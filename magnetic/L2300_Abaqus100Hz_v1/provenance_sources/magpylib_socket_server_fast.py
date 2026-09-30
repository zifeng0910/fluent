#!/usr/bin/env python
"""TCP server for Abaqus <-> Magpylib live magnetic load coupling.

Run with the Abaqus 2025 Python environment that has Magpylib installed::

    I:\SIMULIA\EstProducts\2025\win_b64\tools\SMApy\python3.10\python.exe \
        J:\magpy\magpylib_socket_server.py

Wire protocol: UTF-8 JSON, one object per newline.  After ``hello``, send::

    {"type":"pose", "t_s":0.0123,
     "position_mm":[X,Y,Z], "ur_rad":[UR1,UR2,UR3]}

The response has ``force_N`` and ``torque_Nmm`` arrays.  All incoming
coordinates are in mm; all returned force/torque values are N and N*mm.
Magpylib itself operates internally in SI, so conversions occur only at its
API boundary.
"""
from __future__ import print_function

import argparse
import csv
import json
import math
import os
import socket
import sys
import time

import numpy as np


HOST = '127.0.0.1'
PORT = 65432
MU0 = 4.0 * math.pi * 1e-7
FIELD_FRAME_MODES = ('LEGACY_DRIVER_FRAME', 'ROBOT_LOCAL_TANGENT', 'ROBOT_LOCAL_ROCKING',
                     'ROBOT_LOCAL_PRECESSING_ROCKING', 'ROBOT_LOCAL_ELLIPTIC_ROCKING')
ROBOT_LOCAL_FRAME_MODES = ('ROBOT_LOCAL_TANGENT', 'ROBOT_LOCAL_ROCKING',
                           'ROBOT_LOCAL_PRECESSING_ROCKING', 'ROBOT_LOCAL_ELLIPTIC_ROCKING')


class RigidFrameTransform(object):
    """One authoritative rigid transform between Abaqus and Magpylib.

    The JSON file is the single source of truth.  Abaqus sends positions and
    rotation vectors in its original assembly frame.  The server evaluates
    the magnetic model in the flat-DXF frame and maps polar (force) and axial
    (torque) vectors back with R.T.  R is a proper rotation (det=+1), so both
    vector types use the same mapping.
    """

    def __init__(self, path=None):
        self.path = path
        self.origin_aba_mm = np.zeros(3)
        self.R_aba_to_mag = np.eye(3)
        self.name = 'IDENTITY'
        if path:
            with open(path, 'r') as stream:
                data = json.load(stream)
            self.origin_aba_mm = np.asarray(data['origin_aba_mm'], dtype=float)
            self.R_aba_to_mag = np.asarray(data['R_aba_to_mag'], dtype=float)
            self.name = '{} -> {}'.format(data.get('source_frame', 'ABAQUS'),
                                           data.get('target_frame', 'MAGPYLIB'))
            if self.origin_aba_mm.shape != (3,) or self.R_aba_to_mag.shape != (3, 3):
                raise ValueError('frame transform requires origin[3] and R[3,3]')
            orth_error = np.max(np.abs(np.dot(self.R_aba_to_mag.T,
                                               self.R_aba_to_mag) - np.eye(3)))
            determinant = float(np.linalg.det(self.R_aba_to_mag))
            if orth_error > 1.0e-10 or abs(determinant - 1.0) > 1.0e-10:
                raise ValueError('frame transform is not a proper rotation: orth_error={} det={}'.format(
                    orth_error, determinant))
            pair = data.get('validation_pair')
            if pair:
                mapped = self.position_to_mag(pair['abaqus_rp_mm'])
                expected = np.asarray(pair['magpylib_rp_mm'], dtype=float)
                error = float(np.linalg.norm(mapped - expected))
                if error > 1.0e-8:
                    raise ValueError('frame validation pair mismatch: {:.12g} mm'.format(error))

    def position_to_mag(self, position_aba_mm):
        return np.dot(self.R_aba_to_mag,
                      np.asarray(position_aba_mm, dtype=float) - self.origin_aba_mm)

    def vector_to_mag(self, vector_aba):
        return np.dot(self.R_aba_to_mag, np.asarray(vector_aba, dtype=float))

    def vector_to_aba(self, vector_mag):
        return np.dot(self.R_aba_to_mag.T, np.asarray(vector_mag, dtype=float))


class MagneticCouplingModel(object):
    """Finite drive magnet acting on an axial N52 cylindrical robot magnet.

    The robot is represented by the dipole-equivalent moment of a finite
    axially magnetized N52 cylinder.  This keeps the force/torque evaluation
    compatible with the existing field-gradient coupling while using the
    actual 0.8 mm x 2.0 mm robot magnet geometry.
    """

    def __init__(self, dxf_path, drive_type='sphere', drive_diameter_mm=50.0,
                 drive_height_mm=None, drive_length_mm=None, drive_width_mm=None,
                 br_t=1.46,
                 robot_diameter_mm=0.8, robot_height_mm=2.0,
                 robot_br_t=1.46, robot_moment_Am2=None,
                 robot_mass_mg=10.0,
                 driver_speed_mm_s=5.0,
                 bend_speed_mm_s=None,
                 bend_start_mm=13.49,
                 bend_end_mm=18.56,
                 z_offset_mm=57.0, driver_start_offset_mm=0.0,
                 spin_hz=5.0, cone_half_angle_deg=30.0,
                 gradient_step_mm=0.01, align_origin_mm=None,
                 align_tangent=None, driver_xy_shift_mm=None,
                 trajectory_position_angle_deg=0.0,
                 cone_axis_bias_deg=0.0,
                 cone_frame_azimuth_deg=0.0,
                 tangent_only=False, zero_torque=False,
                 ramp_time_s=0.001, endpoint_taper_mm=1.0,
                 adaptive_lead_mm=0.0, reverse_trajectory=False,
                 analytic_b_t=0.010, analytic_follow_robot=False,
                 field_frame_mode='LEGACY_DRIVER_FRAME', rocking_amplitude_deg=10.0,
                 rocking_cross_amplitude_deg=0.0,
                 rocking_frame_azimuth_deg=-61.37284757596327,
                 precession_frequency_hz=2.5, precession_phase_deg=-83.87284757596327,
                 analytic_gradient_b_t=0.0, analytic_gradient_length_mm=25.0,
                 analytic_gradient_profile='legacy', analytic_gradient_switch_start_s=0.0,
                 analytic_gradient_transition_s=0.0005, analytic_gradient_local_b_t=None,
                 analytic_gradient_local_length_mm=3.75,
                 analytic_gradient_tilt_deg=0.0, analytic_gradient_azimuth_deg=0.0,
                 analytic_rotation_sense=1.0,
                 chirp_start_hz=None, chirp_end_hz=None, chirp_duration_s=0.0,
                 directional_pulse=False, directional_pulse_phase_deg=0.0,
                 directional_pulse_halfwidth_deg=5.0, directional_pulse_ft_n=0.0,
                 directional_pulse_fn_n=0.0, directional_pulse_normal_mag=None):
        try:
            import magpylib as magpy
            import ezdxf
        except ImportError as exc:
            raise RuntimeError(
                'Missing dependency: {}. Use the configured Abaqus Python and install '
                'magpylib, ezdxf and numpy.'.format(exc)
            )
        self.magpy = magpy
        if drive_type not in ('sphere', 'cylinder', 'cuboid', 'analytic'):
            raise ValueError('drive_type must be sphere, cylinder, cuboid, or analytic')
        if br_t is None or float(br_t) <= 0:
            raise ValueError('drive Br/polarization must be positive')
        self.drive_type = drive_type
        self.br_t = float(br_t)
        # Analytic rotating-field amplitude (tesla).  This is deliberately
        # independent of the legacy physical-source Br/polarization input.
        self.analytic_b_t = float(analytic_b_t)
        self.analytic_follow_robot = bool(analytic_follow_robot)
        self.field_frame_mode = str(field_frame_mode).upper()
        if self.field_frame_mode not in FIELD_FRAME_MODES:
            raise ValueError('field_frame_mode must be one of {}'.format(', '.join(FIELD_FRAME_MODES)))
        self.rocking_amplitude_deg = float(rocking_amplitude_deg)
        if not (0.0 <= self.rocking_amplitude_deg < 90.0):
            raise ValueError('rocking_amplitude_deg must satisfy 0 <= amplitude < 90 deg')
        self.rocking_cross_amplitude_deg = float(rocking_cross_amplitude_deg)
        if not (0.0 <= self.rocking_cross_amplitude_deg < 90.0):
            raise ValueError('rocking_cross_amplitude_deg must satisfy 0 <= amplitude < 90 deg')
        self.rocking_frame_azimuth_deg = float(rocking_frame_azimuth_deg)
        self.precession_frequency_hz = float(precession_frequency_hz)
        self.precession_phase_deg = float(precession_phase_deg)
        if self.precession_frequency_hz < 0.0:
            raise ValueError('precession_frequency_hz must be non-negative')
        self.analytic_gradient_b_t = float(analytic_gradient_b_t)
        self.analytic_gradient_length_mm = float(analytic_gradient_length_mm)
        self.analytic_gradient_profile = str(analytic_gradient_profile)
        self.analytic_gradient_switch_start_s = float(analytic_gradient_switch_start_s)
        self.analytic_gradient_transition_s = float(analytic_gradient_transition_s)
        self.analytic_gradient_local_b_t = (self.analytic_gradient_b_t if analytic_gradient_local_b_t is None else float(analytic_gradient_local_b_t))
        self.analytic_gradient_local_length_mm = float(analytic_gradient_local_length_mm)
        # Spatial direction of the prescribed gradient.  Defaults to the
        # historical tangent direction, so existing production jobs are
        # bit-for-bit unchanged unless these optional arguments are set.
        self.analytic_gradient_tilt_deg = float(analytic_gradient_tilt_deg)
        self.analytic_gradient_azimuth_deg = float(analytic_gradient_azimuth_deg)
        self.analytic_rotation_sense = float(analytic_rotation_sense)
        self.directional_pulse = bool(directional_pulse)
        self.directional_pulse_phase_deg = float(directional_pulse_phase_deg)
        self.directional_pulse_halfwidth_deg = float(directional_pulse_halfwidth_deg)
        self.directional_pulse_ft_n = float(directional_pulse_ft_n)
        self.directional_pulse_fn_n = float(directional_pulse_fn_n)
        self.directional_pulse_normal_mag = None if directional_pulse_normal_mag is None else np.asarray(directional_pulse_normal_mag, dtype=float)
        if self.directional_pulse and self.directional_pulse_normal_mag is None:
            raise ValueError('directional pulse requires a fixed normal vector in Magpylib coordinates')
        if self.directional_pulse and np.linalg.norm(self.directional_pulse_normal_mag) < 1e-12:
            raise ValueError('directional pulse normal vector must be non-zero')
        if self.directional_pulse_halfwidth_deg <= 0 or self.directional_pulse_halfwidth_deg >= 180:
            raise ValueError('directional pulse halfwidth must be in (0,180) degrees')
        if self.analytic_rotation_sense not in (-1.0, 1.0):
            raise ValueError('analytic_rotation_sense must be +1 or -1')
        if self.analytic_b_t <= 0:
            raise ValueError('analytic B amplitude must be positive')
        if self.analytic_gradient_b_t < 0 or self.analytic_gradient_length_mm <= 0:
            raise ValueError('analytic gradient amplitude/length must be non-negative/positive')
        if self.analytic_gradient_profile not in ('legacy','broad-to-local'):
            raise ValueError('analytic_gradient_profile must be legacy or broad-to-local')
        if self.analytic_gradient_switch_start_s < 0 or self.analytic_gradient_transition_s <= 0:
            raise ValueError('gradient switch start must be non-negative and transition positive')
        if self.analytic_gradient_local_b_t < 0 or self.analytic_gradient_local_length_mm <= 0:
            raise ValueError('local gradient amplitude/length must be non-negative/positive')
        self.drive_diameter_mm = None if drive_diameter_mm is None else float(drive_diameter_mm)
        self.drive_height_mm = None if drive_height_mm is None else float(drive_height_mm)
        self.drive_length_mm = None if drive_length_mm is None else float(drive_length_mm)
        self.drive_width_mm = None if drive_width_mm is None else float(drive_width_mm)
        if drive_type in ('sphere', 'cylinder') and (self.drive_diameter_mm is None or self.drive_diameter_mm <= 0):
            raise ValueError('drive diameter is required for sphere/cylinder')
        if drive_type == 'cylinder' and (self.drive_height_mm is None or self.drive_height_mm <= 0):
            raise ValueError('drive height is required for cylinder')
        if drive_type == 'cuboid' and any(v is None or v <= 0 for v in (self.drive_length_mm, self.drive_width_mm, self.drive_height_mm)):
            raise ValueError('drive length/width/height are required for cuboid')
        self.robot_diameter_mm = float(robot_diameter_mm)
        self.robot_height_mm = float(robot_height_mm)
        self.robot_br_t = float(robot_br_t)
        # Mass is carried as metadata for the coupled dynamics model. It does
        # not enter Magpylib's magnetic field calculation.
        self.robot_mass_mg = float(robot_mass_mg)
        if robot_moment_Am2 is None:
            radius_m = 0.5 * self.robot_diameter_mm * 1e-3
            height_m = self.robot_height_mm * 1e-3
            volume_m3 = math.pi * radius_m * radius_m * height_m
            self.robot_moment_Am2 = self.robot_br_t * volume_m3 / MU0
        else:
            self.robot_moment_Am2 = float(robot_moment_Am2)
        self.driver_speed_mm_s = float(driver_speed_mm_s)
        self.bend_speed_mm_s = None if bend_speed_mm_s is None else float(bend_speed_mm_s)
        self.bend_start_mm = float(bend_start_mm)
        self.bend_end_mm = float(bend_end_mm)
        self.z_offset_mm = float(z_offset_mm)
        self.driver_start_offset_mm = max(0.0, float(driver_start_offset_mm))
        self.driver_xy_shift_mm = np.asarray(driver_xy_shift_mm if driver_xy_shift_mm is not None else [0.0, 0.0], dtype=float)
        self.trajectory_position_angle_deg = float(trajectory_position_angle_deg)
        self.cone_axis_bias_deg = float(cone_axis_bias_deg)
        # Rigidly rotate the complete biased cone frame about the local pipe
        # tangent.  Zero is the legacy production frame exactly.
        self.cone_frame_azimuth_deg = float(cone_frame_azimuth_deg)
        self.driver_normal_offset_mm = 0.0
        # Initial RP position of the transformed curvenew CEL assembly.
        # Keep this synchronized with vuforc_socket_bridge.f and RP_ROBOT in
        # the active input deck.
        self.robot_initial_position_mm = np.array([11.0983529192, 6.6000750094, -0.00325443519617], dtype=float)
        # PCA axis of the imported Robot_SOLID geometry in assembly coordinates.
        # It is almost parallel to the pipe entrance tangent; using global +X
        # here was the source of the spurious wall-clamping torque.
        self.robot_axis_global = np.array([-0.96270929, 0.12195883, -0.24148886], dtype=float)
        self.robot_axis_global /= np.linalg.norm(self.robot_axis_global)
        self.robot_polarity = 1.0
        self.cone_axis_tangent = False
        self.phase_offset_rad = 0.0
        self.spin_hz = float(spin_hz)
        self.chirp_start_hz = None if chirp_start_hz is None else float(chirp_start_hz)
        self.chirp_end_hz = None if chirp_end_hz is None else float(chirp_end_hz)
        self.chirp_duration_s = max(float(chirp_duration_s), 0.0)
        if self.chirp_start_hz is not None or self.chirp_end_hz is not None:
            if self.chirp_start_hz is None or self.chirp_end_hz is None or self.chirp_duration_s <= 0.0:
                raise ValueError('chirp_start_hz, chirp_end_hz and positive chirp_duration_s are required together')
            if self.chirp_start_hz <= 0.0 or self.chirp_end_hz <= 0.0:
                raise ValueError('chirp frequencies must be positive')
        self.cone_half_angle_deg = float(cone_half_angle_deg)
        self.tangent_only = bool(tangent_only)
        self.zero_torque = bool(zero_torque)
        self.ramp_time_s = max(float(ramp_time_s), 0.0)
        self.endpoint_taper_mm = max(float(endpoint_taper_mm), 0.0)
        # Optional diagnostic lead guard.  When enabled, the magnetic source
        # follows the robot's projected arc position plus a prescribed lead.
        # This is intentionally opt-in because it is a supervisory test mode,
        # not a replacement for the physical fixed-speed magnet trajectory.
        self.adaptive_lead_mm = max(float(adaptive_lead_mm), 0.0)
        # The DXF arclength is stored from the left endpoint toward the
        # right endpoint, while the physical experiment enters from the
        # right and advances toward decreasing arclength.  This switch
        # makes the direction explicit instead of silently reversing the
        # interpretation in a downstream bridge.
        self.reverse_trajectory = bool(reverse_trajectory)
        self.gradient_step_m = float(gradient_step_mm) * 1e-3
        self.curve_mm = self._read_dxf_curve(ezdxf, dxf_path)
        if align_origin_mm is not None and align_tangent is not None:
            self.curve_mm = self._align_curve(self.curve_mm, align_origin_mm, align_tangent)
        self.arc_mm = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.curve_mm, axis=0), axis=1))]
        self._previous_robot_arc_mm = None
        self._local_tangent_sign = None
        self._local_segment_tangent = np.diff(self.curve_mm, axis=0)
        self._local_segment_tangent /= np.linalg.norm(self._local_segment_tangent, axis=1)[:, None]
        self._local_e1 = self._parallel_transport_normals(self._local_segment_tangent)
        if self.field_frame_mode in ROBOT_LOCAL_FRAME_MODES:
            if self.drive_type != 'analytic':
                raise ValueError('{} is defined only for analytic drive_type'.format(self.field_frame_mode))
            if abs(self.cone_axis_bias_deg) > 1.0e-15:
                raise ValueError('{} requires cone_axis_bias_deg=0; legacy global-Z bias is not reused'.format(self.field_frame_mode))

    @staticmethod
    def _align_curve(points, origin, tangent):
        """Rigidly map DXF curve start/tangent into assembly coordinates."""
        p = np.asarray(points, dtype=float)
        a = p[1] - p[0]; a = a / np.linalg.norm(a)
        b = np.asarray(tangent, dtype=float); b = b / np.linalg.norm(b)
        v = np.cross(a, b); s = np.linalg.norm(v); c = float(np.dot(a, b))
        if s < 1e-12:
            R = np.eye(3) if c > 0 else -np.eye(3)
        else:
            vx = np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
            R = np.eye(3) + vx + np.dot(vx, vx) * ((1.0-c)/(s*s))
        q = np.dot(p-p[0], R.T) + np.asarray(origin, dtype=float)
        return q

    @staticmethod
    def _as_vec3(point):
        return np.array([point.x, point.y, point.z], dtype=float)

    def _read_dxf_curve(self, ezdxf, dxf_path):
        """Flatten supported DXF curves and join contiguous pieces."""
        doc = ezdxf.readfile(dxf_path)
        pieces = []
        for entity in doc.modelspace():
            if entity.dxftype() not in ('LINE', 'ARC', 'CIRCLE', 'LWPOLYLINE', 'POLYLINE', 'SPLINE'):
                continue
            try:
                # ezdxf exposes 3-D POLYLINE vertices through the child
                # vertex records rather than through ``flattening``.  The
                # reconstructed pipe centerline is intentionally written as
                # a 3-D POLYLINE, so preserve its Z coordinates here.
                if entity.dxftype() == 'POLYLINE':
                    try:
                        children = list(entity.vertices())
                    except TypeError:
                        children = list(entity.vertices)
                    vertices = np.array([self._as_vec3(child.dxf.location) for child in children], dtype=float)
                else:
                    vertices = np.array([self._as_vec3(point) for point in entity.flattening(0.01)], dtype=float)
            except (AttributeError, TypeError):
                if entity.dxftype() != 'LINE':
                    continue
                vertices = np.array([self._as_vec3(entity.dxf.start), self._as_vec3(entity.dxf.end)])
            if len(vertices) > 1 and np.linalg.norm(vertices[-1] - vertices[0]) > 1e-12:
                pieces.append(vertices)
        if not pieces:
            raise RuntimeError('No supported non-zero length curve found in DXF: ' + dxf_path)
        joined = pieces.pop(0)
        while pieces:
            # Attach to either end.  The previous implementation only
            # appended to joined[-1], so an ARC with one LINE on each end
            # silently dropped the second line.
            start, end = joined[0], joined[-1]
            candidates=[]
            for index,item in enumerate(pieces):
                candidates.extend([(np.linalg.norm(end-item[0]), index, 'append', False),
                                   (np.linalg.norm(end-item[-1]), index, 'append', True),
                                   (np.linalg.norm(start-item[-1]), index, 'prepend', False),
                                   (np.linalg.norm(start-item[0]), index, 'prepend', True)])
            gap,index,where,reverse=min(candidates, key=lambda value:value[0])
            if gap > 0.02:
                break
            item=pieces.pop(index)
            if reverse: item=item[::-1]
            if where=='append': joined=np.vstack((joined,item[1:]))
            else: joined=np.vstack((item[:-1],joined))
        if len(joined) < 2:
            raise RuntimeError('DXF curve has insufficient connected points')
        return self._resample(joined, 0.1)

    @staticmethod
    def _resample(points, ds_mm):
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(lengths)]
        samples = np.r_[np.arange(0.0, s[-1], ds_mm), s[-1]]
        return np.column_stack([np.interp(samples, s, points[:, index]) for index in range(3)])

    @staticmethod
    def rotation_matrix_from_ur(ur_rad):
        """Rodrigues exponential map: Abaqus UR rotation vector -> 3x3 matrix."""
        vector = np.asarray(ur_rad, dtype=float)
        angle = np.linalg.norm(vector)
        if angle < 1e-12:
            return np.eye(3)
        axis = vector / angle
        cross = np.array([[0.0, -axis[2], axis[1]],
                          [axis[2], 0.0, -axis[0]],
                          [-axis[1], axis[0], 0.0]])
        return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * np.dot(cross, cross)

    def _project_arc_mm(self, position_mm):
        p = np.asarray(position_mm, dtype=float)
        delta = self.curve_mm - p.reshape(1, 3)
        idx = int(np.argmin(np.einsum('ij,ij->i', delta, delta)))
        return float(self.arc_mm[idx])

    @staticmethod
    def _parallel_transport_normals(tangents):
        """Build one continuous minimal-rotation normal for every segment."""
        first = np.asarray(tangents[0], dtype=float)
        normal = np.array([0.0, 0.0, 1.0]) - first[2] * first
        if np.linalg.norm(normal) < 1.0e-12:
            normal = np.array([0.0, 1.0, 0.0]) - first[1] * first
        normal /= max(np.linalg.norm(normal), 1.0e-15)
        frames = [normal]
        for previous, current in zip(tangents[:-1], tangents[1:]):
            cross = np.cross(previous, current)
            sine = float(np.linalg.norm(cross))
            cosine = float(np.clip(np.dot(previous, current), -1.0, 1.0))
            if sine > 1.0e-12:
                axis = cross / sine
                angle = math.atan2(sine, cosine)
                candidate = (frames[-1] * math.cos(angle) +
                             np.cross(axis, frames[-1]) * math.sin(angle) +
                             axis * np.dot(axis, frames[-1]) * (1.0 - math.cos(angle)))
            else:
                candidate = frames[-1].copy()
            candidate -= np.dot(candidate, current) * current
            candidate /= max(np.linalg.norm(candidate), 1.0e-15)
            if np.dot(candidate, frames[-1]) < 0.0:
                candidate *= -1.0
            frames.append(candidate)
        return np.asarray(frames)

    def _project_robot_arc_mm(self, position_mm):
        """Project onto curve segments with continuity and endpoint extrapolation."""
        point = np.asarray(position_mm, dtype=float)
        origin = self.curve_mm[:-1]
        edge = np.diff(self.curve_mm, axis=0)
        length2 = np.einsum('ij,ij->i', edge, edge)
        raw_fraction = np.einsum('ij,ij->i', point - origin, edge) / length2
        fraction = np.clip(raw_fraction, 0.0, 1.0)
        fraction[0] = min(raw_fraction[0], 1.0)
        fraction[-1] = max(raw_fraction[-1], 0.0)
        closest = origin + fraction[:, None] * edge
        distance2 = np.einsum('ij,ij->i', closest - point, closest - point)
        candidate_arc = self.arc_mm[:-1] + fraction * np.sqrt(length2)
        minimum = float(distance2.min())
        candidates = np.flatnonzero(distance2 <= minimum + 1.0e-12)
        if self._previous_robot_arc_mm is None or len(candidates) == 1:
            index = int(candidates[0])
        else:
            index = int(candidates[np.argmin(np.abs(candidate_arc[candidates] - self._previous_robot_arc_mm))])
        self._previous_robot_arc_mm = float(candidate_arc[index])
        return self._previous_robot_arc_mm

    def reset_robot_arc_continuity(self):
        self._previous_robot_arc_mm = None

    def _robot_local_frame(self, robot_arc_mm):
        """Return polarity-fixed (c,e1,e2) at the projected robot arc."""
        distance = min(max(float(robot_arc_mm), 0.0), float(self.arc_mm[-1]))
        raw_tangent = self._tangent_at_distance(distance)
        if self._local_tangent_sign is None:
            reference = self.robot_polarity * self.robot_axis_global
            self._local_tangent_sign = 1.0 if np.dot(raw_tangent, reference) > 0.0 else -1.0
        tangent = self._local_tangent_sign * raw_tangent
        index = int(np.searchsorted(self.arc_mm, distance, side='right') - 1)
        index = max(0, min(index, len(self._local_e1) - 1))
        if index < len(self._local_e1) - 1:
            span = max(float(self.arc_mm[index + 1] - self.arc_mm[index]), 1.0e-15)
            weight = min(max((distance - float(self.arc_mm[index])) / span, 0.0), 1.0)
            e1 = (1.0 - weight) * self._local_e1[index] + weight * self._local_e1[index + 1]
        else:
            e1 = self._local_e1[index].copy()
        e1 -= np.dot(e1, tangent) * tangent
        e1 /= max(np.linalg.norm(e1), 1.0e-15)
        e2 = np.cross(tangent, e1)
        e2 /= max(np.linalg.norm(e2), 1.0e-15)
        return tangent, e1, e2

    def _robot_rocking_frame(self, robot_arc_mm):
        """Return RouteA-gauged (c,n_rock,b_rock) in the production PT frame."""
        tangent, e1, e2 = self._robot_local_frame(robot_arc_mm)
        chi = math.radians(self.rocking_frame_azimuth_deg)
        n_rock = math.cos(chi) * e1 + math.sin(chi) * e2
        n_rock /= max(np.linalg.norm(n_rock), 1.0e-15)
        b_rock = np.cross(tangent, n_rock)
        b_rock /= max(np.linalg.norm(b_rock), 1.0e-15)
        return tangent, n_rock, b_rock

    def _command_frequency_hz(self, t_s):
        """Continuous linear chirp frequency; legacy runs remain constant spin_hz."""
        if self.chirp_start_hz is None:
            return float(self.spin_hz)
        tau = max(0.0, min(float(t_s), self.chirp_duration_s))
        return float(self.chirp_start_hz + (self.chirp_end_hz - self.chirp_start_hz) * tau / self.chirp_duration_s)

    def _command_phase_rad(self, t_s):
        """Phase obtained by integrating the commanded frequency exactly."""
        t = max(0.0, float(t_s))
        if self.chirp_start_hz is None:
            return self.phase_offset_rad + self.analytic_rotation_sense * 2.0 * math.pi * self.spin_hz * t
        tc = min(t, self.chirp_duration_s)
        slope = (self.chirp_end_hz - self.chirp_start_hz) / self.chirp_duration_s
        integral = self.chirp_start_hz * tc + 0.5 * slope * tc * tc
        if t > self.chirp_duration_s:
            integral += self.chirp_end_hz * (t - self.chirp_duration_s)
        return self.phase_offset_rad + self.analytic_rotation_sense * 2.0 * math.pi * integral

    def _rocking_phase_rad(self, t_s):
        """Unsigned oscillator phase; cone winding sense is not a rocking control."""
        t = max(0.0, float(t_s))
        if self.chirp_start_hz is None:
            return self.phase_offset_rad + 2.0 * math.pi * self.spin_hz * t
        tc = min(t, self.chirp_duration_s)
        slope = (self.chirp_end_hz - self.chirp_start_hz) / self.chirp_duration_s
        integral = self.chirp_start_hz * tc + 0.5 * slope * tc * tc
        if t > self.chirp_duration_s:
            integral += self.chirp_end_hz * (t - self.chirp_duration_s)
        return self.phase_offset_rad + 2.0 * math.pi * integral

    def _precession_phase_rad(self, t_s):
        """Plane azimuth in the raw parallel-transport (e1,e2) frame."""
        return (math.radians(self.precession_phase_deg) +
                2.0 * math.pi * self.precession_frequency_hz * max(0.0, float(t_s)))

    def _driver_pose(self, t_s, robot_position_mm=None):
        distance = self._distance_at_time(t_s)
        robot_arc_mm = None
        if robot_position_mm is not None and self.field_frame_mode in ROBOT_LOCAL_FRAME_MODES:
            robot_arc_mm = self._project_robot_arc_mm(robot_position_mm)
        elif self.adaptive_lead_mm > 0.0 and robot_position_mm is not None:
            robot_arc_mm = self._project_arc_mm(robot_position_mm)
        if self.adaptive_lead_mm > 0.0 and robot_arc_mm is not None:
            # Never allow the source to fall behind the robot in this
            # diagnostic mode.  The resulting source speed is intentionally
            # reported so it cannot be mistaken for a physical 6 mm/s run.
            distance = max(distance, robot_arc_mm + self.adaptive_lead_mm)
            distance = min(distance, float(self.arc_mm[-1]))
        position = np.array([np.interp(distance, self.arc_mm, self.curve_mm[:, index]) for index in range(3)])
        position[0] += self.driver_xy_shift_mm[0]
        position[1] += self.driver_xy_shift_mm[1]
        if abs(self.trajectory_position_angle_deg) < 1.0e-15:
            # Preserve the historical global-Z trajectory exactly when the
            # new physical trajectory-angle control is not requested.
            position[2] += self.z_offset_mm
        else:
            tangent = self._tangent_at_distance(distance)
            znormal = self._z_reference_normal(tangent)
            alpha = math.radians(self.trajectory_position_angle_deg)
            position += self.z_offset_mm * (math.cos(alpha) * znormal +
                                             math.sin(alpha) * tangent)
        if self.driver_normal_offset_mm:
            position += self.driver_normal_offset_mm * self.driver_normal_dir
        # At t=0, magnetization is tilted toward +Z by the cone half-angle.
        # Its +X component stays fixed while the transverse YZ component
        # self-spins, yielding a cone about the global +X axis.
        angle = self._command_phase_rad(t_s)
        cone_angle = math.radians(self.cone_half_angle_deg)
        if self.cone_axis_tangent:
            # Follow the local DXF tangent at the driver's current arc-length
            # position.  Using curve_mm[1]-curve_mm[0] here kept the field
            # orientation frozen at the inlet and caused Ft to reverse in a
            # bend even though the magnetic force itself remained forward.
            tangent = self._tangent_at_distance(distance)
            if abs(self.cone_axis_bias_deg) < 1.0e-15:
                # Historical basis retained for existing jobs.
                normal = np.cross(tangent, np.array([0., 0., 1.]))
                if np.linalg.norm(normal) < 1e-12:
                    normal = np.cross(tangent, np.array([0., 1., 0.]))
                normal /= np.linalg.norm(normal)
                binormal = np.cross(tangent, normal); binormal /= np.linalg.norm(binormal)
                direction = (math.cos(cone_angle) * tangent +
                             math.sin(cone_angle) * (math.cos(angle) * normal + math.sin(angle) * binormal))
            else:
                znormal = self._z_reference_normal(tangent)
                beta = math.radians(self.cone_axis_bias_deg)
                axis = math.cos(beta) * tangent + math.sin(beta) * znormal
                axis /= max(np.linalg.norm(axis), 1.0e-15)
                e1 = znormal - np.dot(znormal, axis) * axis
                if np.linalg.norm(e1) < 1.0e-12:
                    e1 = np.cross(axis, np.array([0., 1., 0.]))
                e1 /= max(np.linalg.norm(e1), 1.0e-15)
                e2 = np.cross(axis, e1); e2 /= max(np.linalg.norm(e2), 1.0e-15)
                if abs(self.cone_frame_azimuth_deg) > 1.0e-15:
                    chi = math.radians(self.cone_frame_azimuth_deg)
                    # Rodrigues rotation about the authoritative local
                    # tangent; rotate axis, e1 and e2 together.
                    def _rot_about(v, k, a):
                        return (v*math.cos(a) + np.cross(k, v)*math.sin(a) +
                                k*np.dot(k, v)*(1.0-math.cos(a)))
                    axis = _rot_about(axis, tangent, chi); axis /= max(np.linalg.norm(axis), 1.0e-15)
                    e1 = _rot_about(e1, tangent, chi); e1 /= max(np.linalg.norm(e1), 1.0e-15)
                    e2 = _rot_about(e2, tangent, chi); e2 /= max(np.linalg.norm(e2), 1.0e-15)
                direction = (math.cos(cone_angle) * axis +
                             math.sin(cone_angle) * (math.cos(angle) * e1 + math.sin(angle) * e2))
        else:
            initial = np.array([math.cos(cone_angle), 0.0, math.sin(cone_angle)])
            direction = np.array([initial[0], math.cos(angle) * initial[1] - math.sin(angle) * initial[2],
                                  math.sin(angle) * initial[1] + math.cos(angle) * initial[2]])
        return position, direction, float(distance), robot_arc_mm

    @staticmethod
    def _z_reference_normal(tangent):
        """Projected global +Z used by the physical trajectory scan."""
        tangent = np.asarray(tangent, dtype=float)
        tangent /= max(np.linalg.norm(tangent), 1.0e-15)
        normal = np.array([0., 0., 1.]) - tangent[2] * tangent
        if np.linalg.norm(normal) < 1.0e-12:
            yaxis = np.array([0., 1., 0.])
            normal = yaxis - np.dot(yaxis, tangent) * tangent
        return normal / max(np.linalg.norm(normal), 1.0e-15)

    def _tangent_at_distance(self, distance_mm):
        # Blend neighboring polyline directions.  A raw segment switch at a
        # DXF vertex causes a force-direction jump exactly at the bend.
        d = min(max(float(distance_mm), 0.0), float(self.arc_mm[-1]))
        j = int(np.searchsorted(self.arc_mm, d, side='right') - 1)
        j = max(0, min(j, len(self.curve_mm) - 2))
        seg0 = self.curve_mm[j + 1] - self.curve_mm[j]
        seg0 /= max(np.linalg.norm(seg0), 1e-12)
        if j >= len(self.curve_mm) - 2:
            return seg0
        seg1 = self.curve_mm[j + 2] - self.curve_mm[j + 1]
        seg1 /= max(np.linalg.norm(seg1), 1e-12)
        span = max(float(self.arc_mm[j + 1] - self.arc_mm[j]), 1e-12)
        w = min(max((d - float(self.arc_mm[j])) / span, 0.0), 1.0)
        tangent = (1.0 - w) * seg0 + w * seg1
        return tangent / max(np.linalg.norm(tangent), 1e-12)

    def _distance_at_time(self, t_s):
        """Arc-length position with optional slower bend segment."""
        if self.reverse_trajectory:
            # In reverse mode driver_start_offset_mm is an absolute DXF
            # arclength station, not a distance travelled from the inlet.
            d = min(max(float(self.driver_start_offset_mm), 0.0), float(self.arc_mm[-1]))
            t = max(float(t_s), 0.0)
            if self.bend_speed_mm_s is None or self.bend_speed_mm_s <= 0.0:
                return max(d - t * max(self.driver_speed_mm_s, 1.0e-12), 0.0)
            a = max(0.0, min(self.bend_start_mm, self.arc_mm[-1]))
            b = max(a, min(self.bend_end_mm, self.arc_mm[-1]))
            v0 = max(self.driver_speed_mm_s, 1.0e-12)
            vb = max(self.bend_speed_mm_s, 1.0e-12)
            if d > b:
                dt = (d - b) / v0
                if t <= dt:
                    return d - t * v0
                t -= dt; d = b
            if d > a:
                dt = (d - a) / vb
                if t <= dt:
                    return d - t * vb
                t -= dt; d = a
            return max(d - t * v0, 0.0)
        d0 = max(self.driver_start_offset_mm, 0.0)
        t = max(float(t_s), 0.0)
        if self.bend_speed_mm_s is None or self.bend_speed_mm_s <= 0.0:
            return min(d0 + t * self.driver_speed_mm_s, self.arc_mm[-1])
        a = max(d0, min(self.bend_start_mm, self.arc_mm[-1]))
        b = max(a, min(self.bend_end_mm, self.arc_mm[-1]))
        v0 = max(self.driver_speed_mm_s, 1.0e-12); vb = max(self.bend_speed_mm_s, 1.0e-12)
        ta = max(0.0, (a - d0) / v0); tb = ta + max(0.0, (b - a) / vb)
        if t <= ta: return min(d0 + t * v0, self.arc_mm[-1])
        if t <= tb: return min(a + (t - ta) * vb, self.arc_mm[-1])
        return min(b + (t - tb) * v0, self.arc_mm[-1])

    def _drive_scale(self, t_s, distance_mm):
        """Smoothly turn on the drive and taper it to zero at the DXF end."""
        t = max(float(t_s), 0.0)
        if self.ramp_time_s > 0.0 and t < self.ramp_time_s:
            q = min(max(t / self.ramp_time_s, 0.0), 1.0)
            ramp = q * q * (3.0 - 2.0 * q)
        else:
            ramp = 1.0
        if self.field_frame_mode in ROBOT_LOCAL_FRAME_MODES:
            return ramp
        if self.endpoint_taper_mm <= 0.0:
            end_scale = 0.0 if distance_mm >= self.arc_mm[-1] else 1.0
        elif distance_mm >= self.arc_mm[-1]:
            end_scale = 0.0
        elif distance_mm > self.arc_mm[-1] - self.endpoint_taper_mm:
            q = (self.arc_mm[-1] - distance_mm) / self.endpoint_taper_mm
            q = min(max(float(q), 0.0), 1.0)
            end_scale = q * q * (3.0 - 2.0 * q)
        else:
            end_scale = 1.0
        return ramp * end_scale

    def set_driver_normal_offset(self, offset_mm):
        self.driver_normal_offset_mm = float(offset_mm)
        p0 = self.curve_mm[0].copy(); p0[2] += self.z_offset_mm
        tangent = self.curve_mm[1] - self.curve_mm[0]
        tangent /= max(np.linalg.norm(tangent), 1e-12)
        radial = self.robot_initial_position_mm - p0
        radial -= np.dot(radial, tangent) * tangent
        nr = np.linalg.norm(radial)
        self.driver_normal_dir = radial / nr if nr > 1e-12 else np.array([0., 0., 1.])

    def _field_and_gradient(self, source, observer_m):
        magnetic_field = np.asarray(self.magpy.getB(source, observer_m), dtype=float)
        gradient = np.empty((3, 3), dtype=float)
        for column in range(3):
            delta = np.zeros(3)
            delta[column] = self.gradient_step_m
            plus = np.asarray(self.magpy.getB(source, observer_m + delta), dtype=float)
            minus = np.asarray(self.magpy.getB(source, observer_m - delta), dtype=float)
            gradient[:, column] = (plus - minus) / (2.0 * self.gradient_step_m)
        return magnetic_field, gradient

    @staticmethod
    def _minimum_symmetric_tracefree_tensor(moment, force):
        """Minimum-Frobenius-norm A with A=A.T, trace(A)=0 and A*m=F.

        The tensor is the local Maxwell-consistent first spatial derivative of
        B at the robot COM.  No scalar force projection or body-force shortcut
        is used.  Variables are [Axx,Ayy,Azz,Axy,Axz,Ayz].
        """
        m = np.asarray(moment, dtype=float); f = np.asarray(force, dtype=float)
        C = np.array([[m[0],0,0,m[1],m[2],0],
                      [0,m[1],0,m[0],0,m[2]],
                      [0,0,m[2],0,m[0],m[1]],
                      [1,1,1,0,0,0]], dtype=float)
        x = np.linalg.lstsq(C, np.r_[f, 0.0], rcond=None)[0]
        return np.array([[x[0],x[3],x[4]], [x[3],x[1],x[5]], [x[4],x[5],x[2]]], dtype=float)

    def _directional_pulse(self, phase_rad, moment_global, tangent):
        """Return (force, A, weight) for one open-loop phase window."""
        if not self.directional_pulse:
            return np.zeros(3), np.zeros((3,3)), 0.0
        n = self.directional_pulse_normal_mag / max(np.linalg.norm(self.directional_pulse_normal_mag), 1e-15)
        t = np.asarray(tangent, dtype=float); t = t / max(np.linalg.norm(t), 1e-15)
        d = math.atan2(math.sin(phase_rad - math.radians(self.directional_pulse_phase_deg)),
                       math.cos(phase_rad - math.radians(self.directional_pulse_phase_deg)))
        h = math.radians(self.directional_pulse_halfwidth_deg)
        if abs(d) >= h:
            return np.zeros(3), np.zeros((3,3)), 0.0
        w = 0.5 * (1.0 + math.cos(math.pi * d / h))
        target = w * (self.directional_pulse_ft_n * t + self.directional_pulse_fn_n * n)
        A = self._minimum_symmetric_tracefree_tensor(moment_global, target)
        return A.dot(moment_global), A, w

    def _make_drive_source(self, position_mm, direction):
        polarization = np.asarray(direction, dtype=float) * self.br_t
        if self.drive_type == 'sphere':
            return self.magpy.magnet.Sphere(
                diameter=self.drive_diameter_mm * 1e-3,
                polarization=polarization,
                position=np.asarray(position_mm) * 1e-3)
        if self.drive_type == 'cylinder':
            return self.magpy.magnet.Cylinder(
                dimension=(self.drive_diameter_mm * 1e-3, self.drive_height_mm * 1e-3),
                polarization=polarization,
                position=np.asarray(position_mm) * 1e-3)
        return self.magpy.magnet.Cuboid(
            dimension=(self.drive_length_mm * 1e-3, self.drive_width_mm * 1e-3, self.drive_height_mm * 1e-3),
            polarization=polarization,
            position=np.asarray(position_mm) * 1e-3)

    def evaluate(self, t_s, position_mm, ur_rad):
        """Return a JSON-safe load result at the latest Abaqus RP pose."""
        # A corrupted VUAMP time value must never be interpreted as a valid
        # drive time: doing so would clamp the sphere to the DXF endpoint and
        # create an unbounded, non-physical force.  Fail closed so the bridge
        # can zero the load and stop/reconnect instead.
        t_s = float(t_s)
        if (not math.isfinite(t_s)) or t_s < -1.0e-6 or t_s > 10.0:
            raise ValueError('invalid coupling time %.17g s' % t_s)
        position_mm = np.asarray(position_mm, dtype=float)
        ur_rad = np.asarray(ur_rad, dtype=float)
        if position_mm.shape != (3,) or ur_rad.shape != (3,) or not np.isfinite(position_mm).all() or not np.isfinite(ur_rad).all():
            raise ValueError('position_mm and ur_rad must each contain three finite values')
        driver_position_mm, driver_direction, driver_arc_mm, robot_arc_mm = self._driver_pose(t_s, position_mm)
        rotation = self.rotation_matrix_from_ur(ur_rad)
        # Rotate the actual CAD/PCA robot long axis, not global +X.
        moment_global = rotation.dot(self.robot_polarity * self.robot_moment_Am2 * self.robot_axis_global)
        gradient_weight = 0.0
        gradient_length_used = self.analytic_gradient_length_mm
        gradient_b_used = 0.0
        pulse_force_n = np.zeros(3)
        pulse_A = np.zeros((3, 3))
        pulse_weight = 0.0
        if self.drive_type == 'analytic':
            # In analytic mode there is no physical sphere and therefore no
            # meaningful Z/source position.  Build the Frenet basis at the
            # robot's current centerline location to avoid an artificial
            # spatial lead being mistaken for a magnetic source offset.
            field_arc_mm = robot_arc_mm if (self.analytic_follow_robot and
                                            robot_arc_mm is not None and
                                            np.isfinite(robot_arc_mm)) else driver_arc_mm
            phase = (self._rocking_phase_rad(t_s) if self.field_frame_mode in
                     ('ROBOT_LOCAL_ROCKING', 'ROBOT_LOCAL_PRECESSING_ROCKING',
                      'ROBOT_LOCAL_ELLIPTIC_ROCKING')
                     else self._command_phase_rad(t_s))
            ca = math.radians(self.cone_half_angle_deg)
            if self.field_frame_mode in ROBOT_LOCAL_FRAME_MODES:
                if robot_arc_mm is None or not np.isfinite(robot_arc_mm):
                    raise ValueError('{} requires a finite current robot COM position'.format(self.field_frame_mode))
                if self.field_frame_mode in ('ROBOT_LOCAL_ROCKING', 'ROBOT_LOCAL_ELLIPTIC_ROCKING'):
                    tangent, e1, e2 = self._robot_rocking_frame(robot_arc_mm)
                else:
                    tangent, e1, e2 = self._robot_local_frame(robot_arc_mm)
                axis = tangent
                normal, binormal = e1, e2
            else:
                tangent = self._tangent_at_distance(field_arc_mm)
                normal = self._z_reference_normal(tangent)
                binormal = np.cross(tangent, normal); binormal /= max(np.linalg.norm(binormal), 1.0e-15)
                if abs(self.cone_axis_bias_deg) < 1.0e-15:
                    axis = tangent
                    e1, e2 = normal, binormal
                else:
                    beta = math.radians(self.cone_axis_bias_deg)
                    axis = math.cos(beta) * tangent + math.sin(beta) * normal
                    axis /= max(np.linalg.norm(axis), 1.0e-15)
                    e1 = normal - np.dot(normal, axis) * axis
                    e1 /= max(np.linalg.norm(e1), 1.0e-15)
                    e2 = np.cross(axis, e1)
                    e2 /= max(np.linalg.norm(e2), 1.0e-15)
                    if abs(self.cone_frame_azimuth_deg) > 1.0e-15:
                        chi = math.radians(self.cone_frame_azimuth_deg)
                        def _rot_about(v, k, a):
                            return (v*math.cos(a) + np.cross(k, v)*math.sin(a) +
                                    k*np.dot(k, v)*(1.0-math.cos(a)))
                        axis = _rot_about(axis, tangent, chi); axis /= max(np.linalg.norm(axis), 1.0e-15)
                        e1 = _rot_about(e1, tangent, chi); e1 /= max(np.linalg.norm(e1), 1.0e-15)
                        e2 = _rot_about(e2, tangent, chi); e2 /= max(np.linalg.norm(e2), 1.0e-15)
            if self.field_frame_mode == 'ROBOT_LOCAL_ROCKING':
                # RouteA-gauged (tangent,e1,e2) is right-handed. The field
                # rocks in the (c,n_rock) plane about b_rock without winding.
                alpha = math.radians(self.rocking_amplitude_deg) * math.sin(phase)
                magnetic_field_t = self.analytic_b_t * (
                    math.cos(alpha) * tangent + math.sin(alpha) * e1)
            elif self.field_frame_mode == 'ROBOT_LOCAL_PRECESSING_ROCKING':
                psi = self._precession_phase_rad(t_s)
                n_precess = math.cos(psi) * e1 + math.sin(psi) * e2
                n_precess /= max(np.linalg.norm(n_precess), 1.0e-15)
                alpha = math.radians(self.rocking_amplitude_deg) * math.sin(phase)
                magnetic_field_t = self.analytic_b_t * (
                    math.cos(alpha) * tangent + math.sin(alpha) * n_precess)
            elif self.field_frame_mode == 'ROBOT_LOCAL_ELLIPTIC_ROCKING':
                # Fixed RouteA axes. This is a small elliptical perturbation
                # of HEAD-TAIL rocking, not a precessing rocking plane.
                alpha_main = math.radians(self.rocking_amplitude_deg) * math.sin(phase)
                alpha_cross = math.radians(self.rocking_cross_amplitude_deg) * math.cos(phase)
                direction = (tangent - math.tan(alpha_main) * e1 -
                             math.tan(alpha_cross) * e2)
                direction /= max(np.linalg.norm(direction), 1.0e-15)
                magnetic_field_t = self.analytic_b_t * direction
            elif self.field_frame_mode == 'ROBOT_LOCAL_TANGENT':
                transverse = (math.cos(phase) * e1 +
                              self.analytic_rotation_sense * math.sin(phase) * e2)
                magnetic_field_t = self.analytic_b_t * (
                    math.cos(ca) * axis + math.sin(ca) * transverse)
            else:
                # Preserve the historical production equation exactly.  The
                # command phase already contains analytic_rotation_sense.
                transverse = math.cos(phase) * e1 + math.sin(phase) * e2
                magnetic_field_t = self.analytic_b_t * (
                    math.cos(ca) * axis + math.sin(ca) * transverse)
            # Add a weak, localized axial-gradient component without creating
            # an Abaqus magnet body.  The Gaussian envelope is centered on
            # the prescribed driver arc.  Behind the driver (delta_s<0) the
            # gradient points forward; after overtaking it changes sign.
            robot_arc_eval = robot_arc_mm if (robot_arc_mm is not None and
                                              np.isfinite(robot_arc_mm)) else self._project_arc_mm(position_mm)
            delta_s_mm = float(robot_arc_eval - driver_arc_mm)
            if self.analytic_gradient_b_t > 0.0:
                # Optional time-gated cross-fade.  Legacy mode is exactly the
                # historical single Gaussian evaluation.  Broad-to-local
                # represents two physical gradient channels whose currents
                # are cross-faded with a C1 smoothstep; it is not a sum.
                if self.analytic_gradient_profile == 'broad-to-local':
                    x = ((t_s - self.analytic_gradient_switch_start_s) /
                         self.analytic_gradient_transition_s)
                    x = min(1.0, max(0.0, x))
                    gradient_weight = x*x*(3.0-2.0*x)
                    broad_g, broad_l = self.analytic_gradient_b_t, self.analytic_gradient_length_mm
                    local_g, local_l = self.analytic_gradient_local_b_t, self.analytic_gradient_local_length_mm
                    gb = (1.0-gradient_weight)*broad_g + gradient_weight*local_g
                    # Blend the actual force profiles, not L, to avoid a
                    # discontinuous derivative or an artificial intermediate
                    # length scale.
                    env_b = math.exp(-0.5 * (delta_s_mm / broad_l) ** 2)
                    env_l = math.exp(-0.5 * (delta_s_mm / local_l) ** 2)
                    dB_b = -(delta_s_mm / (broad_l*broad_l))*env_b*broad_g*1000.0
                    dB_l = -(delta_s_mm / (local_l*local_l))*env_l*local_g*1000.0
                    dB_ds_T_per_m = (1.0-gradient_weight)*dB_b + gradient_weight*dB_l
                    gradient_length_used = (1.0-gradient_weight)*broad_l + gradient_weight*local_l
                    gradient_b_used = gb
                else:
                    gradient_weight = 1.0
                    gl = self.analytic_gradient_length_mm
                    env = math.exp(-0.5 * (delta_s_mm / gl) ** 2)
                    dB_ds_T_per_m = -(delta_s_mm / (gl * gl)) * env * self.analytic_gradient_b_t * 1000.0
                    gradient_length_used = gl
                    gradient_b_used = self.analytic_gradient_b_t
                grad_axis = self.robot_polarity * tangent
                # Keep the prescribed rotating-field amplitude exactly equal
                # to analytic_b_t.  The gradient is an independent spatial
                # derivative used for force only; adding bgrad to B would
                # violate the |B|=10 mT chirp gate and alter torque amplitude.
                # Optional spatial-gradient direction.  The field direction
                # remains grad_axis (historical production convention), while
                # the spatial derivative direction can be tilted in the local
                # normal/binormal plane.  At tilt=0 this reduces exactly to
                # the previous outer(grad_axis, tangent) tensor.
                gt = math.radians(self.analytic_gradient_tilt_deg)
                ga = math.radians(self.analytic_gradient_azimuth_deg)
                gdir = (math.cos(gt) * tangent +
                        math.sin(gt) * (math.cos(ga) * normal + math.sin(ga) * binormal))
                gdir /= max(np.linalg.norm(gdir), 1.0e-15)
                gradient_t_per_m = np.outer(grad_axis, gdir) * dB_ds_T_per_m
                force_n = moment_global.dot(gradient_t_per_m)
            else:
                delta_s_mm = 0.0
                gradient_t_per_m = np.zeros((3, 3))
                force_n = np.zeros(3)
                gradient_weight = 0.0
                gradient_length_used = self.analytic_gradient_length_mm
                gradient_b_used = 0.0
            torque_nmm = np.cross(moment_global, magnetic_field_t) * 1000.0
        else:
            source = self._make_drive_source(driver_position_mm, driver_direction)
            magnetic_field_t, gradient_t_per_m = self._field_and_gradient(source, position_mm * 1e-3)
            force_n = moment_global.dot(gradient_t_per_m)
            torque_nmm = np.cross(moment_global, magnetic_field_t) * 1000.0
        distance = self._distance_at_time(t_s)
        tangent = self._tangent_at_distance(distance)
        if self.drive_type == 'analytic' and self.directional_pulse:
            pulse_force_n, pulse_A, pulse_weight = self._directional_pulse(
                self._command_phase_rad(t_s), moment_global, tangent)
            force_n = force_n + pulse_force_n
        ft = float(np.dot(force_n, tangent))
        fn_vec = force_n - ft * tangent
        if self.tangent_only:
            force_n = ft * tangent
            fn_vec = np.zeros(3)
        if self.zero_torque:
            torque_nmm = np.zeros(3)
        raw_distance = distance
        drive_scale = self._drive_scale(t_s, raw_distance)
        force_n *= drive_scale
        torque_nmm *= drive_scale
        return {
            'type': 'load', 't_s': float(t_s),
            'force_N': [float(value) for value in force_n],
            'torque_Nmm': [float(value) for value in torque_nmm],
            'driver_position_mm': [float(value) for value in driver_position_mm],
            'driver_arc_mm': float(driver_arc_mm),
            'robot_arc_mm': None if robot_arc_mm is None else float(robot_arc_mm),
            'robot_mass_mg': self.robot_mass_mg,
            'drive_type': self.drive_type,
            'field_frame_mode': self.field_frame_mode,
            'force_tangent_N': ft,
            'force_normal_N': [float(value) for value in fn_vec],
            'force_normal_magnitude_N': float(np.linalg.norm(fn_vec)),
            'gradient_profile': self.analytic_gradient_profile,
            'gradient_weight': float(gradient_weight),
            'gradient_b_used_t': float(gradient_b_used),
            'gradient_length_used_mm': float(gradient_length_used),
            'directional_pulse_enabled': bool(self.directional_pulse),
            'directional_pulse_weight': float(pulse_weight),
            'directional_pulse_force_N': [float(v) for v in pulse_force_n],
            'directional_pulse_gradient_norm_T_per_m': float(np.linalg.norm(pulse_A)),
            # Diagnostic only: expose the magnetic source arc position so a
            # coupled run can verify that the robot does not outrun the
            # prescribed 6 mm/s driver trajectory.
            'drive_scale': float(drive_scale),
            'endpoint_reached': bool(raw_distance >= self.arc_mm[-1]),
            'commanded_frequency_Hz': self._command_frequency_hz(t_s),
            'instantaneous_phase_rad': (self._rocking_phase_rad(t_s) if self.field_frame_mode in
                                        ('ROBOT_LOCAL_ROCKING', 'ROBOT_LOCAL_PRECESSING_ROCKING',
                                         'ROBOT_LOCAL_ELLIPTIC_ROCKING')
                                        else self._command_phase_rad(t_s)),
            'rocking_phase_rad': (self._rocking_phase_rad(t_s) if self.field_frame_mode in
                                  ('ROBOT_LOCAL_ROCKING', 'ROBOT_LOCAL_PRECESSING_ROCKING',
                                   'ROBOT_LOCAL_ELLIPTIC_ROCKING') else None),
            'precession_phase_rad': (self._precession_phase_rad(t_s) if
                                     self.field_frame_mode == 'ROBOT_LOCAL_PRECESSING_ROCKING' else None),
            'precession_frequency_Hz': (self.precession_frequency_hz if
                                        self.field_frame_mode == 'ROBOT_LOCAL_PRECESSING_ROCKING' else None),
            'cone_angle_deg': (None if self.field_frame_mode in
                               ('ROBOT_LOCAL_ROCKING', 'ROBOT_LOCAL_PRECESSING_ROCKING',
                                'ROBOT_LOCAL_ELLIPTIC_ROCKING')
                               else float(self.cone_half_angle_deg)),
            'B_mag_T': float(np.linalg.norm(magnetic_field_t)),
            'B_mag_vec_T': [float(value) for value in magnetic_field_t],
        }


class FrameMappedMagneticModel(object):
    """Protocol-facing wrapper that makes the frame conversion explicit."""

    def __init__(self, magnetic_model, frame_transform, lubrication=None):
        self.magnetic_model = magnetic_model
        self.frame_transform = frame_transform
        self.lubrication = lubrication
        self.last_diagnostics = None

    def evaluate(self, t_s, position_aba_mm, ur_aba_rad):
        position_aba_mm = np.asarray(position_aba_mm, dtype=float)
        ur_aba_rad = np.asarray(ur_aba_rad, dtype=float)
        position_mag_mm = self.frame_transform.position_to_mag(position_aba_mm)
        # An axis-angle rotation vector transforms as a vector under a change
        # of orthonormal basis: [R Q R.T]_vee = R * rotvec(Q).
        ur_mag_rad = self.frame_transform.vector_to_mag(ur_aba_rad)
        result = self.magnetic_model.evaluate(t_s, position_mag_mm, ur_mag_rad)
        phase_diagnostics = {
            key: result.pop(key, None) for key in
            ('rocking_phase_rad', 'precession_phase_rad', 'precession_frequency_Hz')
        }
        force_mag = np.asarray(result['force_N'], dtype=float)
        torque_mag = np.asarray(result['torque_Nmm'], dtype=float)
        force_aba = self.frame_transform.vector_to_aba(force_mag)
        torque_aba = self.frame_transform.vector_to_aba(torque_mag)
        if 'B_mag_vec_T' in result:
            result['B_aba_vec_T'] = [float(v) for v in self.frame_transform.vector_to_aba(np.asarray(result['B_mag_vec_T'], dtype=float))]
            result['B_aba_T'] = float(np.linalg.norm(result['B_aba_vec_T']))
        if self.lubrication is not None:
            lub = self.lubrication.evaluate(t_s, position_aba_mm, ur_aba_rad)
            force_aba += np.asarray(lub['force_N'], dtype=float)
            torque_aba += np.asarray(lub['torque_Nmm'], dtype=float)
            result['lubrication'] = lub
            result['force_N'] = [float(value) for value in force_aba]
            result['torque_Nmm'] = [float(value) for value in torque_aba]
        else:
            result['lubrication'] = None
        result['force_N'] = [float(value) for value in force_aba]
        result['torque_Nmm'] = [float(value) for value in torque_aba]
        # Keep protocol response size unchanged: socket_win32.c intentionally
        # uses a small fixed receive buffer.  Full frame-audit values go to
        # the server log through this side channel, never onto the wire.
        self.last_diagnostics = {
            'position_aba_mm': position_aba_mm.copy(),
            'position_mag_mm': position_mag_mm.copy(),
            'ur_aba_rad': ur_aba_rad.copy(),
            'ur_mag_rad': ur_mag_rad.copy(),
            'force_mag_N': force_mag.copy(),
            'torque_mag_Nmm': torque_mag.copy(),
            # Always project the reported robot pose for telemetry.  This is
            # diagnostic-only and does not enable the adaptive lead guard or
            # change the JSON load response consumed by Abaqus.
            'robot_arc_mm': (float(result['robot_arc_mm'])
                             if result.get('robot_arc_mm') is not None
                             else self.magnetic_model._project_arc_mm(position_mag_mm)),
            'lubrication': result.get('lubrication'),
        }
        self.last_diagnostics.update(phase_diagnostics)
        return result


class NormalLubricationModel(object):
    """Auditable sub-grid normal lubrication correction for head/tail.

    The wall triangles are the verified PIPE_WALL_HELPER inner-surface mesh.
    Runtime activation uses a local triangle plane and a smooth cosine gate;
    the exact signed wall-gap audit remains the post-processing authority.
    Forces oppose closing normal velocity and are applied at the head/tail
    points, with the corresponding moment about the Abaqus RP.
    """
    def __init__(self, wall_triangles_csv, scale=1.0, hcut_mm=0.3008351826678705,
                 hmin_mm=0.001, eta_pa_s=0.00071, radius_mm=0.286460121,
                 robot_radius_mm=0.45, half_length_mm=1.05,
                 robot_axis_aba=( -0.96270929, 0.12195883, -0.24148886)):
        self.scale = float(scale)
        self.hcut_mm = float(hcut_mm)
        self.hmin_mm = float(hmin_mm)
        self.eta_pa_s = float(eta_pa_s)
        self.radius_mm = float(radius_mm)
        self.robot_radius_mm = float(robot_radius_mm)
        self.half_length_mm = float(half_length_mm)
        axis = np.asarray(robot_axis_aba, dtype=float)
        self.robot_axis_aba = axis / max(np.linalg.norm(axis), 1.0e-15)
        self.centroids = []
        self.normals = []
        self.vertices = []
        with open(wall_triangles_csv, 'r') as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
        if not rows:
            raise ValueError('empty lubrication wall triangle file: %s' % wall_triangles_csv)
        # The triangle file stores node ids; a sibling node file is required.
        node_file = wall_triangles_csv.replace('_triangles_exact.csv', '_nodes_exact.csv')
        nodes = {}
        with open(node_file, 'r') as stream:
            for row in csv.DictReader(stream):
                nodes[int(row['node'])] = np.array([float(row['x_mm']), float(row['y_mm']), float(row['z_mm'])])
        for row in rows:
            try:
                tri = np.array([nodes[int(row['n1'])], nodes[int(row['n2'])], nodes[int(row['n3'])]], dtype=float)
            except KeyError:
                continue
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            if norm <= 1.0e-12:
                continue
            normal /= norm
            self.vertices.append(tri)
            self.centroids.append(np.mean(tri, axis=0))
            self.normals.append(normal)
        if not self.vertices:
            raise ValueError('no valid wall triangles in %s' % wall_triangles_csv)
        self.vertices = np.asarray(self.vertices)
        self.centroids = np.asarray(self.centroids)
        self.normals = np.asarray(self.normals)
        self._last_t = None
        self._last_points = None

    @staticmethod
    def _cosine_gate(h, hcut, hmin):
        if h >= hcut:
            return 0.0
        u = max(0.0, min(1.0, (hcut - h) / max(hcut - hmin, 1.0e-15)))
        return 0.5 * (1.0 - math.cos(math.pi * u))

    def _closest_plane(self, point):
        delta = self.centroids - point.reshape(1, 3)
        idx = int(np.argmin(np.einsum('ij,ij->i', delta, delta)))
        tri = self.vertices[idx]
        normal = self.normals[idx]
        signed_plane = float(np.dot(point - tri[0], normal))
        projected = point - signed_plane * normal
        # Clamp to the triangle by selecting the nearest vertex if the
        # projection is outside. This is conservative for runtime activation.
        vdist = np.linalg.norm(tri - projected.reshape(1, 3), axis=1)
        if float(np.min(vdist)) < abs(signed_plane):
            nearest = tri[int(np.argmin(vdist))]
            distance = float(np.linalg.norm(point - nearest))
        else:
            distance = abs(signed_plane)
        return distance, normal, idx, projected

    def evaluate(self, t_s, rp_aba_mm, ur_aba_rad):
        rotation = MagneticCouplingModel.rotation_matrix_from_ur(ur_aba_rad)
        axis = rotation.dot(self.robot_axis_aba)
        axis /= max(np.linalg.norm(axis), 1.0e-15)
        points = np.asarray([rp_aba_mm - self.half_length_mm * axis,
                             rp_aba_mm + self.half_length_mm * axis], dtype=float)
        values = []
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        power_W = 0.0
        for point in points:
            distance, normal, tri_idx, projected = self._closest_plane(point)
            h = distance - self.robot_radius_mm
            inward = point - projected
            inward_norm = np.linalg.norm(inward)
            if inward_norm > 1.0e-12:
                inward = inward / inward_norm
            else:
                inward = -normal
            if self._last_t is None or self._last_points is None or t_s <= self._last_t:
                vclose = 0.0
                vpoint = np.zeros(3)
            else:
                vpoint = (point - self._last_points[len(values)]) / max(t_s - self._last_t, 1.0e-15)
                # inward points from the wall to the robot; closing motion has
                # a negative inward component and is opposed by +inward.
                vclose = max(0.0, -float(np.dot(vpoint, inward)))
            gate = self._cosine_gate(h, self.hcut_mm, self.hmin_mm)
            h_eff = max(h, self.hmin_mm)
            force_mN = 0.0
            if gate > 0.0 and vclose > 0.0:
                # Sphere-plane squeeze-film estimate; all inputs converted to SI.
                force_N = (6.0 * math.pi * self.eta_pa_s * (self.radius_mm * 1e-3) ** 2 *
                           (vclose * 1e-3) / (h_eff * 1e-3) * gate * self.scale)
                force_mN = force_N * 1e3
                fvec = force_N * inward
                total_force += fvec
                total_moment += np.cross(point - rp_aba_mm, fvec)
                # Work rate at the endpoint (mm/s -> m/s).  This is
                # non-positive for the closing-only dissipative force.
                power_W += float(np.dot(fvec, vpoint * 1.0e-3))
            values.append({'h_mm': float(h), 'vclose_mm_s': float(vclose),
                           'force_mN': float(force_mN), 'active': bool(gate > 0.0 and vclose > 0.0),
                           'wall_triangle': int(tri_idx + 1)})
        self._last_t = float(t_s)
        self._last_points = points.copy()
        return {'force_N': [float(v) for v in total_force],
                'torque_Nmm': [float(v) for v in total_moment],
                'head': values[0], 'tail': values[1],
                'power_W': float(power_W), 'scale': self.scale,
                'hcut_mm': self.hcut_mm, 'hmin_mm': self.hmin_mm,
                'eta_pa_s': self.eta_pa_s, 'radius_mm': self.radius_mm}


def send_message(writer, message):
    writer.write((json.dumps(message, separators=(',', ':')) + '\n').encode('utf-8'))
    writer.flush()


def process_client(connection, address, model, telemetry_writer=None, telemetry_stream=None,
                   identity=None, telemetry_interval_s=0.0, frame_log_interval_s=0.0):
    print('Client connected: {}:{}'.format(address[0], address[1]))
    reader = connection.makefile('rb')
    writer = connection.makefile('wb')
    last_telemetry_t = -float('inf')
    last_frame_log_t = -float('inf')
    try:
        try:
            for raw_line in reader:
                try:
                    request = json.loads(raw_line.decode('utf-8'))
                    request_type = request.get('type', request.get('cmd'))
                    if request_type == 'hello':
                        ident = identity or {}
                        mismatches=[]
                        for key in ('job_name','run_uuid'):
                            if request.get(key) and ident.get(key) and request.get(key) != ident.get(key):
                                mismatches.append('{} mismatch'.format(key))
                        if request.get('lubrication_scale') is not None and ident.get('lubrication_scale') is not None:
                            if abs(float(request['lubrication_scale'])-float(ident['lubrication_scale'])) > 1e-12:
                                mismatches.append('lubrication_scale mismatch')
                        if mismatches:
                            send_message(writer, {'type':'hello_nack','protocol':ident.get('protocol','magnetic-coupling-jsonl-v2'),'reason':'; '.join(mismatches)})
                            return True
                        send_message(writer, {'type': 'hello_ack', 'protocol': ident.get('protocol','magnetic-coupling-jsonl-v2'),
                                               'job_name': ident.get('job_name',''), 'lubrication_scale': ident.get('lubrication_scale',0.0),
                                               'run_uuid': ident.get('run_uuid',''), 'server_pid': os.getpid(),
                                               'port': ident.get('port'),
                                               'units': {'length': 'mm', 'force': 'N', 'torque': 'N*mm'}})
                    elif request_type == 'pose':
                        request_t = float(request['t_s'])
                        response = model.evaluate(request_t, request['position_mm'], request['ur_rad'])
                        response['status'] = 'ok'
                        send_message(writer, response)
                        write_telemetry = (telemetry_writer is not None and
                            (telemetry_interval_s <= 0.0 or
                             request_t - last_telemetry_t >= telemetry_interval_s or
                             last_telemetry_t == -float('inf')))
                        if write_telemetry:
                            diagnostics = getattr(model, 'last_diagnostics', None) or {}
                            pm = diagnostics.get('position_mag_mm', [None, None, None])
                            tm = diagnostics.get('torque_mag_Nmm', [None, None, None])
                            lub = diagnostics.get('lubrication') or response.get('lubrication') or {}
                            head = lub.get('head') or {}
                            tail = lub.get('tail') or {}
                            robot_arc_diag = diagnostics.get('robot_arc_mm', response.get('robot_arc_mm'))
                            telemetry_writer.writerow([
                                request.get('t_s'), *request.get('position_mm', [None, None, None]),
                                *request.get('ur_rad', [None, None, None]),
                                *response.get('force_N', [None, None, None]),
                                *response.get('torque_Nmm', [None, None, None]),
                                response.get('driver_arc_mm'), robot_arc_diag,
                                response.get('force_tangent_N'), response.get('force_normal_magnitude_N'),
                                *pm, *tm, response.get('drive_scale'), response.get('endpoint_reached'),
                                head.get('h_mm'), tail.get('h_mm'),
                                head.get('vclose_mm_s'), tail.get('vclose_mm_s'),
                                head.get('force_mN'), tail.get('force_mN'),
                                lub.get('force_N', [None, None, None])[0],
                                lub.get('force_N', [None, None, None])[1],
                                lub.get('force_N', [None, None, None])[2],
                                lub.get('torque_Nmm', [None, None, None])[0],
                                lub.get('torque_Nmm', [None, None, None])[1],
                                lub.get('torque_Nmm', [None, None, None])[2],
                                lub.get('power_W'), head.get('active'), tail.get('active'),
                                head.get('wall_triangle'), tail.get('wall_triangle'),
                                response.get('commanded_frequency_Hz'), response.get('instantaneous_phase_rad'),
                                diagnostics.get('rocking_phase_rad'), diagnostics.get('precession_phase_rad'),
                                diagnostics.get('precession_frequency_Hz'),
                                response.get('cone_angle_deg'), response.get('B_aba_T'),
                                *(response.get('B_aba_vec_T') or [None, None, None])
                            ])
                            if telemetry_stream is not None:
                                telemetry_stream.flush()
                            last_telemetry_t = request_t
                        # Keep diagnostics strictly after the response and make
                        # them non-fatal.  In particular, a probe pose may not
                        # project to the DXF centerline, so robot_arc_mm is None.
                        # Raising here would enqueue a second error JSON after
                        # the valid pose response and desynchronize Abaqus'
                        # next request/response pair.
                        driver_arc = response.get('driver_arc_mm')
                        diagnostics = getattr(model, 'last_diagnostics', None) or {}
                        robot_arc = diagnostics.get('robot_arc_mm', response.get('robot_arc_mm'))
                        gap = None if driver_arc is None or robot_arc is None else driver_arc - robot_arc
                        diagnostics = getattr(model, 'last_diagnostics', None)
                        position_mag = None if diagnostics is None else diagnostics.get('position_mag_mm')
                        torque_mag = None if diagnostics is None else diagnostics.get('torque_mag_Nmm')
                        write_frame_log = (frame_log_interval_s <= 0.0 or
                            request_t - last_frame_log_t >= frame_log_interval_s or
                            last_frame_log_t == -float('inf'))
                        if write_frame_log:
                            print('t={:.9g} s driver_s={} robot_s={} gap={} mm RP_ABA=({:.9g},{:.9g},{:.9g}) mm RP_MAG={} F_ABA=({:.9g},{:.9g},{:.9g}) N Ft_MAG={:.9g} N T_MAG={} T_ABA=({:.9g},{:.9g},{:.9g}) Nmm scale={:.6g}'.format(
                                request_t,
                                'NA' if driver_arc is None else '{:.4f}'.format(driver_arc),
                                'NA' if robot_arc is None else '{:.4f}'.format(robot_arc),
                                'NA' if gap is None else '{:.4f}'.format(gap),
                                request['position_mm'][0], request['position_mm'][1], request['position_mm'][2],
                                'NA' if position_mag is None else '({:.9g},{:.9g},{:.9g})'.format(*position_mag),
                                response['force_N'][0], response['force_N'][1], response['force_N'][2],
                                response['force_tangent_N'], 'NA' if torque_mag is None else '({:.9g},{:.9g},{:.9g})'.format(*torque_mag), response['torque_Nmm'][0], response['torque_Nmm'][1],
                                response['torque_Nmm'][2], response.get('drive_scale', 1.0)))
                            last_frame_log_t = request_t
                    elif request_type == 'close':
                        send_message(writer, {'type': 'close_ack'})
                        break
                    elif request_type == 'shutdown':
                        send_message(writer, {'type': 'shutdown_ack'})
                        return True
                    else:
                        raise ValueError('unsupported message type: {}'.format(request_type))
                except Exception as exc:
                    try:
                        send_message(writer, {'type': 'error', 'message': str(exc)})
                    except (ConnectionResetError, BrokenPipeError, OSError):
                        break
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            print('Client socket closed: {}:{}'.format(type(exc).__name__, exc))
    finally:
        reader.close()
        writer.close()
        connection.close()
        print('Client disconnected: {}:{}'.format(address[0], address[1]))
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=HOST)
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--job-name', default='', help='run identity returned during hello handshake')
    parser.add_argument('--run-uuid', default='', help='run identity UUID returned during hello handshake')
    parser.add_argument('--protocol-version', default='magnetic-coupling-jsonl-v2')
    parser.add_argument('--dxf', default=r'J:\magpy\curveforsphere.dxf')
    parser.add_argument('--robot-diameter-mm', type=float, default=0.8)
    parser.add_argument('--robot-height-mm', type=float, default=2.0)
    parser.add_argument('--drive-type', choices=('sphere', 'cylinder', 'cuboid', 'analytic'), default='sphere')
    parser.add_argument('--drive-diameter-mm', type=float, default=50.0)
    parser.add_argument('--drive-height-mm', type=float)
    parser.add_argument('--drive-length-mm', type=float)
    parser.add_argument('--drive-width-mm', type=float)
    parser.add_argument('--drive-br-t', type=float, default=1.46)
    parser.add_argument('--analytic-b-t', type=float, default=0.010,
                        help='uniform analytic rotating-field amplitude in tesla')
    parser.add_argument('--analytic-follow-robot', action='store_true',
                        help='evaluate analytic field Frenet basis at robot arc position (no sphere/Z offset)')
    parser.add_argument('--field-frame-mode', choices=FIELD_FRAME_MODES, default='LEGACY_DRIVER_FRAME',
                        help='explicit analytic cone frame; legacy driver frame remains the default')
    parser.add_argument('--rocking-amplitude-deg', type=float, default=10.0,
                        help='peak signed field angle for ROBOT_LOCAL_ROCKING')
    parser.add_argument('--rocking-cross-amplitude-deg', type=float, default=0.0,
                        help='fixed RouteA cross-axis peak angle for ROBOT_LOCAL_ELLIPTIC_ROCKING')
    parser.add_argument('--rocking-frame-azimuth-deg', type=float, default=-61.37284757596327,
                        help='RouteA n_rock gauge angle from production e1 toward e2')
    parser.add_argument('--precession-frequency-hz', type=float, default=2.5,
                        help='rocking-plane precession frequency for ROBOT_LOCAL_PRECESSING_ROCKING')
    parser.add_argument('--precession-phase-deg', type=float, default=-83.87284757596327,
                        help='initial rocking-plane azimuth from production e1 toward e2')
    parser.add_argument('--analytic-gradient-b-t', type=float, default=0.0,
                        help='weak localized axial-gradient field amplitude in tesla (0 disables)')
    parser.add_argument('--analytic-gradient-length-mm', type=float, default=25.0,
                        help='Gaussian axial-gradient length scale in mm')
    parser.add_argument('--analytic-gradient-profile', choices=('legacy','broad-to-local'), default='legacy',
                        help='gradient architecture; broad-to-local cross-fades two physical spatial channels')
    parser.add_argument('--analytic-gradient-switch-start-s', type=float, default=0.0,
                        help='absolute simulation time at which broad-to-local cross-fade starts')
    parser.add_argument('--analytic-gradient-transition-s', type=float, default=0.0005,
                        help='smoothstep cross-fade duration in seconds')
    parser.add_argument('--analytic-gradient-local-b-t', type=float, default=None,
                        help='late local-channel amplitude in tesla (defaults to broad amplitude)')
    parser.add_argument('--analytic-gradient-local-length-mm', type=float, default=3.75,
                        help='late local-channel Gaussian length scale in mm')
    parser.add_argument('--analytic-gradient-tilt-deg', type=float, default=0.0,
                        help='tilt of spatial gradient direction from local tangent (deg)')
    parser.add_argument('--analytic-gradient-azimuth-deg', type=float, default=0.0,
                        help='gradient tilt azimuth in local normal/binormal plane (deg)')
    parser.add_argument('--robot-br-t', type=float, default=1.46)
    parser.add_argument('--robot-moment-Am2', type=float, default=None,
                        help='explicit total robot dipole moment in A*m^2; overrides robot-br-t volume conversion')
    parser.add_argument('--robot-mass-mg', type=float, default=10.0)
    parser.add_argument('--driver-speed-mm-s', type=float, default=5.0)
    parser.add_argument('--bend-speed-mm-s', type=float, default=None)
    parser.add_argument('--bend-start-mm', type=float, default=13.49)
    parser.add_argument('--bend-end-mm', type=float, default=18.56)
    parser.add_argument('--z-offset-mm', type=float, default=57.0)
    parser.add_argument('--driver-start-offset-mm', type=float, default=0.0)
    parser.add_argument('--spin-hz', type=float, default=5.0)
    parser.add_argument('--chirp-start-hz', type=float, default=None,
                        help='continuous chirp start frequency (Hz); requires end and duration')
    parser.add_argument('--chirp-end-hz', type=float, default=None,
                        help='continuous chirp end frequency (Hz); requires start and duration')
    parser.add_argument('--chirp-duration-s', type=float, default=0.0,
                        help='chirp duration (s), with phase computed by exact frequency integration')
    parser.add_argument('--directional-pulse', action='store_true',
                        help='enable one Maxwell-consistent directional anti-impact gradient pulse')
    parser.add_argument('--directional-pulse-phase-deg', type=float, default=0.0)
    parser.add_argument('--directional-pulse-halfwidth-deg', type=float, default=5.0)
    parser.add_argument('--directional-pulse-ft-n', type=float, default=0.0,
                        help='positive tangential pulse target in N')
    parser.add_argument('--directional-pulse-fn-n', type=float, default=0.0,
                        help='positive away-from-wall pulse target in N')
    parser.add_argument('--directional-pulse-normal-mag', type=float, nargs=3, default=None,
                        help='fixed n_away vector in Magpylib coordinates')
    parser.add_argument('--align-origin-mm', type=float, nargs=3)
    parser.add_argument('--align-tangent', type=float, nargs=3)
    parser.add_argument('--cone-half-angle-deg', type=float, default=30.0)
    parser.add_argument('--driver-xy-shift-mm', type=float, nargs=2, default=[0.0, 0.0],
                        help='x,y translation of the physical drive trajectory; does not change sphere geometry')
    parser.add_argument('--driver-normal-offset-mm', type=float, default=0.0,
                        help='offset drive trajectory away from robot along initial pipe-normal direction')
    parser.add_argument('--trajectory-position-angle-deg', type=float, default=0.0,
                        help='rotate the physical sphere-center offset from projected +Z toward local tangent')
    parser.add_argument('--cone-axis-bias-deg', type=float, default=0.0,
                        help='rotate the cone axis from local tangent toward projected +Z')
    parser.add_argument('--cone-frame-azimuth-deg', type=float, default=0.0,
                        help='rigidly rotate the entire biased cone frame about local pipe tangent (deg)')
    parser.add_argument('--robot-polarity', type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument('--cone-axis', choices=('global_x','tangent'), default='global_x')
    parser.add_argument('--robot-axis-tangent', action='store_true',
                        help='initialize the axial robot magnet along the aligned inlet tangent')
    parser.add_argument('--robot-moment-axis-aba', type=float, nargs=3, default=None,
                        help='explicit UR=0 physical magnetic-moment axis in the Abaqus frame')
    parser.add_argument('--phase-deg', type=float, default=0.0)
    parser.add_argument('--analytic-rotation-sense', type=float, choices=(-1.0, 1.0), default=1.0,
                        help='analytic rotating-field sense: +1 baseline, -1 reverse; phase at t=0 is unchanged')
    parser.add_argument('--tangent-only', action='store_true',
                        help='return only the local pipe-tangent component of the Magpylib force')
    parser.add_argument('--zero-torque', action='store_true',
                        help='return zero magnetic torque for forward-motion diagnostics')
    parser.add_argument('--ramp-time-s', type=float, default=0.001,
                        help='smoothstep turn-on time for force and torque')
    parser.add_argument('--endpoint-taper-mm', type=float, default=1.0,
                        help='smoothly taper force/torque before the DXF endpoint')
    parser.add_argument('--adaptive-lead-mm', type=float, default=0.0,
                        help='diagnostic mode: keep magnetic source this many mm ahead of robot arc projection')
    parser.add_argument('--reverse-trajectory', action='store_true',
                        help='move the magnetic source toward decreasing DXF arclength; start offset is absolute s')
    parser.add_argument('--frame-transform-json', default=None,
                        help='single-source rigid transform from Abaqus global to Magpylib/DXF frame')
    parser.add_argument('--telemetry-csv', default=None,
                        help='optional CSV path for raw pose/load telemetry; response wire format is unchanged')
    parser.add_argument('--telemetry-interval-s', type=float, default=0.0,
                        help='write telemetry at most once per interval; 0 keeps every pose (load updates unchanged)')
    parser.add_argument('--frame-log-interval-s', type=float, default=0.0,
                        help='print a diagnostic frame at most once per interval; 0 prints every pose')
    parser.add_argument('--lubrication-wall-triangles', default=None,
                        help='verified PIPE_WALL_HELPER triangle CSV; enables head/tail normal lubrication')
    parser.add_argument('--lubrication-scale', type=float, default=0.0,
                        help='normal lubrication correction scale (0=off, 0.5=L1, 1=L2, 2=L3)')
    parser.add_argument('--lubrication-hcut-mm', type=float, default=0.3008351826678705)
    parser.add_argument('--lubrication-hmin-mm', type=float, default=0.001)
    parser.add_argument('--lubrication-eta-pa-s', type=float, default=0.00071)
    parser.add_argument('--lubrication-radius-mm', type=float, default=0.286460121)
    parser.add_argument('--lubrication-robot-radius-mm', type=float, default=0.45)
    parser.add_argument('--lubrication-half-length-mm', type=float, default=1.05)
    args = parser.parse_args()
    # Unit guard: analytic field amplitudes are internal SI tesla.  Refuse
    # suspicious mT-as-T values before binding a production socket.
    if not (0.0 < args.analytic_b_t < 0.02):
        parser.error('analytic-b-t must be in tesla and satisfy 0 < B < 0.02 T')
    if not (0.0 <= args.analytic_gradient_b_t < 0.02):
        parser.error('analytic-gradient-b-t must be in tesla and satisfy 0 <= gradient < 0.02 T')
    if args.analytic_gradient_local_b_t is not None and not (0.0 <= args.analytic_gradient_local_b_t < 0.02):
        parser.error('analytic-gradient-local-b-t must be in tesla and satisfy 0 <= gradient < 0.02 T')
    print('Field units: B={:.3f} mT = {:.6f} T; gradient={:.3f} mT = {:.6f} T'.format(
        args.analytic_b_t*1000.0, args.analytic_b_t,
        args.analytic_gradient_b_t*1000.0, args.analytic_gradient_b_t))
    model = MagneticCouplingModel(
        args.dxf,
        cone_half_angle_deg=args.cone_half_angle_deg,
        drive_type=args.drive_type,
        drive_diameter_mm=args.drive_diameter_mm,
        drive_height_mm=args.drive_height_mm,
        drive_length_mm=args.drive_length_mm,
        drive_width_mm=args.drive_width_mm,
        br_t=args.drive_br_t,
        robot_diameter_mm=args.robot_diameter_mm,
        robot_height_mm=args.robot_height_mm,
        robot_br_t=args.robot_br_t,
        robot_moment_Am2=args.robot_moment_Am2,
        robot_mass_mg=args.robot_mass_mg,
        driver_speed_mm_s=args.driver_speed_mm_s,
        bend_speed_mm_s=args.bend_speed_mm_s,
        bend_start_mm=args.bend_start_mm,
        bend_end_mm=args.bend_end_mm,
        z_offset_mm=args.z_offset_mm,
        driver_start_offset_mm=args.driver_start_offset_mm,
        spin_hz=args.spin_hz,
        align_origin_mm=args.align_origin_mm,
        align_tangent=args.align_tangent,
        driver_xy_shift_mm=args.driver_xy_shift_mm,
        trajectory_position_angle_deg=args.trajectory_position_angle_deg,
        cone_axis_bias_deg=args.cone_axis_bias_deg,
        cone_frame_azimuth_deg=args.cone_frame_azimuth_deg,
        tangent_only=args.tangent_only,
        zero_torque=args.zero_torque,
        ramp_time_s=args.ramp_time_s,
        endpoint_taper_mm=args.endpoint_taper_mm,
        adaptive_lead_mm=args.adaptive_lead_mm,
        reverse_trajectory=args.reverse_trajectory,
        analytic_b_t=args.analytic_b_t,
        analytic_follow_robot=args.analytic_follow_robot,
        field_frame_mode=args.field_frame_mode,
        rocking_amplitude_deg=args.rocking_amplitude_deg,
        rocking_cross_amplitude_deg=args.rocking_cross_amplitude_deg,
        rocking_frame_azimuth_deg=args.rocking_frame_azimuth_deg,
        precession_frequency_hz=args.precession_frequency_hz,
        precession_phase_deg=args.precession_phase_deg,
        analytic_gradient_b_t=args.analytic_gradient_b_t,
        analytic_gradient_length_mm=args.analytic_gradient_length_mm,
        analytic_gradient_profile=args.analytic_gradient_profile,
        analytic_gradient_switch_start_s=args.analytic_gradient_switch_start_s,
        analytic_gradient_transition_s=args.analytic_gradient_transition_s,
        analytic_gradient_local_b_t=args.analytic_gradient_local_b_t,
        analytic_gradient_local_length_mm=args.analytic_gradient_local_length_mm,
        analytic_gradient_tilt_deg=args.analytic_gradient_tilt_deg,
        analytic_gradient_azimuth_deg=args.analytic_gradient_azimuth_deg,
        analytic_rotation_sense=args.analytic_rotation_sense,
        chirp_start_hz=args.chirp_start_hz,
        chirp_end_hz=args.chirp_end_hz,
        chirp_duration_s=args.chirp_duration_s,
        directional_pulse=args.directional_pulse,
        directional_pulse_phase_deg=args.directional_pulse_phase_deg,
        directional_pulse_halfwidth_deg=args.directional_pulse_halfwidth_deg,
        directional_pulse_ft_n=args.directional_pulse_ft_n,
        directional_pulse_fn_n=args.directional_pulse_fn_n,
        directional_pulse_normal_mag=args.directional_pulse_normal_mag,
    )
    model.set_driver_normal_offset(args.driver_normal_offset_mm)
    model.robot_polarity = args.robot_polarity
    model.cone_axis_tangent = (args.cone_axis == 'tangent')
    frame_transform = RigidFrameTransform(args.frame_transform_json)
    if args.robot_moment_axis_aba is not None and args.robot_axis_tangent:
        parser.error('--robot-moment-axis-aba and --robot-axis-tangent are mutually exclusive')
    if args.robot_moment_axis_aba is not None:
        moment_axis = frame_transform.vector_to_mag(args.robot_moment_axis_aba)
        moment_axis /= max(np.linalg.norm(moment_axis), 1.0e-15)
        model.robot_axis_global = moment_axis / model.robot_polarity
    elif args.robot_axis_tangent:
        axis = model.curve_mm[1] - model.curve_mm[0]
        model.robot_axis_global = axis / max(np.linalg.norm(axis), 1.0e-15)
    model.phase_offset_rad = math.radians(args.phase_deg)
    print('TRAJECTORY_DIRECTION={}'.format('REVERSE' if model.reverse_trajectory else 'FORWARD'))
    if model.drive_type == 'analytic':
        print('ANALYTIC_ROTATING_FIELD: B=%.9g T; f=%.9g Hz; cone=%.9g deg; bias=%.9g deg; frame_azimuth=%.9g deg; gradient_B=%.9g T; gradient_L=%.9g mm; grad_tilt=%.9g deg; grad_azimuth=%.9g deg; sense=%+.0f; frame_mode=%s; basis=%s' %
              (model.analytic_b_t, model.spin_hz, model.cone_half_angle_deg,
               model.cone_axis_bias_deg, model.cone_frame_azimuth_deg,
               model.analytic_gradient_b_t, model.analytic_gradient_length_mm,
               model.analytic_gradient_tilt_deg, model.analytic_gradient_azimuth_deg,
               model.analytic_rotation_sense, model.field_frame_mode,
               ('ROBOT_ARC' if model.field_frame_mode in ROBOT_LOCAL_FRAME_MODES
                 else ('ROBOT_ARC' if model.analytic_follow_robot else 'DRIVER_ARC'))))
        if model.field_frame_mode == 'ROBOT_LOCAL_PRECESSING_ROCKING':
            print('PRECESSING_ROCKING: A=%.15g deg; f_rock=%.9g Hz; f_prec=%.9g Hz; psi0=%.15g deg' %
                  (model.rocking_amplitude_deg, model.spin_hz,
                   model.precession_frequency_hz, model.precession_phase_deg))
        if model.field_frame_mode == 'ROBOT_LOCAL_ELLIPTIC_ROCKING':
            print('ELLIPTIC_ROCKING: A_main=%.15g deg; A_cross=%.15g deg; f=%.9g Hz; fixed_routeA_axes=YES' %
                  (model.rocking_amplitude_deg, model.rocking_cross_amplitude_deg,
                   model.spin_hz))
        print('ANALYTIC_GRADIENT_PROFILE: %s; switch_start=%.9g s; transition=%.9g s; local_B=%.9g T; local_L=%.9g mm' %
              (model.analytic_gradient_profile, model.analytic_gradient_switch_start_s,
               model.analytic_gradient_transition_s, model.analytic_gradient_local_b_t,
               model.analytic_gradient_local_length_mm))
        if model.directional_pulse:
            print('DIRECTIONAL_PULSE: phase=%.9g deg; halfwidth=%.9g deg; Ft=%.9g N; Fn_away=%.9g N; n_mag=(%.9g,%.9g,%.9g)' %
                  (model.directional_pulse_phase_deg, model.directional_pulse_halfwidth_deg,
                   model.directional_pulse_ft_n, model.directional_pulse_fn_n,
                   model.directional_pulse_normal_mag[0], model.directional_pulse_normal_mag[1], model.directional_pulse_normal_mag[2]))
    if model.chirp_start_hz is not None:
        print('CHIRP: f0=%.9g Hz; f1=%.9g Hz; duration=%.9g s; phase=integral(2*pi*f dt)' %
              (model.chirp_start_hz, model.chirp_end_hz, model.chirp_duration_s))
    print('DRIVER_START_ARC_MM={:.9g}; DRIVER_SPEED_MM_S={:.9g}'.format(
        model.driver_start_offset_mm, model.driver_speed_mm_s))
    lubrication = None
    if args.lubrication_scale > 0.0:
        if args.lubrication_wall_triangles is None:
            parser.error('--lubrication-wall-triangles is required when --lubrication-scale > 0')
        lubrication = NormalLubricationModel(
            args.lubrication_wall_triangles,
            scale=args.lubrication_scale,
            hcut_mm=args.lubrication_hcut_mm,
            hmin_mm=args.lubrication_hmin_mm,
            eta_pa_s=args.lubrication_eta_pa_s,
            radius_mm=args.lubrication_radius_mm,
            robot_radius_mm=args.lubrication_robot_radius_mm,
            half_length_mm=args.lubrication_half_length_mm)
        print('LUBRICATION: normal head/tail only; scale={:.6g}; h_cut={:.9g} mm ({:.6g}x median CEL edge); h_min={:.9g} mm; eta={:.9g} Pa*s; R_eff={:.9g} mm'.format(
            args.lubrication_scale, args.lubrication_hcut_mm / 0.6016703653357411,
            args.lubrication_hcut_mm, args.lubrication_hmin_mm,
            args.lubrication_eta_pa_s, args.lubrication_radius_mm))
    model = FrameMappedMagneticModel(model, frame_transform, lubrication=lubrication)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(4)
    telemetry_stream = None
    telemetry_writer = None
    if args.telemetry_csv:
        telemetry_stream = open(args.telemetry_csv, 'w', newline='')
        telemetry_writer = csv.writer(telemetry_stream)
        telemetry_writer.writerow([
            't_s','rp_x_aba_mm','rp_y_aba_mm','rp_z_aba_mm','ur1','ur2','ur3',
            'fx_aba_N','fy_aba_N','fz_aba_N','tx_aba_Nmm','ty_aba_Nmm','tz_aba_Nmm',
            'driver_arc_mm','robot_arc_mm','force_tangent_N','force_normal_N',
            'rp_x_mag_mm','rp_y_mag_mm','rp_z_mag_mm','tx_mag_Nmm','ty_mag_Nmm','tz_mag_Nmm',
            'drive_scale','endpoint_reached','h_head_mm','h_tail_mm',
            'vn_head_close_mm_s','vn_tail_close_mm_s','Flub_head_mN','Flub_tail_mN',
            'Flub_x_N','Flub_y_N','Flub_z_N','Mlub_x_Nmm','Mlub_y_Nmm','Mlub_z_Nmm',
            'lub_power_W','active_head','active_tail','wall_tri_head','wall_tri_tail',
            'commanded_frequency_Hz','instantaneous_phase_rad','rocking_phase_rad',
            'precession_phase_rad','precession_frequency_Hz','cone_angle_deg','B_aba_T',
            'Bx_aba_T','By_aba_T','Bz_aba_T'])
        telemetry_stream.flush()
        print('Telemetry CSV: {}'.format(args.telemetry_csv))
    print('Magpylib Socket server listening on {}:{}'.format(args.host, args.port))
    print('Trajectory position angle={} deg; cone-axis bias={} deg'.format(
        args.trajectory_position_angle_deg, args.cone_axis_bias_deg))
    print('Frame transform: {} ({})'.format(frame_transform.name,
                                             args.frame_transform_json or 'identity'))
    print('Send type="shutdown" for a safe server shutdown.')
    try:
        while True:
            connection, address = server.accept()
            identity={'job_name':args.job_name,'run_uuid':args.run_uuid,
                      'lubrication_scale':args.lubrication_scale,'port':args.port,
                      'protocol':args.protocol_version}
            if process_client(connection, address, model, telemetry_writer, telemetry_stream, identity,
                              telemetry_interval_s=args.telemetry_interval_s,
                              frame_log_interval_s=args.frame_log_interval_s):
                break
    except KeyboardInterrupt:
        print('Keyboard interrupt: closing server.')
    finally:
        server.close()
        if telemetry_stream is not None:
            telemetry_stream.close()


if __name__ == '__main__':
    main()
