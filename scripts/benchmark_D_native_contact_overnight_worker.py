"""Hash-reviewed owned native contact branch; no interactive controller dependency.

All persisted native positions, orientations and velocities are read, never reset.
The branch evidence is separate from the campaign supervisor's state machine.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback

import numpy as np
import psutil
import pyvista as pv
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from benchmark_D_native_contact_overnight_common import (
    ROOT, EVID, OUT, CAMPAIGN, atomic, read, sha, stamp, native,
    verify_frozen, prepare_branch,
)
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_recovery_v2_restart import compare, state_vector
from benchmark_C_analytic_reference import compute_magnetic_load
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_coarse_2ms_common import identity

COM_REFERENCE = np.array([.0012060186937156343, 0., 0.])
TUBE_RADIUS_M = .0009
TIME_TOLERANCE_S = 1e-12


def json_rows(path):
    path = Path(path)
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def worker_state(branch_dir, status, **items):
    rec = read(branch_dir / 'worker_state.json')
    rec.update(campaign=CAMPAIGN, status=status, timestamp=stamp(), **items)
    atomic(branch_dir / 'worker_state.json', rec)
    return rec


def branch_event(branch_dir, name, **items):
    with (branch_dir / 'worker_events.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(dict(timestamp=stamp(), event=name, **items), allow_nan=False) + '\n')


def check_control(config):
    if sha(EVID / 'window.json') != config['_reviewed_window_sha256']:
        raise RuntimeError('IMMUTABLE_WINDOW_CHANGED_DURING_NATIVE_BRANCH')
    window = read(EVID / 'window.json')
    if time.time() >= float(window['hard_deadline_epoch']):
        raise RuntimeError('TIME_BUDGET_EXHAUSTED')
    if read(OUT / 'stop_request.json') or read(EVID / 'stop_request.json'):
        raise RuntimeError('STOP_REQUESTED_AT_NATIVE_BOUNDARY')
    if config.get('window_id') and config['window_id'] != window.get('window_id'):
        raise RuntimeError('IMMUTABLE_WINDOW_ID_MISMATCH')


def checkpoint_integrity(metadata):
    """Native CASE/DATA audit independent of a dt-specific timestep-index formula."""
    import h5py
    result = dict(timestamp=stamp(), status='PASS', files={}, timesteps_advanced=0,
                  time_s=float(metadata['time_s']))
    if not np.isfinite(result['time_s']):
        raise RuntimeError('NONFINITE_NATIVE_CHECKPOINT_TIME')
    for kind in ('case', 'data'):
        path = Path(metadata[kind])
        if not path.is_absolute():
            path = ROOT / path
        if not path.is_file() or path.stat().st_size < 1024:
            raise RuntimeError('MISSING_OR_EMPTY_NATIVE_CHECKPOINT_' + kind)
        digest = sha(path)
        if digest != metadata[kind + '_sha256']:
            raise RuntimeError('NATIVE_CHECKPOINT_HASH_MISMATCH_' + kind)
        with h5py.File(path, 'r') as handle:
            if not list(handle):
                raise RuntimeError('EMPTY_NATIVE_HDF5_' + kind)
            group = 'settings/Rampant Variables' if kind == 'case' else 'settings/Data Variables'
            if group in handle and not len(handle[group][()]):
                raise RuntimeError('EMPTY_NATIVE_HDF5_SETTINGS_' + kind)
        result['files'][kind] = dict(path=str(path), size_bytes=path.stat().st_size,
                                     sha256=digest, HDF5_readable=True)
    return result


def expected_checkpoint_state(metadata):
    if 'native_state' in metadata:
        return metadata['native_state']
    if 'row' in metadata and metadata['row'] is not None:
        return state_vector(metadata['row'])
    raise RuntimeError('NATIVE_CHECKPOINT_HAS_NO_VERIFIED_POSE_VELOCITY')


def native_restart_gate(solver, branch_dir, output_dir, checkpoint, config):
    """Zero-step continuity; no old C clearance gate and no state assignment."""
    from benchmark_C_fineA_theta0 import summarize_csv
    before = float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    result = dict(timestamp=stamp(), status='RUNNING', timesteps_advanced=0,
                  pose_reset_performed=False, velocity_reset_performed=False,
                  checkpoint_integrity=checkpoint_integrity(checkpoint))
    try:
        if abs(before - float(checkpoint['time_s'])) > TIME_TOLERANCE_S or abs(before - config['start_time_s']) > TIME_TOLERANCE_S:
            raise RuntimeError('RESTART_CONTINUITY_FAIL_NATIVE_TIME')
        wall = native_state(solver, 'robot_wall')
        component = native_state(solver, 'robot_component_fluid')
        want = expected_checkpoint_state(checkpoint)
        errors = compare(wall, want)
        component_errors = compare(component, want)
        result.update(native_time_s=before, native_state=wall, native_errors=errors,
                      component_errors=component_errors, q_norm=float(np.linalg.norm(wall['q'])))
        if abs(result['q_norm'] - 1) > 1e-6 or any(item['status'] != 'PASS' for group in (errors, component_errors) for item in group.values()):
            raise RuntimeError('RESTART_CONTINUITY_FAIL_NATIVE_POSE_VELOCITY')
        s = solver.settings
        if abs(float(solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)")) - 1e-50) > 1e-60:
            raise RuntimeError('FROZEN_CUTOFF_CHANGED')
        if not s.setup.dynamic_mesh.options.six_dof.enabled.get_state() or s.setup.general.operating_conditions.gravity.enable.get_state():
            raise RuntimeError('FROZEN_SIX_DOF_GRAVITY_CHANGED')
        gravity = s.setup.dynamic_mesh.options.six_dof.gravity.get_state()
        if any(float(gravity[key]) != 0 for key in 'xyz'):
            raise RuntimeError('FROZEN_SIX_DOF_GRAVITY_CHANGED')
        nodes = s.setup.dynamic_mesh.dynamic_zones
        for zone in ('robot_wall', 'robot_component_fluid'):
            name = next(name for name in nodes.keys() if nodes[name].zone.get_state() == zone)
            node = nodes[name]
            if not node.motion.six_dof.enabled.get_state() or bool(node.motion.six_dof.passive.get_state()) != (zone == 'robot_component_fluid'):
                raise RuntimeError('FROZEN_ACTIVE_PASSIVE_SIX_DOF_CHANGED')
        robot = surface_mesh(solver.fields.field_data, 'robot_wall')
        base = pv.read(branch_dir / 'fielddata' / 'robot_0000.vtp')
        rotation = Rotation.from_quat(np.array(wall['q'])[[1, 2, 3, 0]])
        transformed = rotation.apply(base.points - COM_REFERENCE) + np.asarray(wall['com'])
        err = float(cKDTree(robot.points).query(transformed)[0].max())
        result['native_quaternion_surface_error_m'] = err
        if err > 1e-9:
            raise RuntimeError('RESTART_CONTINUITY_FAIL_NATIVE_SURFACE')
        force, torque, source = compute_magnetic_load(wall['com'], np.array(wall['q'])[[1, 2, 3, 0]], before)
        expected_f = checkpoint.get('Fmag_N')
        expected_t = checkpoint.get('Tmag_COM_Nm', checkpoint.get('Tmag_Nm'))
        if expected_f is None or expected_t is None:
            row = checkpoint.get('row', {})
            expected_f = [float(row['Fmag_' + axis + '_N']) for axis in 'xyz']
            expected_t = [float(row['Tmag_' + axis + '_Nm']) for axis in 'xyz']
        ferr = float(np.max(abs(force - np.asarray(expected_f))))
        terr = float(np.max(abs(torque - np.asarray(expected_t))))
        result.update(Fmag_N=force.tolist(), Tmag_COM_Nm=torque.tolist(), source_state=source,
                      F_error_N=ferr, T_error_Nm=terr)
        if ferr > 1e-13 or terr > 1e-14:
            raise RuntimeError('RESTART_CONTINUITY_FAIL_ABSOLUTE_MAGNETIC_LOAD')
        export = output_dir / 'restart_official_cells.csv'
        s.file.export.ascii(file_name=str(export), surface_name_list=[], delimiter='comma',
                            quantities=['overset-cell-type', 'overset-donor-size-ratio', 'cell-volume'], location='cell-center')
        stats = summarize_csv(export)
        donors = solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR', zone_names=['background_water', 'robot_component_fluid'])
        valid = sum(int(np.count_nonzero(donors[zone] > 0)) for zone in ('background_water', 'robot_component_fluid'))
        gap = float(TUBE_RADIUS_M - np.linalg.norm(robot.points[:, 1:3], axis=1).max())
        stats.update(invalid_donors=max(0, stats['receptors'] - valid), signed_gap_m=gap,
                     donor_length_ratio_policy='WARNING_ONLY', old_C_clearance_gate_applied=False)
        result['overset'] = stats
        if stats['total_cells'] != 4699301 or stats['orphans'] or stats['invalid_donors'] or stats['cell_type_counts']['-3'] or stats['minimum_volume_m3'] <= 0 or gap < -config['penetration_tolerance_m']:
            raise RuntimeError('RESTART_CONTINUITY_FAIL_CONNECTIVITY_VOLUME_PENETRATION')
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)")) - before) > TIME_TOLERANCE_S:
            raise RuntimeError('RESTART_VALIDATOR_ADVANCED_DYNAMICS')
        result['status'] = 'PASS'
        atomic(branch_dir / 'source_restart_validation.json', result)
        return result
    except Exception as error:
        result.update(status='FAIL', error=repr(error))
        atomic(branch_dir / 'source_restart_validation.json', result)
        raise


def collect(solver, branch_dir, library, pipe, pipe_sign, config):
    solver.settings.setup.user_defined.execute_on_demand(lib_name='overwrite_semantics_snapshot::' + library)
    actual_time = float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    snapshots = [item for item in json_rows(branch_dir / 'theta_lifecycle_node.jsonl')
                 if item.get('stage') == 'BOUNDARY_SNAPSHOT' and abs(item['CURRENT_TIME'] - actual_time) <= TIME_TOLERANCE_S]
    if not snapshots:
        raise RuntimeError('MISSING_ACTUAL_NATIVE_DT_BOUNDARY_SNAPSHOT')
    direct = snapshots[-1]
    if not np.isfinite(np.r_[direct['CG'], direct['Q'], direct['velocity'], direct['omega']]).all():
        raise RuntimeError('NONFINITE_ACTUAL_NATIVE_DT_STATE')
    direct_q_norm = float(np.linalg.norm(direct['Q']))
    if abs(direct_q_norm - 1.) > 1e-6:
        raise RuntimeError('DYNAMIC_HARD_GATE:RAW_NATIVE_QUATERNION')
    actual = native_state(solver, 'robot_wall')
    direct_state = dict(com=direct['CG'], q=direct['Q'], velocity=direct['velocity'], omega=direct['omega'])
    direct_errors = compare(actual, direct_state)
    if any(item['status'] != 'PASS' for item in direct_errors.values()):
        raise RuntimeError('NATIVE_SDK_STATE_DISAGREES_WITH_DIRECT_DT_SNAPSHOT')
    component = native_state(solver, 'robot_component_fluid')
    shared = compare(component, actual)
    if any(item['status'] != 'PASS' for item in shared.values()):
        raise RuntimeError('ACTIVE_PASSIVE_BODY_STATE_MISMATCH')
    solver.settings.setup.user_defined.execute_on_demand(lib_name='lifecycle_connectivity::' + library)
    connectivity = read(branch_dir / 'native_connectivity_current.json')
    zones = connectivity['zones']
    stats = {key: sum(zone[key] for zone in zones) for key in ('total', 'orphan', 'invalid_donors', 'unidentified', 'nonpositive_volume', 'nonfinite_volume')}
    stats['minimum_volume_m3'] = min(zone['minimum_volume_m3'] for zone in zones)
    if abs(connectivity['time_s'] - actual_time) > TIME_TOLERANCE_S:
        raise RuntimeError('CONNECTIVITY_NATIVE_TIME_MISMATCH')
    robot = surface_mesh(solver.fields.field_data, 'robot_wall')
    radius = np.linalg.norm(robot.points[:, 1:3], axis=1)
    signed_gap = float(TUBE_RADIUS_M - radius.max())
    # Analytic cylinder radial distance is exact for the actual linear surface
    # geometry: the convex radial norm attains its maximum at a face vertex.
    index = int(np.argmax(radius))
    closest = robot.points[index]
    normal = np.array([0., -closest[1], -closest[2]]) / radius[index]
    meshed_gap = float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance'] * pipe_sign))
    quaternion = np.asarray(actual['q'])
    rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
    axial_direction = rotation.apply([1., 0., 0.])
    force, torque, source = compute_magnetic_load(actual['com'], quaternion[[1, 2, 3, 0]], actual_time)
    native_index = int(solver.scheme.eval("(rpgetvar 'time-step)"))
    record = dict(time_s=actual_time, native_step_index=native_index, native_state=actual,
                  evidence_branch=config['branch'], logical_branch=config.get('logical_branch', config['branch']), dt_s=config['dt_s'],
                  shared_body_errors=shared, connectivity=stats, signed_gap_m=signed_gap,
                  physical_gap_m=signed_gap, meshed_pipe_signed_gap_m=meshed_gap,
                  maximum_penetration_m=max(0., -signed_gap), closest_surface_point_m=closest.tolist(),
                  inward_normal=normal.tolist(), tube_radius_m=TUBE_RADIUS_M,
                  gap_definition='IDEAL_CYLINDER_EXACT_FOR_ACTUAL_TRIANGULATED_ROBOT_SURFACE',
                  q_norm=float(np.linalg.norm(quaternion)), tilt_angle_rad=float(math.acos(np.clip(axial_direction[0], -1., 1.))),
                  direct_DT_Q_wxyz=direct['Q'], direct_DT_Q_norm=direct_q_norm, direct_DT_state_errors=direct_errors,
                  omega_magnitude_rad_s=float(np.linalg.norm(actual['omega'])),
                  Fmag_N=force.tolist(), Tmag_Nm=torque.tolist(), source_state=source)
    reasons = []
    if stats['total'] != 4699301:
        reasons.append('CELL_COUNT')
    for key in ('orphan', 'invalid_donors', 'unidentified', 'nonpositive_volume', 'nonfinite_volume'):
        if stats[key]:
            reasons.append(key.upper())
    if stats['minimum_volume_m3'] <= 0:
        reasons.append('VOLUME')
    if abs(record['q_norm'] - 1) > 1e-6:
        reasons.append('QUATERNION')
    if not np.isfinite(np.r_[actual['com'], actual['q'], actual['velocity'], actual['omega'], force, torque, signed_gap]).all():
        reasons.append('NONFINITE')
    if signed_gap < -config['penetration_tolerance_m']:
        reasons.append('PHYSICAL_PENETRATION')
    record.update(hard_failures=reasons, status='PASS' if not reasons else 'FAIL')
    name = f'state_{native_index:04d}'
    atomic(branch_dir / (name + '.json'), record)
    robot.save(branch_dir / 'fielddata' / (name + '_robot.vtp'))
    return record


def snapshot_velocity(solver, branch_dir, record):
    """Export actual FieldData; export faults never silently become visual PASS."""
    from ansys.fluent.core.fields.field_data_interfaces import ScalarFieldDataRequest
    native_index = record['native_step_index']
    t = record['time_s']
    name = 'benchmark_d_native_contact_velocity_midplane'
    planes = solver.settings.results.surfaces.plane_surface
    if name not in planes.keys():
        planes.create(name=name)
        planes[name].method = 'xy-plane'
        planes[name].z = 0.
    fd = solver.fields.field_data
    fluid = surface_mesh(fd, name)
    velocity = np.asarray(fd.get_field_data(ScalarFieldDataRequest(field_name='velocity-magnitude', surfaces=[name], node_value=True, boundary_value=False))[name], dtype=float)
    if not len(velocity) or len(velocity) != fluid.n_points or not np.isfinite(velocity).all():
        raise RuntimeError('ACTUAL_VELOCITY_FIELDDATA_INVALID')
    fluid.point_data['velocity_m_s'] = velocity
    velocity_path = branch_dir / 'fielddata' / f'state_{native_index:04d}_velocity.vtp'
    fluid.save(velocity_path)
    surface_mesh(fd, 'overset_component').save(branch_dir / 'fielddata' / f'state_{native_index:04d}_component.vtp')
    info = read(branch_dir / 'snapshots.json', {'frames': []})
    if not any(abs(frame['time_s'] - t) < TIME_TOLERANCE_S for frame in info['frames']):
        info['frames'].append(dict(native_step_index=native_index, time_s=t,
                                  robot_path=str((branch_dir / 'fielddata' / f'state_{native_index:04d}_robot.vtp').relative_to(ROOT)),
                                  velocity_path=str(velocity_path.relative_to(ROOT)), actual_FieldData=True,
                                  velocity_points=int(fluid.n_points), velocity_min_m_s=float(velocity.min()), velocity_max_m_s=float(velocity.max())))
    atomic(branch_dir / 'snapshots.json', info)
    if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)")) - t) > TIME_TOLERANCE_S:
        raise RuntimeError('FIELDDATA_EXPORT_ADVANCED_DYNAMICS')


def save_native(solver, branch_dir, output_dir, record, label, verified=True):
    case_path = output_dir / (label + '.cas.h5')
    data_path = output_dir / (label + '.dat.h5')
    if case_path.exists() or data_path.exists():
        raise RuntimeError('NATIVE_CHECKPOINT_OVERWRITE_REFUSED')
    actual_time = float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    actual = native_state(solver, 'robot_wall')
    errors = compare(actual, record['native_state'])
    if abs(actual_time - record['time_s']) > TIME_TOLERANCE_S or any(item['status'] != 'PASS' for item in errors.values()):
        raise RuntimeError('NATIVE_CHECKPOINT_HISTORY_STATE_MISMATCH')
    solver.settings.file.write_case(file_name=str(case_path))
    solver.settings.file.write_data(file_name=str(data_path))
    force, torque, source = compute_magnetic_load(actual['com'], np.asarray(actual['q'])[[1, 2, 3, 0]], actual_time)
    metadata = dict(timestamp=stamp(), status='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS' if verified else 'SAVED_HASHED_STOP_PENDING_REVIEW',
                    step=record.get('native_step_index', int(solver.scheme.eval("(rpgetvar 'time-step)"))),
                    native_step_index=record.get('native_step_index'), time_s=actual_time, native_state=actual,
                    evidence_branch=record.get('evidence_branch'), logical_branch=record.get('logical_branch'),
                    state_errors=errors, case=str(case_path), data=str(data_path),
                    case_sha256=sha(case_path), data_sha256=sha(data_path),
                    case_size_bytes=case_path.stat().st_size, data_size_bytes=data_path.stat().st_size,
                    Fmag_N=force.tolist(), Tmag_COM_Nm=torque.tolist(), source_state=source,
                    signed_gap_m=record.get('signed_gap_m'), dt_s=record.get('dt_s'),
                    large_native_files_local_only=True)
    metadata['integrity'] = checkpoint_integrity(metadata)
    metadata['files'] = metadata['integrity']['files']
    atomic(branch_dir / (label + '_checkpoint.json'), metadata)
    return metadata


def compile_and_hook(solver, branch_dir, output_dir, config):
    library = 'libD_overnight_' + config['branch'] + '_v1'
    path = output_dir / library
    source = ROOT / config['contact_source_path']
    magnetic = output_dir / 'magnetic_logging_copy.c'
    header = output_dir / 'lifecycle_config.h'
    for file_path, key in ((source, 'contact_source_sha256'), (magnetic, 'magnetic_logging_copy_sha256'), (header, 'configuration_header_sha256')):
        if sha(file_path) != config[key]:
            raise RuntimeError('REVIEWED_BRANCH_SOURCE_CONFIG_CHANGED_' + key)
    kernel = ROOT / 'fluent_udf/l2300_contact_impulse_math.h'
    if sha(kernel) != config['mathematical_kernel_sha256']:
        raise RuntimeError('VALIDATED_IMPULSE_KERNEL_CHANGED')
    headers = [header, kernel]
    if config.get('route', 'NATIVE_OVERWRITE') != 'NATIVE_OVERWRITE':
        geometry = output_dir / 'load_contact_geometry.h'
        if sha(geometry) != config.get('geometry_header_sha256'):
            raise RuntimeError('REVIEWED_LOAD_GEOMETRY_HEADER_CHANGED')
        headers.append(geometry)
    solver.settings.setup.user_defined.compiled_udf(library_name=str(path), source_files=[str(source), str(magnetic)],
                                                  header_files=[str(item) for item in headers], use_built_in_compiler=True)
    dlls = [path / 'win64' / kind / 'libudf.dll' for kind in ('3ddp_host', '3ddp_node')]
    latest = max(file_path.stat().st_mtime for file_path in (source, magnetic, *headers))
    if any(not dll.is_file() or dll.stat().st_mtime < latest for dll in dlls):
        raise RuntimeError('ACTUAL_FRESH_HOST_NODE_DLLS_REQUIRED')
    solver.settings.setup.user_defined.load(udf_library_name=str(path))
    nodes = solver.settings.setup.dynamic_mesh.dynamic_zones
    for zone in ('robot_wall', 'robot_component_fluid'):
        name = next(name for name in nodes.keys() if nodes[name].zone.get_state() == zone)
        nodes[name].motion.motion_def = 'lifecycle_magnetic_state_probe::' + library
    contact = solver.settings.setup.dynamic_mesh.options.contact_detection
    route = config.get('route', 'NATIVE_OVERWRITE')
    if route == 'NATIVE_OVERWRITE':
        contact.enabled = True
        contact.face_zones = ['robot_wall', 'pipe_wall']
        contact.proximity_threshold = config['threshold_m']
        contact.contact_udf = 'l2300_native_contact_lifecycle::' + library
        contact.flow_control.enabled = False
        contact.verbosity = 1
    else:
        # Reviewed conditional fallback contributes normal F/T in properties;
        # simultaneous native contact impulses would double the response.
        contact.enabled = False
    atomic(branch_dir / 'compile_hook_gate.json', dict(status='PASS', timestamp=stamp(), library=library,
           source_sha256=config['contact_source_sha256'], config_sha256=sha(branch_dir / 'configuration.json'),
           dll_sha256={str(dll.relative_to(ROOT)): sha(dll) for dll in dlls}, contact_state=contact.get_state(),
           magnetic_formula_delegated_once=True, route=route,
           true_solver_contact_only=route == 'NATIVE_OVERWRITE',
           native_contact_disabled_for_load_path=route != 'NATIVE_OVERWRITE'))
    return library


def merge_contact_events(branch_dir, records, config):
    """Node records are physical applications; host records are transport copies."""
    callbacks = json_rows(branch_dir / 'native_contact_callback_trace_node.jsonl')
    events = []
    applied_by_step = {}
    for callback in callbacks:
        analytic_contact = config.get('route', 'NATIVE_OVERWRITE') != 'NATIVE_OVERWRITE' and callback.get('analytic_contact_point_count', 0) > 0
        if not callback.get('moving_body') or (callback.get('contact_face_count', 0) <= 0 and not analytic_contact):
            continue
        for key in ('contact_point', 'contact_normal', 'get_velocity', 'get_omega'):
            if key in callback and not np.isfinite(np.asarray(callback[key])).all():
                raise RuntimeError('NONFINITE_CONTACT_' + key)
        if not np.isfinite(callback.get('impulse_N_s', 0)):
            raise RuntimeError('NONFINITE_CONTACT_IMPULSE')
        event = dict(callback, evidence_branch=config['branch'])
        matches = [record for record in records if abs(record['time_s'] - (float(callback['CURRENT_TIME']) + config['dt_s'])) < TIME_TOLERANCE_S]
        if matches:
            completed = matches[0]
            event.update(completed_time_s=completed['time_s'], completed_native_state=completed['native_state'],
                         completed_signed_gap_m=completed['signed_gap_m'])
        events.append(event)
        if callback.get('action') == 'APPLIED' and (callback.get('overwrite_called') or callback.get('physical_impulse_applied')):
            key = (callback['CURRENT_TIME'], callback.get('timestep_index'), callback.get('dynamic_zone_id'), callback.get('opposite_zone_id'))
            point = np.asarray(callback['contact_point'])
            previous = applied_by_step.setdefault(key, [])
            if any(np.linalg.norm(point - old_point) <= 20e-6 for old_point in previous):
                raise RuntimeError('DUPLICATE_PHYSICAL_CONTACT_IMPULSE')
            previous.append(point)
    (branch_dir / 'contact_events.jsonl').write_text(''.join(json.dumps(event, allow_nan=False) + '\n' for event in events), encoding='utf-8')
    if events:
        keys = sorted(set().union(*(event.keys() for event in events)))
        with (branch_dir / 'contact_events.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            for event in events:
                writer.writerow({key: json.dumps(value, allow_nan=False) if isinstance(value, (dict, list)) else value for key, value in event.items()})
    return events


def classify_failure(error):
    text = str(error)
    if text == 'TIME_BUDGET_EXHAUSTED':
        return text
    if 'RESOURCE' in text or 'MEMORY' in text:
        return 'RESOURCE_HARD_STOP'
    if 'RESTART_' in text:
        return 'RESTART_CONTINUITY_FAIL'
    if 'PHYSICAL_PENETRATION' in text:
        return 'CONTACT_TIMESTEP_PENETRATION'
    if 'DUPLICATE' in text:
        return 'CALLBACK_DUPLICATION'
    if 'NONFINITE' in text:
        return 'NONFINITE_CONTACT_OR_STATE'
    if 'DYNAMIC_HARD_GATE' in text:
        return 'NUMERICAL_DYNAMIC_GATE'
    return 'WORKER_LIFECYCLE_TERMINATION'


def fatal_transcript_evidence(branch_dir):
    """A Python/transport disconnect alone is not a Fluent fatal result."""
    from benchmark_C_coarse_commit_audit import redact
    path = branch_dir / 'solver.trn'
    if not path.is_file():
        return []
    patterns = r'(?i)(fatal error|floating point exception|segmentation (violation|fault)|SIGSEGV|^\s*Error at (host|node)\b)'
    found = []
    with path.open('r', encoding='utf-8', errors='replace') as stream:
        for number, line in enumerate(stream, 1):
            if re.search(patterns, line):
                found.append(dict(source=str(path.relative_to(ROOT)), line_number=number,
                                  excerpt=redact(line.strip())[:500]))
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--branch', required=True)
    branch = parser.parse_args().branch
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', branch):
        raise RuntimeError('INVALID_REVIEWED_BRANCH_DIRECTORY_NAME')
    verify_frozen()
    OUT.mkdir(parents=True, exist_ok=True)
    import msvcrt
    lock = (OUT / 'native_contact_worker.lock').open('a+b')
    lock.seek(0)
    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
    if (EVID / 'branches' / branch / 'worker_identity.json').exists():
        raise RuntimeError('PREVIOUS_BRANCH_ATTEMPT_REQUIRES_DISTINCT_REVIEWED_RECOVERY_SPECIFICATION')
    branch_dir, output_dir, config = prepare_branch(branch)
    route = config.get('route', 'NATIVE_OVERWRITE')
    allowed = route == 'NATIVE_OVERWRITE' and config.get('mode') == 3
    allowed = allowed or route in ('SDOF_LOAD_PATH', 'COMPLIANT_LOAD_PATH') and config.get('mode') in (2, 3)
    allowed = allowed or route == 'SDOF_LOAD_PATH' and config.get('mode') == 0 and config.get('load_disabled_reference') is True
    if not allowed:
        raise RuntimeError('UNREVIEWED_CONTACT_ROUTE_OR_MODE')
    for key in ('start_time_s', 'end_time_s', 'dt_s', 'penetration_tolerance_m'):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise RuntimeError('INVALID_REVIEWED_BRANCH_CONFIGURATION_' + key)
    if config['end_time_s'] > .002 + TIME_TOLERANCE_S:
        raise RuntimeError('UNAUTHORIZED_DYNAMICS_BEYOND_2MS')
    intervals = (config['end_time_s'] - config['start_time_s']) / config['dt_s']
    limit = int(round(intervals))
    if limit < 1 or abs(intervals - limit) > 1e-8:
        raise RuntimeError('TARGET_TIME_NOT_ON_REVIEWED_NATIVE_TIMESTEP')
    review = read(branch_dir / 'launch_review.json')
    if review.get('status') != 'PASS':
        raise RuntimeError('CONCRETE_BRANCH_LAUNCH_REVIEW_REQUIRED')
    for relative, digest in review.get('files_sha256', {}).items():
        if sha(ROOT / relative) != digest:
            raise RuntimeError('REVIEWED_LAUNCH_INPUT_CHANGED_' + relative)
    config['_reviewed_window_sha256'] = read(EVID / 'configuration.json')['window_sha256']
    sys.stdout = (branch_dir / 'worker_stdout.log').open('a', buffering=1)
    sys.stderr = (branch_dir / 'worker_stderr.log').open('a', buffering=1)
    created = psutil.Process().create_time()
    atomic(branch_dir / 'worker_identity.json', dict(pid=os.getpid(), created=created, branch=branch, timestamp=stamp()))
    import benchmark_D_native_worker as memory_module
    memory_module.EVID = branch_dir
    memory_module.OUT = output_dir
    profile = memory_module.Profile()
    solver = None
    last = None
    initial = None
    records = []
    status = 'INCOMPLETE'
    first_event_iteration = None
    try:
        check_control(config)
        worker_state(branch_dir, 'RESOURCE_WAIT', branch=branch, worker_pid=os.getpid(), worker_created=created,
                     worker_alive=True, solver_alive=False, completed_new_steps=0, current_time_s=config['start_time_s'])
        stable = None
        while True:
            check_control(config)
            if native():
                raise RuntimeError('EXISTING_NATIVE_ENGINE_DUPLICATE_LAUNCH_REFUSED')
            sample = profile.sample()
            okay = sample['available_gib'] >= 12 and sample['commit_headroom_gib'] >= 19 and sample['disk_free_gib'] >= 35
            stable = (stable or time.monotonic()) if okay else None
            if stable is not None and time.monotonic() - stable >= 60:
                break
            time.sleep(5)
        verify_frozen()
        checkpoint = read(ROOT / config['source_checkpoint'])
        checkpoint_integrity(checkpoint)
        # A prelaunch resource sample is a waiting condition, not a runtime
        # failure. Admission has now been continuously safe for sixty seconds.
        profile.abort = None
        check_control(config)
        if native():
            raise RuntimeError('EXISTING_NATIVE_ENGINE_DUPLICATE_LAUNCH_REFUSED')
        import ansys.fluent.core as pyfluent
        worker_state(branch_dir, 'NATIVE_RESTART', branch=branch, solver_alive=False)
        solver = pyfluent.launch_fluent(mode='solver', dimension=3, precision='double', processor_count=1,
             ui_mode='no_gui_or_graphics', start_timeout=180, start_watchdog=False,
             fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe', cwd=str(output_dir))
        profile.solver = solver
        profile.stage = 'NATIVE_RESTART'
        profile.sample()
        profile.thread.start()
        solver.transcript.start(str(branch_dir / 'solver.trn'))
        solver.settings.file.read_case(file_name=str(Path(checkpoint['case'])))
        solver.settings.file.read_data(file_name=str(Path(checkpoint['data'])))
        profile.check('RESTART_VALIDATION')
        restart_result = native_restart_gate(solver, branch_dir, output_dir, checkpoint, config)
        check_control(config)
        library = compile_and_hook(solver, branch_dir, output_dir, config)
        fd = solver.fields.field_data
        pipe = surface_mesh(fd, 'pipe_wall')
        pipe.save(branch_dir / 'fielddata' / 'pipe.vtp')
        pipe_sign = float(np.sign(pv.PolyData(np.array([restart_result['native_state']['com']])).compute_implicit_distance(pipe)['implicit_distance'][0]))
        last = collect(solver, branch_dir, library, pipe, pipe_sign, config)
        initial = last
        for key, official_key in (('orphan', 'orphans'), ('invalid_donors', 'invalid_donors'), ('total', 'total_cells')):
            if last['connectivity'][key] != restart_result['overset'][official_key]:
                raise RuntimeError('NATIVE_CONNECTIVITY_OFFICIAL_CROSSCHECK_FAIL')
        if abs(last['connectivity']['minimum_volume_m3'] - restart_result['overset']['minimum_volume_m3']) > 1e-25:
            raise RuntimeError('NATIVE_VOLUME_OFFICIAL_CROSSCHECK_FAIL')
        atomic(branch_dir / 'native_connectivity_crosscheck.json', dict(status='PASS', timestamp=stamp(),
               native=last['connectivity'], official=restart_result['overset']))
        if last['hard_failures']:
            raise RuntimeError('RESTART_CONTINUITY_FAIL_' + ','.join(last['hard_failures']))
        calc = solver.settings.solution.run_calculation
        # Only the explicitly authorized branch timestep changes here. Native
        # absolute time, phase, COM/Q/v/omega and magnetic law are untouched.
        calc.parameters.time_step_size = config['dt_s']
        calc.parameters.max_iter_per_time_step = 2
        if abs(float(solver.scheme.eval("(rpgetvar 'physical-time-step)")) - config['dt_s']) > TIME_TOLERANCE_S:
            raise RuntimeError('REVIEWED_BRANCH_TIMESTEP_NOT_INSTALLED')
        atomic(branch_dir / 'native_history.json', dict(branch=branch, initial_state=initial, start_time_s=config['start_time_s'],
               dt_s=config['dt_s'], end_time_s=config['end_time_s'], records=[]))
        latest_checkpoint = save_native(solver, branch_dir, output_dir, last, f"native_{last['native_step_index']:04d}")
        atomic(branch_dir / 'latest_verified_checkpoint.json', dict(status='PASS', timestamp=stamp(),
               metadata=str((branch_dir / f"native_{last['native_step_index']:04d}_checkpoint.json").relative_to(ROOT)),
               time_s=last['time_s'], checkpoint=latest_checkpoint))
        for iteration in range(0, limit + 1):
            if iteration:
                check_control(config)
                profile.check('ACTUAL_TRANSIENT_SOLVE')
                worker_state(branch_dir, 'WORKER_RUNNING', branch=branch, completed_new_steps=iteration - 1,
                             target_new_steps=limit, current_time_s=last['time_s'], solver_alive=True)
                atomic(branch_dir / f'before_step_{iteration:02d}.json', dict(time_s=last['time_s'], native_state=native_state(solver, 'robot_wall')))
                calc.dual_time_iterate(time_step_count=1, max_iter_per_step=2)
                last = collect(solver, branch_dir, library, pipe, pipe_sign, config)
                last['dt_s'] = config['dt_s']
                last['local_iteration'] = iteration
                records.append(last)
                if abs(last['time_s'] - (config['start_time_s'] + iteration * config['dt_s'])) > TIME_TOLERANCE_S:
                    raise RuntimeError('ABSOLUTE_NATIVE_TIME_RESET_OR_ADVANCE_MISMATCH')
                atomic(branch_dir / 'native_history.json', dict(branch=branch, initial_state=initial, start_time_s=config['start_time_s'],
                       dt_s=config['dt_s'], end_time_s=config['end_time_s'], records=records))
                events = merge_contact_events(branch_dir, records, config)
                if events and first_event_iteration is None:
                    first_event_iteration = iteration
                branch_event(branch_dir, 'NATIVE_STEP_COMPLETE', time_s=last['time_s'], new_steps=iteration,
                             contact_callback_count=len(events), signed_gap_m=last['signed_gap_m'])
                if last['hard_failures']:
                    raise RuntimeError('DYNAMIC_HARD_GATE:' + ','.join(last['hard_failures']))
                profile.check('POST_NATIVE_TIMESTEP_AUDIT')
                label = f"native_{last['native_step_index']:04d}"
                latest_checkpoint = save_native(solver, branch_dir, output_dir, last, label)
                atomic(branch_dir / 'latest_verified_checkpoint.json', dict(status='PASS', timestamp=stamp(),
                       metadata=str((branch_dir / (label + '_checkpoint.json')).relative_to(ROOT)),
                       time_s=last['time_s'], checkpoint=latest_checkpoint))
            try:
                snapshot_velocity(solver, branch_dir, last)
            except Exception as export_error:
                atomic(branch_dir / 'fielddata' / f"export_failure_{last['native_step_index']:04d}.json", dict(timestamp=stamp(), error=repr(export_error), time_s=last['time_s']))
                branch_event(branch_dir, 'FIELDDATA_EXPORT_FAILED', error=repr(export_error), time_s=last['time_s'])
            if iteration and abs(last['time_s'] - .0019) < TIME_TOLERANCE_S:
                atomic(branch_dir / 'checkpoint_1p900ms_checkpoint.json', latest_checkpoint)
            if iteration and abs(last['time_s'] - .002) < TIME_TOLERANCE_S:
                atomic(branch_dir / 'checkpoint_2p000ms_checkpoint.json', latest_checkpoint)
        if abs(last['time_s'] - config['end_time_s']) > TIME_TOLERANCE_S:
            raise RuntimeError('BRANCH_TARGET_NATIVE_TIME_NOT_REACHED')
        # Boundary probe establishes inherited native state without inventing a
        # dynamics step after the authorized final 2 ms time.
        actual = native_state(solver, 'robot_wall')
        atomic(branch_dir / 'final_next_start.json', dict(time_s=last['time_s'], native_state=actual,
               timesteps_advanced=0, validation_kind='NATIVE_BOUNDARY_READ; actual next PROPERTIES_ENTRY exists for nonterminal callbacks'))
        atomic(branch_dir / 'final_native_checkpoint.json', latest_checkpoint)
        verify_frozen()
        status = 'NATIVE_BRANCH_DATA_COMPLETE' if route == 'NATIVE_OVERWRITE' else 'LOAD_PATH_BRANCH_DATA_COMPLETE'
        atomic(branch_dir / 'branch_result.json', dict(status=status, branch=branch, timestamp=stamp(), stage=config.get('stage'),
               route=route, dt_s=config['dt_s'], start_time_s=config['start_time_s'], end_time_s=config['end_time_s'],
               actual_contact_triggered=first_event_iteration is not None, first_contact_completed_iteration=first_event_iteration,
               completed_new_steps=len(records), last_state=last, final_checkpoint=str((branch_dir / 'final_native_checkpoint.json').relative_to(ROOT)),
               previous_evidence_preserved=True))
        worker_state(branch_dir, 'AWAITING_OFFLINE_BRANCH_REVIEW', current_time_s=last['time_s'], completed_new_steps=len(records), solver_alive=True)
    except Exception as error:
        status = classify_failure(error)
        fatal_evidence = fatal_transcript_evidence(branch_dir)
        if fatal_evidence:
            status = 'FLUENT_FATAL_ERROR'
        failure = dict(timestamp=stamp(), branch=branch, error=repr(error), failure_class=status,
                       traceback=traceback.format_exc(), latest_state=last, completed_new_steps=len(records), profile_abort=profile.abort,
                       actual_fluent_fatal_evidence=fatal_evidence)
        if solver is not None and last is not None:
            try:
                actual = native_state(solver, 'robot_wall')
                actual_time = float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                provisional = dict(time_s=actual_time, native_state=actual,
                                   native_step_index=int(solver.scheme.eval("(rpgetvar 'time-step)")),
                                   signed_gap_m=last.get('signed_gap_m'), dt_s=config['dt_s'], not_a_numerical_gate_PASS=True)
                if abs(actual_time - last['time_s']) <= TIME_TOLERANCE_S and all(item['status'] == 'PASS' for item in compare(actual, last['native_state']).values()):
                    # Numerical contact failure still needs an actual terminal
                    # boundary read for response attribution and dt diagnosis.
                    # This never upgrades penetration/connectivity to PASS.
                    atomic(branch_dir / 'final_next_start.json', dict(time_s=actual_time, native_state=actual,
                           timesteps_advanced=0, numerical_status=last['status'],
                           validation_kind='ZERO_STEP_FAILED_BRANCH_NATIVE_BOUNDARY; not a numerical gate PASS'))
                failure['saved_native_checkpoint'] = save_native(solver, branch_dir, output_dir, provisional, 'failure_native', verified=False)
            except Exception as save_error:
                failure['checkpoint_save_error'] = repr(save_error)
        atomic(branch_dir / 'failure.json', failure)
        worker_state(branch_dir, 'TIME_BUDGET_EXHAUSTED' if status == 'TIME_BUDGET_EXHAUSTED' else 'AWAITING_OFFLINE_FAILURE_REVIEW',
                     branch=branch, error=repr(error), failure_class=status, current_time_s=last['time_s'] if last else config['start_time_s'])
        traceback.print_exc()
    finally:
        if solver is not None:
            import benchmark_C_coarse_2ms_worker as exit_module
            exit_module.EVID = branch_dir
            try:
                exit_module.exit_owned(solver, profile)
            except Exception as error:
                atomic(branch_dir / 'closure_failure.json', dict(error=repr(error), timestamp=stamp()))
        profile.stop.set()
        if profile.thread.is_alive():
            profile.thread.join(10)
        if profile.rows:
            atomic(branch_dir / 'memory_summary.json', dict(timestamp=stamp(), samples=len(profile.rows),
                   peak_project_working_set_gib=max(row['project_working_set_gib'] for row in profile.rows),
                   peak_project_private_bytes_gib=max(row['project_private_bytes_gib'] for row in profile.rows),
                   peak_system_commit_percent=100 * max(row['commit_fraction'] for row in profile.rows),
                   minimum_available_gib=min(row['available_gib'] for row in profile.rows), abort=profile.abort))
        closed = not any(identity(rec) for rec in profile.owned.values())
        atomic(branch_dir / 'worker_exit.json', dict(timestamp=stamp(), status=status, worker_pid=os.getpid(), worker_created=created,
               solver_closed=closed, completed_new_steps=len(records), current_time_s=last['time_s'] if last else config['start_time_s']))
        worker_state(branch_dir, read(branch_dir / 'worker_state.json').get('status', status), worker_alive=False, solver_alive=not closed)


if __name__ == '__main__':
    main()
