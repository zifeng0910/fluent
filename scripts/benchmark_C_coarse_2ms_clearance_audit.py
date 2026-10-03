"""Offline review of a saved clearance stop; never connects or advances Fluent."""
import json, re, shutil
import h5py
import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_2ms_common import *
from benchmark_C_recovery_v2_restart import compare, state_vector


def main():
    verify_frozen()
    failure = read(EVID/'failure.json')
    pose = read(EVID/'failure_pose.json')
    checkpoint = read(EVID/'stop_0073.json')
    if pose['hard_failures'] != ['CLEARANCE'] or checkpoint['step'] != 73:
        raise RuntimeError('This audit is specific to the archived step73 clearance stop')
    if failure['stop_checkpoint'] != checkpoint:
        raise RuntimeError('Failure and standalone stop checkpoint differ')
    st = read(EVID/'state.json'); sup = read(EVID/'supervisor_state.json')
    owned = read(EVID/'owned_engine.json')
    identities = {
        'supervisor_alive': identity({'pid':sup['supervisor_pid'], 'created':sup['supervisor_created'], 'name':'pythonw.exe'}) is not None,
        'worker_alive': identity({'pid':st['worker_pid'], 'created':st['worker_created'], 'name':'pythonw.exe'}) is not None,
        'owned_engines_alive': [identity(p) is not None for p in owned['processes']],
    }
    if identities['supervisor_alive'] or identities['worker_alive'] or any(identities['owned_engines_alive']):
        raise RuntimeError('Campaign owner still running; offline terminal review refused')
    archive = EVID/'failure_review/archive_0073'
    archive.mkdir(parents=True, exist_ok=True)
    archived = {}
    for name in ['failure.json', 'failure_pose.json', 'stop_0073.json', 'state.json',
                 'supervisor_state.json', 'dynamic.json', 'engine_exit.json']:
        source = EVID/name; dest = archive/name
        if not dest.exists(): shutil.copy2(source, dest)
        # State may later change to the reviewed terminal status. Other evidence is immutable.
        if name not in ['state.json', 'supervisor_state.json'] and sha(dest) != sha(source):
            raise RuntimeError('Archived failure evidence changed: '+name)
        archived[str(dest.relative_to(ROOT)).replace('\\','/')] = sha(dest)
    stop_audit = checkpoint_audit(checkpoint)
    checkpoints = {str(i):checkpoint_audit(read(EVID/f'checkpoint_{i:04d}.json')) for i in [50,60,70]}
    with h5py.File(checkpoint['data'], 'r') as f:
        raw = f['settings/Data Variables'][()].tobytes().decode('utf-8')
    native_time = float(re.search(r'\(flow-time\s+([^\s)]+)\)', raw)[1])
    native_step = int(re.search(r'\(time-step\s+(\d+)\)', raw)[1])
    if native_step != 73 or abs(native_time-checkpoint['time_s']) > 1e-12:
        raise RuntimeError('Native HDF5 time/step differs from checkpoint metadata')
    saved_state_match = compare(checkpoint['native_state'], state_vector(checkpoint['row']))
    if any(v['status'] != 'PASS' for v in saved_state_match.values()):
        raise RuntimeError('Native saved pose or velocity differs from history')
    fd = EVID/'fielddata'
    pipe = pv.read(fd/'pipe.vtp').triangulate()
    robot = pv.read(fd/'robot_failure_0073.vtp')
    reference = pv.read(fd/'robot_0000.vtp')
    ns = checkpoint['native_state']; q = np.asarray(ns['q'])
    rotation = Rotation.from_quat(q[[1,2,3,0]])
    expected = rotation.apply(reference.points-np.asarray(COM0))+np.asarray(ns['com'])
    if expected.shape != robot.points.shape:
        raise RuntimeError('Actual and reference point inventories differ')
    point_error = float(np.linalg.norm(expected-robot.points, axis=1).max())
    if point_error > 1e-8:
        raise RuntimeError('Full native pose does not reproduce saved actual wall')
    cells, closest = pipe.find_closest_cell(robot.points, return_closest_point=True)
    distances = np.linalg.norm(robot.points-closest, axis=1)
    index = int(distances.argmin()); gap = float(distances[index])
    if abs(gap-pose['physical_clearance_m']) > 1e-10 or gap >= 0.0001:
        raise RuntimeError('Independent geometry does not confirm clearance hard stop')
    rows = history(); last_valid = next(r for r in reversed(rows) if float(r['robot_wall_clearance_m']) >= .0001)
    official = [read(fd/f'connectivity_{i:04d}.json') for i in range(41,74)]
    connectivity_valid = all(s['total_cells']==4699301 and s['orphans']==0 and s['invalid_donors']==0
                             and s['minimum_volume_m3']>0 and s['cell_type_counts']['-3']==0 for s in official)
    if not connectivity_valid: raise RuntimeError('Additional connectivity blocker requires separate diagnosis')
    resources = read(EVID/'memory_summary.json')
    exit_rec = read(EVID/'engine_exit.json')
    if resources['abort'] or not exit_rec['owned_engine_closed']:
        raise RuntimeError('Resource stop or unclosed owned engine requires separate diagnosis')
    sources = [EVID/'failure.json', EVID/'failure_pose.json', EVID/'stop_0073.json',
               EVID/'dynamic_history.csv', EVID/'memory_summary.json', EVID/'engine_exit.json',
               fd/'robot_failure_0073.vtp', fd/'robot_0000.vtp', fd/'pipe.vtp',
               fd/'connectivity_0073.json', Path(pose['official_export'])]
    audit = {
        'timestamp':stamp(), 'status':'PASS', 'diagnostic_timesteps_advanced':0,
        'Fluent_session_launched_or_connected':False,
        'classification':'PHYSICAL_CLEARANCE_HARD_GATE_REQUIRES_USER_REVIEW',
        'campaign_result':'INCOMPLETE', 'step':native_step, 'native_time_s':native_time,
        'last_all_gate_valid_step':int(last_valid['step']), 'last_all_gate_valid_time_s':float(last_valid['time_s']),
        'last_all_gate_valid_clearance_m':float(last_valid['robot_wall_clearance_m']),
        'minimum_clearance_required_m':.0001, 'measured_clearance_m':gap,
        'clearance_shortfall_m':.0001-gap,
        'geometry_method':'Actual robot vertex to closest actual triangulated pipe surface; unsigned Euclidean distance',
        'closest_robot_vertex':index, 'closest_robot_xyz_m':robot.points[index].tolist(),
        'closest_pipe_triangle':int(cells[index]), 'closest_pipe_xyz_m':closest[index].tolist(),
        'native_pose_to_actual_geometry_max_error_m':point_error,
        'native_state':ns, 'native_state_vs_history':saved_state_match,
        'orientation_magnitude_rad':float(rotation.magnitude()), 'static_rotation_bound_rad':.35,
        'COM_translation_magnitude_m':float(np.linalg.norm(np.asarray(ns['com'])-COM0)),
        'static_envelope_exceeded_is_only_warning':True,
        'actual_connectivity_all_steps41_to73_valid':True,
        'orphan_count':0, 'invalid_donor_count':0,
        'minimum_cell_volume_m3':pose['connectivity']['minimum_volume_m3'],
        'local_orphan_spacing_diagnosis':'NOT_APPLICABLE: no orphan or invalid donor at failure',
        'donor_length_ratio_failure_step':checkpoint['row']['donor_length_ratio_max'],
        'donor_length_ratio_policy':'WARNING_ONLY',
        'stop_checkpoint_integrity':stop_audit, 'checkpoint_integrity':checkpoints,
        'checkpoint_review_kind':'Offline HDF5/hash/native saved time and geometry audit; no cold solver reload',
        'resources':resources, 'process_identities':identities,
        'owned_solver_closed':True, 'unrelated_processes_touched':[],
        'repairs_attempted_for_clearance':0,
        'reason_no_unchanged_retry':'Step73 saved physical gap is already below the frozen0.10mm gate. Restart cannot satisfy that gate. Remeshing cannot change this saved physical pose.',
        'required_user_decision':'Any continuation requires explicit new authorization for a changed physical configuration or clearance policy; current frozen scope ends at this stop.',
        'source_sha256':{str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sources},
        'archived_original_evidence_sha256':archived,
    }
    atomic(EVID/'failure_review/clearance_audit.json', audit)
    print(json.dumps({'audit_status':audit['status'], 'classification':audit['classification'],
                      'step':native_step, 'time_s':native_time, 'clearance_mm':1000*gap,
                      'stop_checkpoint_integrity':stop_audit['status']}))


if __name__ == '__main__': main()
