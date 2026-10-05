"""Offline gates for the native contact campaign. Never connects to Fluent.

Tolerances are declared before observing any overnight branch. Finite mesh/time
resolution is separated from callback mathematics and native state inheritance.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from benchmark_D_native_contact_overnight_common import ROOT, EVID, OUT, CAMPAIGN, read, atomic, sha, stamp
from benchmark_C_analytic_reference import compute_magnetic_load

MASS = 8.904428864007322e-6
INERTIA = np.array([[7.257810693523445e-13, 2.782693607479792e-28, -1.9327591150313555e-29],
                    [2.782693607479792e-28, 3.929384741151496e-12, -2.4813784672159598e-29],
                    [-1.9327591150313555e-29, -2.4813784672159598e-29, 3.9293847411514897e-12]])
TIME_TOL = 1e-12
PENETRATION_TOL = 5e-6
TOLERANCES = {
    'penetration_m': PENETRATION_TOL, 'q_norm_error': 1e-6,
    'cluster_distance_m': 20e-6, 'impulse_math_v_m_s': 1e-10,
    'impulse_math_omega_rad_s': 1e-7, 'impulse_math_J_N_s': 1e-16,
    'completed_response_relative_error': .10, 'completed_response_direction_cosine': .99,
    'inheritance_COM_m': 1e-10, 'inheritance_Q_rad': 1e-9,
    'inheritance_v_m_s': 1e-8, 'inheritance_omega_rad_s': 1e-6,
    'magnetic_F_N': 1e-13, 'magnetic_T_Nm': 1e-14,
    'dt_comparison': {'timing_coarse_steps': 1, 'contact_position_m': 20e-6,
        'signed_gap_m': 5e-6, 'penetration_m': 5e-6, 'impulse_relative': .10,
        'final_v_relative': .10, 'final_v_absolute_m_s': .001,
        'final_omega_relative': .10, 'final_omega_absolute_rad_s': 1.,
        'max_tilt_rad': math.radians(.5), 'peak_omega_relative': .10,
        'event_count_absolute': 1, 'event_count_relative': .20},
    'rationale': '5um penetration is below 1% of robot diameter; 20um cluster is the validated dedup resolution. '
        'Response 10%/.99 is the authorized load acceptance. DT comparison uses one coarse step for '
        'contact onset sampling, 10% impulse/state error and .5deg tilt for this development baseline; '
        'these are adequacy gates, not an experimental accuracy or mesh convergence claim.'}


def rows(path):
    p = Path(path)
    return [json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()] if p.exists() else []


def rot(q):
    return Rotation.from_quat(np.asarray(q, dtype=float)[[1, 2, 3, 0]])


def angle(a, b):
    return float((rot(a) * rot(b).inv()).magnitude())


def norm(a):
    return float(np.linalg.norm(a))


def difference(a, b):
    aa, bb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    err, den, length = norm(aa-bb), norm(bb), norm(aa)
    return {'norm_error': err, 'max_abs': float(np.max(abs(aa-bb))),
            'relative_error': err/den if den else None,
            'direction_cosine': float(np.dot(aa, bb)/(length*den)) if length*den else None}


def state_difference(a, b):
    errs = {'com_m': norm(np.asarray(a['com'])-b['com']), 'orientation_rad': angle(a['q'], b['q']),
            'v_m_s': norm(np.asarray(a['velocity'])-b['velocity']),
            'omega_rad_s': norm(np.asarray(a['omega'])-b['omega'])}
    errs['status'] = 'PASS' if errs['com_m'] <= 1e-10 and errs['orientation_rad'] <= 1e-9 and \
        errs['v_m_s'] <= 1e-8 and errs['omega_rad_s'] <= 1e-6 else 'FAIL'
    return errs


def absolute_path(path):
    p = Path(path)
    return p if p.is_absolute() else ROOT/p


def checkpoint_audit(path):
    meta = read(path)
    accepted = meta.get('status') in ('PASS', 'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS')
    if not meta or not accepted:
        return {'status': 'FAIL', 'reason': 'Verified native checkpoint metadata missing', 'metadata': str(path)}
    files = meta.get('files') or meta.get('integrity', {}).get('files', {})
    if not files and all(meta.get(k) for k in ('case', 'data', 'case_sha256', 'data_sha256')):
        files = {k: {'path': meta[k], 'sha256': meta[k+'_sha256'], 'size_bytes': meta[k+'_size_bytes']} for k in ('case', 'data')}
    checks = {}
    for kind in ('case', 'data'):
        info = files.get(kind, {})
        p = absolute_path(info.get('path', '__missing__'))
        okay = p.is_file() and p.stat().st_size == info.get('size_bytes') and sha(p) == info.get('sha256')
        hdf = False
        if okay:
            with h5py.File(p, 'r') as fp:
                hdf = bool(list(fp))
        checks[kind] = {'path': str(p), 'hash_size_PASS': bool(okay), 'HDF5_readable': bool(hdf)}
    return {'status': 'PASS' if all(x['hash_size_PASS'] and x['HDF5_readable'] for x in checks.values()) else 'FAIL',
            'time_s': meta.get('time_s'), 'native_state': meta.get('native_state'), 'files': checks}


def magnetic_audit(b):
    path = b/'magnetic_history.csv'
    errors = [0., 0.]
    count = 0
    if path.exists():
        with path.open(newline='', encoding='utf-8-sig') as fp:
            for r in csv.DictReader(fp):
                t = float(r['time_s'])
                F, T, _ = compute_magnetic_load([float(r[f'cg_{a}_m']) for a in 'xyz'],
                    [float(r[f'q{i}']) for i in (1, 2, 3, 0)], t)
                errors[0] = max(errors[0], float(np.max(abs(F-[float(r[f'Fmag_{a}_N']) for a in 'xyz']))))
                errors[1] = max(errors[1], float(np.max(abs(T-[float(r[f'Tmag_{a}_Nm']) for a in 'xyz']))))
                count += 1
    return {'status': 'PASS' if count and errors[0] <= 1e-13 and errors[1] <= 1e-14 else 'FAIL',
            'actual_property_calls': count, 'max_F_error_N': errors[0], 'max_T_error_Nm': errors[1],
            'actual_absolute_time_and_pose_used': True, 'phase_ramp_offset_not_restarted': True}


def independent_impulse(e):
    v, w = np.asarray(e['get_velocity']), np.asarray(e['get_omega'])
    r, n = np.asarray(e['contact_point'])-e['DT_CG'], np.asarray(e['contact_normal'])
    R = rot(e['DT_Q']).as_matrix()
    I = R@INERTIA@R.T
    vn = float(n@(v+np.cross(w, r)))
    lever = np.cross(r, n)
    J = max(0., -vn/(1/MASS+lever@np.linalg.solve(I, lever)))
    V, W = v+J*n/MASS, w+np.linalg.solve(I, J*lever)
    return V, W, J, vn, float(n@(V+np.cross(W, r)))


def reference_c_rows():
    p = ROOT/'evidence/benchmark_C_coarse_2ms/dynamic_history.csv'
    with p.open(newline='', encoding='utf-8-sig') as fp:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fp)]


def precontact_comparison(states, trace, dt):
    # The unmodified 25us branch is a strict same-dt replay; a refined timestep
    # necessarily changes discretization, so its comparison explicitly reports it.
    applied = [r for r in trace if r.get('action') == 'APPLIED']
    cutoff = min((r['CURRENT_TIME'] for r in applied), default=float('inf'))
    ref = reference_c_rows()
    comparisons = []
    for st in states:
        if st['time_s'] > cutoff+TIME_TOL:
            continue
        match = next((r for r in ref if abs(r['time_s']-st['time_s']) <= TIME_TOL), None)
        if match is None:
            continue  # No interpolation, and no claim for an unmatched time.
        ns = st['native_state']
        old = {'com': [match[f'com_{a}_m'] for a in 'xyz'], 'q': [match[f'q{i}'] for i in range(4)],
               'velocity': [match[f'v{a}_m_s'] for a in 'xyz'],
               'omega': [match[f'omega_{a}_rad_s'] for a in 'xyz']}
        diff = state_difference(ns, old)
        F, T, _ = compute_magnetic_load(old['com'], np.asarray(old['q'])[[1, 2, 3, 0]], match['time_s'])
        diff['F_difference_N'] = norm(np.asarray(st['Fmag_N'])-F)
        diff['T_difference_Nm'] = norm(np.asarray(st['Tmag_Nm'])-T)
        if abs(dt-25e-6) <= TIME_TOL:
            passed = diff['status'] == 'PASS' and diff['F_difference_N'] <= 1e-13 and diff['T_difference_Nm'] <= 1e-14
        else:
            passed = diff['com_m'] <= 1e-6 and diff['orientation_rad'] <= .005 and diff['v_m_s'] <= .001 and \
                diff['omega_rad_s'] <= max(2., .01*norm(old['omega']))
        diff.update(time_s=st['time_s'], status='PASS' if passed else 'FAIL')
        comparisons.append(diff)
    return {'status': 'PASS' if comparisons and all(r['status'] == 'PASS' for r in comparisons) else 'FAIL',
            'comparison_kind': 'SAME_DT_NATIVE_REPLAY' if abs(dt-25e-6) <= TIME_TOL else 'REFINED_DT_DISCRETIZATION_CHECK',
            'same_dt_tolerance': {'COM_m': 1e-10, 'Q_rad': 1e-9, 'v_m_s': 1e-8, 'omega_rad_s': 1e-6},
            'refined_dt_tolerance': {'COM_m': 1e-6, 'Q_rad': .005, 'v_m_s': .001, 'omega_relative': .01, 'omega_floor_rad_s': 2.},
            'exact_time_records': comparisons, 'exact_time_count': len(comparisons), 'interpolation_used': False,
            'cutoff_callback_time_s': cutoff if math.isfinite(cutoff) else None,
            'note': 'F/T law checked exactly at each native pose separately. Refined-dt pose differences are reported as integration differences.'}


def parts_for(name):
    b = EVID/'branches'/name
    config = read(b/'configuration.json')
    parts = [str(n) for n in config.get('history_prefix_branches', [])] + [name]
    if len(set(parts)) != len(parts):
        raise RuntimeError('A lifecycle prefix is listed more than once')
    return [EVID/'branches'/n for n in parts]


def resolve_branch(name):
    return read(EVID/'branch_resolution.json').get(name, name)


def part_cutoffs(parts):
    cutoffs = {}
    for i, part in enumerate(parts):
        if i+1 == len(parts):
            cutoffs[part.name] = float('inf')
            continue
        spec = read(parts[i+1]/'configuration.json')
        source = read(absolute_path(spec['source_checkpoint']))
        cutoff = float(source['time_s'])
        if abs(cutoff-float(spec['start_time_s'])) > TIME_TOL:
            raise RuntimeError('Lifecycle checkpoint time does not equal continuation start')
        cutoffs[part.name] = cutoff
    return cutoffs


def closure_review(part):
    normal = read(part/'worker_exit.json').get('solver_closed') is True and \
        read(part/'engine_exit.json').get('owned_engine_closed') is True
    recovery = read(part/'recovery_process_review.json')
    worker = recovery.get('worker_identity', {})
    owned = recovery.get('owned_engine_identities', [])
    exact = worker.get('pid') is not None and worker.get('created') is not None and bool(worker.get('name')) and \
        bool(owned) and all(r.get('pid') is not None and r.get('created') is not None and r.get('name') for r in owned)
    recovered = recovery.get('status') == 'PASS' and recovery.get('worker_absent') is True and \
        recovery.get('all_owned_engines_absent') is True and recovery.get('no_native_engine_present') is True and exact
    return {'status':'PASS' if normal or recovered else 'FAIL',
            'basis':'WORKER_EXIT_AND_ENGINE_EXIT' if normal else 'EXACT_REGISTERED_PROCESS_RECOVERY_REVIEW' if recovered else 'MISSING_CLOSURE_PROOF',
            'recovery_review':recovery if recovered else None}


def branch_rows(parts, filename):
    rr = []
    cutoffs = part_cutoffs(parts)
    for part in parts:
        for r in rows(part/filename):
            cutoff = cutoffs[part.name]
            if 'callback_trace' in filename:
                keep = r.get('CURRENT_TIME', -float('inf')) < cutoff-TIME_TOL
            elif r.get('stage') == 'BOUNDARY_SNAPSHOT':
                keep = r.get('CURRENT_TIME', 0) <= cutoff+TIME_TOL
            else:
                keep = r.get('property_time', r.get('CURRENT_TIME', 0)) <= cutoff+TIME_TOL
            if not keep:
                continue
            r['_evidence_branch'] = part.name
            rr.append(r)
    return rr


def history_chain(name):
    parts = parts_for(name)
    hist = []
    initial = read(parts[0]/'native_history.json').get('initial_state')
    cutoffs = part_cutoffs(parts)
    exclusions = []
    for part in parts:
        ph = read(part/'native_history.json')
        original = ph.get('records', [])
        rr = [r for r in original if r['time_s'] <= cutoffs[part.name]+TIME_TOL]
        discarded = [r for r in original if r['time_s'] > cutoffs[part.name]+TIME_TOL]
        excluded_callbacks = [r for r in rows(part/'native_contact_callback_trace_node.jsonl')
            if r['CURRENT_TIME'] >= cutoffs[part.name]-TIME_TOL]
        if math.isfinite(cutoffs[part.name]):
            exclusions.append({'branch':part.name, 'verified_restart_time_s':cutoffs[part.name],
                'original_history_sha256':sha(part/'native_history.json'),
                'excluded_completed_records':[{'time_s':r['time_s'],'native_step_index':r['native_step_index']} for r in discarded],
                'excluded_callback_indices':[r['callback_index'] for r in excluded_callbacks],
                'reason':'Collected or callback tail after the last complete saved native boundary is preserved in original files, excluded from recovered trajectory, and replayed from that boundary exactly once.'})
        for i, r in enumerate(rr, 1):
            r['_evidence_branch'] = part.name
            r['_local_iteration'] = i
        if hist:
            before = ph.get('initial_state', {})
            if not before or abs(before['time_s']-hist[-1]['time_s']) > TIME_TOL or \
                state_difference(before['native_state'], hist[-1]['native_state'])['status'] != 'PASS':
                raise RuntimeError('Lifecycle continuation prefix does not inherit the last verified native state')
        hist.extend(rr)
    if exclusions:
        atomic(EVID/'branches'/name/'lifecycle_prefix_exclusions.json',
               {'timestamp':stamp(),'original_files_preserved':True,'parts':exclusions})
    return initial, hist


def event_audit(b, history, initial, dt, parts=None):
    parts = parts or [b]
    raw = branch_rows(parts, 'native_contact_callback_trace_node.jsonl')
    trace = [r for r in raw if r.get('moving_body') and r.get('contact_face_count', 0) > 0]
    sem = branch_rows(parts, 'theta_lifecycle_node.jsonl')
    applied = [r for r in trace if r.get('action') == 'APPLIED']
    groups = []
    for e in trace:
        group = next((g for g in groups if abs(g['time_s']-e['CURRENT_TIME']) <= 1e-13 and
            g['body'] == e['dynamic_zone_id'] and g['opposite'] == e['opposite_zone_id'] and
            norm(np.asarray(g['point'])-e['contact_point']) <= 20e-6), None)
        if group is None:
            group = {'time_s': e['CURRENT_TIME'], 'body': e['dynamic_zone_id'], 'opposite': e['opposite_zone_id'],
                     'point': e['contact_point'], 'callbacks': []}
            groups.append(group)
        group['callbacks'].append(e)
    duplicates = [g for g in groups if sum(e['action'] == 'APPLIED' for e in g['callbacks']) > 1]
    illegal = [e for e in trace if bool(e.get('overwrite_called')) != (e.get('action') == 'APPLIED')]
    checks = []
    for e in applied:
        V, W, J, vn, vna = independent_impulse(e)
        post = next((s for s in history if abs(s['time_s']-(e['CURRENT_TIME']+dt)) <= TIME_TOL), None)
        idx = history.index(post) if post is not None else -1
        next_record = history[idx+1] if idx >= 0 and idx+1 < len(history) else None
        next_dir = EVID/'branches'/next_record['_evidence_branch'] if next_record else b
        before_next = read(next_dir/f'before_step_{next_record["_local_iteration"]:02d}.json') if next_record else {}
        terminal = idx == len(history)-1
        if not before_next and terminal:
            before_next = read(b/'final_next_start.json')
            if not before_next and closure_review(b)['status'] == 'PASS' and \
                read(b/'branch_result.json').get('status') == 'RECOVERED_NATIVE_BRANCH_DATA_COMPLETE':
                saved = read(b/'final_native_checkpoint.json')
                if saved.get('native_state') and abs(saved.get('time_s',0)-post['time_s']) <= TIME_TOL:
                    before_next = {'time_s':saved['time_s'],'native_state':saved['native_state'],
                        'validation_kind':'SAVED_NATIVE_BOUNDARY_AFTER_OFFLINE_PROCESS_CLOSURE_REVIEW',
                        'timesteps_advanced':0,'actual_next_dynamics_NOT_claimed':True}
        post_ns = post['native_state'] if post is not None else None
        intended_v, intended_w = V-np.asarray(e['get_velocity']), W-np.asarray(e['get_omega'])
        dv = np.asarray(post_ns['velocity'])-e['get_velocity'] if post_ns else np.zeros(3)
        dw = np.asarray(post_ns['omega'])-e['get_omega'] if post_ns else np.zeros(3)
        change = {'velocity': difference(dv, intended_v), 'omega': difference(dw, intended_w)}
        # Multiple distinct clusters at one boundary can apply a sequence. Final
        # native state is compared to the LAST requested motion, not each interim.
        same_boundary = [a for a in applied if abs(a['CURRENT_TIME']-e['CURRENT_TIME']) <= 1e-13]
        final_e = same_boundary[-1]
        sequential_interim = e is not final_e
        response = all(d['relative_error'] is not None and d['relative_error'] <= .10 and
            d['direction_cosine'] is not None and d['direction_cosine'] >= .99 for d in change.values())
        if sequential_interim:
            following = same_boundary[same_boundary.index(e)+1]
            response = difference(following['get_velocity'], V)['max_abs'] <= 1e-8 and \
                difference(following['get_omega'], W)['max_abs'] <= 1e-6
        inherited = state_difference(before_next['native_state'], post_ns) if before_next and post_ns else {'status': 'FAIL'}
        entry = next((r for r in sem if r.get('stage') == 'PROPERTIES_ENTRY' and post is not None and
            abs(r['CURRENT_TIME']-post['time_s']) <= TIME_TOL and r['property_time'] > post['time_s']+TIME_TOL), None)
        entry_pass = bool(entry) and difference(entry['velocity'], post_ns['velocity'])['max_abs'] <= 1e-8 and \
            difference(entry['omega'], post_ns['omega'])['max_abs'] <= 1e-6
        if terminal:
            entry_pass = inherited['status'] == 'PASS'  # Explicit zero-step terminal native boundary; no extra solve.
        math_pass = difference(e['requested_velocity'], V)['max_abs'] <= 1e-10 and \
            difference(e['requested_omega'], W)['max_abs'] <= 1e-7 and abs(e['impulse_N_s']-J) <= 1e-16 and \
            norm(e['contact_normal']) > .999999 and norm(e['contact_normal']) < 1.000001 and J > 0 and abs(vna) <= 1e-9
        checks.append({'callback_index': e['callback_index'], 'evidence_branch':e['_evidence_branch'], 'callback_time_s': e['CURRENT_TIME'],
            'completed_time_s': post['time_s'] if post is not None else None,
            'independent_J_N_s': J, 'independent_vn_before_m_s': vn, 'independent_vn_after_m_s': vna,
            'independent_impulse_math_PASS': bool(math_pass), 'predicted_velocity_m_s': V.tolist(),
            'predicted_omega_rad_s': W.tolist(), 'actual_completed_native_state': post_ns,
            'expected_vs_actual_change': change, 'completed_response_PASS': bool(response),
            'interim_distinct_cluster_validated_against_next_callback': sequential_interim,
            'next_start': before_next, 'next_start_errors': inherited, 'properties_entry_PASS': bool(entry_pass),
            'inheritance_evidence_available':bool(before_next), 'actual_properties_entry_available':bool(entry),
            'terminal_zero_step_boundary_only': terminal, 'next_step_inherited_PASS': inherited['status'] == 'PASS' and bool(entry_pass),
            'immediate_reread_used_as_PASS': False})
    finite = all(np.isfinite(np.r_[e['get_velocity'], e['get_omega'], e['requested_velocity'], e['requested_omega'],
        e['contact_point'], e['contact_normal'], e['impulse_N_s']]).all() for e in trace)
    host = [r for r in branch_rows(parts, 'native_contact_callback_trace_host.jsonl') if r.get('moving_body') and r.get('contact_face_count', 0) > 0]
    host_same = len(host) == len(trace) and all(all(a[k] == z[k] for k in ['CURRENT_TIME', 'action', 'contact_point', 'impulse_N_s'])
        for a, z in zip(trace, host))
    persisted = bool(checks) and all(c['independent_impulse_math_PASS'] and c['completed_response_PASS'] and c['next_step_inherited_PASS'] for c in checks)
    audit = {'status': 'PASS' if not duplicates and not illegal and finite and host_same and persisted else 'FAIL',
        'callback_count': len(trace), 'physical_impulse_application_count': len(applied),
        'physical_event_group_count': sum(any(e['action'] == 'APPLIED' for e in g['callbacks']) for g in groups),
        'dedup_PASS': not duplicates and not illegal, 'finite_PASS': bool(finite), 'host_node_same_event': host_same,
        'host_is_copy_not_additional_impulse': True, 'persistence_PASS': bool(persisted),
        'groups': [{k: v for k, v in g.items() if k != 'callbacks'} | {
            'callback_count': len(g['callbacks']), 'applied_count': sum(e['action'] == 'APPLIED' for e in g['callbacks']),
            'skipped_duplicate_count': sum(e['action'] == 'SKIPPED_DUPLICATE' for e in g['callbacks'])} for g in groups],
        'response_checks': checks, 'duplicate_impulse_groups': [{k: v for k, v in g.items() if k != 'callbacks'} for g in duplicates]}
    enriched = []
    for e in raw:
        out = dict(e)
        link = next((c for c in checks if c['callback_index'] == e['callback_index'] and c['evidence_branch'] == e['_evidence_branch']), None)
        if link:
            out['completed_state_validation'] = link
        enriched.append(out)
    (b/'contact_events.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in enriched), encoding='utf-8')
    keys = ['callback_index', 'CURRENT_TIME', 'timestep_index', 'moving_body', 'contact_face_count', 'action',
            'overwrite_called', 'normal_velocity_before', 'impulse_N_s', 'normal_velocity_after']
    with (b/'contact_events.csv').open('w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=keys+['contact_point_json', 'normal_json', 'completed_state_json', 'next_start_json'])
        writer.writeheader()
        for e in enriched:
            link = e.get('completed_state_validation', {})
            writer.writerow({k: e.get(k) for k in keys} | {'contact_point_json': json.dumps(e['contact_point']),
                'normal_json': json.dumps(e['contact_normal']), 'completed_state_json': json.dumps(link.get('actual_completed_native_state')),
                'next_start_json': json.dumps(link.get('next_start'))})
    return audit, trace


def metrics(states, trace, dt):
    applied = [e for e in trace if e.get('action') == 'APPLIED']
    gap = np.array([s.get('signed_gap_m', s['physical_gap_m']) for s in states])
    ns = [s['native_state'] for s in states]
    tilt = [float(math.acos(np.clip(rot(s['q']).apply([1., 0., 0.])[0], -1, 1))) for s in ns]
    positions = np.asarray([s['com'] for s in ns])
    first = applied[0] if applied else None
    episodes = []
    for e in applied:
        if not episodes or e['CURRENT_TIME']-episodes[-1][-1]['CURRENT_TIME'] > 1.5*dt:
            episodes.append([])
        episodes[-1].append(e)
    separation_boundaries = []
    # A detection-free solved boundary after a contact episode is a measured
    # proximity separation. This does not assert a zero-gap physical collision.
    for ep in episodes:
        end = ep[-1]['CURRENT_TIME']+dt
        later = next((s for s in states if s['time_s'] > end+TIME_TOL), None)
        if later and not any(abs(e['CURRENT_TIME']-(later['time_s']-dt)) <= TIME_TOL for e in trace):
            separation_boundaries.append(later['time_s'])
    imax = int(np.argmax(np.maximum(-gap, 0)))
    return {'first_contact_callback_time_s': first['CURRENT_TIME'] if first else None,
        'first_contact_completed_time_s': first['CURRENT_TIME']+dt if first else None,
        'first_contact_point_m': first['contact_point'] if first else None,
        'first_contact_normal': first['contact_normal'] if first else None,
        'first_contact_actual_mesh_gap_m': first['ideal_tube_gap_from_actual_vertices_m'] if first else None,
        'first_contact_tilt_rad': float(math.acos(np.clip(rot(first['DT_Q']).apply([1., 0., 0.])[0], -1, 1))) if first else None,
        'first_contact_omega_rad_s': first['get_omega'] if first else None,
        'first_impulse_N_s': first['impulse_N_s'] if first else None,
        'physical_impulse_application_count': len(applied), 'contact_episode_count': len(episodes),
        'separation_count': len(separation_boundaries), 'recontact_count': max(0, len(episodes)-1),
        'separation_times_s': separation_boundaries,
        'episode_definition': 'Adjacent impulse boundaries separated by <=1.5dt constitute one native proximity-contact episode; callbacks within a boundary are deduplicated.',
        'contact_geometry_note': 'Native threshold remains100um: first callback may be at positive gap. Callback CURRENT_TIME is start boundary and mesh vertices belong to that boundary; completed contact response occurs one dt later.',
        'minimum_signed_gap_m': float(gap.min()), 'maximum_penetration_m': float(np.maximum(-gap, 0).max()),
        'maximum_penetration_time_s': states[imax]['time_s'], 'peak_impulse_N_s': max((e['impulse_N_s'] for e in applied), default=0.),
        'maximum_tilt_rad': max(tilt), 'maximum_tilt_deg': math.degrees(max(tilt)),
        'peak_omega_rad_s': max(norm(s['omega']) for s in ns),
        'COM_displacement_from_restart_m': (positions[-1]-positions[0]).tolist(),
        'maximum_lateral_displacement_from_restart_m': float(np.linalg.norm(positions[:, 1:]-positions[0, 1:], axis=1).max()),
        'final_COM_m': ns[-1]['com'], 'final_Q_wxyz': ns[-1]['q'],
        'final_v_m_s': ns[-1]['velocity'], 'final_omega_rad_s': ns[-1]['omega']}


def branch_review(name):
    b = EVID/'branches'/name
    config, result, hh = read(b/'configuration.json'), read(b/'branch_result.json'), read(b/'native_history.json')
    if not config or not hh:
        raise RuntimeError('Actual branch configuration/history required: '+name)
    parts = parts_for(name)
    initial, hist = history_chain(name)
    part_audits = []
    for part in parts:
        pr = read(part/'source_restart_validation.json') or read(part/'C70_restart_validation.json')
        if not pr:
            pr = next((read(p) for p in part.glob('*restart_validation.json')), {})
        prefix_checkpoint = None
        if part != b:
            latest = read(part/'latest_verified_checkpoint.json')
            if latest.get('metadata'):
                prefix_checkpoint = checkpoint_audit(absolute_path(latest['metadata']))
            else:
                prefix_checkpoint = checkpoint_audit(part/'final_native_checkpoint.json')
        closure = closure_review(part)
        part_audits.append({'branch':part.name, 'restart_continuity_PASS':pr.get('status') == 'PASS',
            'owned_engine_closed':closure['status'] == 'PASS','closure_review':closure,
            'prefix_checkpoint':prefix_checkpoint, 'magnetic':magnetic_audit(part)})
    if not initial:
        candidates = sorted(b.glob('state_*.json'))
        initial = read(candidates[0]) if candidates else None
    if not hist or not initial:
        raise RuntimeError('Native initial and completed state evidence missing')
    dt = float(config['dt_s'])
    start = float(initial['time_s'])
    end = float(config.get('end_time_s', result.get('end_time_s', .0019)))
    states = [initial, *hist]
    allfinite = all(np.isfinite(np.r_[s['native_state']['com'], s['native_state']['q'],
        s['native_state']['velocity'], s['native_state']['omega'], s.get('signed_gap_m', s['physical_gap_m'])]).all() for s in states)
    connectivity = [s['connectivity'] for s in states]
    numerical = {'finite': bool(allfinite), 'cell_count': all(c['total'] == 4699301 for c in connectivity),
        'orphan_zero': all(c['orphan'] == 0 for c in connectivity),
        'invalid_donors_zero': all(c['invalid_donors'] == 0 for c in connectivity),
        'unidentified_zero': all(c.get('unidentified', 0) == 0 for c in connectivity),
        'positive_volume': all(c['minimum_volume_m3'] > 0 and c['nonpositive_volume'] == 0 and c['nonfinite_volume'] == 0 for c in connectivity),
        'quaternion': all(abs(norm(s['native_state']['q'])-1) <= 1e-6 for s in states),
        'actual_time_sequence': all(abs(s['time_s']-(start+(i+1)*dt)) <= TIME_TOL for i, s in enumerate(hist))}
    full = abs(hist[-1]['time_s']-end) <= TIME_TOL and len(hist) == round((end-start)/dt)
    mag = {'status':'PASS' if all(p['magnetic']['status'] == 'PASS' for p in part_audits) else 'FAIL',
        'parts':[{ 'branch':p['branch'], **p['magnetic']} for p in part_audits]}
    restart = read(b/'source_restart_validation.json') or read(b/'C70_restart_validation.json')
    if not restart:
        restart = next((read(p) for p in b.glob('*restart_validation.json')), {})
    cp = checkpoint_audit(b/'final_native_checkpoint.json')
    if cp['status'] == 'PASS':
        cp['state_errors'] = state_difference(cp['native_state'], hist[-1]['native_state'])
        cp['status'] = 'PASS' if cp['state_errors']['status'] == 'PASS' and abs(cp['time_s']-hist[-1]['time_s']) <= TIME_TOL else 'FAIL'
    event, trace = event_audit(b, hist, initial, dt, parts)
    pre = precontact_comparison(states, trace, dt) if abs(start-.00175) <= TIME_TOL else {
        'status': 'INHERITED_REVIEWED_POSTCONTACT_CHECKPOINT', 'note': 'Production continues the selected D checkpoint; precontact comparison is retained in selected micro branch.'}
    mm = metrics(states, trace, dt)
    numerical['penetration'] = mm['maximum_penetration_m'] <= PENETRATION_TOL
    closed = all(p['owned_engine_closed'] for p in part_audits)
    memories = [read(part/'memory_summary.json') for part in parts]
    gates = {'restart_continuity': all(p['restart_continuity_PASS'] for p in part_audits), 'actual_magnetic_law': mag['status'] == 'PASS',
        'checkpoint_integrity': cp['status'] == 'PASS', 'owned_engine_closed': closed,
        'prefix_checkpoint_integrity':all(p['prefix_checkpoint'] is None or p['prefix_checkpoint']['status'] == 'PASS' for p in part_audits),
        'resource_history':all(m and not m.get('abort') for m in memories),
        'completed_branch_result':result.get('status') in ('NATIVE_BRANCH_DATA_COMPLETE', 'RECOVERED_NATIVE_BRANCH_DATA_COMPLETE', 'LOAD_PATH_BRANCH_DATA_COMPLETE'),
        'contact_response_and_dedup': event['status'] == 'PASS',
        'precontact_C_vs_D': pre['status'] in ('PASS', 'INHERITED_REVIEWED_POSTCONTACT_CHECKPOINT'), 'target_time_reached': full,
        **numerical}
    blockers = [k for k, v in gates.items() if not v]
    status = 'PASS' if not blockers else 'FAIL'
    failure = read(b/'failure.json')
    penetration_refinement_gates = [key for key in gates if key not in ('penetration','target_time_reached',
        'checkpoint_integrity','completed_branch_result','contact_response_and_dedup')]
    event_safety = event['dedup_PASS'] and event['finite_PASS'] and event['host_node_same_event'] and \
        all(c['independent_impulse_math_PASS'] and c['completed_response_PASS'] and
            (c['next_step_inherited_PASS'] if c['inheritance_evidence_available'] else c['terminal_zero_step_boundary_only'])
            for c in event['response_checks'])
    if failure.get('failure_class') == 'CONTACT_TIMESTEP_PENETRATION' and not numerical['penetration'] and \
        all(gates[k] for k in penetration_refinement_gates) and event_safety:
        status = 'PENETRATION_REQUIRES_DT'
    failure_class = failure.get('failure_class')
    if status == 'FAIL' and not failure_class:
        if not gates['precontact_C_vs_D']:
            failure_class = 'PRECONTACT_TRAJECTORY_FAIL'
        elif not event['dedup_PASS']:
            failure_class = 'CALLBACK_DUPLICATION'
        elif not event['finite_PASS'] or any(not numerical[k] for k in numerical if k != 'penetration'):
            failure_class = 'NUMERICAL_DYNAMIC_GATE'
        elif not event['response_checks'] or any(not c['independent_impulse_math_PASS'] or not c['completed_response_PASS'] for c in event['response_checks']):
            failure_class = 'CONTACT_RESPONSE'
        elif any(not c['next_step_inherited_PASS'] for c in event['response_checks']):
            failure_class = 'EVENT_STATE_INHERITANCE'
        elif not gates['restart_continuity']:
            failure_class = 'RESTART_CONTINUITY_FAIL'
        elif not gates['actual_magnetic_law']:
            failure_class = 'FROZEN_MAGNETIC_LAW_FAIL'
        elif not gates['resource_history']:
            failure_class = 'RESOURCE_HARD_STOP'
        elif not gates['owned_engine_closed']:
            failure_class = 'OWNERSHIP_CLOSURE_FAIL'
        elif not gates['checkpoint_integrity'] or not gates['prefix_checkpoint_integrity']:
            failure_class = 'CHECKPOINT_INTEGRITY_FAIL'
        elif not gates['target_time_reached'] or not gates['completed_branch_result']:
            failure_class = 'WORKER_LIFECYCLE_TERMINATION'
        else:
            failure_class = 'REVIEW_INCOMPLETE'
    report = {'timestamp': stamp(), 'campaign': CAMPAIGN, 'branch': name, 'status': status,
        'start_time_s': start, 'target_time_s': end, 'end_time_s': hist[-1]['time_s'], 'dt_s': dt,
        'completed_new_steps': len(hist), 'gates': gates, 'blockers': blockers, 'tolerances': TOLERANCES,
        'failure_class':failure_class if status != 'PASS' else None,
        'logical_branch': config.get('logical_branch',name), 'lifecycle_parts':part_audits,
        'failure':failure,'penetration_refinement_classification': {
            'classification_only_NOT_micro_PASS':status == 'PENETRATION_REQUIRES_DT',
            'reviewed_safety_gates':penetration_refinement_gates,'contact_math_and_dedup_safe':bool(event_safety),
            'missing_terminal_inheritance_or_final_checkpoint_not_claimed_PASS':status == 'PENETRATION_REQUIRES_DT'},
        'native_restart': restart, 'native_checkpoint': cp, 'magnetic_load_validation': mag,
        'precontact_C_vs_D': pre, 'contact_validation': event, 'metrics': mm,
        'numerical_peaks': {'orphan_peak': max(c['orphan'] for c in connectivity),
            'invalid_donor_peak': max(c['invalid_donors'] for c in connectivity),
            'minimum_volume_m3': min(c['minimum_volume_m3'] for c in connectivity),
            'maximum_q_norm_error': max(abs(norm(s['native_state']['q'])-1) for s in states)},
        'resource_summary': read(b/'memory_summary.json'), 'reviewed_files_sha256': {
            str(p.relative_to(ROOT)): sha(p) for part in parts for p in [part/'configuration.json', part/'native_history.json', part/'branch_result.json',
                part/'native_contact_callback_trace_node.jsonl', part/'native_contact_callback_trace_host.jsonl',
                part/'source_restart_validation.json', part/'worker_exit.json', part/'engine_exit.json', part/'memory_summary.json',
                b/'contact_events.jsonl', b/'contact_events.csv'] if p.is_file()}, 'no_solver_connection': True}
    atomic(b/'actual_magnetic_load_validation.json', mag)
    atomic(b/'precontact_C_vs_D.json', pre)
    atomic(b/'review.json', report)
    return report


def compare_dt(a, z):
    aa, zz = a['metrics'], z['metrics']
    t = TOLERANCES['dt_comparison']
    errors = {'contact_timing_s': abs(aa['first_contact_completed_time_s']-zz['first_contact_completed_time_s']),
        'contact_point_m': norm(np.asarray(aa['first_contact_point_m'])-zz['first_contact_point_m']),
        'minimum_signed_gap_m': abs(aa['minimum_signed_gap_m']-zz['minimum_signed_gap_m']),
        'peak_penetration_m': abs(aa['maximum_penetration_m']-zz['maximum_penetration_m']),
        'first_impulse_relative': abs(aa['first_impulse_N_s']-zz['first_impulse_N_s'])/max(abs(zz['first_impulse_N_s']), 1e-18),
        'peak_impulse_relative': abs(aa['peak_impulse_N_s']-zz['peak_impulse_N_s'])/max(abs(zz['peak_impulse_N_s']), 1e-18),
        'final_v_m_s': norm(np.asarray(aa['final_v_m_s'])-zz['final_v_m_s']),
        'final_omega_rad_s': norm(np.asarray(aa['final_omega_rad_s'])-zz['final_omega_rad_s']),
        'maximum_tilt_rad': abs(aa['maximum_tilt_rad']-zz['maximum_tilt_rad']),
        'peak_omega_relative': abs(aa['peak_omega_rad_s']-zz['peak_omega_rad_s'])/max(zz['peak_omega_rad_s'], 1.),
        'impulse_count_difference': abs(aa['physical_impulse_application_count']-zz['physical_impulse_application_count']),
        'episode_count_difference': abs(aa['contact_episode_count']-zz['contact_episode_count'])}
    count_allowance = max(t['event_count_absolute'], math.ceil(t['event_count_relative']*zz['physical_impulse_application_count']))
    checks = {'both_full_micro_PASS': a['status'] == z['status'] == 'PASS',
        'same_physical_end_time': abs(a['end_time_s']-z['end_time_s']) <= TIME_TOL,
        'timing': errors['contact_timing_s'] <= a['dt_s']+TIME_TOL,
        'position': errors['contact_point_m'] <= t['contact_position_m'],
        'gap': errors['minimum_signed_gap_m'] <= t['signed_gap_m'],
        'penetration': errors['peak_penetration_m'] <= t['penetration_m'],
        'first_impulse': errors['first_impulse_relative'] <= t['impulse_relative'],
        'peak_impulse': errors['peak_impulse_relative'] <= t['impulse_relative'],
        'final_v': errors['final_v_m_s'] <= max(t['final_v_absolute_m_s'], t['final_v_relative']*norm(zz['final_v_m_s'])),
        'final_omega': errors['final_omega_rad_s'] <= max(t['final_omega_absolute_rad_s'], t['final_omega_relative']*norm(zz['final_omega_rad_s'])),
        'tilt': errors['maximum_tilt_rad'] <= t['max_tilt_rad'],
        'peak_omega': errors['peak_omega_relative'] <= t['peak_omega_relative'],
        'event_counts': errors['impulse_count_difference'] <= count_allowance and errors['episode_count_difference'] <= 1}
    return {'status': 'PASS' if all(checks.values()) else 'MATERIAL_DIFFERENCE',
            'branches': [a['branch'], z['branch']], 'errors': errors, 'checks': checks, 'tolerances': t,
            'exact_common_final_time_s': a['end_time_s'], 'no_interpolation': True}


def select_dt():
    names = {n: resolve_branch(n) for n in ('micro25', 'micro12p5', 'micro6p25')}
    reports = {n: read(EVID/'branches'/names[n]/'review.json') for n in names}
    comparisons = []
    selected = None
    for coarse, fine in [('micro25', 'micro12p5'), ('micro12p5', 'micro6p25')]:
        a, z = reports[coarse], reports[fine]
        if not a or not z:
            continue
        if not a.get('metrics', {}).get('first_contact_completed_time_s') or not z.get('metrics', {}).get('first_contact_completed_time_s'):
            comparisons.append({'status': 'MATERIAL_DIFFERENCE', 'branches': [coarse, fine], 'reason': 'Contact response evidence absent'})
            continue
        comparison = compare_dt(a, z)
        comparisons.append(comparison)
        if comparison['status'] == 'PASS':
            selected = coarse
            break
    if selected:
        status, reason = 'PASS', 'Largest timestep meeting all predeclared gates against the next factor-two refinement.'
    elif reports['micro6p25']:
        status, reason = 'FAIL', '12.5us versus6.25us did not meet adequate convergence; no smaller dt is authorized automatically.'
    else:
        status, reason = 'REFINEMENT_REQUIRED', '25us versus12.5us materially differs or lacks a full valid branch;6.25us comparison is needed.'
    result = {'timestamp': stamp(), 'campaign': CAMPAIGN, 'status': status, 'selected_branch': selected,
        'selected_dt_s': reports[selected]['dt_s'] if selected else None,
        'selected_physical_branch':names[selected] if selected else None,
        'reason': reason, 'comparisons': comparisons, 'predeclared_tolerances': TOLERANCES,
        'penetration_tolerance_m': PENETRATION_TOL, 'mesh_convergence_claim': False,
        'failure_class':'DT_RESOLUTION' if status == 'FAIL' else None,
        'review_sha256': {n: sha(EVID/'branches'/names[n]/'review.json') for n, r in reports.items() if r}}
    atomic(EVID/'contact_dt_selection.json', result)
    return result


def final_report():
    selection = read(EVID/'contact_dt_selection.json')
    state = read(EVID/'state.json')
    prod_name = resolve_branch(state.get('production_branch', 'production'))
    prod = read(EVID/'branches'/prod_name/'review.json')
    micro_name = selection.get('selected_physical_branch') or (resolve_branch(selection['selected_branch']) if selection.get('selected_branch') else None)
    micro = read(EVID/'branches'/str(micro_name)/'review.json') if micro_name else {}
    gates = {'dt_selected': selection.get('status') == 'PASS', 'selected_micro_PASS': micro.get('status') == 'PASS',
        'production_PASS': prod.get('status') == 'PASS', 'final_time_2ms': abs(prod.get('end_time_s', 0)-.002) <= TIME_TOL}
    merged = []
    trace = []
    checkpoint_19 = checkpoint_20 = {}
    if micro and prod:
        mi, mh = history_chain(micro_name)
        start, ph = history_chain(prod_name)
        if not start:
            candidates = sorted((EVID/'branches'/prod_name).glob('state_*.json'))
            start = read(candidates[0]) if candidates else {}
        continuity = state_difference(start['native_state'], mh[-1]['native_state']) if start else {'status': 'FAIL'}
        gates['production_inherits_selected_D_checkpoint'] = continuity['status'] == 'PASS'
        gates['same_production_dt'] = abs(prod['dt_s']-selection['selected_dt_s']) <= TIME_TOL
        merged = [mi, *mh, *ph]
        trace = [e for branch in (micro_name, prod_name) for e in branch_rows(parts_for(branch), 'native_contact_callback_trace_node.jsonl')
                 if e.get('moving_body') and e.get('contact_face_count', 0) > 0]
        checkpoint_19 = micro['native_checkpoint']
        checkpoint_20 = prod['native_checkpoint']
    good = bool(merged) and all(gates.values())
    all_memory = [r for b in (EVID/'branches').glob('*') for r in rows(b/'memory_samples.jsonl')]
    resources = {'samples': len(all_memory), 'peak_project_working_set_gib': max((r.get('project_working_set_gib', 0) for r in all_memory), default=0),
        'peak_project_private_bytes_gib': max((r.get('project_private_bytes_gib', 0) for r in all_memory), default=0),
        'peak_system_commit_percent': max((100*r.get('commit_fraction', 0) for r in all_memory), default=0),
        'minimum_physical_available_gib': min((r.get('available_gib', float('inf')) for r in all_memory), default=None),
        'minimum_disk_free_gib': min((r.get('disk_free_gib', float('inf')) for r in all_memory), default=None),
        'boot_CommitPeak_not_campaign_peak': True}
    result = {'timestamp': stamp(), 'campaign': CAMPAIGN, 'status': 'BENCHMARK_D_NATIVE_CONTACT_2MS_PASS' if good else 'INCOMPLETE',
        'route': 'NATIVE_OVERWRITE', 'numerical_gates': gates, 'selected_dt_s': selection.get('selected_dt_s'),
        'selected_micro_branch': micro_name, 'production_branch': prod_name,
        'restart_source': 'Original Benchmark C native step70 /1.750ms followed by selected D1.900ms checkpoint',
        'precontact_C_vs_D': micro.get('precontact_C_vs_D'), 'checkpoint_1p9ms': checkpoint_19, 'checkpoint_2ms': checkpoint_20,
        'baseline_25us_strict_C_vs_D':read(EVID/'branches'/resolve_branch('micro25')/'review.json').get('precontact_C_vs_D'),
        'resource_summary': resources, 'metrics': metrics(merged, trace, selection['selected_dt_s']) if merged else {},
        'numerical_peaks': {k: (min(micro['numerical_peaks'][k], prod['numerical_peaks'][k]) if k == 'minimum_volume_m3' else
            max(micro['numerical_peaks'][k], prod['numerical_peaks'][k])) for k in micro.get('numerical_peaks', {})} if prod else {},
        'visual_review_status': read(EVID/'visual_review.json').get('status', 'NOT_REVIEWED'),
        'fine_reference': 'FINE_REFERENCE_NOT_AVAILABLE_AT_2MS', 'mesh_convergence_claim': False,
        'scientific_scope': 'Rigid frictionless e_n=0 normal native reaction triggered by proximity threshold0.100mm. '
            'A positive ideal gap at an event is not a zero-gap wall impact. Compare measured trajectories without assigning '
            'all differences to contact alone; no friction, material stress or experimental restitution claim.',
        'comparison_C_source': 'evidence/benchmark_C_coarse_2ms/dynamic_history.csv',
        'C_free_nearwall_reference_final_time_s': max(r['time_s'] for r in reference_c_rows()),
        'C_vs_D_postcontact_note': 'C ended at its near-wall clearance gate; no free C2ms reference. No interpolated2ms C comparison.'}
    atomic(EVID/'final_report.json', result)
    return result


def main():
    parser = argparse.ArgumentParser()
    args = parser.add_mutually_exclusive_group(required=True)
    args.add_argument('--branch')
    args.add_argument('--select-dt', action='store_true')
    args.add_argument('--final', action='store_true')
    selected = parser.parse_args()
    try:
        out = branch_review(selected.branch) if selected.branch else select_dt() if selected.select_dt else final_report()
        print(json.dumps({'status': out['status'], 'branch': out.get('branch'), 'selected_branch': out.get('selected_branch'),
                          'selected_dt_s': out.get('selected_dt_s'), 'blockers': out.get('blockers')}))
    except Exception as exc:
        failure = {'timestamp': stamp(), 'status': 'FAIL', 'error': repr(exc), 'no_solver_connection': True}
        dest = EVID/'branches'/selected.branch/'review.json' if selected.branch else EVID/('contact_dt_selection.json' if selected.select_dt else 'final_report.json')
        atomic(dest, failure)
        print(json.dumps(failure))
        raise


if __name__ == '__main__':
    main()
