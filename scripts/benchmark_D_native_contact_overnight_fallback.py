"""Conditional local load-path campaign. This module never reopens native semantics.

The supervisor admits this finite job only after archived native-route evidence.
Numerical reviewers run as separate subprocesses so the waiting controller does
not retain NumPy, SciPy or VTK memory before the next single-solver admission.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

from benchmark_D_native_contact_overnight_common import (
    ROOT, EVID, OUT, atomic, read, sha, stamp, native, rows,
    event, state, verify_frozen, prepare_branch, review_launch, identity,
)

SOURCE = 'fluent_udf/l2300_load_contact_overnight.c'
SELF = 'scripts/benchmark_D_native_contact_overnight_fallback.py'
WORKER = 'scripts/benchmark_D_native_contact_overnight_worker.py'
EXIT_REVIEW = 'scripts/benchmark_D_native_contact_overnight_owned_exit_review.py'
BASE_GEOMETRY = OUT / 'load_contact_body_points.h'
RESOURCE_FAILURES = {'RESOURCE_HARD_STOP', 'RESOURCE_BLOCKER', 'STOPPED_RESOURCE_BLOCKER'}
TIME_TOL = 1e-12


class FallbackResourceBlocker(RuntimeError):
    """Resource evidence is never contact-instability or compliant admission."""


def process_record(pid=None):
    import psutil
    p = psutil.Process(pid or os.getpid())
    return dict(pid=p.pid, created=p.create_time(), name=p.name())


def retained_applied_events(name, cutoff=None):
    """Count only applied intervals belonging to the saved recovered path.

    Earlier attempts retain their failed tail on disk. The next part's native
    checkpoint defines the retained boundary; the current attempt is also
    trimmed at its latest verified checkpoint when preparing a recovery.
    """
    config = read(EVID/'branches'/name/'configuration.json')
    names = config.get('history_prefix_branches', [])+[name]
    if len(names) != len(set(names)):
        raise RuntimeError('DUPLICATED_FALLBACK_HISTORY_PREFIX')
    retained = []
    for i, part in enumerate(names):
        c = read(EVID/'branches'/part/'configuration.json')
        boundary = cutoff if i == len(names)-1 else read(EVID/'branches'/names[i+1]/'configuration.json')['start_time_s']
        boundary = float('inf') if boundary is None else float(boundary)
        for e in rows(EVID/'branches'/part/'native_contact_callback_trace_node.jsonl'):
            if e.get('physical_impulse_applied') and e['CURRENT_TIME']+c['dt_s'] <= boundary+TIME_TOL:
                retained.append({**e, '_evidence_branch':part})
    return retained


def resource_failure(name, failure):
    b = EVID/'branches'/name
    evidence = [p.relative_to(ROOT).as_posix() for p in
                (b/'failure.json', b/'memory_summary.json', b/'memory_samples.jsonl', b/'memory_failure.json',
                 b/'latest_verified_checkpoint.json') if p.is_file()]
    report = dict(status='STOPPED_RESOURCE_BLOCKER', timestamp=stamp(), branch=name,
                  failure_class=failure.get('failure_class'), failure=failure,
                  evidence_files=evidence, evidence_sha256={p:sha(ROOT/p) for p in evidence},
                  contact_instability_claim=False, compliant_eligible=False,
                  guards_unchanged=True, new_solver_launch=False)
    atomic(EVID/'fallback_resource_blocker.json', report)
    control('STOPPED_RESOURCE_BLOCKER', branch=name, resource_failure=report)
    raise FallbackResourceBlocker('RESOURCE_HARD_STOP:'+name)


def register_actual_worker(active):
    """Persist the worker below a possible pythonw launcher, using exact ID."""
    if active.get('kind') != 'worker':
        return active
    launcher = identity(active.get('launcher_identity', active))
    if launcher:
        children = []
        for p in launcher.children(recursive=True):
            try:
                cmd = p.cmdline()
                if any(Path(a).name == Path(WORKER).name for a in cmd) and '--branch' in cmd and active['branch'] in cmd:
                    children.append(process_record(p.pid))
            except Exception:
                continue
        if children:
            active['launch_descendant_identities'] = children
            control('FALLBACK_WORKER_RUNNING', active_job=active)
    recorded = read(EVID/'branches'/active['branch']/'worker_identity.json')
    if recorded:
        # The worker registers before RESOURCE_WAIT and before engine launch.
        actual = {**recorded, 'name':active.get('registered_worker_identity', {}).get('name', 'pythonw.exe')}
        p = identity(actual)
        if p:
            actual['name'] = p.name()
        active['registered_worker_identity'] = actual
        control('FALLBACK_WORKER_RUNNING', active_job=active)
    return active


def wait_registered_worker(name):
    """Adopt an extant exact child; never let a second helper launch beside it."""
    saved = read(EVID/'fallback_state.json')
    active = saved.get('active_job') or {}
    registered = read(EVID/'branches'/name/'worker_identity.json')
    if registered:
        registered = {**registered, 'name':active.get('registered_worker_identity', {}).get('name', 'pythonw.exe')}
    candidates = [registered] if registered else []
    if active.get('branch') == name:
        candidates += [active.get('registered_worker_identity', {}), active.get('launcher_identity', active)]
        candidates += active.get('launch_descendant_identities', [])
    live = [r for r in candidates if identity(r)]
    if not live:
        return False
    adopted = dict(active, kind='worker', branch=name, adopted_by_helper_identity=process_record(),
                   registered_worker_identity=registered or active.get('registered_worker_identity'))
    control('FALLBACK_EXISTING_CHILD_ADOPTED', active_job=adopted, adopted_identities=live)
    while any(identity(r) for r in candidates):
        if time.time() >= read(EVID/'window.json')['hard_deadline_epoch']:
            atomic(OUT/'stop_request.json', dict(reason='TIME_BUDGET_EXHAUSTED', timestamp=stamp()))
        time.sleep(5)
        if not registered:
            registered = read(EVID/'branches'/name/'worker_identity.json')
            if registered:
                registered = {**registered, 'name':'pythonw.exe'}
                candidates.append(registered)
                adopted['registered_worker_identity'] = registered
                control('FALLBACK_EXISTING_CHILD_ADOPTED', active_job=adopted)
    if native():
        raise RuntimeError('ADOPTED_FALLBACK_CHILD_EXITED_WITH_NATIVE_ENGINE_PRESENT')
    control('FALLBACK_EXISTING_CHILD_FINISHED', active_job=None, last_job=name)
    deadline()
    return True


def control(status, **items):
    r = read(EVID / 'fallback_state.json')
    r.update(status=status, timestamp=stamp(), helper_identity=process_record())
    r.update(items)
    atomic(EVID / 'fallback_state.json', r)


def deadline():
    verify_frozen()
    if time.time() >= read(EVID / 'window.json')['hard_deadline_epoch']:
        atomic(OUT / 'stop_request.json', dict(timestamp=stamp(), reason='TIME_BUDGET_EXHAUSTED'))
        raise RuntimeError('TIME_BUDGET_EXHAUSTED')
    if read(OUT / 'stop_request.json') or read(EVID / 'stop_request.json'):
        raise RuntimeError('STOP_REQUESTED_AT_NATIVE_BOUNDARY')


def geometry_review():
    """Actual canonical geometry versus independently exported completed poses."""
    import numpy as np
    import pyvista as pv
    from scipy.spatial.transform import Rotation
    canonical = ROOT / 'evidence/benchmark_D_contact_baseline/fielddata/robot_0000.vtp'
    base = pv.read(canonical)
    body = np.asarray(base.points, dtype=float) - [.0012060186937156343, 0., 0.]
    examples = []
    previous = ROOT / 'evidence/benchmark_D_contact_overwrite_semantics'
    for branch in ('A1', 'controlled'):
        for p in sorted((previous / branch).glob('state_*.json')):
            st = read(p)
            exported = previous / branch / 'fielddata' / (p.stem + '_robot.vtp')
            if not exported.is_file() or not st.get('native_state'):
                continue
            ns = st['native_state']
            R = Rotation.from_quat(np.asarray(ns['q'])[[1, 2, 3, 0]])
            xyz = R.apply(body) + ns['com']
            current = pv.read(exported)
            gap = .0009 - np.linalg.norm(xyz[:, 1:], axis=1).max()
            actual = .0009 - np.linalg.norm(current.points[:, 1:], axis=1).max()
            examples.append(dict(path=p.relative_to(ROOT).as_posix(), time_s=st['time_s'],
                                 analytic_gap_m=float(gap), exported_gap_m=float(actual),
                                 gap_error_m=float(abs(gap-actual))))
    if not examples or max(x['gap_error_m'] for x in examples) > 1e-9:
        raise RuntimeError('LOAD_CONTACT_GEOMETRY_AUDIT_FAIL')
    BASE_GEOMETRY.parent.mkdir(parents=True, exist_ok=True)
    text = '#define LOAD_BODY_POINT_COUNT ' + str(len(body)) + '\n'
    text += 'static const double load_body_points[LOAD_BODY_POINT_COUNT][3]={\n'
    text += ''.join('{'+','.join(format(float(x), '.17g') for x in p)+'},\n' for p in body)
    text += '};\n'
    if BASE_GEOMETRY.exists() and BASE_GEOMETRY.read_text() != text:
        raise RuntimeError('Reviewed canonical geometry changed')
    BASE_GEOMETRY.write_text(text)
    review = dict(status='PASS', timestamp=stamp(), source=canonical.relative_to(ROOT).as_posix(),
                  canonical_sha256=sha(canonical), body_points=len(body), COM_reference_m=[.0012060186937156343, 0., 0.],
                  body_header=BASE_GEOMETRY.relative_to(ROOT).as_posix(), body_header_sha256=sha(BASE_GEOMETRY),
                  ideal_tube_radius_m=.0009, examples=examples,
                  maximum_gap_error_m=max(x['gap_error_m'] for x in examples),
                  numerical_geometry_tolerance_m=1e-9, approximation='Exact for actual canonical linear triangulated surface',
                  finite_rotation_prediction=True, physical_activation='predicted signed gap<=0 AND inward-normal velocity<0',
                  no_positive_clearance_barrier=True, solver_launched=False)
    atomic(EVID / 'fallback_geometry_review.json', review)
    return review


def prepare_capability():
    deadline()
    geometry = geometry_review()
    text = (ROOT / SOURCE).read_text()
    forbidden = ('SDOF_Overwrite_Motion(', 'SDOF_Get_Motion(', 'DT_VEL_CG(dt)[i]=', 'DT_OMEGA_CG(dt)[i]=')
    if any(token in text for token in forbidden):
        raise RuntimeError('LOAD_PATH_SOURCE_HAS_FORBIDDEN_STATE_MUTATION')
    review = dict(status='PASS', timestamp=stamp(), activated=False,
                  route='SDOF_LOAD_PATH', job=SELF, job_sha256=sha(ROOT / SELF),
                  source=SOURCE, source_sha256=sha(ROOT / SOURCE),
                  worker=WORKER, worker_sha256=sha(ROOT / WORKER),
                  geometry_review_sha256=sha(EVID / 'fallback_geometry_review.json'),
                  geometry_header_sha256=geometry['body_header_sha256'],
                  native_force_path_already_validated=True, conditional_Fluent_compile_and_controlled_validation_required=True,
                  no_motion_overwrite=True, original_magnetic_delegate_once=True,
                  no_manual_fluid_force=True, maximum_time_s=.002,
                  activation_requires_archived_native_failure=True,
                  compliant_only_after_measured_impulse_load_instability=True)
    atomic(EVID / 'fallback_launch_review.json', review)
    return review


def check_capability():
    v = read(EVID / 'fallback_launch_review.json')
    geometry = read(EVID / 'fallback_geometry_review.json')
    for actual, want in ((sha(ROOT / SELF), v.get('job_sha256')), (sha(ROOT / SOURCE), v.get('source_sha256')),
                         (sha(ROOT / WORKER), v.get('worker_sha256')), (sha(BASE_GEOMETRY), v.get('geometry_header_sha256')),
                         (sha(EVID / 'fallback_geometry_review.json'), v.get('geometry_review_sha256'))):
        if actual != want:
            raise RuntimeError('CONDITIONAL_FALLBACK_REVIEW_HASH_MISMATCH')
    if v.get('status') != 'PASS' or geometry.get('status') != 'PASS':
        raise RuntimeError('CONDITIONAL_FALLBACK_CAPABILITY_NOT_REVIEWED')


def stage_branch(name, dt, mode, *, checkpoint=None, route='SDOF_LOAD_PATH', model=0,
                 stiffness=0., damping=0., prefixes=None, initial_events=0):
    deadline()
    b, o = EVID/'branches'/name, OUT/'branches'/name
    if (b/'worker_identity.json').exists() or (b/'branch_result.json').exists():
        raise RuntimeError('UNCHANGED_FALLBACK_BRANCH_RELAUNCH_REFUSED:'+name)
    source_checkpoint = checkpoint or read(EVID/'configuration.json')['source_checkpoint']
    meta = read(ROOT/source_checkpoint)
    specification = dict(branch=name, route=route, dt_s=dt, mode=mode,
                         start_time_s=float(meta['time_s']), end_time_s=.002,
                         stage='FALLBACK_LOAD_VALIDATION' if mode in (0, 2) else 'FALLBACK_MICRORUN',
                         source_checkpoint=source_checkpoint, contact_source_path=SOURCE,
                         penetration_tolerance_m=5e-6, iterations_per_step=2,
                         window_id=read(EVID/'window.json')['window_id'],
                         hard_deadline_epoch=read(EVID/'window.json')['hard_deadline_epoch'],
                         load_disabled_reference=mode == 0, load_contact_model=model,
                         stiffness_N_m=stiffness, damping_N_s_m=damping,
                         history_prefix_branches=prefixes or [], initial_load_events=initial_events,
                         bounded_one_event_validation=mode == 2,
                         baseline_completed_2ms_PASS_not_claimed_from_controlled=mode == 2)
    atomic(EVID/'branch_specs'/f'{name}.json', specification)
    b, o, config = prepare_branch(name)
    header = o/'load_contact_geometry.h'
    body = BASE_GEOMETRY.read_text()
    parameters = '#define LOAD_TUBE_RADIUS_M .0009\n#define LOAD_INITIAL_EVENT_TOTAL '+str(initial_events)+'\n'
    parameters += '#define LOAD_CONTACT_MODEL '+str(model)+'\n'
    parameters += '#define LOAD_STIFFNESS_N_M '+format(stiffness, '.17g')+'\n'
    parameters += '#define LOAD_DAMPING_N_S_M '+format(damping, '.17g')+'\n'
    header.write_text(parameters + body)
    config.update(geometry_header_sha256=sha(header), geometry_body_header_sha256=sha(BASE_GEOMETRY))
    atomic(b/'configuration.json', config)
    review_launch(name)
    launch = read(b/'launch_review.json')
    launch['files_sha256'][header.relative_to(ROOT).as_posix()] = sha(header)
    launch['files_sha256'][(ROOT/SELF).relative_to(ROOT).as_posix()] = sha(ROOT/SELF)
    atomic(b/'launch_review.json', launch)
    event('FALLBACK_BRANCH_REVIEWED', branch=name, route=route, mode=mode, dt_s=dt, start_time_s=config['start_time_s'], end_time_s=.002)
    return b


def job(args, label, worker=False, closure=False):
    import psutil
    deadline()
    check_capability()
    if worker and native():
        raise RuntimeError('UNKNOWN_NATIVE_ENGINE_REFUSES_FALLBACK_LAUNCH')
    logs = EVID/'fallback_jobs';logs.mkdir(exist_ok=True)
    path = ROOT/WORKER if worker else ROOT/EXIT_REVIEW if closure else ROOT/SELF
    with (logs/(label+'_stdout.log')).open('a') as out, (logs/(label+'_stderr.log')).open('a') as err:
        p = subprocess.Popen([str(ROOT/'.venv/Scripts/pythonw.exe'), '-u', str(path), *args], cwd=ROOT,
                             stdout=out, stderr=err, creationflags=subprocess.CREATE_NO_WINDOW)
        owner = process_record(p.pid)
        active = dict(**owner, kind='worker' if worker else 'offline_review',
                      branch=args[args.index('--branch')+1] if worker else None,
                      job=path.relative_to(ROOT).as_posix(), label=label, args=args,
                      helper_identity=process_record(), launcher_identity=owner)
        control('FALLBACK_WORKER_RUNNING' if worker else 'FALLBACK_OFFLINE_REVIEW', active_job=active)
        while True:
            if worker:
                active = register_actual_worker(active)
            child_live = worker and (identity(active.get('registered_worker_identity', {})) or
                         any(identity(r) for r in active.get('launch_descendant_identities', [])))
            if p.poll() is not None and not child_live:
                break
            if time.time() >= read(EVID/'window.json')['hard_deadline_epoch']:
                atomic(OUT/'stop_request.json', dict(reason='TIME_BUDGET_EXHAUSTED', timestamp=stamp()))
            time.sleep(5)
        if worker:
            active = register_actual_worker(active)
    if worker and native():
        branch=args[args.index('--branch')+1]
        if 'Owned engine not idle after SDK exit' in read(EVID/'branches'/branch/'closure_failure.json').get('error',''):
            job(['--branch',branch],branch+'_owned_exit_review',closure=True)
    if native():
        raise RuntimeError('OWNED_FALLBACK_WORKER_DID_NOT_CLOSE_NATIVE_ENGINE')
    control('FALLBACK_JOB_FINISHED', active_job=None, last_job=label, returncode=p.returncode,
            last_child_identity=active.get('registered_worker_identity') or owner)
    deadline()
    return p.returncode


def run_branch(name, dt, mode, _logical_name=None, **kwargs):
    logical = _logical_name or name
    resolution = read(EVID/'fallback_branch_resolution.json')
    if _logical_name is None:
        name = resolution.get(logical, name)
    b = EVID/'branches'/name
    config = read(b/'configuration.json')
    if config:
        if abs(config['dt_s']-dt)>TIME_TOL or config['mode'] != mode:
            raise RuntimeError('FALLBACK_RESOLUTION_CONFIG_MISMATCH:'+name)
        kwargs.update(checkpoint=config['source_checkpoint'], prefixes=config.get('history_prefix_branches', []),
                      initial_events=config.get('initial_load_events', 0), route=config['route'],
                      model=config.get('load_contact_model', 0), stiffness=config.get('stiffness_N_m', 0),
                      damping=config.get('damping_N_s_m', 0))
    if not (b/'worker_exit.json').exists():
        adopted = wait_registered_worker(name)
        if not adopted and not (b/'worker_identity.json').exists():
            stage_branch(name, dt, mode, **kwargs)
            job(['--branch', name], name, worker=True)
    if native():
        raise RuntimeError('NATIVE_ENGINE_PRESENT_WHILE_REVIEWING_FALLBACK_BRANCH:'+name)
    failure = read(b/'failure.json')
    if not failure and not read(b/'branch_result.json'):
        failure = dict(timestamp=stamp(), failure_class='WORKER_LIFECYCLE_TERMINATION',
                       reason='Registered worker ended without complete result or failure record; no numerical-fatal evidence exists.')
        atomic(b/'fallback_lifecycle_failure.json', failure)
    if failure.get('failure_class') in RESOURCE_FAILURES or failure.get('profile_abort'):
        resource_failure(name, failure)
    if failure.get('failure_class') == 'WORKER_LIFECYCLE_TERMINATION':
        detail=failure.get('error','')+' '+failure.get('traceback','')
        external=any(marker in detail for marker in ('RpcError','grpc.','UNAVAILABLE','ConnectionResetError',
            'BrokenPipeError','ProcessLookupError','FluentConnection','connection closed','connection terminated'))
        if failure.get('error') and not external:
            raise RuntimeError('FALLBACK_WORKER_CODE_DEFECT_REQUIRES_REVIEW:'+name)
        worker_record=read(b/'worker_identity.json');worker_record.setdefault('name','pythonw.exe')
        owned=read(b/'owned_engine.json').get('processes',[])
        if identity(worker_record) or any(identity(r) for r in owned):
            raise RuntimeError('FALLBACK_LIFECYCLE_OWNERS_NOT_CLOSED:'+name)
        # Later native states must be retained. A verified checkpoint, rather
        # than an unverified failed solve state, is the only recovery source.
        latest = read(b/'latest_verified_checkpoint.json')
        count = read(EVID/'fallback_state.json').get('lifecycle_restarts', 0)
        native_restarts=len(read(EVID/'lifecycle_repair_history.json',{'repairs':[]})['repairs'])
        if count+native_restarts >= 2 or latest.get('status') != 'PASS' or not latest.get('metadata'):
            raise RuntimeError('FALLBACK_LIFECYCLE_REPAIRS_EXHAUSTED_OR_NO_VERIFIED_CHECKPOINT')
        metadata=read(ROOT/latest['metadata'])
        if metadata.get('status')!='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS':
            raise RuntimeError('FALLBACK_RECOVERY_CHECKPOINT_NOT_NUMERICALLY_VERIFIED')
        for kind in ('case','data'):
            path=Path(metadata[kind]);path=path if path.is_absolute() else ROOT/path
            if not path.is_file() or sha(path)!=metadata[kind+'_sha256']:
                raise RuntimeError('FALLBACK_RECOVERY_NATIVE_BYTES_CHANGED')
        atomic(b/'recovery_process_review.json',dict(status='PASS',timestamp=stamp(),worker_identity=worker_record,
            owned_engine_identities=owned,worker_absent=True,all_owned_engines_absent=True,
            no_native_engine_present=True,no_process_terminated=True))
        if not read(b/'memory_summary.json'):
            samples=rows(b/'memory_samples.jsonl')
            if not samples or any(m['available_gib']<3 or m['commit_fraction']>=.95 or
                m['project_working_set_gib']>=15 or m['disk_free_gib']<15 for m in samples):
                resource_failure(name,dict(failure_class='RESOURCE_HARD_STOP',reason='Preserved resource samples do not support recovery'))
            atomic(b/'memory_summary.json',dict(timestamp=stamp(),samples=len(samples),abort=None,
                peak_project_working_set_gib=max(m['project_working_set_gib'] for m in samples),
                peak_project_private_bytes_gib=max(m['project_private_bytes_gib'] for m in samples),
                peak_system_commit_percent=100*max(m['commit_fraction'] for m in samples),
                minimum_available_gib=min(m['available_gib'] for m in samples),reconstructed_from_preserved_raw_samples=True))
        from_name = name
        name = name+'_recovery'+str(count+1)
        prefixes = read(b/'configuration.json').get('history_prefix_branches', [])+[from_name]
        events_applied = len(retained_applied_events(from_name, float(read(ROOT/latest['metadata'])['time_s'])))
        control('FALLBACK_NATIVE_CHECKPOINT_RECOVERY', lifecycle_restarts=count+1,
                source_checkpoint=latest['metadata'], original_failure=b.relative_to(ROOT).as_posix())
        resolution[logical] = name
        atomic(EVID/'fallback_branch_resolution.json', resolution)
        return run_branch(name, dt, mode, _logical_name=logical, **{**kwargs, 'checkpoint':latest['metadata'], 'prefixes':prefixes,
                                         'initial_events':events_applied if mode == 2 else 0})
    return name


def state_numeric_gates(st, allow_penetration=False):
    import numpy as np
    ns, c = st.get('native_state', {}), st.get('connectivity', {})
    try:
        finite = bool(np.isfinite(np.r_[ns['com'], ns['q'], ns['velocity'], ns['omega'],
                                      st.get('signed_gap_m', st['physical_gap_m']), st['Fmag_N'], st['Tmag_Nm']]).all())
        qvalid = abs(float(np.linalg.norm(ns['q']))-1)<=1e-6
        positive = c['minimum_volume_m3']>0 and c['nonpositive_volume']==0 and c['nonfinite_volume']==0
        gap = st.get('signed_gap_m', st['physical_gap_m']) >= -5e-6
    except (KeyError, TypeError, ValueError):
        finite=qvalid=positive=gap=False
    reasons = set(st.get('hard_failures', []))
    return dict(finite=finite, quaternion=qvalid, cell_count=c.get('total')==4699301,
                orphan_zero=c.get('orphan')==0, invalid_donors_zero=c.get('invalid_donors')==0,
                unidentified_zero=c.get('unidentified')==0, positive_volume=positive,
                penetration=gap or allow_penetration,
                no_other_hard_failure=not (reasons-({'PHYSICAL_PENETRATION'} if allow_penetration else set())))


def bounded_magnetic_audit(parts, start, end):
    import numpy as np
    import benchmark_D_native_contact_overnight_review as core
    errors=[0., 0.]; count=0; finite=True
    cutoffs=core.part_cutoffs(parts)
    for part in parts:
        p=part/'magnetic_history.csv'
        if not p.is_file():
            return dict(status='FAIL', reason='Actual magnetic delegate CSV missing', branch=part.name)
        with p.open(newline='',encoding='utf-8-sig') as fp:
            for r in csv.DictReader(fp):
                t=float(r['time_s'])
                if t<start-TIME_TOL or t>min(end,cutoffs[part.name])+TIME_TOL:
                    continue
                xyz=[float(r[f'cg_{a}_m']) for a in 'xyz'];q=[float(r[f'q{i}']) for i in (1,2,3,0)]
                F,T,_=core.compute_magnetic_load(xyz,q,t)
                actualF=[float(r[f'Fmag_{a}_N']) for a in 'xyz']; actualT=[float(r[f'Tmag_{a}_Nm']) for a in 'xyz']
                finite=finite and bool(np.isfinite(np.r_[xyz,q,F,T,actualF,actualT]).all())
                errors[0]=max(errors[0],float(np.max(abs(F-actualF))))
                errors[1]=max(errors[1],float(np.max(abs(T-actualT))))
                count+=1
    return dict(status='PASS' if count and finite and errors[0]<=1e-13 and errors[1]<=1e-14 else 'FAIL',
                actual_property_calls=count, finite_PASS=finite, max_F_error_N=errors[0],max_T_error_Nm=errors[1],
                absolute_native_interval_s=[start,end], actual_absolute_time_and_pose_used=True)


def actual_next_start(core, history, post, final_dir, terminal_allowed=False):
    idx=history.index(post)
    next_record=history[idx+1] if idx+1<len(history) else None
    terminal=next_record is None
    before={}
    if next_record:
        part=EVID/'branches'/next_record['_evidence_branch']
        before=read(part/f'before_step_{next_record["_local_iteration"]:02d}.json')
    elif terminal_allowed:
        before=read(final_dir/'final_next_start.json')
    inherited=core.state_difference(before['native_state'],post['native_state']) if before else {'status':'FAIL'}
    if before and abs(before.get('time_s',float('inf'))-post['time_s'])>TIME_TOL:
        inherited={'status':'FAIL','reason':'Next native boundary time mismatch'}
    return inherited, terminal, next_record


def controlled_review(control_name, reference_name):
    import numpy as np
    import benchmark_D_native_contact_overnight_review as core
    control_dir = EVID/'branches'/control_name
    initial, history = core.history_chain(control_name)
    reference_initial, reference = core.history_chain(reference_name)
    parts=core.parts_for(control_name);reference_parts=core.parts_for(reference_name)
    dt = read(control_dir/'configuration.json')['dt_s']
    applied = [e for e in core.branch_rows(parts,'native_contact_callback_trace_node.jsonl') if e.get('physical_impulse_applied')]
    sem=core.branch_rows(parts,'theta_lifecycle_node.jsonl')
    if not applied:
        result = dict(status='NO_LOAD_CONTACT_WITHIN_AUTHORIZED_INTERVAL', timestamp=stamp(),
                      controlled_branch=control_name, reference_branch=reference_name, maximum_time_s=.002,
                      controlled_scope='One predicted zero-gap approach event only; no production PASS', solver_launched=False)
        atomic(EVID/'fallback_controlled_validation.json', result)
        return result
    checks = []
    for e in applied:
        V, W, J, vn, _ = core.independent_impulse(e)
        post = next((s for s in history if abs(s['time_s']-(e['CURRENT_TIME']+dt)) <= 1e-12), None)
        free = next((s for s in reference if abs(s['time_s']-(e['CURRENT_TIME']+dt)) <= 1e-12), None)
        if not post or not free:
            checks.append(dict(status='FAIL', reason='Exact matched controlled/reference endpoint absent'))
            continue
        expected = {'velocity':V-np.asarray(e['get_velocity']), 'omega':W-np.asarray(e['get_omega'])}
        changes = {key:core.difference(np.asarray(post['native_state'][key])-free['native_state'][key], want)
                   for key, want in expected.items()}
        inherited,terminal,next_record=actual_next_start(core,history,post,control_dir)
        entry = next((s for s in sem if s.get('stage') == 'PROPERTIES_ENTRY'
                      and next_record is not None and s['_evidence_branch']==next_record['_evidence_branch']
                      and abs(s['CURRENT_TIME']-post['time_s']) <= 1e-12 and s['property_time'] > post['time_s']+1e-12), None)
        entry_pass = bool(entry) and core.norm(np.asarray(entry['velocity'])-post['native_state']['velocity']) <= 1e-8 and core.norm(np.asarray(entry['omega'])-post['native_state']['omega']) <= 1e-6
        direction = all(x['relative_error'] is not None and x['relative_error'] <= .10 and x['direction_cosine'] is not None and x['direction_cosine'] >= .99 for x in changes.values())
        math_pass = core.norm(V-np.asarray(e['requested_velocity'])) <= 1e-10 and core.norm(W-np.asarray(e['requested_omega'])) <= 1e-7 and abs(J-e['impulse_N_s']) <= 1e-16 and vn < 0
        ref_gates=state_numeric_gates(free,allow_penetration=True)
        checked = dict(status='PASS' if direction and math_pass and inherited['status']=='PASS' and entry_pass and all(ref_gates.values()) else 'FAIL',
                       event=e, actual_completed_state=post, no_contact_reference_state=free,
                       expected_vs_observed_changes=changes, independent_math_PASS=bool(math_pass),
                       reference_endpoint_numerical_gates=ref_gates, observed_response_PASS=bool(direction),
                       next_start=inherited, actual_next_properties_entry=entry,
                       actual_next_step_inheritance_PASS=bool(entry_pass and inherited['status']=='PASS'))
        checks.append(checked)
    first_completed = min(e['CURRENT_TIME']+dt for e in applied)
    controlled_states = [s for s in [initial, *history] if s['time_s'] <= first_completed+dt+1e-12]
    valid_numerics = all(s.get('status') == 'PASS' and all(state_numeric_gates(s).values()) for s in controlled_states)
    reference_states=[s for s in [reference_initial,*reference] if s and s['time_s']<=first_completed+TIME_TOL]
    valid_reference=bool(reference_states) and all(all(state_numeric_gates(s,allow_penetration=True).values()) for s in reference_states)
    closed=all(core.closure_review(p)['status']=='PASS' for p in parts)
    reference_closed=all(core.closure_review(p)['status']=='PASS' for p in reference_parts)
    magnetic=bounded_magnetic_audit(parts,initial['time_s'],first_completed+dt)
    reference_magnetic=bounded_magnetic_audit(reference_parts,reference_initial['time_s'],first_completed)
    restart=all(read(p/'source_restart_validation.json').get('status')=='PASS' for p in [*parts,*reference_parts])
    compile_pass=all(read(p/'compile_hook_gate.json').get('status')=='PASS' and
                     read(p/'compile_hook_gate.json').get('magnetic_formula_delegated_once') is True and
                     read(p/'compile_hook_gate.json').get('native_contact_disabled_for_load_path') is True
                     for p in [*parts,*reference_parts])
    passed = len(applied)==1 and bool(checks) and all(x['status']=='PASS' for x in checks) and valid_numerics and valid_reference and closed and reference_closed and restart and compile_pass and magnetic['status']=='PASS' and reference_magnetic['status']=='PASS'
    report = dict(status='PASS' if passed else 'FAIL', timestamp=stamp(),
                  result='LOAD_PATH_CONTACT_RESPONSE_VALIDATED' if passed else 'LOAD_PATH_CONTROLLED_NOT_VALIDATED',
                  controlled_branch=control_name, reference_branch=reference_name,
                  controlled_source_sha256=sha(ROOT/SOURCE), dt_s=dt, response_checks=checks,
                  physical_event_count=len(applied), all_controlled_numerical_states_PASS=valid_numerics,
                  bounded_free_reference_numerical_PASS=valid_reference,reference_owned_engine_closed=reference_closed,
                  controlled_magnetic_audit=magnetic,reference_magnetic_audit=reference_magnetic,
                  source_restart_PASS=restart,actual_compile_hook_and_delegate_once_PASS=compile_pass,
                  controlled_gate_interval_s=[initial['time_s'], first_completed+dt],
                  later_diagnostic_states_are_not_part_of_one_event_gate=True,
                  later_diagnostic_failure=read(control_dir/'failure.json') or None,
                  owned_engine_closed=closed, no_motion_overwrite=True, completed_delta_subtracts_matched_fluid_magnetic_reference=True,
                  reference_note='An intentionally free no-contact branch may stop at penetration. Its finite exact matched endpoint is reference evidence, never a physical-contact production PASS.',
                  maximum_time_s=.002, controlled_scope='Exactly one load interval plus actual next-step inheritance; not production PASS')
    atomic(EVID/'fallback_controlled_validation.json', report)
    return report


def load_event_audit(b, history, initial, dt, parts=None):
    import numpy as np
    import benchmark_D_native_contact_overnight_review as core
    parts = parts or [b]
    raw = core.branch_rows(parts, 'native_contact_callback_trace_node.jsonl')
    trace = [r for r in raw if r.get('moving_body') and r.get('analytic_contact_point_count', 0)>0]
    applied = [r for r in trace if r.get('physical_impulse_applied')]
    checks = []
    duplicates = []
    external = read(EVID/'fallback_controlled_validation.json')
    admission = external.get('status')=='PASS' and external.get('controlled_source_sha256')==sha(ROOT/SOURCE)
    for e in applied:
        boundary = [a for a in applied if abs(a['CURRENT_TIME']-e['CURRENT_TIME']) < 1e-13 and a['dynamic_zone_id']==e['dynamic_zone_id']]
        if len(boundary)>1:
            duplicates.append(e['CURRENT_TIME'])
        r = np.asarray(e['contact_point'])-e['DT_CG']
        impulse = np.asarray(e['contact_normal'])*e['impulse_N_s']
        load_math = core.norm(np.asarray(e['Fcontact_N'])*dt-impulse)<=1e-12 and core.norm(np.asarray(e['Tcontact_COM_Nm'])*dt-np.cross(r,impulse))<=1e-14
        if e.get('load_contact_model', 0)==0:
            V,W,J,vn,_ = core.independent_impulse(e)
            load_math = load_math and abs(J-e['impulse_N_s'])<=1e-16 and core.norm(V-e['requested_velocity'])<=1e-10 and core.norm(W-e['requested_omega'])<=1e-7
        else:
            Fn = e['stiffness_N_m']*max(0., -e['gap_predicted_m'])+e['damping_N_s_m']*max(0., -e['normal_velocity_before'])
            load_math = load_math and abs(e['impulse_N_s']-Fn*dt)<=1e-12
        same_epoch = [a for a in trace if abs(a['CURRENT_TIME']-e['CURRENT_TIME']) < 1e-13 and a['dynamic_zone_id']==e['dynamic_zone_id']]
        cached = all(core.norm(np.asarray(a['Fcontact_N'])-e['Fcontact_N'])<=1e-12 and core.norm(np.asarray(a['Tcontact_COM_Nm'])-e['Tcontact_COM_Nm'])<=1e-14 for a in same_epoch)
        post = next((s for s in history if abs(s['time_s']-(e['CURRENT_TIME']+dt))<=1e-12), None)
        inherit,terminal,next_record=actual_next_start(core,history,post,b,terminal_allowed=True) if post else ({'status':'FAIL'},False,None)
        sem = core.branch_rows(parts, 'theta_lifecycle_node.jsonl')
        entry = next((s for s in sem if s.get('stage')=='PROPERTIES_ENTRY' and post is not None and next_record is not None and
            s['_evidence_branch']==next_record['_evidence_branch'] and abs(s['CURRENT_TIME']-post['time_s'])<=1e-12 and s['property_time']>post['time_s']+1e-12), None)
        entry_pass = terminal or bool(entry) and core.norm(np.asarray(entry['velocity'])-post['native_state']['velocity'])<=1e-8 and core.norm(np.asarray(entry['omega'])-post['native_state']['omega'])<=1e-6
        completed=bool(post) and all(state_numeric_gates(post,allow_penetration=True).values()) and admission
        inherited=inherit['status']=='PASS' and entry_pass
        gate = load_math and cached and completed and e['gap_predicted_m']<=0 and e['normal_velocity_before']<0 and inherited and not e.get('overwrite_called')
        checks.append(dict(status='PASS' if gate else 'FAIL', callback_index=e['callback_index'], load_interval_math_PASS=bool(load_math),
                           cached_property_re_evaluations_do_not_accumulate=bool(cached), native_next_start=inherit,
                           actual_next_properties_entry_PASS=bool(entry_pass and not terminal), terminal_native_read_only=terminal,
                           independent_impulse_math_PASS=bool(load_math),completed_response_PASS=bool(completed),
                           next_step_inherited_PASS=bool(inherited),inheritance_evidence_available=bool(post and inherit.get('status')=='PASS'),
                           terminal_zero_step_boundary_only=bool(terminal),
                           response_validation_basis='Measured one-event LOAD integration gate plus each finite native completed boundary/inheritance; no per-event instantaneous overwrite equality claim'))
    host_applied = [r for r in core.branch_rows(parts, 'native_contact_callback_trace_host.jsonl') if r.get('physical_impulse_applied')]
    host_same = len(host_applied)==len(applied) and all(all(a[k]==h[k] for k in ('CURRENT_TIME','contact_point','impulse_N_s','Fcontact_N','Tcontact_COM_Nm')) for a,h in zip(applied,host_applied))
    finite = all(np.isfinite(np.r_[e['Fcontact_N'],e['Tcontact_COM_Nm'],e['impulse_N_s'],e['requested_velocity'],e['requested_omega']]).all() for e in trace)
    passed = admission and bool(applied) and not duplicates and host_same and finite and all(c['status']=='PASS' for c in checks)
    result = dict(status='PASS' if passed else 'FAIL', callback_count=len(trace), physical_impulse_application_count=len(applied),
                  dedup_PASS=not duplicates, host_node_same_event=host_same, finite_PASS=bool(finite),
                  controlled_load_integration_PASS=admission, controlled_gate_sha256=sha(EVID/'fallback_controlled_validation.json'),
                  persistence_PASS=bool(checks) and all(c['status']=='PASS' for c in checks), response_checks=checks,
                  no_instantaneous_load_path_setter_claim=True, no_manual_fluid_force=True)
    (b/'contact_events.jsonl').write_text(''.join(json.dumps(e,allow_nan=False)+'\n' for e in raw))
    with (b/'contact_events.csv').open('w', newline='') as fp:
        keys=('CURRENT_TIME','timestep_index','action','physical_impulse_applied','impulse_N_s','normal_velocity_before','normal_velocity_after','gap_current_m','gap_predicted_m')
        w=csv.DictWriter(fp,fieldnames=keys);w.writeheader();w.writerows({k:e.get(k) for k in keys} for e in raw)
    return result, trace


def review_branch(name):
    import benchmark_D_native_contact_overnight_review as core
    original_metrics = core.metrics
    def load_metrics(states, trace, dt):
        # A routine no-load properties evaluation is not continuing contact.
        # This distinction lets detection-free positive-gap boundaries count
        # as separation after an actual applied-load episode.
        active_trace = [e for e in trace if e.get('load_interval_J_N_s', 0)>0]
        out = original_metrics(states, active_trace, dt)
        out['contact_geometry_note']='Analytic actual canonical surface: load activates only when finite-rotation predicted next gap<=0 and normal velocity is approaching.'
        out['episode_definition']='Adjacent applied normal-load intervals form one analytic approach-contact episode; properties re-evaluations are not new impulses.'
        applied=[e for e in trace if e.get('physical_impulse_applied')]
        out['peak_contact_load_N']=max((math.sqrt(sum(x*x for x in e['Fcontact_N'])) for e in applied),default=0)
        return out
    core.event_audit=load_event_audit
    core.metrics=load_metrics
    result=core.branch_review(name)
    result['route']=read(EVID/'branches'/name/'configuration.json')['route']
    atomic(EVID/'branches'/name/'review.json', result)
    return result


def select_load(reports, destination):
    import benchmark_D_native_contact_overnight_review as core
    comparisons=[];chosen=None
    for coarse,fine in zip(reports,reports[1:]):
        if coarse.get('status')!='PASS' or fine.get('status')!='PASS':
            comparisons.append(dict(status='FAIL',reason='Full contact branch gates did not pass',branches=[coarse.get('branch'),fine.get('branch')]))
            continue
        c=core.compare_dt(coarse,fine);comparisons.append(c)
        if c['status']=='PASS':chosen=coarse;break
    result=dict(status='PASS' if chosen else 'REFINEMENT_REQUIRED',timestamp=stamp(),route=chosen.get('route') if chosen else None,
                selected_branch=chosen['branch'] if chosen else None,selected_physical_branch=chosen['branch'] if chosen else None,
                selected_dt_s=chosen['dt_s'] if chosen else None,comparisons=comparisons,
                validation_interval_s=[.00175,.002],minimum_1p9ms_checkpoint_retained=True,
                production_replay_not_needed_if_selected_2ms_already_verified=True,mesh_convergence_claim=False)
    atomic(EVID/destination,result);return result


def consolidate(name, selection):
    import benchmark_D_native_contact_overnight_review as core
    report=read(EVID/'branches'/name/'review.json')
    initial,history=core.history_chain(name)
    route=report['route']
    final_dir=EVID/'branches'/name
    checkpoint_19_path=final_dir/'checkpoint_1p900ms_checkpoint.json'
    if not checkpoint_19_path.is_file():
        for part in core.parts_for(name):
            for path in part.glob('*checkpoint.json'):
                metadata=read(path)
                if metadata.get('status')=='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS' and abs(metadata.get('time_s',0)-.0019)<=TIME_TOL:
                    if core.checkpoint_audit(path).get('status')=='PASS':
                        atomic(checkpoint_19_path,{**metadata,'retained_metadata_source':path.relative_to(ROOT).as_posix(),
                            'retained_metadata_source_sha256':sha(path),'large_native_files_not_copied':True})
                        break
            if checkpoint_19_path.is_file():break
    checkpoint_19=core.checkpoint_audit(checkpoint_19_path)
    memory_rows=[m for part in core.parts_for(name) for m in rows(part/'memory_samples.jsonl')]
    resources_PASS=bool(memory_rows) and all(m['available_gib']>=3 and m['commit_fraction']<.95 and
        m['project_working_set_gib']<15 and m['disk_free_gib']>=15 for m in memory_rows)
    gates=dict(selected_contact_dt_PASS=selection.get('status')=='PASS',full_selected_branch_PASS=report.get('status')=='PASS',
               actual_native_2ms=bool(history) and abs(history[-1]['time_s']-.002)<=1e-12,
               controlled_load_response_PASS=read(EVID/'fallback_controlled_validation.json').get('status')=='PASS',
               native_2ms_checkpoint_PASS=report.get('native_checkpoint',{}).get('status')=='PASS',
               owned_engine_closed=bool(report.get('lifecycle_parts')) and all(x.get('owned_engine_closed') for x in report.get('lifecycle_parts',[])),
               native_1p9ms_checkpoint_PASS=checkpoint_19.get('status')=='PASS',runtime_resource_history_PASS=resources_PASS)
    memories=[read(p/'memory_summary.json') for p in (EVID/'branches').glob('*') if (p/'memory_summary.json').is_file()]
    resources=dict(peak_project_working_set_gib=max((m.get('peak_project_working_set_gib',0) for m in memories),default=0),
                   peak_project_private_bytes_gib=max((m.get('peak_project_private_bytes_gib',0) for m in memories),default=0),
                   peak_system_commit_percent=max((m.get('peak_system_commit_percent',0) for m in memories),default=0))
    final=dict(status='NUMERICAL_PASS' if all(gates.values()) else 'INCOMPLETE',timestamp=stamp(),route=route,
               selected_dt_s=selection.get('selected_dt_s'),selected_micro_branch=name,production_branch=name,
               restart_source='C70 /1.750ms; continuous validated load-path branch to2.000ms; no replay',
               numerical_gates=gates,metrics=report['metrics'],numerical_peaks=report['numerical_peaks'],
               precontact_C_vs_D=report.get('precontact_C_vs_D'),resource_summary=resources,
               checkpoint_1p9ms=checkpoint_19,
               checkpoint_2ms=report['native_checkpoint'],validation_and_final_scope_s=[.00175,.002],
               physical_contact_definition='Analytic predicted zero-gap approach, not native100um proximity detection',
               fine_reference='FINE_REFERENCE_NOT_AVAILABLE_AT_2MS',mesh_convergence_claim=False,
               visual_review_status=read(EVID/'visual_review.json').get('status','NOT_REVIEWED'),
               result='BENCHMARK_D_LOAD_PATH_CONTACT_2MS_NUMERICAL_PASS')
    atomic(EVID/'fallback_result.json',final);atomic(EVID/'final_report.json',final)
    atomic(EVID/'contact_dt_selection.json',selection)
    return final


def run():
    deadline();check_capability()
    if not read(EVID/'native_route_failure.json'):
        raise RuntimeError('FALLBACK_REQUIRES_ARCHIVED_NATIVE_ROUTE_FAILURE')
    event('CONDITIONAL_LOAD_PATH_STARTED',controlled_validation_end_time_s=.002,not_a_production_PASS=True)
    control('FALLBACK_LOAD_VALIDATION')
    controlled=None
    for label,dt in (('25',25e-6),('12p5',12.5e-6),('6p25',6.25e-6)):
        ref=run_branch('load_reference'+label,dt,0)
        con=run_branch('load_controlled'+label,dt,2)
        job(['--controlled-review',con,'--reference',ref],con+'_review')
        controlled=read(EVID/'fallback_controlled_validation.json')
        if controlled.get('status')=='PASS':break
        if controlled.get('status')=='NO_LOAD_CONTACT_WITHIN_AUTHORIZED_INTERVAL':
            raise RuntimeError('NO_LOAD_CONTACT_WITHIN_AUTHORIZED_INTERVAL')
        failure=read(EVID/'branches'/con/'failure.json')
        if failure.get('failure_class') not in ('PHYSICAL_PENETRATION','CONTACT_TIMESTEP_PENETRATION','DT_RESOLUTION','CONTACT_RESPONSE'):
            raise RuntimeError('LOAD_CONTROLLED_FAILURE_REQUIRES_CONCRETE_REPAIR:'+str(failure.get('failure_class')))
        event('LOAD_CONTROLLED_DT_REFINEMENT',branch=con,dt_s=dt,archived_failure=con)
    if not controlled or controlled.get('status')!='PASS':
        raise RuntimeError('LOAD_CONTROLLED_RESPONSE_REPAIRS_EXHAUSTED')
    reports=[]
    for label,dt in (('25',25e-6),('12p5',12.5e-6),('6p25',6.25e-6)):
        name=run_branch('load_micro'+label,dt,3)
        job(['--review-branch',name],name+'_review')
        reports.append(read(EVID/'branches'/name/'review.json'))
        if len(reports)>=2:
            job(['--select-load',*[r['branch'] for r in reports]],'load_dt_selection')
            selection=read(EVID/'fallback_dt_selection.json')
            if selection.get('status')=='PASS':
                job(['--final-load',selection['selected_branch']],'load_final')
                return read(EVID/'fallback_result.json')
    # Numerical compliance is conditional on measured instability, not a
    # substitute for resource/restart/lifecycle failures or missing evidence.
    failures=[read(EVID/'branches'/r['branch']/'failure.json') for r in reports]
    unstable=any(f.get('failure_class') in ('PHYSICAL_PENETRATION','CONTACT_TIMESTEP_PENETRATION') for f in failures)
    if not unstable:
        raise RuntimeError('LOAD_DT_ADEQUACY_NOT_REACHED_WITH_AUTHORIZED_REFINEMENTS')
    evidence=[e for r in reports for e in rows(EVID/'branches'/r['branch']/'native_contact_callback_trace_node.jsonl')
              if e.get('physical_impulse_applied') and e.get('normal_velocity_before',0)<0]
    if not evidence:
        raise RuntimeError('COMPLIANT_PARAMETERS_REQUIRE_MEASURED_LOAD_EVENT')
    closing=max(abs(e['normal_velocity_before']) for e in evidence)
    m_eff=max(e['effective_contact_mass_kg'] for e in evidence)
    target=5e-6;k=m_eff*(closing/target)**2;c=2*math.sqrt(k*m_eff)
    derivation=dict(status='DERIVED_FOR_CONDITIONAL_INVESTIGATION',timestamp=stamp(),effective_mass_kg=m_eff,
                    closing_speed_m_s=closing,target_penetration_m=target,stiffness_N_m=k,damping_N_s_m=c,damping_ratio=1,
                    stiffness_formula='m_eff*(abs(vn)/target_penetration)^2',critical_damping_formula='2*sqrt(k*m_eff)',
                    measured_impulse_load_instability=failures,arbitrary_positive_gap_repulsion=False,
                    generic_fatal_nonfinite_or_resource_failures_do_not_establish_load_instability=True,
                    contact_dt_s=[12.5e-6,6.25e-6],duration_to_contact_penetration_s=target/closing,
                    dimensionless_contact_frequencies=[step*math.sqrt(k/m_eff) for step in [12.5e-6,6.25e-6]],
                    adjacent_dt_and_stiffness_sensitivity_required=True)
    atomic(EVID/'compliant_parameter_derivation.json',derivation)
    comp=[]
    for name,dt,scale in (('compliant_micro12p5_k1',12.5e-6,1),('compliant_micro6p25_k1',6.25e-6,1),('compliant_micro6p25_k2',6.25e-6,2)):
        b=run_branch(name,dt,3,route='COMPLIANT_LOAD_PATH',model=1,stiffness=k*scale,damping=c*math.sqrt(scale))
        job(['--review-branch',b],b+'_review');comp.append(read(EVID/'branches'/b/'review.json'))
    job(['--select-compliant',*[r['branch'] for r in comp]],'compliant_selection')
    selection=read(EVID/'fallback_dt_selection.json')
    if selection.get('status')!='PASS':
        raise RuntimeError('COMPLIANT_DT_STIFFNESS_SENSITIVITY_REPAIRS_EXHAUSTED')
    job(['--final-load',selection['selected_branch']],'compliant_final')
    return read(EVID/'fallback_result.json')


def main():
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--prepare',action='store_true');g.add_argument('--run',action='store_true')
    g.add_argument('--controlled-review');p.add_argument('--reference')
    g.add_argument('--review-branch');g.add_argument('--select-load',nargs='+')
    g.add_argument('--select-compliant',nargs='+');g.add_argument('--final-load')
    a=p.parse_args()
    try:
        if a.prepare:result=prepare_capability()
        elif a.run:result=run()
        elif a.controlled_review:result=controlled_review(a.controlled_review,a.reference)
        elif a.review_branch:result=review_branch(a.review_branch)
        elif a.select_load:result=select_load([read(EVID/'branches'/x/'review.json') for x in a.select_load],'fallback_dt_selection.json')
        elif a.select_compliant:
            import benchmark_D_native_contact_overnight_review as core
            reports=[read(EVID/'branches'/x/'review.json') for x in a.select_compliant]
            result=select_load(reports[:2],'fallback_dt_selection.json')
            stiffness=core.compare_dt(reports[1],reports[2]) if all(r.get('status')=='PASS' for r in reports[1:]) else {'status':'FAIL'}
            result['adjacent_stiffness_comparison']=stiffness
            if stiffness['status']!='PASS':result['status']='FAIL'
            atomic(EVID/'fallback_dt_selection.json',result)
        else:result=consolidate(a.final_load,read(EVID/'fallback_dt_selection.json'))
        print(json.dumps(dict(status=result.get('status'),route=result.get('route'),selected_branch=result.get('selected_branch'))))
    except Exception as exc:
        if a.run:
            report=dict(status='STOPPED_RESOURCE_BLOCKER' if isinstance(exc,FallbackResourceBlocker) else
                        'TIME_BUDGET_EXHAUSTED' if 'TIME_BUDGET_EXHAUSTED' in str(exc) else 'HARD_BLOCKER',
                        timestamp=stamp(),reason=repr(exc),traceback=traceback.format_exc(),solver_alive=bool(native()))
            atomic(EVID/'fallback_result.json',report);control(report['status'],reason=report['reason'])
        raise


if __name__=='__main__':main()
