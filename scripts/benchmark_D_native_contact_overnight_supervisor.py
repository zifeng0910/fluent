"""Windows-owned contact FSM. No LLM or interactive process owns solver survival."""
import json, msvcrt, os, subprocess, sys, time, traceback
import psutil
from benchmark_D_native_contact_overnight_common import *

TERMINAL={'COMPLETE','HARD_BLOCKER','TIME_BUDGET_EXHAUSTED','STOPPED_RESOURCE_BLOCKER','DT_CONVERGENCE_NOT_REACHED'}
WORKER='scripts/benchmark_D_native_contact_overnight_worker.py'
REVIEW='scripts/benchmark_D_native_contact_overnight_review.py'
RENDER='scripts/benchmark_D_native_contact_overnight_render.py'
FALLBACK='scripts/benchmark_D_native_contact_overnight_fallback.py'
EXIT_REVIEW='scripts/benchmark_D_native_contact_overnight_owned_exit_review.py'

def verified_completion_result(report):
    """The visual gate cannot relabel a LOAD result as native overwrite."""
    gates=report.get('numerical_gates',{})
    if not gates or not all(value is True for value in gates.values()):
        raise RuntimeError('Full numerical gates are required before COMPLETE')
    route=report.get('route')
    required=('dt_selected','selected_micro_PASS','production_PASS','final_time_2ms',
        'production_inherits_selected_D_checkpoint','same_production_dt') if route=='NATIVE_OVERWRITE' else (
        'selected_contact_dt_PASS','full_selected_branch_PASS','actual_native_2ms',
        'controlled_load_response_PASS','native_2ms_checkpoint_PASS','owned_engine_closed')
    if not all(gates.get(key) is True for key in required):
        raise RuntimeError('Route-specific full numerical gates are required')
    for key,target in [('checkpoint_1p9ms',.0019),('checkpoint_2ms',.002)]:
        cp=report.get(key,{})
        if cp.get('status')!='PASS' or abs(cp.get('time_s',0)-target)>1e-12:
            raise RuntimeError('Verified1.9ms/2ms native checkpoints are required')
    if route=='NATIVE_OVERWRITE' and report.get('status')=='BENCHMARK_D_NATIVE_CONTACT_2MS_PASS':
        return 'BENCHMARK_D_NATIVE_CONTACT_2MS_PASS'
    if route in {'SDOF_LOAD_PATH','COMPLIANT_LOAD_PATH'} and report.get('status')=='NUMERICAL_PASS':
        controlled=read(EVID/'fallback_controlled_validation.json')
        if controlled.get('status')!='PASS' or not gates.get('controlled_load_response_PASS'):
            raise RuntimeError('Measured controlled LOAD response is required')
        if report.get('checkpoint_2ms',{}).get('status')!='PASS':
            raise RuntimeError('Verified native2ms checkpoint is required')
        return 'BENCHMARK_D_'+('SDOF_LOAD_CONTACT' if route=='SDOF_LOAD_PATH' else 'COMPLIANT_LOAD_CONTACT')+'_2MS_PASS'
    raise RuntimeError('Unverified route/result combination cannot become COMPLETE')

def live_fallback_worker():
    """A dead controller does not imply its resource-waiting worker is dead."""
    matches=[]
    for path in (EVID/'branches').glob('*/worker_identity.json'):
        config=read(path.parent/'configuration.json')
        if config.get('route') not in {'SDOF_LOAD_PATH','COMPLIANT_LOAD_PATH'}:continue
        rec=read(path);rec.setdefault('name','pythonw.exe')
        process=identity(rec)
        if not process:continue
        if not any(Path(part).name==Path(WORKER).name for part in process.cmdline()):
            raise RuntimeError('Registered fallback worker command mismatch')
        matches.append(dict(rec,kind='fallback_worker',job=WORKER,branch=path.parent.name,
            key='adopted_fallback_worker_'+str(rec['pid']),started=stamp()))
    if len(matches)>1:raise RuntimeError('Multiple live fallback workers; no new launch allowed')
    return matches[0] if matches else None

def resume_fallback_controller(previous):
    """Resume an idempotent pipeline only after its exact child has closed."""
    if live_fallback_worker() or native():
        raise RuntimeError('Fallback child/engine is still active')
    history=read(EVID/'fallback_controller_recovery_history.json',{'repairs':[]})
    if len(history['repairs'])>=read(EVID/'configuration.json')['max_lifecycle_restarts']:
        fail('Fallback controller lifecycle recoveries exhausted',failure_class='WORKER_LIFECYCLE_TERMINATION')
        return None
    review=dict(timestamp=stamp(),previous_controller=previous,
        failure_class='WORKER_LIFECYCLE_TERMINATION',
        concrete_change='Resume reviewed idempotent helper after existing child closure; retain completed branch results and latest native checkpoints',
        no_active_fallback_worker=True,no_native_engine=True,window_sha256=sha(EVID/'window.json'))
    history['repairs'].append(review);atomic(EVID/'fallback_controller_recovery_history.json',history)
    event('FALLBACK_CONTROLLER_RECOVERY_REVIEWED',review=review)
    state('FALLBACK_LOAD_VALIDATION',route=read(EVID/'state.json').get('route','SDOF_LOAD_PATH'))
    return launch('fallback',FALLBACK,['--run'])

def control(status,**data):
    r=read(EVID/'supervisor_state.json');r.update(status=status,timestamp=stamp(),
        supervisor_pid=os.getpid(),supervisor_created=psutil.Process().create_time(),supervisor_alive=True)
    r.update(data)
    atomic(EVID/'supervisor_state.json',r)

def fail(reason,**data):
    state('HARD_BLOCKER',reason=reason,**data);event('HARD_BLOCKER',reason=reason,**data)


def stop_on_dt_convergence_failure(result):
    """Make an unresolved timestep gate terminal; never enter fallback."""
    reason = result.get('reason', 'Frozen timestep convergence gate failed')
    failure_class = result.get('failure_class', 'DT_RESOLUTION')
    blocker = 'Frozen 12.5us versus 6.25us convergence failed solely at the impulse-count gate; no production or fallback is authorized.'
    state('DT_CONVERGENCE_NOT_REACHED', reason=reason, failure_class=failure_class,
        current_blocker=blocker, native_status='NOT_DISPROVEN_BY_DT_CONVERGENCE',
        selected_dt_s=None, production_restart='NOT_STARTED', fallback_route='NOT_ALLOWED',
        pending_stage=None, active_branch=None, worker_alive=False, solver_alive=False,
        next_action='Stop. Await explicit user authorization for any finer timestep or alternate route.')
    stop = dict(timestamp=stamp(), reason='DT_CONVERGENCE_NOT_REACHED', source='offline timestep selection',
        do_not_run_finer_or_fallback_route=True, result_status=result.get('status'),
        result_reason=reason)
    atomic(EVID/'stop_request.json', stop)
    event('DT_CONVERGENCE_NOT_REACHED', result_status=result.get('status'), reason=reason,
        no_fallback=True, no_production=True)
    return None

def checked_code(path):
    review=read(EVID/'launch_review.json')
    if review.get('status')!='PASS':raise RuntimeError('Local launch readiness review absent')
    if sha(EVID/'configuration.json')!=review['configuration_sha256']:raise RuntimeError('Campaign configuration changed')
    if path not in review['supporting_code_sha256']:raise RuntimeError('Unreviewed job/code: '+path)
    for job,digest in review['supporting_code_sha256'].items():
        if sha(ROOT/job)!=digest:raise RuntimeError('Reviewed supporting code changed: '+job)
    verify_frozen()

def launch(kind,job,args,branch=None):
    checked_code(job)
    if kind in {'worker','fallback'} and native():raise RuntimeError('Existing native engine: second launch refused')
    key=f'{kind}_{branch or "campaign"}_{int(time.time()*1000)}'
    logs=EVID/'jobs';logs.mkdir(exist_ok=True)
    stdout=(logs/(key+'_stdout.log')).open('a');stderr=(logs/(key+'_stderr.log')).open('a')
    try:
        p=subprocess.Popen([str(ROOT/'.venv/Scripts/pythonw.exe'),'-u',str(ROOT/job),*args],cwd=str(ROOT),
            stdout=stdout,stderr=stderr,creationflags=subprocess.CREATE_NO_WINDOW)
    finally:stdout.close();stderr.close()
    item=dict(kind=kind,job=job,branch=branch,pid=p.pid,created=psutil.Process(p.pid).create_time(),
        name=psutil.Process(p.pid).name(),started=stamp(),stdout=str((logs/(key+'_stdout.log')).relative_to(ROOT)),
        stderr=str((logs/(key+'_stderr.log')).relative_to(ROOT)),key=key)
    atomic(EVID/'active_job.json',item);event('JOB_STARTED',process_identity=item)
    return p,item

def result_for(job):
    if job['kind'] in {'worker','fallback_worker'}:return read(EVID/'branches'/job['branch']/'branch_result.json')
    if job['kind']=='review':return read(EVID/'branches'/job['branch']/'review.json')
    if job['kind']=='select':return read(EVID/'contact_dt_selection.json')
    if job['kind']=='final':return read(EVID/'final_report.json')
    if job['kind']=='render':return read(EVID/'render_manifest.json')
    if job['kind']=='closure':return read(EVID/'branches'/job['branch']/'recovery_process_review.json')
    return {}

def lifecycle_recovery(branch):
    """Resume only a proven complete native boundary; preserve the failed attempt."""
    b=EVID/'branches'/branch;failure=read(b/'failure.json')
    if failure and failure.get('failure_class')!='WORKER_LIFECYCLE_TERMINATION':return None
    if failure:
        detail=failure.get('error','')+' '+failure.get('traceback','')
        external_lifecycle=any(word in detail for word in ['RpcError','grpc.','UNAVAILABLE','ConnectionResetError',
            'BrokenPipeError','ProcessLookupError','FluentConnection','connection closed','connection terminated'])
        if not external_lifecycle:return None
    if native():return None
    worker_record=read(b/'worker_identity.json');worker_record.setdefault('name','pythonw.exe')
    if identity(worker_record):return None
    owned=read(b/'owned_engine.json').get('processes',[])
    if any(identity(r) for r in owned):return None
    process_review=dict(status='PASS',timestamp=stamp(),worker_identity=worker_record,
        owned_engine_identities=owned,worker_absent=True,all_owned_engines_absent=True,
        no_native_engine_present=True,no_process_terminated=True,
        evidence='Exact registered PID/creation/name checks and global native-engine inventory')
    atomic(b/'recovery_process_review.json',process_review)
    if not read(b/'memory_summary.json'):
        samples=rows(b/'memory_samples.jsonl')
        if not samples or any(m['available_gib']<3 or m['commit_fraction']>=.95 or
            m['project_working_set_gib']>=15 or m['disk_free_gib']<15 for m in samples):return None
        atomic(b/'memory_summary.json',dict(timestamp=stamp(),samples=len(samples),
            peak_project_working_set_gib=max(m['project_working_set_gib'] for m in samples),
            peak_project_private_bytes_gib=max(m['project_private_bytes_gib'] for m in samples),
            peak_system_commit_percent=100*max(m['commit_fraction'] for m in samples),
            minimum_available_gib=min(m['available_gib'] for m in samples),abort=None,
            reconstructed_from_preserved_raw_samples=True))
    spec=specification(branch);logical=spec.get('logical_branch',branch)
    history=read(EVID/'lifecycle_repair_history.json',{'repairs':[]})
    used=[r for r in history['repairs'] if r['logical_branch']==logical]
    fallback_restarts=read(EVID/'fallback_state.json').get('lifecycle_restarts',0)
    if len(history['repairs'])+fallback_restarts>=read(EVID/'configuration.json')['max_lifecycle_restarts']:return None
    pointer=read(b/'latest_verified_checkpoint.json')
    metadata_path=pointer.get('metadata')
    if not metadata_path:return None
    metadata=read(ROOT/metadata_path)
    if metadata.get('status')!='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS':return None
    if abs(metadata['time_s']-spec['end_time_s'])<1e-12:
        # A missing branch result after final save is reviewed; no time is replayed.
        history=read(b/'native_history.json');records=history.get('records',[])
        if not records or abs(records[-1]['time_s']-metadata['time_s'])>=1e-12:return None
        if any(r.get('status')!='PASS' or r.get('hard_failures') for r in records):return None
        if records[-1]['native_state']!=metadata['native_state']:return None
        for kind in ['case','data']:
            pth=ROOT/metadata[kind] if not os.path.isabs(metadata[kind]) else Path(metadata[kind])
            if not pth.is_file() or sha(pth)!=metadata[kind+'_sha256']:return None
        atomic(b/'final_native_checkpoint.json',metadata)
        atomic(b/'worker_exit.json',dict(timestamp=stamp(),status='LIFECYCLE_EXIT_AFTER_VERIFIED_FINAL_CHECKPOINT',
            solver_closed=True,closure_basis='Recorded worker absent and no native engine present; no engine was terminated',
            current_time_s=metadata['time_s'],completed_new_steps=len(records)))
        atomic(b/'branch_result.json',dict(status='RECOVERED_NATIVE_BRANCH_DATA_COMPLETE',branch=branch,
            timestamp=stamp(),stage=spec['stage'],route=spec.get('route','NATIVE_OVERWRITE'),dt_s=spec['dt_s'],
            start_time_s=spec['start_time_s'],end_time_s=spec['end_time_s'],completed_new_steps=len(records),
            last_state=records[-1],final_checkpoint=str((b/'final_native_checkpoint.json').relative_to(ROOT)),
            reconstructed_from_verified_native_boundary_only=True))
        return ('review',branch)
    if metadata['time_s']>=spec['end_time_s'] or metadata['time_s']<spec['start_time_s']:return None
    for kind in ['case','data']:
        pth=ROOT/metadata[kind] if not os.path.isabs(metadata[kind]) else Path(metadata[kind])
        if not pth.is_file() or sha(pth)!=metadata[kind+'_sha256']:return None
    new=logical+'_recovery'+str(len(used)+1)
    new_spec={**spec,'branch':new,'logical_branch':logical,'source_checkpoint':metadata_path,
        'start_time_s':metadata['time_s'],'history_prefix_branches':spec.get('history_prefix_branches',[])+[branch],
        'recovery_reason':'WORKER_LIFECYCLE_TERMINATION; newer verified native boundary retained'}
    atomic(EVID/'branch_specs'/f'{new}.json',new_spec)
    plan=dict(plan_id=new,timestamp=stamp(),failure_class='WORKER_LIFECYCLE_TERMINATION',logical_branch=logical,
        previous_branch=branch,new_branch=new,source_checkpoint=metadata_path,
        source_checkpoint_sha256=sha(ROOT/metadata_path),source_time_s=metadata['time_s'],
        concrete_change='New worker identity/cold native restart from latest verified boundary, not C70 replay',
        configuration_sha256=sha(EVID/'branch_specs'/f'{new}.json'),zero_step_native_gate_required=True)
    history['repairs'].append(plan);atomic(EVID/'lifecycle_repair_history.json',history)
    atomic(EVID/'plans'/f'{new}.json',plan)
    mapping=read(EVID/'branch_resolution.json');mapping[logical]=new;atomic(EVID/'branch_resolution.json',mapping)
    event('LIFECYCLE_NATIVE_RECOVERY_REVIEWED',**plan)
    return ('worker',new)

def branch_spec(branch,dt,end,stage,checkpoint=None):
    c=read(EVID/'configuration.json')
    path=EVID/'branch_specs'/f'{branch}.json'
    if path.exists():return read(path)
    checkpoint=checkpoint or c['source_checkpoint']
    metadata=read(ROOT/checkpoint)
    start=float(metadata['time_s'])
    item=dict(branch=branch,route='NATIVE_OVERWRITE',dt_s=dt,start_time_s=start,end_time_s=end,
        stage=stage,source_checkpoint=checkpoint,contact_source_path=c['contact_source_path'],mode=3,
        penetration_tolerance_m=c['penetration_tolerance_m'],iterations_per_step=2,
        hard_deadline_epoch=read(EVID/'window.json')['hard_deadline_epoch'])
    atomic(path,item);event('BRANCH_SPEC_CREATED',branch=branch,spec_sha256=sha(path));return item

def next_worker(branch,dt,end,stage,checkpoint=None):
    b=EVID/'branches'/branch
    if (b/'worker_identity.json').exists() or (b/'branch_result.json').exists():
        raise RuntimeError('Unchanged branch relaunch refused: '+branch)
    branch_spec(branch,dt,end,stage,checkpoint);review_launch(branch)
    state(stage,active_branch=branch,current_time_s=read(b/'configuration.json')['start_time_s'])
    return launch('worker',WORKER,['--branch',branch],branch)

def native_failure(branch,review):
    """Only numerical contact-route failures qualify for conditional fallback."""
    failure=read(EVID/'branches'/branch/'failure.json')
    info=dict(branch=branch,review=review,failure=failure,timestamp=stamp())
    atomic(EVID/'native_route_failure.json',info)
    # A malformed/incomplete restart, resources and lifecycle are not contact-model evidence.
    cls=review.get('failure_class') or failure.get('failure_class','UNCLASSIFIED')
    route_classes={'CALLBACK_DUPLICATION','CONTACT_RESPONSE','EVENT_STATE_INHERITANCE','CONTACT_TIMING','DT_RESOLUTION','PHYSICAL_PENETRATION','CONTACT_TIMESTEP_PENETRATION'}
    if cls not in route_classes:
        fail('Contact route not disproven; concrete failure needs review',failure_class=cls,failure_evidence='native_route_failure.json');return None
    plan=read(EVID/'fallback_launch_review.json')
    if plan.get('status')!='PASS':
        fail('Validated load-path fallback launch capability is not yet verified',failure_class=cls,
             native_status='NATIVE_MULTI_EVENT_CONTACT_NOT_VIABLE',failure_evidence='native_route_failure.json');return None
    state('FALLBACK_LOAD_VALIDATION',native_status='NATIVE_MULTI_EVENT_CONTACT_NOT_VIABLE',route='SDOF_LOAD_PATH')
    event('ROUTE_SWITCHED',from_route='NATIVE_OVERWRITE',to_route='SDOF_LOAD_PATH',failure_class=cls)
    return launch('fallback',FALLBACK,['--run'])

def main():
    EVID.mkdir(parents=True,exist_ok=True);OUT.mkdir(parents=True,exist_ok=True)
    sys.stdout=(EVID/'supervisor_stdout.log').open('a',buffering=1);sys.stderr=(EVID/'supervisor_stderr.log').open('a',buffering=1)
    lock=(EVID/'supervisor.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    window=read(EVID/'window.json');stable=None;p=None
    active=read(EVID/'active_job.json')
    if active.get('status')=='FINISHED':active={}
    if not active:
        # Close the tiny Popen->journal race without inventing connection data.
        owned=[]
        for identity_file in (EVID/'branches').glob('*/worker_identity.json'):
            record=read(identity_file);record.setdefault('name','pythonw.exe')
            process=identity(record)
            if process:
                command=process.cmdline()
                if not any(Path(part).name=='benchmark_D_native_contact_overnight_worker.py' for part in command):
                    raise RuntimeError('Registered branch PID is not the expected worker command')
                owned.append(record)
        if len(owned)>1:raise RuntimeError('More than one registered native contact worker')
        if owned:
            record=owned[0]
            active=dict(record,kind='worker',job=WORKER,key='adopted_'+str(record['pid']),started=stamp())
            atomic(EVID/'active_job.json',active)
    if active:
        if active['kind']=='fallback':
            helper=read(EVID/'fallback_state.json').get('helper_identity',{})
            if helper and identity(helper):
                if not any(Path(part).name==Path(FALLBACK).name for part in identity(helper).cmdline()):
                    raise RuntimeError('Registered fallback helper command mismatch')
                active.update(launcher_identity={k:active[k] for k in ['pid','created','name']},
                    **{k:helper[k] for k in ['pid','created','name']})
                atomic(EVID/'active_job.json',active)
        if active['kind']=='worker':
            registered=read(EVID/'branches'/active['branch']/'worker_identity.json')
            registered.setdefault('name','pythonw.exe')
            actual_worker=identity(registered) if registered.get('pid') else None
            if actual_worker:
                cmd=actual_worker.cmdline()
                if not any(Path(part).name=='benchmark_D_native_contact_overnight_worker.py' for part in cmd):
                    raise RuntimeError('Registered native worker command mismatch')
                # Windows venv redirectors can have a different launcher PID.
                active.update(launcher_identity={k:active[k] for k in ['pid','created','name']},
                    **{k:registered[k] for k in ['pid','created','name']})
                atomic(EVID/'active_job.json',active)
        owner=identity(active)
        if owner:
            event('EXISTING_JOB_ADOPTED',process_identity=active)
        else:
            active.update(returncode='UNKNOWN_AFTER_CONTROLLER_RESTART')
    event('SUPERVISOR_STARTED',hard_deadline_epoch=window['hard_deadline_epoch'])
    while True:
        now=time.time();campaign=read(EVID/'state.json');status=campaign.get('status','STARTUP')
        control(status,active_job=active or None,seconds_to_hard_deadline=window['hard_deadline_epoch']-now)
        if active:
            alive=(p.poll() is None) if p is not None else bool(identity(active))
            if alive:
                if now>=window['hard_deadline_epoch'] and active['kind'] in {'worker','fallback','fallback_worker'}:
                    atomic(OUT/'stop_request.json',dict(reason='TIME_BUDGET_EXHAUSTED',timestamp=stamp()))
                    state('DEADLINE_GRACEFUL_CHECKPOINT_WAIT',active_branch=active.get('branch'))
                if active['kind'] in {'worker','fallback_worker'}:
                    ws=read(EVID/'branches'/active['branch']/'worker_state.json')
                    control(ws.get('status',status),active_job=active,worker_state=ws)
                    state(read(EVID/'state.json').get('status',status),active_branch=active['branch'],
                        current_time_s=ws.get('current_time_s',campaign.get('current_time_s')),
                        worker_alive=bool(identity(active)),solver_alive=ws.get('solver_alive',False),
                        worker_status=ws.get('status'),worker_state_timestamp=ws.get('timestamp'))
                time.sleep(5);continue
            result=result_for(active);returncode=p.returncode if p else active.get('returncode')
            event('JOB_FINISHED',key=active['key'],kind=active['kind'],branch=active.get('branch'),returncode=returncode,result_status=result.get('status'))
            done={**active,'status':'FINISHED','returncode':returncode,'finished':stamp()}
            atomic(EVID/'jobs'/(active['key']+'.json'),done);atomic(EVID/'active_job.json',done)
            kind=active['kind'];branch=active.get('branch');active={};p=None
            if kind=='fallback':
                child=live_fallback_worker()
                if child:
                    child['previous_controller']=done
                    active=child;atomic(EVID/'active_job.json',active)
                    event('SURVIVING_FALLBACK_WORKER_ADOPTED',process_identity=child)
                    continue
            if native():
                if kind=='worker' and 'Owned engine not idle after SDK exit' in read(EVID/'branches'/branch/'closure_failure.json').get('error',''):
                    state('OWNED_EXIT_REVIEW',active_branch=branch,worker_alive=False,solver_alive=True)
                    p,active=launch('closure',EXIT_REVIEW,['--branch',branch],branch)
                    continue
                fail('Native engine remains after registered worker closure; no further solver allowed');continue
            if now>=window['hard_deadline_epoch']:
                state('TIME_BUDGET_EXHAUSTED',last_completed_result=result,solver_alive=False);continue
            if kind=='closure':
                if result.get('status')!='PASS':
                    fail('Exact owned post-SDK-exit closure did not pass',branch=branch);continue
                event('OWNED_EXIT_REVIEW_PASS',branch=branch)
                p,active=launch('review',REVIEW,['--branch',branch],branch);continue
            if kind=='fallback_worker':
                recovery=resume_fallback_controller(done.get('previous_controller',done))
                if recovery:p,active=recovery
                continue
            if kind=='worker':
                if not result:
                    recovery=lifecycle_recovery(branch)
                    if recovery:
                        next_kind,next_branch=recovery
                        if next_kind=='review':p,active=launch('review',REVIEW,['--branch',next_branch],next_branch)
                        else:
                            # Admission and continuity remain mandatory in the new worker.
                            review_launch(next_branch)
                            state(specification(next_branch)['stage'],active_branch=next_branch)
                            p,active=launch('worker',WORKER,['--branch',next_branch],next_branch)
                        continue
                    failure=read(EVID/'branches'/branch/'failure.json')
                    if failure:
                        p,active=launch('review',REVIEW,['--branch',branch],branch);continue
                    fail('WORKER_LIFECYCLE_TERMINATION; no newer verified checkpoint or recoveries exhausted',branch=branch);continue
                p,active=launch('review',REVIEW,['--branch',branch],branch);continue
            if kind=='review':
                spec=read(EVID/'branch_specs'/f'{branch}.json');stage=spec.get('stage');logical=spec.get('logical_branch',branch)
                if stage=='NATIVE_PRODUCTION':
                    if result.get('status')!='PASS':
                        switched=native_failure(branch,result)
                        if switched:p,active=switched
                        continue
                    state('POSTPROCESS',production_branch=branch,route='NATIVE_OVERWRITE')
                    p,active=launch('final',REVIEW,['--final']);continue
                if result.get('status') not in {'PASS','PENETRATION_REQUIRES_DT','DT_RESOLUTION_REQUIRES_REFINEMENT'}:
                    switched=native_failure(branch,result)
                    if switched:p,active=switched
                    continue
                if logical=='micro25':
                    state('NATIVE_DT_12P5',micro25_status=result['status']);continue
                state('DT_SELECTION');p,active=launch('select',REVIEW,['--select-dt']);continue
            if kind=='select':
                if result.get('status')=='REFINEMENT_REQUIRED':
                    if (EVID/'branches'/resolve_branch('micro6p25')/'review.json').exists():
                        stop_on_dt_convergence_failure(dict(result, failure_class='DT_RESOLUTION'))
                    else:state('NATIVE_DT_6P25')
                    continue
                if result.get('status')!='PASS':
                    stop_on_dt_convergence_failure(dict(result, failure_class='DT_RESOLUTION'))
                    continue
                chosen=result['selected_branch'];dt=result['selected_dt_s']
                review=read(EVID/'branches'/resolve_branch(chosen)/'review.json')
                if review.get('status')!='PASS':raise RuntimeError('Selected micro-run has not passed all gates')
                state('NATIVE_PRODUCTION',selected_branch=chosen,selected_dt_s=dt,micro_run='BENCHMARK_D_NATIVE_CONTACT_MICRORUN_PASS')
                event('MICRORUN_PASS',branch=chosen);event('DT_SELECTED',branch=chosen,dt_s=dt)
                continue
            if kind=='final':
                if result.get('status') not in {'BENCHMARK_D_NATIVE_CONTACT_2MS_PASS','NUMERICAL_PASS'}:
                    fail('Final numerical consolidation did not pass',report_status=result.get('status'));continue
                event('TWO_MS_NUMERICAL_PASS',report='final_report.json');state('POSTPROCESS',numerical_2ms='PASS')
                p,active=launch('render',RENDER,[]);continue
            if kind=='render':
                if result.get('status')!='RENDERED_AWAITING_VISUAL_REVIEW':
                    fail('Offscreen rendering failed; preserved numerical result');continue
                state('VISUAL_REVIEW',numerical_2ms='PASS',solver_alive=False)
                event('VISUAL_REVIEW_READY',manifest='render_manifest.json');continue
            if kind=='fallback':
                fallback=read(EVID/'fallback_result.json')
                if not fallback:
                    recovery=resume_fallback_controller(done)
                    if recovery:p,active=recovery
                elif fallback.get('status') in {'TIME_BUDGET_EXHAUSTED','STOPPED_RESOURCE_BLOCKER'}:
                    state(fallback['status'],fallback_result=fallback,solver_alive=False,worker_alive=False)
                elif fallback.get('status')!='NUMERICAL_PASS':fail('Fallback stopped at a concrete gate',fallback_result=fallback)
                else:
                    state('POSTPROCESS',route=fallback.get('route','SDOF_LOAD_PATH'),numerical_2ms='PASS')
                    p,active=launch('render',RENDER,[])
                continue
        if status in TERMINAL:
            control(status,supervisor_alive=False,active_job=None);return
        if now>=window['hard_deadline_epoch']:
            state('TIME_BUDGET_EXHAUSTED',solver_alive=False,worker_alive=False);event('HARD_DEADLINE_REACHED');continue
        if status=='STARTUP':
            if read(EVID/'launch_review.json').get('status')=='PASS':
                checked_code(WORKER);state('NATIVE_MICRORUN_25US');event('LOCAL_CAMPAIGN_ARMED');continue
            # Preparation is a finite local prerequisite, never permission/LLM waiting.
            control('STARTUP',readiness='Verified worker/report/source artifacts are being staged')
            time.sleep(5);continue
        if status=='VISUAL_REVIEW':
            vr=read(EVID/'visual_review.json')
            if vr.get('status')=='PASS':
                if native() or live_fallback_worker():raise RuntimeError('Owned solver/worker closure required before COMPLETE')
                verify_frozen();report=read(EVID/'final_report.json')
                completion=verified_completion_result(report)
                report.update(visual_review_status='PASS',visual_review_timestamp=vr.get('timestamp'),
                    visual_review_sha256=sha(EVID/'visual_review.json'),completion_result=completion)
                atomic(EVID/'final_report.json',report)
                state('COMPLETE',result=completion,route=report['route'],visual_review='PASS',solver_alive=False,worker_alive=False)
                event('COMPLETE',route=report['route'],result=completion);continue
            if vr.get('status')=='FAIL':fail('Actual visual review failed',visual_review='FAIL');continue
            time.sleep(10);continue
        if status=='DT_SELECTION':
            p,active=launch('select',REVIEW,['--select-dt']);continue
        expected={'NATIVE_MICRORUN_25US':('micro25',25e-6), 'NATIVE_DT_12P5':('micro12p5',12.5e-6),
                  'NATIVE_DT_6P25':('micro6p25',6.25e-6)}
        if status in expected or status=='RESOURCE_WAIT' or status=='NATIVE_PRODUCTION':
            stage=campaign.get('pending_stage',status) if status=='RESOURCE_WAIT' else status
            okay,m=admission();stable=(stable or time.monotonic()) if okay else None
            atomic(EVID/'resource_latest.json',m)
            with (EVID/'resource_samples.jsonl').open('a') as f:f.write(json.dumps(m)+'\n')
            seconds=time.monotonic()-stable if stable else 0
            if not stable or seconds<60:
                state('RESOURCE_WAIT',pending_stage=stage,resource=m,stable_admission_seconds=seconds);time.sleep(5);continue
            stable=None
            if stage=='NATIVE_PRODUCTION':
                chosen=campaign['selected_branch'];dt=campaign['selected_dt_s']
                meta=EVID/'branches'/resolve_branch(chosen)/'final_native_checkpoint.json'
                if not meta.exists():raise RuntimeError('Validated1.900ms native checkpoint missing')
                branch='production'+{'micro25':'25','micro12p5':'12p5','micro6p25':'6p25'}[chosen]
                branch=resolve_branch(branch);existing=EVID/'branches'/branch
                if (existing/'branch_result.json').exists() or (existing/'failure.json').exists():
                    p,active=launch('review',REVIEW,['--branch',branch],branch)
                elif (existing/'worker_identity.json').exists():
                    recovery=lifecycle_recovery(branch)
                    if not recovery:
                        fail('WORKER_LIFECYCLE_TERMINATION; no verified newer production checkpoint',branch=branch);continue
                    kind,branch=recovery
                    if kind=='review':p,active=launch('review',REVIEW,['--branch',branch],branch)
                    else:
                        review_launch(branch);p,active=launch('worker',WORKER,['--branch',branch],branch)
                else:p,active=next_worker(branch,dt,.002,stage,meta.relative_to(ROOT).as_posix())
                state(stage,active_branch=branch,production_branch=branch,selected_branch=chosen,selected_dt_s=dt,route='NATIVE_OVERWRITE')
            else:
                logical,dt=expected[stage];branch=resolve_branch(logical)
                existing=EVID/'branches'/branch
                if (existing/'branch_result.json').exists() or (existing/'failure.json').exists():
                    p,active=launch('review',REVIEW,['--branch',branch],branch)
                elif (existing/'worker_identity.json').exists():
                    recovery=lifecycle_recovery(branch)
                    if not recovery:
                        fail('WORKER_LIFECYCLE_TERMINATION; no verified newer native checkpoint',branch=branch);continue
                    kind,recovered=recovery
                    if kind=='review':p,active=launch('review',REVIEW,['--branch',recovered],recovered)
                    else:
                        review_launch(recovered);state(stage,active_branch=recovered)
                        p,active=launch('worker',WORKER,['--branch',recovered],recovered)
                else:p,active=next_worker(branch,dt,.0019,stage)
            continue
        fail('Unrecognized local FSM state',previous_status=status)

if __name__=='__main__':
    try:main()
    except Exception as e:
        atomic(EVID/'supervisor_failure.json',dict(timestamp=stamp(),error=repr(e),traceback=traceback.format_exc()))
        control('SUPERVISOR_FAILURE',supervisor_alive=False);raise
