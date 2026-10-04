"""One owned native solver per independent branch, sequential actual transient diagnostics."""
import argparse,csv,json,math,os,shutil,sys,time,traceback
import numpy as np
import psutil,pyvista as pv
from scipy.spatial.transform import Rotation
from benchmark_D_overwrite_common import *
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_recovery_v2_restart import compare
from benchmark_C_analytic_reference import compute_magnetic_load
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_coarse_2ms_common import checkpoint_audit,identity

def collect(solver,b,library,pipe,sign):
    solver.settings.setup.user_defined.execute_on_demand(lib_name='overwrite_semantics_snapshot::'+library)
    t=float(solver.scheme.eval("(rpgetvar 'flow-time)"));actual=native_state(solver,'robot_wall')
    component=native_state(solver,'robot_component_fluid');shared=compare(component,actual)
    if any(x['status']!='PASS' for x in shared.values()):raise RuntimeError('Active/passive body state mismatch')
    solver.settings.setup.user_defined.execute_on_demand(lib_name='lifecycle_connectivity::'+library)
    connectivity=read(b/'native_connectivity_current.json');zones=connectivity['zones']
    stats={k:sum(z[k] for z in zones) for k in ['total','orphan','invalid_donors','unidentified','nonpositive_volume','nonfinite_volume']}
    stats['minimum_volume_m3']=min(z['minimum_volume_m3'] for z in zones)
    if abs(connectivity['time_s']-t)>1e-12:raise RuntimeError('Connectivity time mismatch')
    robot=surface_mesh(solver.fields.field_data,'robot_wall')
    gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign))
    q=np.array(actual['q']);F,T,source=compute_magnetic_load(actual['com'],q[[1,2,3,0]],t)
    record=dict(time_s=t,native_step_index=int(solver.scheme.eval("(rpgetvar 'time-step)")),native_state=actual,
        shared_body_errors=shared,connectivity=stats,physical_gap_m=gap,q_norm=float(np.linalg.norm(q)),
        Fmag_N=F.tolist(),Tmag_Nm=T.tolist(),source_state=source)
    reasons=[]
    if stats['total']!=4699301:reasons.append('CELL_COUNT')
    for k in ['orphan','invalid_donors','unidentified','nonpositive_volume','nonfinite_volume']:
        if stats[k]:reasons.append(k.upper())
    if stats['minimum_volume_m3']<=0:reasons.append('VOLUME')
    if abs(record['q_norm']-1)>1e-6:reasons.append('QUATERNION')
    if not np.isfinite(np.r_[actual['com'],actual['q'],actual['velocity'],actual['omega'],F,T,gap]).all():reasons.append('NONFINITE')
    if gap < -1e-9:reasons.append('PHYSICAL_PENETRATION')
    record.update(hard_failures=reasons,status='PASS' if not reasons else 'FAIL')
    name=f"state_{record['native_step_index']:04d}"
    atomic(b/(name+'.json'),record);robot.save(b/'fielddata'/(name+'_robot.vtp'))
    return record

def save_native(solver,b,o,record,label):
    import h5py
    c=o/(label+'.cas.h5');d=o/(label+'.dat.h5')
    if c.exists() or d.exists():raise RuntimeError('Independent diagnostic checkpoint overwrite refused')
    solver.settings.file.write_case(file_name=str(c));solver.settings.file.write_data(file_name=str(d))
    integrity={}
    for key,p in [('case',c),('data',d)]:
        with h5py.File(p,'r') as f:
            if not list(f):raise RuntimeError('Empty native checkpoint')
        integrity[key]=dict(path=str(p.relative_to(ROOT)),sha256=sha(p),size_bytes=p.stat().st_size,HDF5_readable=True)
    actual=native_state(solver,'robot_wall');errors=compare(actual,record['native_state'])
    if any(x['status']!='PASS' for x in errors.values()):raise RuntimeError('Native checkpoint state mismatch')
    atomic(b/(label+'_checkpoint.json'),dict(timestamp=stamp(),status='PASS',native_state=actual,time_s=record['time_s'],
        state_errors=errors,files=integrity,large_native_files_local_only=True))

def compile_and_hook(solver,b,o,config):
    library='libD_overwrite_'+config['branch']+'_v1';path=o/library
    source=ROOT/'fluent_udf/l2300_contact_overwrite_semantics.c';magnetic=o/'magnetic_logging_copy.c';header=o/'lifecycle_config.h'
    for p,key in [(source,'contact_source_sha256'),(magnetic,'magnetic_logging_copy_sha256'),(header,'configuration_header_sha256')]:
        if sha(p)!=config[key]:raise RuntimeError('Reviewed branch source/config changed')
    if sha(ROOT/'fluent_udf/l2300_contact_impulse_math.h')!=config['mathematical_kernel_sha256']:raise RuntimeError('Validated impulse kernel changed')
    solver.settings.setup.user_defined.compiled_udf(library_name=str(path),source_files=[str(source),str(magnetic)],
        header_files=[str(header),str(ROOT/'fluent_udf/l2300_contact_impulse_math.h')],use_built_in_compiler=True)
    dlls=[path/'win64'/kind/'libudf.dll' for kind in ['3ddp_host','3ddp_node']]
    latest=max(p.stat().st_mtime for p in [source,magnetic,header])
    if any(not p.is_file() or p.stat().st_mtime<latest for p in dlls):raise RuntimeError('Actual fresh host/node DLLs required')
    solver.settings.setup.user_defined.load(udf_library_name=str(path))
    nodes=solver.settings.setup.dynamic_mesh.dynamic_zones
    for zone in ['robot_wall','robot_component_fluid']:
        n=next(n for n in nodes.keys() if nodes[n].zone.get_state()==zone)
        nodes[n].motion.motion_def='lifecycle_magnetic_state_probe::'+library
    contact=solver.settings.setup.dynamic_mesh.options.contact_detection
    contact.enabled=True;contact.face_zones=['robot_wall','pipe_wall'];contact.proximity_threshold=config['threshold_m']
    contact.contact_udf='l2300_native_contact_lifecycle::'+library;contact.flow_control.enabled=False;contact.verbosity=1
    atomic(b/'compile_hook_gate.json',dict(status='PASS',timestamp=stamp(),library=library,
        source_sha256=config['contact_source_sha256'],config_sha256=sha(b/'configuration.json'),
        dll_sha256={str(p.relative_to(ROOT)):sha(p) for p in dlls},contact_state=contact.get_state(),
        magnetic_formula_delegated_once=True,true_solver_contact_only=True))
    return library

def merged_trace(b,branch):
    trace=rows(b/'native_contact_callback_trace_host.jsonl')
    node=rows(b/'native_contact_callback_trace_node.jsonl')
    prefix={'read_only':'read_only_callback_trace','noop':'noop_overwrite_trace','controlled':'controlled_overwrite_trace'}.get(branch,branch+'_contact_trace')
    for dest,data in [(EVID/(prefix+'.jsonl'),trace),(b/'native_contact_callback_trace.jsonl',trace)]:
        dest.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in data))
    if (b/'native_contact_callback_trace_host.csv').exists():shutil.copy2(b/'native_contact_callback_trace_host.csv',b/'native_contact_callback_trace.csv')
    return trace,node

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--branch',choices=BRANCHES,required=True);branch=parser.parse_args().branch
    verify_frozen()
    import msvcrt
    lock=(OUT/'semantics_worker.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    if branch=='controlled' and read(EVID/'controlled_write_admission.json').get('status')!='PASS':raise RuntimeError('Semantics and large-signal admission required')
    b,o,config=prepare_branch(branch)
    review=read(b/'launch_review.json')
    for relative,digest in review.get('files_sha256',{}).items():
        if sha(ROOT/relative)!=digest:raise RuntimeError('Reviewed launch input changed: '+relative)
    if review.get('status')!='PASS':raise RuntimeError('Concrete launch review required')
    sys.stdout=(b/'worker_stdout.log').open('a',buffering=1);sys.stderr=(b/'worker_stderr.log').open('a',buffering=1)
    atomic(b/'worker_identity.json',dict(pid=os.getpid(),created=psutil.Process().create_time(),branch=branch,timestamp=stamp()))
    import benchmark_D_native_worker as memory_module
    memory_module.EVID=b;memory_module.OUT=o
    profile=memory_module.Profile();solver=None;last=None;records=[];status='INCOMPLETE'
    try:
        state('RESOURCE_WAIT',branch=branch,worker_pid=os.getpid(),worker_created=psutil.Process().create_time(),worker_alive=True,solver_alive=False,completed_new_steps=0,current_time_s=.00175)
        stable=None
        while True:
            if native():raise RuntimeError('Existing native engine; duplicate launch refused')
            m=profile.sample();okay=m['available_gib']>=12 and m['commit_headroom_gib']>=19 and m['disk_free_gib']>=35
            stable=(stable or time.monotonic()) if okay else None
            if stable and time.monotonic()-stable>=60:break
            if read(OUT/'stop_request.json'):raise RuntimeError('Authorized diagnostic stopped before launch')
            time.sleep(5)
        verify_frozen();checkpoint=read(ROOT/config['source_checkpoint']);checkpoint_audit(checkpoint)
        import ansys.fluent.core as pyfluent
        state('NATIVE_RESTART',branch=branch,solver_alive=False)
        solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,
            ui_mode='no_gui_or_graphics',start_timeout=180,start_watchdog=False,
            fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(o))
        profile.solver=solver;profile.sample();profile.thread.start();solver.transcript.start(str(b/'solver.trn'))
        solver.settings.file.read_case(file_name=checkpoint['case']);solver.settings.file.read_data(file_name=checkpoint['data'])
        import benchmark_C_coarse_native_resume as restart
        restart.EVID=b;restart.OUT=o;restart_result=restart.validation(solver,checkpoint,70)
        atomic(b/'C70_restart_validation.json',restart_result)
        library=compile_and_hook(solver,b,o,config)
        fd=solver.fields.field_data;pipe=surface_mesh(fd,'pipe_wall');robot=surface_mesh(fd,'robot_wall');pipe.save(b/'fielddata/pipe.vtp')
        sign=float(np.sign(pv.PolyData(np.array([restart_result['native_state']['com']])).compute_implicit_distance(pipe)['implicit_distance'][0]))
        last=collect(solver,b,library,pipe,sign)
        for key,k in [('orphan','orphans'),('invalid_donors','invalid_donors'),('total','total_cells')]:
            if last['connectivity'][key]!=restart_result['overset'][k]:raise RuntimeError('Native connectivity crosscheck disagrees with official field export')
        if abs(last['connectivity']['minimum_volume_m3']-restart_result['overset']['minimum_volume_m3'])>1e-25:raise RuntimeError('Native volume audit differs from official field export')
        if last['hard_failures']:raise RuntimeError(last['hard_failures'])
        atomic(b/'native_connectivity_crosscheck.json',dict(status='PASS',timestamp=stamp(),native=last['connectivity'],official=restart_result['overset']))
        calc=solver.settings.solution.run_calculation
        if abs(float(solver.scheme.eval("(rpgetvar 'physical-time-step)"))-config['dt_s'])>1e-12:raise RuntimeError('Frozen/current branch timestep mismatch')
        calc.parameters.max_iter_per_time_step=2
        limit=5  # Exact1.750->1.875ms diagnostic interval, includes next-step start
        first_event_iteration=None
        for iteration in range(1,limit+1):
            profile.check('ACTUAL_TRANSIENT_SOLVE')
            if read(OUT/'stop_request.json'):raise RuntimeError('Stop requested at timestep boundary')
            state('WORKER_RUNNING',branch=branch,completed_new_steps=iteration-1,current_time_s=last['time_s'],solver_alive=True)
            before=dict(time_s=last['time_s'],native_state=native_state(solver,'robot_wall'))
            atomic(b/f'before_step_{iteration:02d}.json',before)
            calc.dual_time_iterate(time_step_count=1,max_iter_per_step=2)
            last=collect(solver,b,library,pipe,sign);records.append(last)
            if abs(last['time_s']-(.00175+iteration*config['dt_s']))>1e-12:raise RuntimeError('Absolute time reset/advance mismatch')
            trace,node=merged_trace(b,branch)
            atomic(b/'native_history.json',dict(branch=branch,start_time_s=.00175,dt_s=config['dt_s'],records=records))
            moving=[r for r in trace if r['moving_body'] and r['contact_face_count']>0]
            if moving and first_event_iteration is None:first_event_iteration=iteration
            if last['hard_failures']:raise RuntimeError('Dynamic hard gate: '+str(last['hard_failures']))
            profile.check('POST_TIMESTEP_AUDIT')
            event('LIFECYCLE_STEP_COMPLETE',branch=branch,time_s=last['time_s'],new_steps=iteration,actual_callback_count=len(moving),physical_gap_m=last['physical_gap_m'])
            if branch=='controlled' and first_event_iteration is not None and iteration>=first_event_iteration+1:break
        status='DIAGNOSTIC_DATA_COMPLETE' if first_event_iteration is not None else 'ACTUAL_CONTACT_NOT_TRIGGERED'
        save_native(solver,b,o,last,'final_native')
        verify_frozen()
        atomic(b/'branch_result.json',dict(status=status,branch=branch,timestamp=stamp(),actual_contact_triggered=first_event_iteration is not None,
            first_contact_completed_iteration=first_event_iteration,completed_new_steps=len(records),last_state=last,
            previous_C_and_zero_step_D_preserved=True,read_only_has_no_overwrite=branch not in ['A1','A2'] or all(not r['overwrite_called'] for r in rows(b/'native_contact_callback_trace_host.jsonl'))))
        state('AWAITING_LIFECYCLE_REVIEW',branch=branch,current_time_s=last['time_s'],completed_new_steps=len(records),solver_alive=True)
    except Exception as e:
        atomic(b/'failure.json',dict(timestamp=stamp(),branch=branch,error=repr(e),traceback=traceback.format_exc(),latest_state=last))
        if solver is not None and last is not None:
            try:
                actual=native_state(solver,'robot_wall');actual_time=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                provisional=dict(time_s=actual_time,native_state=actual,not_a_numerical_gate_PASS=True)
                atomic(b/'failure_native_state.json',provisional)
                save_native(solver,b,o,provisional,'failure_native')
            except Exception as failure:atomic(b/'failure_checkpoint_error.json',dict(error=repr(failure)))
        state('AWAITING_LIFECYCLE_REPAIR_REVIEW',branch=branch,error=repr(e));traceback.print_exc()
    finally:
        if solver is not None:
            import benchmark_C_coarse_2ms_worker as exit_module
            exit_module.EVID=b
            try:exit_module.exit_owned(solver,profile)
            except Exception as e:atomic(b/'closure_failure.json',dict(error=repr(e),timestamp=stamp()))
        profile.stop.set()
        if profile.thread.is_alive():profile.thread.join(10)
        if profile.rows:atomic(b/'memory_summary.json',dict(timestamp=stamp(),samples=len(profile.rows),
            peak_project_working_set_gib=max(r['project_working_set_gib'] for r in profile.rows),
            peak_project_private_bytes_gib=max(r['project_private_bytes_gib'] for r in profile.rows),
            peak_system_commit_percent=100*max(r['commit_fraction'] for r in profile.rows),abort=profile.abort))
        atomic(b/'worker_exit.json',dict(timestamp=stamp(),status=status,worker_pid=os.getpid(),
            solver_closed=not any(identity(r) for r in profile.owned.values())))
        state(read(EVID/'state.json').get('status','INCOMPLETE'),worker_alive=False,solver_alive=any(identity(r) for r in profile.owned.values()))

if __name__=='__main__':main()
