"""Finite local coarse workflow owned by Windows Scheduler, no LLM heartbeat."""
import argparse,msvcrt,os,subprocess,sys,time,traceback
import psutil
from benchmark_C_coarse_common import *

def checked_job(script,python):
    state('PREPARATION_JOB',job=script)
    with (EVID/(Path(script).stem+'.stdout.log')).open('a') as out,(EVID/(Path(script).stem+'.stderr.log')).open('a') as err:
        p=subprocess.Popen([str(ROOT/python),'-u',str(ROOT/script)],cwd=ROOT,stdout=out,stderr=err,creationflags=subprocess.CREATE_NO_WINDOW)
        state('PREPARATION_JOB',job=script,job_pid=p.pid,job_created=psutil.Process(p.pid).create_time())
        result=p.wait()
    if result:raise RuntimeError(f'Preparation job {script} exited {result}')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--geometry-pid',type=int);ap.add_argument('--geometry-created',type=float);args=ap.parse_args()
    EVID.mkdir(parents=True,exist_ok=True);OUT.mkdir(parents=True,exist_ok=True)
    lock=(EVID/'pipeline.lock').open('a+b');lock.seek(0);lock.write(b'0');lock.flush();lock.seek(0)
    try:msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    except OSError:raise RuntimeError('Coarse pipeline already owns the campaign')
    if read(EVID/'state.json').get('status') in ['COMPLETE','HARD_BLOCKER','RESOURCE_BLOCKER']:raise RuntimeError('Terminal evidence exists; explicit reviewed continuation is needed')
    solver=None;profile=None;state('PREPARATION',execution_owner='Windows Task Scheduler',scope='Finite four static poses then 10/20/40 steps; no 2ms, no medium/fine recovery',original_fine_deadline_extended=False)
    try:
        preserve()
        if not args.geometry_pid and read(EVID/'geometry.json').get('status')!='GENERATED':
            checked_job('scripts/benchmark_C_coarse_geometry.py','.venv/Scripts/python.exe')
        while read(EVID/'geometry.json').get('status')=='RUNNING':
            if args.geometry_pid is None:raise RuntimeError('Geometry running but no known process identity')
            try:
                p=psutil.Process(args.geometry_pid)
                if abs(p.create_time()-args.geometry_created)>.01 or 'benchmark_C_coarse_geometry.py' not in ' '.join(p.cmdline()):raise RuntimeError('Geometry identity mismatch')
            except psutil.NoSuchProcess:raise RuntimeError('Geometry process disappeared before verified completion')
            state('GEOMETRY_WAIT',geometry_pid=args.geometry_pid,geometry_created=args.geometry_created,geometry_progress=read(EVID/'geometry.json').get('regions'))
            time.sleep(10)
        if read(EVID/'geometry.json').get('status')!='GENERATED':raise RuntimeError('Verified coarse geometry missing')
        if not (OUT/'validated_component.cas.h5').exists():checked_job('scripts/benchmark_C_coarse_extract_component.py','.venv-prime/Scripts/python.exe')
        if not (OUT/'coarse_raw.cas.h5').exists():checked_job('scripts/benchmark_C_coarse_prime_mesh.py','.venv-prime/Scripts/python.exe')
        checked_job('scripts/benchmark_C_coarse_component_identity.py','.venv/Scripts/python.exe')
        mesh=read(EVID/'mesh.json')
        if not mesh.get('cell_count_target_pass'):raise RuntimeError(f'Background sizing review required: total cells={mesh.get("total_cells")} > 5M')
        session=read(OUT/'private_session_connection.json') if native() else None
        if native() and not session:raise RuntimeError('Another Fluent exists without registered credentials; launch refused')
        if session:
            existing=psutil.Process(session['fluent_host_pid'])
            if abs(existing.create_time()-session['host_created'])>.01:raise RuntimeError('Original solver identity changed')
            checked_job('scripts/benchmark_C_coarse_layer_probe.py','.venv/Scripts/python.exe')
        mem=memory();state('RESOURCE_ADMISSION',memory=mem)
        if not session and (mem['available_gib']<12 or mem['commit_fraction']>.8):raise RuntimeError('Measured coarse launch RAM admission failed (12GiB available /80% commit)')
        if psutil.disk_usage(str(ROOT)).free/2**30<35:raise RuntimeError('35GiB disk admission failed')
        profile=MemoryProfile();profile.start();state('FLUENT_LAUNCH')
        import ansys.fluent.core as pyfluent
        if session:
            solver=pyfluent.connect_to_fluent(ip=session['ip'],port=session['port'],password=session['password'],cleanup_on_exit=False,start_transcript=False,start_watchdog=False)
            if int(solver.connection_properties.fluent_host_pid)!=session['fluent_host_pid']:raise RuntimeError('Reconnected host mismatch')
            if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)")))>1e-12:raise RuntimeError('Reconnection expected unadvanced static state')
        else:
            solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,ui_mode='no_gui_or_graphics',start_timeout=180,fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(OUT))
        profile.solver=solver;profile.sample('after_Fluent_launch')
        solver.transcript.start(str(EVID/'solver.trn'));messages=[];solver.transcript.register_callback(messages.append,keep_new_lines=True)
        commands=[{'pid':p.pid,'name':p.name(),'command':p.cmdline()} for p in native()]
        atomic(EVID/'serial_capability.json',{'timestamp':stamp(),'requested_processor_count':1,'additional_parallel_options_requested':False,'installed_launcher_rule':'Local single core emits no -t or -cnf (core.scheduler.build_parallel_options). Native mode recorded below; no 2/4-core trial.','actual_native_processes':commands,'true_serial':len(commands)==1 and all(not any(x.startswith('-mport') or x=='-host' or x=='-node' for x in p['command']) for p in commands)})
        from benchmark_C_coarse_static import run as static
        static(solver,profile,already_initialized=bool(session))
        from benchmark_C_coarse_dynamic import run as dynamic
        dynamic(solver,profile)
        profile.check('final_validation');preserve()
        solver.settings.mesh.check();solver.exit();solver=None;profile.close()
        from benchmark_C_coarse_report import write_report
        report=write_report()
        if report['coarse_screening']!='PASS':raise RuntimeError(report['current_blocker'])
        state('COMPLETE',final_report=str(EVID/'final_report.json'))
    except Exception as e:
        atomic(EVID/'failure.json',{'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc(),'state':read(EVID/'state.json')})
        resource=bool(profile and profile.abort) or 'RAM admission' in str(e) or 'disk admission' in str(e)
        state('RESOURCE_BLOCKER' if resource else 'HARD_BLOCKER',error=repr(e),time_steps_advanced=read(EVID/'dynamic.json').get('completed_steps',0))
        try:
            from benchmark_C_coarse_report import write_report
            final=write_report();final['current_blocker']=repr(e);atomic(EVID/'final_report.json',final)
        except Exception as report_error:atomic(EVID/'report_error.json',{'error':repr(report_error),'timestamp':stamp()})
        raise
    finally:
        if solver is not None:
            try:solver.exit()
            except Exception as e:atomic(EVID/'exit_error.json',{'timestamp':stamp(),'error':repr(e)})
        if profile is not None and profile.thread.is_alive():profile.close()

if __name__=='__main__':main()
