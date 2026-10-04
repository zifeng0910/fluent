"""One owned Fluent worker. Runs hash-reviewed D jobs sequentially, initially zero steps."""
import json, msvcrt, os, runpy, sys, threading, time, traceback
import psutil
from benchmark_D_common import *
from benchmark_C_coarse_commit_audit import system
from benchmark_C_coarse_2ms_common import identity, checkpoint_audit, mpi_node_associations

ALLOWED = {'benchmark_D_live_audit.py','benchmark_D_static_semantics.py'}

class Profile:
    def __init__(self):
        self.solver=None;self.owned={};self.abort=None;self.stage='ADMISSION';self.rows=[]
        self.lock=threading.RLock();self.stop=threading.Event()
        self.thread=threading.Thread(target=self.monitor,daemon=True)
    def register(self):
        if self.solver is None:return
        cp=self.solver.connection_properties;hosts=[]
        for pid in [cp.fluent_host_pid,cp.cortex_pid]:
            p=psutil.Process(pid);self.owned[p.pid]=dict(pid=p.pid,created=p.create_time(),name=p.name(),ownership='SDK host/cortex')
            if p.name().lower()=='fl2610.exe':hosts.append(p)
        for item in mpi_node_associations(hosts,native()):
            p=psutil.Process(item['pid']);self.owned[p.pid]={**item,'name':p.name()}
        atomic(EVID/'owned_engine.json',dict(timestamp=stamp(),worker_pid=os.getpid(),worker_created=psutil.Process().create_time(),processes=list(self.owned.values())))
    def sample(self):
        with self.lock:
            self.register();processes=[]
            for r in [dict(pid=os.getpid(),created=psutil.Process().create_time(),name=psutil.Process().name()),*self.owned.values()]:
                p=identity(r)
                if p:
                    m=p.memory_info();processes.append({**r,'working_set_gib':m.rss/2**30,'private_bytes_gib':m.private/2**30})
            rec={**system(),'stage':self.stage,'processes':processes,'disk_free_gib':psutil.disk_usage(str(ROOT)).free/2**30,
                 'project_working_set_gib':sum(p['working_set_gib'] for p in processes),'project_private_bytes_gib':sum(p['private_bytes_gib'] for p in processes)}
            if rec['available_gib']<3 or rec['commit_fraction']>=.95 or rec['project_working_set_gib']>=15 or rec['disk_free_gib']<15:self.abort='RESOURCE_HARD_STOP'
            self.rows.append(rec);atomic(EVID/'memory_latest.json',rec)
            with (EVID/'memory_samples.jsonl').open('a') as fp:fp.write(json.dumps(rec)+'\n')
            return rec
    def monitor(self):
        while not self.stop.wait(5):
            try:self.sample()
            except Exception as e:self.abort='MEMORY_MONITOR_FAILURE';atomic(EVID/'memory_failure.json',dict(error=repr(e),timestamp=stamp()))
    def check(self,stage):
        self.stage=stage
        if self.abort:raise RuntimeError(self.abort)

def main():
    initialize();sys.stdout=(EVID/'worker_stdout.log').open('a',buffering=1);sys.stderr=(EVID/'worker_stderr.log').open('a',buffering=1)
    if read(EVID/'state.json').get('status') in {'HARD_BLOCKER_REQUIRES_REVIEW','NATIVE_CONTACT_BASELINE_NOT_VIABLE','COMPLETE'}:
        print('Terminal Benchmark D state requires an explicit reviewed configuration before launch.',flush=True)
        return
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    solver=None;profile=Profile()
    try:
        config=read(EVID/'configuration.json')
        if read(EVID/'contact_impulse_unit_validation.json').get('status')!='PASS':raise RuntimeError('Offline impulse PASS required')
        for p,digest in config['frozen_files_sha256'].items():
            if sha(ROOT/p)!=digest:raise RuntimeError('Frozen evidence changed: '+p)
        if native():raise RuntimeError('Existing native engine; duplicate launch refused')
        state('RESOURCE_WAIT',worker_pid=os.getpid(),worker_created=psutil.Process().create_time(),worker_alive=True,solver_alive=False)
        stable=None
        while True:
            m=profile.sample()
            okay=m['available_gib']>=12 and m['commit_headroom_gib']>=19 and m['disk_free_gib']>=35
            stable=(stable or time.monotonic()) if okay else None
            if stable is not None and time.monotonic()-stable>=60:break
            if read(OUT/'stop_request.json'):raise RuntimeError('STOP_REQUESTED_BEFORE_LAUNCH')
            time.sleep(5)
        checkpoint=read(ROOT/config['source_checkpoint_metadata']);checkpoint_audit(checkpoint)
        if native():raise RuntimeError('Another native engine appeared during admission')
        import ansys.fluent.core as pyfluent
        state('NATIVE_RESTART',worker_alive=True,solver_alive=False);profile.stage='NATIVE_RESTART'
        solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,
            ui_mode='no_gui_or_graphics',start_timeout=180,start_watchdog=False,
            fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(OUT))
        profile.solver=solver;profile.sample();profile.thread.start()
        solver.transcript.start(str(EVID/'solver.trn'))
        solver.settings.file.read_case(file_name=checkpoint['case']);solver.settings.file.read_data(file_name=checkpoint['data'])
        import benchmark_C_coarse_native_resume as restart
        restart.EVID=EVID;restart.OUT=OUT
        rec=restart.validation(solver,checkpoint,70)
        atomic(EVID/'step70_restart_continuity.json',{**rec,'milestone':'BENCHMARK_D_STEP70_RESTART_PASS'})
        profile.check('READY');state('READY',solver_alive=True,worker_alive=True,time_s=rec['native_time_s'],timesteps_advanced=0)
        event('STEP70_NATIVE_RESTART_PASS',time_s=rec['native_time_s'])
        consumed=set();last_activity=time.monotonic()
        while not read(OUT/'stop_request.json'):
            profile.check('READY')
            command=read(OUT/'session_command.json');command_id=command.get('command_id')
            if command_id and command_id not in consumed:
                job=ROOT/command['job']
                if job.parent!=ROOT/'scripts' or job.name not in ALLOWED or sha(job)!=command['job_sha256']:raise RuntimeError('Unreviewed D job refused')
                consumed.add(command_id);atomic(EVID/'active_job.json',{**command,'status':'RUNNING','timestamp':stamp()})
                state('ZERO_DYNAMICS_JOB_RUNNING',active_job=command['job'])
                before=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                try:
                    runpy.run_path(str(job))['run'](solver,dict(root=ROOT,evidence=EVID,out=OUT,profile=profile,command=command))
                except Exception as e:
                    after=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                    if abs(after-before)>1e-12:raise RuntimeError('Failed diagnostic advanced dynamics') from e
                    failure={**command,'status':'FAILED','timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc(),'time_before_s':before,'time_after_s':after}
                    atomic(EVID/'jobs'/str(command_id)/'failure.json',failure)
                    atomic(EVID/'active_job.json',failure)
                    state('AWAITING_REPAIR_REVIEW',active_job=command['job'],error=repr(e),solver_alive=True,worker_alive=True)
                    event('ZERO_DYNAMICS_JOB_FAILED',command_id=command_id,error=repr(e))
                    last_activity=time.monotonic()
                    continue
                after=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                if abs(after-before)>1e-12:raise RuntimeError('Diagnostic job advanced time')
                atomic(EVID/'active_job.json',{**command,'status':'COMPLETE','timestamp':stamp(),'time_before_s':before,'time_after_s':after})
                state('READY',active_job=None,time_s=after,timesteps_advanced=0);event('ZERO_DYNAMICS_JOB_COMPLETE',command_id=command_id,job=command['job'])
                last_activity=time.monotonic()
            if time.monotonic()-last_activity>5400:raise RuntimeError('IDLE_AUDIT_WORKER_CLOSED')
            time.sleep(1)
    except Exception as e:
        atomic(EVID/'worker_failure.json',dict(timestamp=stamp(),error=repr(e),traceback=traceback.format_exc()))
        state('AWAITING_REPAIR_REVIEW',error=repr(e));traceback.print_exc()
    finally:
        if solver is not None:
            # Reuse exact PID/creation verified closure; output root explicitly scoped to D.
            import benchmark_C_coarse_2ms_worker as exit_module
            exit_module.EVID=EVID
            try:exit_module.exit_owned(solver,profile)
            except Exception as e:atomic(EVID/'closure_failure.json',dict(error=repr(e),timestamp=stamp()))
        profile.stop.set()
        if profile.thread.is_alive():profile.thread.join(10)
        state(read(EVID/'state.json').get('status','STOPPED'),worker_alive=False,solver_alive=any(identity(x) for x in profile.owned.values()))
        if profile.rows:atomic(EVID/'memory_summary.json',dict(timestamp=stamp(),samples=len(profile.rows),
            peak_project_working_set_gib=max(x['project_working_set_gib'] for x in profile.rows),
            peak_project_private_bytes_gib=max(x['project_private_bytes_gib'] for x in profile.rows),
            peak_system_commit_percent=100*max(x['commit_fraction'] for x in profile.rows),abort=profile.abort))

if __name__=='__main__':main()
