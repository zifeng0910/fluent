"""Resume only the verified, still-live step28 engine under unchanged guards."""
import csv,msvcrt,psutil,time,traceback,sys
from pathlib import Path
# The scheduler starts pythonw directly; raw files avoid a console/shell parent.
_logs=Path(__file__).resolve().parents[1]/'evidence/benchmark_C_coarse_A'
sys.stdout=(_logs/'scheduler_resume_0028_stdout.log').open('a',buffering=1)
sys.stderr=(_logs/'scheduler_resume_0028_stderr.log').open('a',buffering=1)
from benchmark_C_coarse_common import *
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_recovery_v2_restart import compare,state_vector

def main():
    lock=(EVID/'pipeline.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    old=read(EVID/'state.json');cp=read(OUT/'private_session_connection.json');checkpoint=read(EVID/'checkpoint_0028.json')
    if old['status']!='RESOURCE_BLOCKER' or psutil.pid_exists(old['pid']):raise RuntimeError('Previous worker is not stopped')
    if checkpoint.get('timesteps_advanced')!=0 or checkpoint['status']!='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS':raise RuntimeError('Verified checkpoint missing')
    if any(sha(Path(checkpoint[k]))!=checkpoint[k+'_sha256'] for k in ['case','data']):raise RuntimeError('Checkpoint hash changed')
    host=psutil.Process(cp['fluent_host_pid'])
    if abs(host.create_time()-cp['host_created'])>.01:raise RuntimeError('Original engine changed; launch forbidden')
    preserve();samples=[]
    state('RESOURCE_RECHECK',execution_owner='Windows Task Scheduler / direct pythonw worker',current_step=28,target_step=40,pose=None,error=None,job='scripts/benchmark_C_coarse_resume_resource.py',job_pid=psutil.Process().pid,job_created=psutil.Process().create_time(),review='resource_resume_0028_review.json')
    for i in range(13):
        sample=memory();samples.append(sample)
        atomic(EVID/'resource_recheck_0028.json',{'timestamp':stamp(),'samples':samples,'admission':'All 13 samples over >=60s: available >=4GiB and commit <=90%; original runtime guards 3GiB/95%/15GiB unchanged'})
        if i<12:time.sleep(5)
    if any(x['available_gib']<4 or x['commit_fraction']>.90 for x in samples):
        state('RESOURCE_BLOCKER',error='Stable resource admission failed; no timesteps advanced',current_step=28);return
    if psutil.disk_usage(str(ROOT)).free/2**30<35:raise RuntimeError('35GiB disk guard')
    import ansys.fluent.core as pyfluent
    solver=pyfluent.connect_to_fluent(ip=cp['ip'],port=cp['port'],password=cp['password'],cleanup_on_exit=True,start_transcript=False,start_watchdog=False)
    profile=MemoryProfile();profile.solver=solver;profile.start()
    try:
        if int(solver.connection_properties.fluent_host_pid)!=host.pid:raise RuntimeError('Wrong engine')
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-.0007)>1e-12:raise RuntimeError('Native time differs')
        expected=state_vector(checkpoint['row'])
        errors={z:compare(native_state(solver,z),expected) for z in ['robot_wall','robot_component_fluid']}
        if any(v['status']!='PASS' for group in errors.values() for v in group.values()):raise RuntimeError('Native COM/orientation/velocity changed')
        atomic(EVID/'resource_resume_0028_review.json',{'timestamp':stamp(),'decision':'PASS','native_host_pid':host.pid,'native_node_pid':40656,'new_solver_launched':False,'case_or_data_reloaded':False,'native_time_s':.0007,'native_errors':errors,'runtime_guards_unchanged':True,'stable_resources_verified':True,'frozen_file_hashes_verified':True})
        solver.transcript.start(str(EVID/'solver_resume_0028.trn'))
        from benchmark_C_coarse_dynamic import run
        run(solver,profile,resume_step=28)
        preserve();solver.settings.mesh.check()
        solver.exit();solver=None;profile.close()
        from benchmark_C_coarse_report import write_report
        report=write_report()
        if report['coarse_screening']!='PASS':raise RuntimeError('Final screening gate failed')
        state('COMPLETE',current_step=40,target_step=40,final_report=str(EVID/'final_report.json'))
    except Exception as e:
        atomic(EVID/'resume_0028_failure.json',{'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc(),'dynamic':read(EVID/'dynamic.json')})
        state('RESOURCE_BLOCKER' if str(e)=='COARSE_RESOURCE_LIMIT' else 'HARD_BLOCKER',error=repr(e),current_step=read(EVID/'dynamic.json').get('completed_steps'))
        if solver is not None:
            step=read(EVID/'dynamic.json').get('completed_steps',28)
            c=OUT/f'resume_stop_{step:04d}.cas.h5';d=OUT/f'resume_stop_{step:04d}.dat.h5'
            if not c.exists() and not d.exists():
                solver.settings.file.write_case(file_name=str(c));solver.settings.file.write_data(file_name=str(d))
                atomic(EVID/'resume_stop_checkpoint.json',{'step':step,'case':str(c),'data':str(d),'case_sha256':sha(c),'data_sha256':sha(d),'native_state':native_state(solver,'robot_wall'),'status':'SAVED_VALIDATION_PENDING'})
        raise
    finally:
        if solver is not None:solver.exit()
        if profile.thread.is_alive():profile.close()
        from benchmark_C_coarse_report import write_report
        write_report()

if __name__=='__main__':main()
