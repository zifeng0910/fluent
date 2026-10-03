"""Close only proven stale registered processes after verified full1ms completion."""
import argparse,json,os,sys,time,traceback
import psutil
from benchmark_C_coarse_common import ROOT,EVID,read,atomic,stamp,sha,native,preserve,state
from benchmark_C_coarse_commit_audit import system

def identity(record):
    try:
        p=psutil.Process(record['pid'])
        if abs(p.create_time()-record['created'])>.01 or p.name()!=record['name']:raise RuntimeError('Registered PID reused or process identity changed')
        return p
    except psutil.NoSuchProcess:return None

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);args=ap.parse_args()
    config=read(args.config);cont=EVID/'continuation_20h';path=cont/'finalization_result.json'
    if config['action']!='CLOSE_PROVEN_STALE_COMPLETED_ENGINE':raise RuntimeError('Unknown finalization action')
    audit=read(ROOT/config['audit'])
    if sha(ROOT/config['audit'])!=config['audit_sha256'] or audit['status']!='PROVEN_STALE_COMPLETED_OWNED_ENGINE':raise RuntimeError('Staleness review changed')
    preserve(); campaign=read(EVID/'state.json');check=read(EVID/'checkpoint_0040.json');verification=read(EVID/'step40_checkpoint_verification.json')
    if campaign['status']!='COMPLETE' or verification['status']!='PASS' or read(EVID/'dynamic.json')['completed_steps']!=40:raise RuntimeError('Full verified1ms required')
    if psutil.pid_exists(campaign['job_pid']):raise RuntimeError('Dynamics worker still alive')
    for k in ['case','data']:
        if sha(check[k])!=check[k+'_sha256']:raise RuntimeError('Step40 checkpoint hash changed')
    records=audit['owned_processes']
    expected={r['pid'] for r in records if r['name'].lower() in ['fl2610.exe','fl_mpi2610.exe']}
    if {p.pid for p in native()}-expected:raise RuntimeError('Other native engine exists; refuse broad cleanup')
    before=system();cpu={}
    for r in records:
        p=identity(r)
        if p:cpu[p.pid]=sum(p.cpu_times()[:2])
    time.sleep(5)
    deltas={}
    for r in records:
        p=identity(r)
        if p:
            deltas[p.pid]=sum(p.cpu_times()[:2])-cpu[p.pid]
            if deltas[p.pid]>.25:raise RuntimeError('Registered completed engine is not idle; do not terminate')
    rec={'timestamp':stamp(),'status':'RUNNING','action':config['action'],'before':before,'idle_CPU_seconds_over5s':deltas,
         'timesteps_advanced':0,'new_solver_launched':False,'connection_attempted':False,'soft_exit_already_attempted_by_completed_worker':True,
         'original_server_info_deleted':not audit['server_info_file_exists'],'terminated':[],'checkpoint40_hashes_verified':True,'unrelated_processes_touched':[]}
    atomic(path,rec)
    # Exact recorded identities only; never recurse into unknown descendants.
    order={'fl_mpi2610.exe':0,'fl2610.exe':1,'cx2610.exe':2}
    for r in sorted(records,key=lambda r:order[r['name'].lower()]):
        p=identity(r)
        if p:
            private=p.memory_info().private/2**30;p.terminate()
            try:p.wait(timeout=10)
            except psutil.TimeoutExpired:raise RuntimeError('Exact owned process did not terminate')
            rec['terminated'].append({'pid':r['pid'],'created':r['created'],'name':r['name'],'private_bytes_gib':private})
            atomic(path,rec)
    if native():raise RuntimeError('Native engine still exists after scoped closure')
    rec.update(status='PASS',timestamp=stamp(),after=system(),owned_engine_closed=True)
    rec['system_commit_released_gib']=before['commit_gib']-rec['after']['commit_gib']
    atomic(path,rec)
    state('COMPLETE',current_step=40,time_steps_advanced=40,time_s=.001,solver_alive=False,worker_alive=False,scheduler_enabled=False,memory=rec['after'],failure_file=None,error=None,review='final_completion_verification.json',next_state='COARSE_DEVELOPMENT_BASELINE')

if __name__=='__main__':
    try:main()
    except Exception as e:
        atomic(EVID/'continuation_20h/finalization_job_failure.json',{'timestamp':stamp(),'status':'FAIL','error':repr(e),'traceback':traceback.format_exc()})
        raise
