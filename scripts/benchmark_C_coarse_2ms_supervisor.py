"""Detached single launch owner; stops for evidence-based review, never blind retries."""
import msvcrt, os, subprocess, sys, time, traceback
import psutil
from benchmark_C_coarse_2ms_common import *

def control(status,**data):
    rec=read(EVID/'supervisor_state.json');rec.update(status=status,timestamp=stamp(),
        supervisor_pid=os.getpid(),supervisor_created=psutil.Process().create_time(),**data)
    atomic(EVID/'supervisor_state.json',rec)

def main():
    sys.stdout=(EVID/'supervisor_stdout.log').open('a',buffering=1);sys.stderr=(EVID/'supervisor_stderr.log').open('a',buffering=1)
    lock=(EVID/'supervisor.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    stable=None;worker=None;renderer=None
    config=read(EVID/'configuration.json');launch=read(EVID/'launch_review.json')
    if sha(EVID/'configuration.json')!=launch['configuration_sha256']:raise RuntimeError('Reviewed configuration changed')
    for path,digest in launch['supporting_code_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Reviewed code changed: '+path)
    previous=read(EVID/'supervisor_state.json')
    existing={'pid':previous.get('worker_pid',0),'created':previous.get('worker_created',0),'name':'pythonw.exe'}
    if identity(existing):raise RuntimeError('Existing registered worker owns this campaign')
    event('SUPERVISOR_STARTED',deadline_epoch=config['deadline_epoch'])
    control('STARTED',supervisor_alive=True)
    while True:
        now=time.time();campaign=read(EVID/'state.json')
        if worker:
            if worker.poll() is None:
                control('WORKER_RUNNING',worker_pid=worker.pid,worker_created=psutil.Process(worker.pid).create_time())
                time.sleep(10);continue
            event('WORKER_EXITED',returncode=worker.returncode,campaign_status=campaign['status'])
            control('WORKER_EXITED',worker_pid=None,worker_returncode=worker.returncode);worker=None
            if campaign['status']!='AWAITING_RENDER':
                control('AWAITING_REPAIR_REVIEW',worker_pid=None,supervisor_alive=False);return
        if campaign['status']=='AWAITING_RENDER':
            if native():
                state('AWAITING_REPAIR_REVIEW',error='Owned engine still alive before offscreen rendering')
                control('AWAITING_REPAIR_REVIEW',supervisor_alive=False);return
            if renderer is None:
                log=(EVID/'render_stdout.log').open('a');err=(EVID/'render_stderr.log').open('a')
                renderer=subprocess.Popen([str(ROOT/'.venv/Scripts/pythonw.exe'),str(ROOT/'scripts/benchmark_C_coarse_2ms_render.py')],
                    cwd=str(ROOT),stdout=log,stderr=err,creationflags=subprocess.CREATE_NO_WINDOW)
                control('RENDERING',render_pid=renderer.pid);event('RENDER_STARTED',pid=renderer.pid)
            if renderer.poll() is None:time.sleep(10);continue
            if renderer.returncode!=0:
                state('AWAITING_REPAIR_REVIEW',error='Offscreen render failed; inspect render_stderr.log')
                control('AWAITING_REPAIR_REVIEW',render_pid=None,supervisor_alive=False);return
            state('AWAITING_VISUAL_REVIEW',worker_alive=False,solver_alive=False)
            control('AWAITING_VISUAL_REVIEW',render_pid=None,supervisor_alive=False)
            event('AWAITING_VISUAL_REVIEW',manifest='render_manifest.json');return
        if campaign['status'] not in ['PREPARED','RESOURCE_WAIT']:
            control(campaign['status'],supervisor_alive=False);return
        if now>=config['deadline_epoch']:
            state('TIME_BUDGET_EXHAUSTED',solver_alive=False,worker_alive=False)
            control('TIME_BUDGET_EXHAUSTED',supervisor_alive=False);return
        ok,m=admission();stable=(stable or now) if ok else None
        state('RESOURCE_WAIT',current_step=40,memory=m,stable_admission_seconds=now-stable if stable else 0)
        control('RESOURCE_WAIT',memory=m)
        if stable is None or now-stable<60:time.sleep(10);continue
        verify_frozen()
        if native():raise RuntimeError('Existing engine: refuse second solver')
        if read(EVID/'supervisor_state.json').get('worker_launched_once'):
            raise RuntimeError('A prior launch needs a distinct reviewed repair; unchanged retry refused')
        worker=subprocess.Popen([str(ROOT/'.venv/Scripts/pythonw.exe'),str(ROOT/'scripts/benchmark_C_coarse_2ms_worker.py')],
            cwd=str(ROOT),creationflags=subprocess.CREATE_NO_WINDOW)
        control('WORKER_RUNNING',worker_pid=worker.pid,worker_created=psutil.Process(worker.pid).create_time(),worker_launched_once=True)
        event('WORKER_LAUNCHED',pid=worker.pid,one_rank=True)
        time.sleep(10)

if __name__=='__main__':
    try:main()
    except Exception as e:
        atomic(EVID/'supervisor_failure.json',{'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc()})
        control('SUPERVISOR_FAILURE',supervisor_alive=False);raise
