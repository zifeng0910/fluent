"""Single local launch owner; consumes hashed plans inside one immutable 20h window."""
import argparse, json, msvcrt, os, subprocess, sys, time, traceback
from datetime import datetime,timezone
import psutil
from benchmark_C_coarse_common import ROOT,EVID,atomic,read,sha,stamp,native
from benchmark_C_coarse_commit_audit import system

CONT=EVID/'continuation_20h'
TERMINAL={'COMPLETE','TIME_BUDGET_EXHAUSTED','HARD_BLOCKER_REQUIRES_REVIEW'}

def event(kind,**data):
    with (CONT/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps({'timestamp':stamp(),'event':kind,**data})+'\n')

def set_state(status,**data):
    rec=read(CONT/'state.json');rec.update(status=status,timestamp=stamp(),supervisor_pid=os.getpid(),supervisor_created=psutil.Process().create_time(),**data)
    atomic(CONT/'state.json',rec);return rec

def admission(m):
    return m['available_gib']>=12 and m['commit_headroom_gib']>=19 and psutil.disk_usage(str(ROOT)).free/2**30>=35

def validate_plan(p):
    window=read(CONT/'window.json');state=read(CONT/'state.json')
    if p['window_id']!=window['window_id'] or p['failure_timestamp']!=state['failure_timestamp']:raise RuntimeError('Stale or different window plan')
    if p['decision'] not in ['DIAGNOSE','REPAIR','HARD_BLOCKER']:raise RuntimeError('Unknown plan decision')
    for rel in p['evidence_files']:
        path=(ROOT/rel).resolve();path.relative_to(ROOT.resolve())
        if not path.is_file():raise RuntimeError('Missing plan evidence')
    if p['decision']=='HARD_BLOCKER':return
    path=(ROOT/p['job']).resolve();path.relative_to((ROOT/'scripts').resolve())
    if not path.is_file() or sha(path)!=p['job_sha256']:raise RuntimeError('Job source not reviewed')
    # Only this proven continuation worker may launch a solver. Diagnostics are read-only audits.
    allowed={'scripts/benchmark_C_coarse_native_resume.py','scripts/benchmark_C_coarse_commit_audit.py'}
    if p['job'] not in allowed:raise RuntimeError('New job needs supervisor capability review; not allowlisted')
    if p['job'].endswith('native_resume.py'):
        if p['decision']!='REPAIR' or not p.get('concrete_change'):raise RuntimeError('Concrete resource change required')
        config_path=(ROOT/p['config']).resolve();config_path.relative_to(CONT.resolve())
        if sha(config_path)!=p['configuration_sha256']:raise RuntimeError('Configuration hash differs')
        config=read(config_path)
        if config.get('target_step')!=40 or config.get('processor_count')!=1 or config.get('ui_mode')!='no_gui_or_graphics':raise RuntimeError('Frozen scope violation')
        for rel,digest in p.get('supporting_code_sha256',{}).items():
            if sha(ROOT/rel)!=digest:raise RuntimeError('Supporting reviewed code changed')
        used=state.get('repairs',[])
        if sum(x['failure_class']==p['failure_class'] for x in used)>=2:raise RuntimeError('Two distinct repairs exhausted for this failure class')
        if any(x['concrete_change']==p['concrete_change'] for x in used):raise RuntimeError('Unchanged retry forbidden')

def main():
    CONT.mkdir(exist_ok=True)
    sys.stdout=(CONT/'supervisor_stdout.log').open('a',buffering=1);sys.stderr=(CONT/'supervisor_stderr.log').open('a',buffering=1)
    lock=(CONT/'supervisor.lock').open('a+b');lock.seek(0);lock.write(b'0');lock.flush();lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    window=read(CONT/'window.json')
    if not window or window['duration_hours']!=20:raise RuntimeError('Immutable authorized 20-hour window required')
    old=read(CONT/'state.json')
    if old.get('status') in TERMINAL:return
    # A restarted supervisor never blindly relaunches if an old worker/engine remains.
    if old.get('worker_pid'):
        try:
            p=psutil.Process(old['worker_pid'])
            if abs(p.create_time()-old['worker_created'])<.01:
                set_state('AWAITING_REPAIR_REVIEW',error='Previous owned worker is still alive; monitor and review before supervisor resume');return
        except psutil.Error:pass
    state=set_state(old.get('status','RESOURCE_WAIT'),deadline_epoch=window['deadline_epoch'])
    child=None;active=None;stable_since=None;last_audit=0
    event('SUPERVISOR_STARTED',deadline_epoch=window['deadline_epoch'])
    while True:
        state=read(CONT/'state.json');now=time.time();m=system()
        if child is not None:
            if child.poll() is None:
                set_state('WORKER_RUNNING' if now<window['deadline_epoch'] else 'DEADLINE_GRACEFUL_CHECKPOINT_WAIT',memory=m,worker_pid=child.pid,worker_created=state['worker_created'],campaign_progress=read(EVID/'state.json'))
                # Do not kill a solver during an RPC; worker checkpoints at the next safe boundary.
                time.sleep(10);continue
            code=child.returncode;campaign=read(EVID/'state.json')
            event('WORKER_EXITED',returncode=code,campaign_status=campaign.get('status'),plan_id=active['plan_id'])
            child=None
            if campaign.get('status')=='COMPLETE' and not native():
                set_state('COMPLETE',campaign_progress=campaign,worker_pid=None,completed_at=stamp());return
            failure_timestamp=stamp()
            set_state('AWAITING_REPAIR_REVIEW',worker_pid=None,failure_timestamp=failure_timestamp,
                      campaign_progress=campaign,last_returncode=code,last_plan_id=active['plan_id'],error='Read archived failure and diagnose before a distinct repair')
            active=None;stable_since=None
        if now>=window['deadline_epoch']:
            set_state('TIME_BUDGET_EXHAUSTED',worker_pid=None,campaign_progress=read(EVID/'state.json'));event('DEADLINE_REACHED');return
        if now-last_audit>=60:
            with (CONT/'resource_samples.jsonl').open('a') as f:f.write(json.dumps(m)+'\n')
            last_audit=now
        plan_path=CONT/'repair_plan.json';plan=read(plan_path)
        if plan and plan.get('plan_id') not in state.get('consumed_plan_ids',[]):
            try:validate_plan(plan)
            except Exception as e:
                set_state('AWAITING_REPAIR_REVIEW',plan_rejection=repr(e),memory=m);time.sleep(10);continue
            if plan['decision']=='HARD_BLOCKER':
                set_state('HARD_BLOCKER_REQUIRES_REVIEW',reason=plan['root_cause']);event('HARD_BLOCKER',reason=plan['root_cause']);return
            if native():
                set_state('AWAITING_REPAIR_REVIEW',error='Native Fluent already exists; launch refused',memory=m);time.sleep(10);continue
            if plan['decision']=='REPAIR':
                stable_since=(stable_since or now) if admission(m) else None
                set_state('RESOURCE_WAIT',memory=m,stable_admission_seconds=now-stable_since if stable_since else 0,
                    admission={'physical_available_gib_min':12,'commit_headroom_gib_min':19,'disk_free_gib_min':35,'stable_seconds':60},
                    admission_basis='Prior resource-stop commit 45.84GiB vs postexit 29.27GiB =16.57GiB observed system increase; reserve19GiB including margin. Runtime3GiB/95% guards unchanged.')
                if stable_since is None or now-stable_since<60:time.sleep(10);continue
            command=[str(ROOT/'.venv/Scripts/pythonw.exe'),str(ROOT/plan['job'])]
            if plan.get('config'):command+=['--config',str(ROOT/plan['config'])]
            child=subprocess.Popen(command,cwd=str(ROOT),creationflags=subprocess.CREATE_NO_WINDOW)
            created=psutil.Process(child.pid).create_time();active=plan
            repairs=state.get('repairs',[])
            if plan['decision']=='REPAIR':repairs=repairs+[{'plan_id':plan['plan_id'],'failure_class':plan['failure_class'],'concrete_change':plan['concrete_change'],'timestamp':stamp()}]
            set_state('WORKER_RUNNING',worker_pid=child.pid,worker_created=created,consumed_plan_ids=state.get('consumed_plan_ids',[])+[plan['plan_id']],repairs=repairs,plan_rejection=None)
            atomic(CONT/(plan['plan_id']+'_consumed_plan.json'),plan)
            event('PLAN_DISPATCHED',plan_id=plan['plan_id'],job=plan['job'],worker_pid=child.pid)
        time.sleep(10)

if __name__=='__main__':
    try:main()
    except Exception as e:
        atomic(CONT/'supervisor_failure.json',{'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc()});raise
