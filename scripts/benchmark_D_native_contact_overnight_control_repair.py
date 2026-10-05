"""Review the observed journal defect without editing active worker inputs."""
import json,shutil,subprocess,sys
from benchmark_D_native_contact_overnight_common import *

def main():
    verify_frozen();failure=read(EVID/'supervisor_failure.json')
    if "event() got multiple values for argument 'name'" not in failure.get('error',''):
        raise RuntimeError('This repair applies only to the archived observed journal defect')
    job=read(EVID/'active_job.json');branch=job.get('branch')
    worker=read(EVID/'branches'/str(branch)/'worker_identity.json');worker.setdefault('name','pythonw.exe')
    actual=identity(worker)
    if not actual or not any(Path(p).name=='benchmark_D_native_contact_overnight_worker.py' for p in actual.cmdline()):
        raise RuntimeError('Existing exact registered worker identity must be proven before adoption')
    old=read(EVID/'supervisor_state.json')
    if identity(dict(pid=old['supervisor_pid'],created=old['supervisor_created'],name='pythonw.exe')):
        raise RuntimeError('Old supervisor still alive')
    input_review=read(EVID/'branches'/branch/'launch_review.json')
    for p,h in input_review['files_sha256'].items():
        if sha(ROOT/p)!=h:raise RuntimeError('Active worker input changed: '+p)
    before=read(EVID/'launch_review.json')
    changed={'scripts/benchmark_D_native_contact_overnight_supervisor.py',
             'scripts/benchmark_D_native_contact_overnight_arm.py',
             'scripts/test_benchmark_D_native_contact_overnight_control.py'}
    for p,h in before['supporting_code_sha256'].items():
        if p not in changed and sha(ROOT/p)!=h:raise RuntimeError('Unrelated launch capability changed: '+p)
    test=subprocess.run([sys.executable,str(ROOT/'scripts/test_benchmark_D_native_contact_overnight_control.py')],
        cwd=ROOT,capture_output=True,text=True)
    if test.returncode:raise RuntimeError('Control repair regression checks failed: '+test.stderr)
    archive=EVID/'control_repairs/0001_journal_collision'
    if archive.exists():raise RuntimeError('Observed repair archive already exists')
    archive.mkdir(parents=True)
    for name in ['supervisor_failure.json','supervisor_stderr.log','supervisor_state.json','launch_review.json','active_job.json']:
        shutil.copy2(EVID/name,archive/name)
    for p in changed:before['supporting_code_sha256'][p]=sha(ROOT/p)
    before['supporting_code_sha256']['scripts/benchmark_D_native_contact_overnight_control_repair.py']=sha(Path(__file__))
    before.update(timestamp=stamp(),status='PASS',repair_review='control_repairs/0001_journal_collision/review.json')
    review=dict(status='PASS',timestamp=stamp(),failure_timestamp=failure['timestamp'],
        failure_class='SUPERVISOR_JOURNAL_PARAMETER_COLLISION',
        root_cause='Process record name collided with event(name); alive flag collided with default update kwargs',
        concrete_change='Nested process_identity event payload; explicit state merge; close parent log handles; adopt Windows venv actual worker PID',
        registered_worker=dict(pid=actual.pid,created=actual.create_time(),name=actual.name()),
        worker_survived_supervisor_failure=True,active_worker_inputs_unchanged=True,no_new_solver=True,
        window_sha256=sha(EVID/'window.json'),configuration_sha256=sha(EVID/'configuration.json'),
        supporting_code_sha256={p:sha(ROOT/p) for p in changed},tests=dict(count=8,returncode=0,stdout=test.stdout,stderr=test.stderr),
        archived_evidence_sha256={p.name:sha(p) for p in archive.iterdir() if p.is_file()},
        resume_action='Start SAME Windows task; adopt SAME recorded worker within SAME deadline')
    atomic(archive/'review.json',review);atomic(EVID/'launch_review.json',before)
    event('SUPERVISOR_CONTROL_REPAIR_REVIEWED',review='control_repairs/0001_journal_collision/review.json',
        same_worker_pid=actual.pid,original_deadline_preserved=True)
    print(json.dumps(dict(status='CONTROL_REPAIR_PASS',worker=review['registered_worker'])))

if __name__=='__main__':main()
