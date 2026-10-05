"""Reviewed future-stage control repair; preserves the existing CFD worker."""
import ast,json,shutil,subprocess,sys,time
import psutil
from benchmark_D_native_contact_overnight_common import *

CHANGED={'scripts/benchmark_D_native_contact_overnight_supervisor.py',
         'scripts/benchmark_D_native_contact_overnight_render.py',
         'scripts/test_benchmark_D_native_contact_overnight_control.py'}
ADDED={'scripts/benchmark_D_native_contact_overnight_fallback.py',
       'fluent_udf/l2300_load_contact_overnight.c',
       'scripts/test_benchmark_D_native_contact_overnight_fallback.py',
       'scripts/benchmark_D_native_contact_overnight_activation_audit.py',
       'scripts/benchmark_D_native_contact_overnight_capability_upgrade.py'}


def actual_owners(branch):
    worker=read(EVID/'branches'/branch/'worker_identity.json');worker.setdefault('name','pythonw.exe')
    p=identity(worker)
    if not p or not any(Path(arg).name=='benchmark_D_native_contact_overnight_worker.py' for arg in p.cmdline()):
        raise RuntimeError('Actual registered active worker identity missing')
    result=[dict(pid=p.pid,created=p.create_time(),name=p.name())]
    for entry in read(EVID/'branches'/branch/'owned_engine.json')['processes']:
        process=psutil.Process(entry['pid'])
        if abs(process.create_time()-entry['created'])>=.01:
            raise RuntimeError('Native process creation identity changed')
        result.append(dict(pid=process.pid,created=process.create_time(),name=process.name()))
    return result


def main():
    verify_frozen();window=read(EVID/'window.json')
    if time.time()>=window['hard_deadline_epoch']:raise RuntimeError('Original hard deadline expired')
    archive=EVID/'control_repairs/0002_conditional_route_capability'
    if archive.exists():raise RuntimeError('This reviewed capability upgrade already exists; do not rerun')
    old=read(EVID/'launch_review.json');supervisor=read(EVID/'supervisor_state.json');active=read(EVID/'active_job.json')
    if active.get('kind')!='worker':raise RuntimeError('This upgrade requires the existing native worker')
    branch=active['branch'];sup=identity(dict(pid=supervisor['supervisor_pid'],created=supervisor['supervisor_created'],name='pythonw.exe'))
    if not sup or not any(Path(arg).name=='benchmark_D_native_contact_overnight_supervisor.py' for arg in sup.cmdline()):
        raise RuntimeError('Exact registered supervisor identity mismatch')
    branch_review=read(EVID/'branches'/branch/'launch_review.json')
    for path,digest in branch_review['files_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Active worker input changed: '+path)
    for path,digest in old['supporting_code_sha256'].items():
        if path not in CHANGED and sha(ROOT/path)!=digest:raise RuntimeError('Unreviewed supporting-code change: '+path)
    render_review=read(EVID/'capability_review/render_route_compatibility_review.json')
    if sha(ROOT/'scripts/benchmark_D_native_contact_overnight_render.py')!=render_review['proposed_script_sha256_utf8_LF']:
        raise RuntimeError('Applied renderer differs from independently reviewed patch')
    tests=[]
    for script,count in [('scripts/test_benchmark_D_native_contact_overnight_control.py',13),
                         ('scripts/test_benchmark_D_native_contact_overnight_fallback.py',3)]:
        tested=subprocess.run([sys.executable,str(ROOT/script)],cwd=ROOT,capture_output=True,text=True)
        if tested.returncode:raise RuntimeError('Specific control/interface regression failed: '+tested.stderr)
        tests.append(dict(script=script,count=count,status='PASS',returncode=0,stderr=tested.stderr))
    for path in CHANGED|ADDED:
        if path.endswith('.py'):ast.parse((ROOT/path).read_text())
    fallback=read(EVID/'fallback_launch_review.json')
    for key,path in [('job_sha256',fallback['job']),('source_sha256',fallback['source']),('worker_sha256',fallback['worker'])]:
        if sha(ROOT/path)!=fallback[key]:raise RuntimeError('Future fallback capability hash is stale: '+path)
    if fallback.get('status')!='PASS':raise RuntimeError('Conditional fallback review absent')
    source_review=read(EVID/'fallback_source_review.json')
    if source_review.get('actual_source_sha256')!=fallback['source_sha256']:
        raise RuntimeError('Independent LOAD source math review is stale')
    before=actual_owners(branch)
    archive.mkdir(parents=True)
    for name in ['launch_review.json','supervisor_state.json','active_job.json']:
        shutil.copy2(EVID/name,archive/name)
    new={**old,'timestamp':stamp(),'status':'PASS',
         'conditional_route_capability_review':'control_repairs/0002_conditional_route_capability/review.json'}
    new['supporting_code_sha256']={**old['supporting_code_sha256'],**{path:sha(ROOT/path) for path in CHANGED|ADDED}}
    record=dict(status='PASS_REVIEWED_RELOAD_PENDING',timestamp=stamp(),
        failure_class='FUTURE_ROUTE_SCHEMA_AND_CONTROLLER_CHILD_OWNERSHIP',
        root_cause='Fallback result rejected by native-only renderer; COMPLETE mislabeled native; helper death could leave an admitting child; root live flags retained stale admission data',
        concrete_change='Route-specific numerical/render completion gates and exact surviving-child adoption; mirror active worker progress; retain original task/window and same CFD worker',
        superseded_controller=dict(pid=sup.pid,created=sup.create_time(),name=sup.name(),
            loaded_revision_sha256=old['supporting_code_sha256']['scripts/benchmark_D_native_contact_overnight_supervisor.py']),
        new_controller_sha256=new['supporting_code_sha256']['scripts/benchmark_D_native_contact_overnight_supervisor.py'],
        existing_worker_and_engine_identities=before,active_worker_inputs_sha256=branch_review['files_sha256'],
        active_worker_inputs_unchanged=True,worker_or_engine_termination_authorized=False,
        target_process_is_only_exact_superseded_controller=True,tests=tests,
        renderer_independent_guard_cases=len(render_review['pure_guard_tests']),
        conditional_fallback_not_activated=True,no_measured_LOAD_or_full2ms_PASS_claim=True,
        window_sha256=sha(EVID/'window.json'),configuration_sha256=sha(EVID/'configuration.json'),
        supporting_code_sha256={path:new['supporting_code_sha256'][path] for path in CHANGED|ADDED},
        original_immutable_window_preserved=True)
    atomic(archive/'review.json',record);atomic(EVID/'launch_review.json',new)
    # Only the recorded superseded controller is stopped. Its exact worker and
    # native engines survive; stopping the Scheduler task/tree is not used.
    if not identity(record['superseded_controller']):raise RuntimeError('Superseded controller identity changed before reload')
    sup.terminate();sup.wait(timeout=10)
    if actual_owners(branch)!=before:raise RuntimeError('Worker/engine ownership changed during controller reload')
    for path,digest in branch_review['files_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Active input changed during controller reload')
    for _ in range(20):
        task=subprocess.run(['powershell.exe','-NoProfile','-Command',
            "(Get-ScheduledTask -TaskName 'Fluent-Benchmark-D-Native-Contact-Overnight').State.ToString()"],
            capture_output=True,text=True,check=True)
        if task.stdout.strip()!='Running':break
        time.sleep(.5)
    else:raise RuntimeError('Old task has not closed; no duplicate supervisor launched')
    restarted=subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',
        str(ROOT/'scripts/run_benchmark_D_native_contact_overnight.ps1')],capture_output=True,text=True,check=True)
    adopted=None
    for _ in range(40):
        now=read(EVID/'supervisor_state.json')
        if now.get('supervisor_pid')!=record['superseded_controller']['pid'] and now.get('supervisor_alive'):
            adopted=identity(dict(pid=now['supervisor_pid'],created=now['supervisor_created'],name='pythonw.exe'))
            if adopted and read(EVID/'active_job.json').get('pid')==before[0]['pid']:break
        time.sleep(.5)
    else:raise RuntimeError('Same task has not confirmed adoption; existing worker remains untouched')
    if actual_owners(branch)!=before:raise RuntimeError('Existing CFD owners changed after reload')
    record.update(status='PASS',timestamp=stamp(),same_worker_and_native_engines_after_reload=True,
        new_supervisor=dict(pid=adopted.pid,created=adopted.create_time(),name=adopted.name()),
        resume_stdout=restarted.stdout,no_new_solver=True)
    atomic(archive/'review.json',record)
    event('CONDITIONAL_CAPABILITY_CONTROL_REPAIR_VERIFIED',review=str((archive/'review.json').relative_to(EVID)),same_worker_pid=before[0]['pid'])
    print(json.dumps(dict(status='PASS',same_worker_pid=before[0]['pid'],new_supervisor_pid=adopted.pid,original_window_preserved=True)))


if __name__=='__main__':main()
