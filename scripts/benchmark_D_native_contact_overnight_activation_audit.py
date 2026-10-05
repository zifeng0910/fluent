"""Record a real local launch milestone; never connects to or launches Fluent."""
import json
import subprocess
import psutil
from benchmark_D_native_contact_overnight_common import *


def main():
    verify_frozen()
    review = read(EVID / 'launch_review.json')
    if review.get('status') != 'PASS':
        raise RuntimeError('Launch review missing')
    for path, digest in review['supporting_code_sha256'].items():
        if sha(ROOT / path) != digest:
            raise RuntimeError('Reviewed control code changed: ' + path)
    supervisor = read(EVID / 'supervisor_state.json')
    sup = identity(dict(pid=supervisor['supervisor_pid'],
                        created=supervisor['supervisor_created'], name='pythonw.exe'))
    if not sup:
        raise RuntimeError('Exact local supervisor is absent')
    active = read(EVID / 'active_job.json')
    if active.get('kind') != 'worker' or not identity(active):
        raise RuntimeError('Actual registered worker absent')
    b = EVID / 'branches' / active['branch']
    worker = read(b / 'worker_state.json')
    inputs = read(b / 'launch_review.json')
    for path, digest in inputs['files_sha256'].items():
        if sha(ROOT / path) != digest:
            raise RuntimeError('Active worker input changed: ' + path)
    owned = read(b / 'owned_engine.json')['processes']
    actual = []
    for item in owned:
        p = psutil.Process(item['pid'])
        if abs(p.create_time() - item['created']) >= .01:
            raise RuntimeError('Native engine identity changed')
        actual.append(dict(pid=p.pid, created=p.create_time(), name=p.name()))
    if not actual:
        raise RuntimeError('No owned native engine')
    restart = read(b / 'source_restart_validation.json')
    compile_gate = read(b / 'compile_hook_gate.json')
    latest = read(b / 'latest_verified_checkpoint.json')
    if restart.get('status') != 'PASS' or compile_gate.get('status') != 'PASS':
        raise RuntimeError('Actual native restart or compile gate failed')
    checkpoint = read(ROOT / latest['metadata'])
    if latest.get('status') != 'PASS' or checkpoint['status'] != 'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS':
        raise RuntimeError('No verified actual solved native checkpoint')
    if checkpoint['time_s'] <= restart['native_time_s']:
        raise RuntimeError('Launch milestone must include actual dynamics')
    task = subprocess.run(['powershell.exe', '-NoProfile', '-Command',
        "Get-ScheduledTask -TaskName 'Fluent-Benchmark-D-Native-Contact-Overnight' | "
        "Select-Object TaskName,State | ConvertTo-Json -Compress"],
        check=True, capture_output=True, text=True)
    scheduler = json.loads(task.stdout)
    if scheduler['State'] != 4 and scheduler['State'] != 'Running':
        raise RuntimeError('Local scheduler task is not running')
    evidence = ['window.json', 'configuration.json', 'launch_review.json',
                'control_validation.json', 'source_change_review.json',
                'control_repairs/0001_journal_collision/review.json']
    evidence += [str((b / name).relative_to(EVID)) for name in
                 ['source_restart_validation.json', 'compile_hook_gate.json']]
    evidence += [str((ROOT / latest['metadata']).relative_to(EVID))]
    record = dict(status='LOCAL_CAMPAIGN_ARMED_AND_ACTUAL_DYNAMICS_VERIFIED',
        timestamp=stamp(), campaign=CAMPAIGN, windows_task=scheduler,
        window=read(EVID / 'window.json'), configuration_sha256=sha(EVID / 'configuration.json'),
        supervisor=dict(pid=sup.pid, created=sup.create_time(), name=sup.name()),
        worker={key: active[key] for key in ['branch', 'pid', 'created', 'name']},
        owned_native_engines=actual, native_restart='PASS', actual_UDF_compile='PASS',
        source_time_s=restart['native_time_s'], latest_verified_time_s=checkpoint['time_s'],
        latest_verified_checkpoint=latest['metadata'],
        active_worker_inputs_unchanged=True, no_new_solver=True,
        solver_survived_observed_supervisor_failure=True,
        numerical_2ms='INCOMPLETE', micro_run='INCOMPLETE', timestep_selection='PENDING',
        visual_review='PENDING', no_full_campaign_pass_claim=True,
        evidence_files_sha256={name: sha(EVID / name) for name in evidence})
    atomic(EVID / 'activation_audit.json', record)
    print(json.dumps({key: record[key] for key in ['status', 'latest_verified_time_s', 'numerical_2ms']}))


if __name__ == '__main__':
    main()
