"""Execute the reviewed micro12p5 shutdown after a saved resource stop.

This job advances no dynamics. The stopped branch's resource failure remains
FAIL. Only the exact registered node and its SDK services can be closed.
"""
import argparse
import time

import h5py
import psutil

from benchmark_D_native_contact_overnight_common import *


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', required=True)
    args = parser.parse_args()
    plan_path = (ROOT / args.plan).resolve()
    if plan_path.parent != (EVID / 'control_repairs/0004_staged_owned_service_exit').resolve():
        raise RuntimeError('Only the reviewed second staged closure plan is accepted')
    plan = read(plan_path)
    if plan.get('job_sha256') != sha(Path(__file__)):
        raise RuntimeError('Reviewed closure code changed')
    if plan.get('status') != 'REVIEWED_FOR_EXACT_OWNED_CLOSURE_ONLY':
        raise RuntimeError('Explicit reviewed closure plan required')
    if (plan_path.parent / 'review.json').exists():
        raise RuntimeError('Closure action already completed')
    verify_frozen()
    if sha(EVID / 'window.json') != plan['window_sha256'] or sha(EVID / 'configuration.json') != plan['configuration_sha256']:
        raise RuntimeError('Original window or configuration changed')
    branch = EVID / 'branches/micro12p5'
    if sha(branch / 'failure.json') != plan['original_failure_sha256'] or sha(branch / 'owned_engine.json') != plan['owned_engine_sha256']:
        raise RuntimeError('Original failure/ownership changed')
    if 'Owned engine not idle after SDK exit' not in read(branch / 'closure_failure.json').get('error', ''):
        raise RuntimeError('Recorded public SDK exit is required')
    worker = read(branch / 'worker_identity.json')
    worker.setdefault('name', 'pythonw.exe')
    supervisor = read(EVID / 'supervisor_state.json')
    sup_identity = dict(pid=supervisor['supervisor_pid'], created=supervisor['supervisor_created'], name='pythonw.exe')
    if identity(worker) or identity(sup_identity):
        raise RuntimeError('Worker or supervisor still owns the campaign')
    cp = plan['saved_1p9ms_checkpoint']
    for kind in ('case', 'data'):
        path = Path(cp[kind])
        if path.stat().st_size != cp[kind + '_size_bytes'] or sha(path) != cp[kind + '_sha256']:
            raise RuntimeError('Preserved emergency checkpoint changed')
        with h5py.File(path, 'r') as handle:
            if not list(handle):
                raise RuntimeError('Preserved emergency checkpoint unreadable')
    owned = read(branch / 'owned_engine.json')['processes']
    nodes = [rec for rec in owned if rec['name'].lower() == 'fl_mpi2610.exe']
    hosts = [rec for rec in owned if rec['name'].lower() in ('fl2610.exe', 'cx2610.exe')]
    if len(nodes) != 1 or len(hosts) != 2 or len(owned) != 3:
        raise RuntimeError('Expected one registered compute node and two SDK services')
    allowed = {rec['pid'] for rec in owned if identity(rec)}
    if any(process.pid not in allowed for process in native()):
        raise RuntimeError('Unrelated native engine present; no action')
    transcript = branch / 'solver.trn'
    observations = []
    deadline = time.time() + 120
    node = nodes[0]
    while (process := identity(node)) is not None:
        if time.time() + 5 > deadline:
            raise RuntimeError('Compute node remains busy; no cleanup')
        before = sum(process.cpu_times()[:2])
        time.sleep(5)
        process = identity(node)
        delta = sum(process.cpu_times()[:2]) - before if process else 0.0
        observation = dict(timestamp=stamp(), node_CPU_seconds_over5s=delta)
        observations.append(observation)
        atomic(plan_path.parent / 'observations.json', dict(observations=observations))
        if identity(worker) or identity(sup_identity) or sha(transcript) != plan['transcript_sha256']:
            raise RuntimeError('Ownership or post-exit transcript changed')
        if delta <= .05:
            if process:
                process.terminate()
                process.wait(timeout=15)
            break
    if identity(node):
        raise RuntimeError('Compute node closure not confirmed')
    terminated_services = []
    for rec in hosts:
        if identity(worker) or sha(transcript) != plan['transcript_sha256']:
            raise RuntimeError('Worker/transcript changed before SDK service closure')
        process = identity(rec)
        if process:
            process.terminate()
            process.wait(timeout=15)
            terminated_services.append(rec)
    if any(identity(rec) for rec in owned) or native():
        raise RuntimeError('Native engine closure not confirmed')
    proof = dict(status='PASS', timestamp=stamp(), SDK_exit_requested=True,
        SDK_exit_returned=True, owned_engine_closed=True, worker_identity=worker,
        owned_engine_identities=owned, worker_absent=True,
        all_owned_engines_absent=True, no_native_engine_present=True,
        compute_node_idle_threshold_CPU_seconds_over5s=.05,
        compute_node_absence_before_host_closure=True, observations=observations,
        terminated_services=terminated_services, unrelated_processes_touched=[],
        checkpoint_time_s=cp['time_s'], emergency_checkpoint_integrity_PASS=True,
        original_exit_failure_preserved=True, original_resource_failure_preserved=True,
        resource_failure_not_waived=True, no_new_solver=True, no_dynamics_advanced=True,
        plan_sha256=sha(plan_path), job_sha256=sha(Path(__file__)))
    atomic(plan_path.parent / 'review.json', proof)
    atomic(branch / 'engine_exit.json', proof)
    atomic(branch / 'recovery_process_review.json', proof)
    print('Exact owned compute node and SDK services closed; resource failure retained.')


if __name__ == '__main__':
    main()
