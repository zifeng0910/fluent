"""Close only exact idle owned leftovers after a recorded public SDK exit."""
import argparse,time
import psutil
from benchmark_D_native_contact_overnight_common import *


def worker_is_absent(branch):
    worker=read(branch/'worker_identity.json');worker.setdefault('name','pythonw.exe')
    if not worker.get('pid') or identity(worker):raise RuntimeError('Worker absence must be proven')
    return worker


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--branch',required=True);args=parser.parse_args()
    b=EVID/'branches'/args.branch
    if b.resolve().parent!=(EVID/'branches').resolve():raise RuntimeError('Invalid branch path')
    verify_frozen();worker=worker_is_absent(b)
    failure=read(b/'closure_failure.json')
    if 'Owned engine not idle after SDK exit' not in failure.get('error',''):
        raise RuntimeError('This job applies only to the recorded completed SDK exit and idle-gate failure')
    pointer=read(b/'latest_verified_checkpoint.json');meta=read(ROOT/pointer['metadata'])
    if pointer.get('status')!='PASS' or meta.get('status')!='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS':
        raise RuntimeError('Preserved verified native checkpoint required')
    for kind in ('case','data'):
        path=Path(meta[kind]);path=path if path.is_absolute() else ROOT/path
        if not path.is_file() or sha(path)!=meta[kind+'_sha256']:
            raise RuntimeError('Native checkpoint bytes changed')
    owned=read(b/'owned_engine.json')['processes']
    if not owned or any(not r.get('pid') or not r.get('created') or not r.get('name') for r in owned):
        raise RuntimeError('Exact engine identity records required')
    archive=b/'closure_repairs/0001_transient_exit_busy'
    if archive.exists():raise RuntimeError('Closure repair already recorded; unchanged cleanup refused')
    archive.mkdir(parents=True)
    for name in ('closure_failure.json','worker_exit.json','owned_engine.json'):
        shutil.copy2(b/name,archive/name)
    if (EVID/'state.json').is_file():shutil.copy2(EVID/'state.json',archive/'campaign_state_before.json')
    transcript=b/'solver.trn';last_change=transcript.stat().st_mtime
    observations=[];terminated=[];wait_end=time.time()+120
    while True:
        worker_is_absent(b)
        remaining=[(rec,identity(rec)) for rec in owned];remaining=[(r,p) for r,p in remaining if p]
        if not remaining:break
        if time.time()+5>wait_end:raise RuntimeError('Owned exit remains busy; no forced cleanup')
        before={p.pid:sum(p.cpu_times()[:2]) for _,p in remaining};time.sleep(5)
        deltas={r['pid']:sum(identity(r).cpu_times()[:2])-before[r['pid']] for r,_ in remaining if identity(r)}
        observations.append(dict(timestamp=stamp(),CPU_seconds_over5s=deltas))
        if transcript.stat().st_mtime!=last_change:raise RuntimeError('Transcript changed after SDK exit; no forced cleanup')
        if any(delta>.05 for delta in deltas.values()):continue
        # Same existing idle threshold. The compute node closes before its host.
        for rec,_ in sorted(remaining,key=lambda pair:0 if pair[0]['name'].lower()=='fl_mpi2610.exe' else 1):
            p=identity(rec)
            if p:p.terminate();terminated.append(rec)
        psutil.wait_procs([p for _,p in remaining],timeout=10)
    if native():raise RuntimeError('Another native engine remains; no further solver admitted')
    proof=dict(status='PASS',timestamp=stamp(),SDK_exit_requested=True,SDK_exit_returned=True,
        owned_engine_closed=True,worker_identity=worker,owned_engine_identities=owned,
        worker_absent=True,all_owned_engines_absent=True,no_native_engine_present=True,
        no_process_terminated=not bool(terminated),terminated=terminated,unrelated_processes_touched=[],
        observations=observations,idle_threshold_CPU_seconds_over5s=.05,
        idle_threshold_unchanged=True,checkpoint_time_s=meta['time_s'],checkpoint_metadata_sha256=sha(ROOT/pointer['metadata']),
        original_exit_failure_preserved=True,no_new_solver=True,no_dynamics_advanced=True)
    atomic(archive/'review.json',proof);atomic(b/'engine_exit.json',proof)
    atomic(b/'recovery_process_review.json',proof)
    print('Exact owned post-SDK-exit leftovers closed; original native boundary preserved.')


if __name__=='__main__':main()
