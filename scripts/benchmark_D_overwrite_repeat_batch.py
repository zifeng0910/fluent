"""Finite A2 -> B1 -> B2 diagnostic sequence; no scheduler or physical writes.

Waits for the already reviewed A2 worker, verifies each native checkpoint and
owned closure, then executes the two user-authorized no-op repeats serially.
"""
import json, subprocess, sys, time
import psutil
from benchmark_D_overwrite_common import *
from benchmark_D_overwrite_review import integrity

INPUTS=['scripts/benchmark_D_overwrite_worker.py','scripts/benchmark_D_overwrite_common.py',
        'fluent_udf/l2300_contact_overwrite_semantics.c']

def alive_identity(info):
    try:
        p=psutil.Process(info['pid'])
        return abs(p.create_time()-info['created'])<.01 and p.is_running()
    except psutil.NoSuchProcess:return False

def verify_closed(branch):
    info=read(EVID/branch/'worker_identity.json')
    # The checkpoint/closure result can precede process exit by a few ms.
    while alive_identity(info):time.sleep(10)
    integrity(branch)
    if native():raise RuntimeError('Native engine exists; no further launch allowed')
    verify_frozen()

def main():
    reviewed=read(EVID/'A1/launch_review.json')['files_sha256']
    verify_closed('A2')
    for branch in ['B1','B2']:
        if any(sha(ROOT/p)!=reviewed[p] for p in INPUTS):raise RuntimeError('Reviewed worker inputs changed')
        b,o,c=prepare_branch(branch)
        paths=INPUTS+[str((b/'configuration.json').relative_to(ROOT)),
                     str((o/'lifecycle_config.h').relative_to(ROOT)),str((o/'magnetic_logging_copy.c').relative_to(ROOT))]
        atomic(b/'launch_review.json',dict(status='PASS',timestamp=stamp(),branch=branch,
            decision='Reviewed same-value overwrite repeat; independent C70, five steps to1.875ms; previous worker closed and checkpoint gates PASS',
            files_sha256={p:sha(ROOT/p) for p in paths},finite_repeat_batch=True))
        if native():raise RuntimeError('Native engine appeared before launch')
        with (b/'worker_stdout.log').open('w') as stdout,(b/'worker_stderr.log').open('w') as stderr:
            process=subprocess.Popen([sys.executable,str(ROOT/'scripts/benchmark_D_overwrite_worker.py'),'--branch',branch],
                cwd=ROOT,stdout=stdout,stderr=stderr,creationflags=subprocess.CREATE_NO_WINDOW)
            atomic(EVID/'repeat_batch.json',dict(timestamp=stamp(),branch=branch,launcher_pid=process.pid,
                scope=['B1','B2'],physical_write_allowed=False))
            code=process.wait()
        if code:raise RuntimeError(f'{branch} worker failed with exit {code}')
        verify_closed(branch)
        print(branch+' verified complete',flush=True)
    atomic(EVID/'repeat_batch.json',dict(timestamp=stamp(),status='COMPLETE',branches=['A2','B1','B2'],physical_write_allowed=False))
    print('Four repeats ready for offline semantics review',flush=True)

if __name__=='__main__':
    try:main()
    except Exception as e:
        atomic(EVID/'repeat_batch_failure.json',dict(timestamp=stamp(),reason=str(e)))
        raise
