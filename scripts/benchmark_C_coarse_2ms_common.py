"""Isolated 1->2 ms continuation; the accepted 1 ms evidence remains immutable."""
import csv, os, time
from pathlib import Path
from benchmark_C_recovery_v2_common import atomic, read, sha, stamp, native, mpi_node_associations
from benchmark_C_coarse_commit_audit import system

ROOT=Path(__file__).resolve().parents[1]
EVID=ROOT/'evidence/benchmark_C_coarse_2ms'
OUT=ROOT/'live_cases/benchmark_C_coarse_2ms'
BASE=ROOT/'evidence/benchmark_C_coarse_A'
CAMPAIGN='BENCHMARK_C_COARSE_2MS_CONTINUATION'
DT=25e-6
COM0=[.0012060186937156343,0.,0.]

def state(status, **data):
    rec=read(EVID/'state.json');rec.update(campaign=CAMPAIGN,status=status,timestamp=stamp(),**data)
    atomic(EVID/'state.json',rec);return rec

def event(name, **data):
    import json
    with (EVID/'events.jsonl').open('a',encoding='utf-8') as fp:
        fp.write(json.dumps({'timestamp':stamp(),'event':name,**data},allow_nan=False)+'\n')

def identity(record):
    import psutil
    try:
        p=psutil.Process(record['pid'])
        return p if abs(p.create_time()-record['created'])<.01 and p.name().lower()==record['name'].lower() else None
    except (psutil.Error,KeyError):return None

def verify_frozen():
    rec=read(EVID/'configuration.json')
    for path,digest in rec['frozen_files_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Frozen file changed: '+path)

def checkpoint_audit(metadata, check_hash=True):
    import h5py
    step=int(metadata['step']);t=float(metadata['time_s'])
    if abs(t-step*DT)>1e-12:raise RuntimeError('Checkpoint time/step mismatch')
    result={'step':step,'time_s':t,'files':{},'status':'PASS','timesteps_advanced':0}
    for kind in ['case','data']:
        p=Path(metadata[kind])
        if not p.is_file() or p.stat().st_size<1024:raise RuntimeError('Missing/empty checkpoint '+kind)
        digest=sha(p) if check_hash else metadata[kind+'_sha256']
        if digest!=metadata[kind+'_sha256']:raise RuntimeError('Checkpoint hash mismatch '+kind)
        if kind+'_size_bytes' in metadata and p.stat().st_size!=metadata[kind+'_size_bytes']:
            raise RuntimeError('Checkpoint file size mismatch '+kind)
        with h5py.File(p,'r') as f:
            if not list(f):raise RuntimeError('Empty HDF5 '+kind)
            # Read stored native variables, rather than only opening the container.
            group='settings/Rampant Variables' if kind=='case' else 'settings/Data Variables'
            if group in f:
                content=f[group][()]
                if not len(content):raise RuntimeError('Empty native HDF5 variables')
        result['files'][kind]={'path':str(p),'size_bytes':p.stat().st_size,'sha256':digest,'HDF5_readable':True}
    return result

def admission():
    import psutil
    m=system();m['disk_free_gib']=psutil.disk_usage(str(ROOT)).free/2**30
    return m['available_gib']>=12 and m['commit_headroom_gib']>=19 and m['disk_free_gib']>=35,m

def history():
    with (EVID/'dynamic_history.csv').open(newline='') as fp:return list(csv.DictReader(fp))
