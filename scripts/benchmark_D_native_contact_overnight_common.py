"""Isolated native contact campaign. Control helpers import no numerical runtime."""
from pathlib import Path
import json, os, re, shutil, time
from benchmark_C_recovery_v2_common import atomic, read, sha, stamp, native
ROOT=Path(__file__).resolve().parents[1]
EVID=ROOT/'evidence/benchmark_D_native_contact_overnight'
OUT=ROOT/'live_cases/benchmark_D_native_contact_overnight'
CAMPAIGN='BENCHMARK_D_NATIVE_CONTACT_OVERNIGHT'
DT=25e-6
BRANCHES={'micro25':3,'micro12p5':3,'micro6p25':3,
          'production25':3,'production12p5':3,'production6p25':3}

def state(status, **data):
    rec=read(EVID/'state.json');rec.update(campaign=CAMPAIGN,status=status,timestamp=stamp(),**data)
    atomic(EVID/'state.json',rec);return rec

def event(name, **data):
    EVID.mkdir(parents=True,exist_ok=True)
    with (EVID/'events.jsonl').open('a',encoding='utf-8') as f:
        f.write(json.dumps(dict(timestamp=stamp(),event=name,**data),allow_nan=False,default=str)+'\n')

def rows(path):
    p=Path(path)
    return [json.loads(s) for s in p.read_text(encoding='utf-8-sig').splitlines() if s.strip()] if p.exists() else []

def identity(record):
    import psutil
    try:
        p=psutil.Process(int(record['pid']))
        return p if abs(p.create_time()-float(record['created']))<.01 and p.name().lower()==record['name'].lower() else None
    except (psutil.Error,KeyError,TypeError,ValueError):return None

def verify_frozen():
    config=read(EVID/'configuration.json')
    for path,digest in config['frozen_files_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Frozen source/evidence changed: '+path)
    window=read(EVID/'window.json')
    if sha(EVID/'window.json')!=config['window_sha256'] or window['campaign_id']!=CAMPAIGN:
        raise RuntimeError('Immutable overnight window changed')

def admission():
    import psutil
    from benchmark_C_coarse_commit_audit import system
    m=system();m['disk_free_gib']=psutil.disk_usage(str(ROOT)).free/2**30
    c=read(EVID/'configuration.json')['admission']
    return all(m[k]>=c[k] for k in ['available_gib','commit_headroom_gib','disk_free_gib']),m

def specification(branch):
    path=EVID/'branch_specs'/f'{branch}.json'
    c=read(path)
    if not c or c.get('branch')!=branch:raise RuntimeError('Missing reviewed branch specification: '+branch)
    return c

def resolve_branch(branch):
    return read(EVID/'branch_resolution.json').get(branch,branch)

def prepare_branch(branch):
    b=EVID/'branches'/branch;o=OUT/'branches'/branch
    if (b/'branch_result.json').exists():raise RuntimeError('Completed branch cannot be replayed')
    if (b/'configuration.json').exists():
        c=read(b/'configuration.json')
        for p,k in [(ROOT/c['contact_source_path'],'contact_source_sha256'),
                    (o/'magnetic_logging_copy.c','magnetic_logging_copy_sha256'),
                    (o/'lifecycle_config.h','configuration_header_sha256'),
                    (ROOT/'fluent_udf/l2300_contact_impulse_math.h','mathematical_kernel_sha256')]:
            if sha(p)!=c[k]:raise RuntimeError('Prepared input changed: '+str(p))
        return b,o,c
    c=specification(branch);b.mkdir(parents=True,exist_ok=True);o.mkdir(parents=True,exist_ok=True)
    (b/'fielddata').mkdir(exist_ok=True)
    shutil.copy2(ROOT/'evidence/benchmark_D_contact_baseline/fielddata/robot_0000.vtp',b/'fielddata/robot_0000.vtp')
    shutil.copy2(ROOT/'evidence/benchmark_D_contact_baseline/dynamic_history.csv',b/'dynamic_history.csv')
    original=ROOT/'fluent_udf/l2300_abaqus100hz_6dof.c';source=original.read_text();changed=source;replace={}
    for macro,name in [('LOAD_CSV','magnetic_load_validation.csv'),('HISTORY_CSV','magnetic_history.csv'),('PROBE_CSV','magnetic_orientation_probe.csv')]:
        old=re.search(r'#define '+macro+r' .*',source).group(0)
        new='#define '+macro+' '+json.dumps((b/name).as_posix());changed=changed.replace(old,new);replace[new]=old
    magnetic=o/'magnetic_logging_copy.c';magnetic.write_text(changed)
    reverse=changed
    for new,old in replace.items():reverse=reverse.replace(new,old)
    if reverse!=source:raise RuntimeError('Magnetic formulas changed')
    names={'TRACE_HOST_JSON':'native_contact_callback_trace_host.jsonl','TRACE_NODE_JSON':'native_contact_callback_trace_node.jsonl',
           'TRACE_HOST_CSV':'native_contact_callback_trace_host.csv','TRACE_NODE_CSV':'native_contact_callback_trace_node.csv',
           'STATE_HOST_JSON':'native_state_lifecycle_host.jsonl','STATE_NODE_JSON':'native_state_lifecycle_node.jsonl',
           'CONNECTIVITY_JSON':'native_connectivity_current.json','SEMANTICS_HOST_JSON':'theta_lifecycle_host.jsonl',
           'SEMANTICS_NODE_JSON':'theta_lifecycle_node.jsonl'}
    header=o/'lifecycle_config.h';header.write_text('#define LIFECYCLE_MODE '+str(c.get('mode',3))+'\n'+
        '\n'.join('#define '+k+' '+json.dumps((b/v).as_posix()) for k,v in names.items())+'\n')
    c.update(contact_source_sha256=sha(ROOT/c['contact_source_path']),mathematical_kernel_sha256=sha(ROOT/'fluent_udf/l2300_contact_impulse_math.h'),
             magnetic_logging_copy_sha256=sha(magnetic),magnetic_original_sha256=sha(original),only_magnetic_log_paths_changed=True,
             configuration_header_sha256=sha(header),threshold_m=.0001,mode=c.get('mode',3),
             same_returned_theta_only=True,no_manual_Detect_Contact_call=True,impulse_restitution=0,friction_coefficient=0)
    atomic(b/'configuration.json',c);return b,o,c

def review_launch(branch):
    b,o,c=prepare_branch(branch);files=[b/'configuration.json',EVID/'configuration.json',EVID/'window.json',
        ROOT/c['source_checkpoint'],
        ROOT/c['contact_source_path'],o/'magnetic_logging_copy.c',o/'lifecycle_config.h',
        ROOT/'fluent_udf/l2300_contact_impulse_math.h',ROOT/'scripts/benchmark_D_native_contact_overnight_worker.py',
        ROOT/'scripts/benchmark_D_native_contact_overnight_common.py']
    atomic(b/'launch_review.json',dict(status='PASS',timestamp=stamp(),branch=branch,
        files_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in files},
        gate='Verified staged code/config; actual cold native continuity remains mandatory before dynamics'))
    return b,o,c
