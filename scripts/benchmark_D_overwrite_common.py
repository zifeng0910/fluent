"""Independent lifecycle diagnostics; previous Benchmark C/D evidence is read-only."""
from pathlib import Path
from benchmark_C_recovery_v2_common import atomic,read,sha,stamp,native
ROOT=Path(__file__).resolve().parents[1]
EVID=ROOT/'evidence/benchmark_D_contact_overwrite_semantics'
OUT=ROOT/'live_cases/benchmark_D_contact_overwrite_semantics'
CAMPAIGN='BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS'
DT=25e-6
BRANCHES={'A1':0,'A2':0,'B1':1,'B2':1,'controlled':2}

def state(status,**items):
    rec=read(EVID/'state.json');rec.update(campaign=CAMPAIGN,status=status,timestamp=stamp(),**items);atomic(EVID/'state.json',rec);return rec

def event(name,**items):
    import json
    with (EVID/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(dict(timestamp=stamp(),event=name,**items),allow_nan=False)+'\n')

def verify_frozen():
    for p,h in read(EVID/'configuration.json')['frozen_files_sha256'].items():
        if sha(ROOT/p)!=h:raise RuntimeError('Frozen evidence/source changed: '+p)

def rows(path):
    import json
    p=Path(path)
    return [json.loads(s) for s in p.read_text().splitlines() if s.strip()] if p.exists() else []

def prepare_branch(branch):
    import json,re,shutil
    b=EVID/branch;o=OUT/branch
    if (b/'branch_result.json').exists() or (b/'worker_identity.json').exists():raise RuntimeError('Existing branch must be reviewed; unchanged replay refused')
    if (b/'configuration.json').exists():
        config=read(b/'configuration.json')
        for p,k in [(ROOT/'fluent_udf/l2300_contact_overwrite_semantics.c','contact_source_sha256'),
                    (o/'magnetic_logging_copy.c','magnetic_logging_copy_sha256'),
                    (o/'lifecycle_config.h','configuration_header_sha256'),
                    (ROOT/'fluent_udf/l2300_contact_impulse_math.h','mathematical_kernel_sha256')]:
            if sha(p)!=config[k]:raise RuntimeError('Prepared branch changed before launch: '+str(p))
        return b,o,config
    b.mkdir(parents=True,exist_ok=True);o.mkdir(parents=True,exist_ok=True);(b/'fielddata').mkdir(exist_ok=True)
    shutil.copy2(ROOT/'evidence/benchmark_D_contact_baseline/fielddata/robot_0000.vtp',b/'fielddata/robot_0000.vtp')
    shutil.copy2(ROOT/'evidence/benchmark_D_contact_baseline/dynamic_history.csv',b/'dynamic_history.csv')
    original=ROOT/'fluent_udf/l2300_abaqus100hz_6dof.c';source=original.read_text();changed=source;replacements={}
    for macro,name in [('LOAD_CSV','magnetic_load_validation.csv'),('HISTORY_CSV','magnetic_history.csv'),('PROBE_CSV','magnetic_orientation_probe.csv')]:
        pattern=r'#define '+macro+r' .*';old=re.search(pattern,source).group(0);new='#define '+macro+' '+json.dumps((b/name).as_posix());changed=changed.replace(old,new);replacements[new]=old
    magnetic=o/'magnetic_logging_copy.c';magnetic.write_text(changed)
    reversed_copy=changed
    for new,old in replacements.items():reversed_copy=reversed_copy.replace(new,old)
    if reversed_copy!=source:raise RuntimeError('Magnetic formula changed')
    names={'TRACE_HOST_JSON':'native_contact_callback_trace_host.jsonl','TRACE_NODE_JSON':'native_contact_callback_trace_node.jsonl',
        'TRACE_HOST_CSV':'native_contact_callback_trace_host.csv','TRACE_NODE_CSV':'native_contact_callback_trace_node.csv',
        'STATE_HOST_JSON':'native_state_lifecycle_host.jsonl','STATE_NODE_JSON':'native_state_lifecycle_node.jsonl',
        'CONNECTIVITY_JSON':'native_connectivity_current.json','SEMANTICS_HOST_JSON':'theta_lifecycle_host.jsonl','SEMANTICS_NODE_JSON':'theta_lifecycle_node.jsonl'}
    header=o/'lifecycle_config.h';header.write_text('#define LIFECYCLE_MODE '+str(BRANCHES[branch])+'\n'+
        '\n'.join('#define '+k+' '+json.dumps((b/v).as_posix()) for k,v in names.items())+'\n')
    config=dict(branch=branch,mode=BRANCHES[branch],dt_s=DT,
        threshold_m=.0001,source_checkpoint='evidence/benchmark_C_coarse_2ms/checkpoint_0070.json',
        contact_source_sha256=sha(ROOT/'fluent_udf/l2300_contact_overwrite_semantics.c'),
        mathematical_kernel_sha256=sha(ROOT/'fluent_udf/l2300_contact_impulse_math.h'),
        magnetic_logging_copy_sha256=sha(magnetic),magnetic_original_sha256=sha(original),only_magnetic_log_paths_changed=True,
        configuration_header_sha256=sha(header),official_2026R1_callback_pattern=True,
        no_manual_Detect_Contact_call=True,no_immediate_Get_after_Overwrite_gate=True)
    atomic(b/'configuration.json',config);return b,o,config
