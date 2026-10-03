"""Verify step40 offline and prepare one bounded continuation, without launching."""
import shutil, time
from benchmark_C_coarse_2ms_common import *

def main():
    if (EVID/'configuration.json').exists():raise RuntimeError('Existing 2 ms configuration: initialization refused')
    if native():raise RuntimeError('An existing Fluent engine prevents preparation')
    if read(BASE/'final_completion_verification.json').get('status')!='BENCHMARK_C_COARSE_SCREENING_PASS':
        raise RuntimeError('Verified 1 ms baseline required')
    checkpoint=read(BASE/'checkpoint_0040.json')
    if checkpoint['step']!=40 or abs(checkpoint['time_s']-.001)>1e-12:raise RuntimeError('Only native step40 is authorized')
    audit=checkpoint_audit(checkpoint)
    EVID.mkdir(exist_ok=True);OUT.mkdir(exist_ok=True);(EVID/'fielddata').mkdir(exist_ok=True)
    old_window=read(BASE/'continuation_20h/window.json')
    # Conservative inherited ceiling; no reset/extension of either historical window.
    deadline=old_window['deadline_epoch']
    if time.time()>=deadline:raise RuntimeError('Original authorized 20h ceiling already expired')
    frozen=['fluent_udf/l2300_abaqus100hz_6dof.c','scripts/benchmark_C_analytic_reference.py',
        'evidence/benchmark_C_coarse_A/dynamic_history.csv','evidence/benchmark_C_coarse_A/state.json',
        'evidence/benchmark_C_coarse_A/final_report.json','evidence/benchmark_C_coarse_A/checkpoint_0040.json',
        'evidence/benchmark_C_coarse_A/static.json','evidence/benchmark_C_coarse_A/continuation_20h/window.json',
        'evidence/benchmark_C_fineA_free_6dof_history.csv']
    config={'campaign':CAMPAIGN,'authorization':'User attached e10740f1: native step40->80, unchanged mesh/physics/one-rank, local continuous worker, actual FieldData and visual QA.',
        'start_step':40,'target_step':80,'dt_s':DT,'max_iterations_per_time_step':2,'mesh_cells':4699301,
        'processor_count':1,'dimension':3,'precision':'double','ui_mode':'no_gui_or_graphics',
        'checkpoint_metadata':'evidence/benchmark_C_coarse_A/checkpoint_0040.json',
        'expected_case_size_bytes':audit['files']['case']['size_bytes'],'expected_data_size_bytes':audit['files']['data']['size_bytes'],
        'deadline_epoch':deadline,'deadline_policy':'Inherit original 20h ceiling, never reset/extend. New explicit task authorizes 2ms scope.',
        'admission':{'available_gib':12,'commit_headroom_gib':19,'disk_free_gib':35,'stable_seconds':60},
        'runtime_guards':{'available_gib':3,'commit_fraction':.95,'project_working_set_gib':15,'disk_free_gib':15},
        'fielddata_snapshot_steps':list(range(40,81,2)), 'checkpoint_steps':[50,60,70,80],
        'static_envelope_policy':'Warning if exceeded and actual dynamic connectivity/clearance remains valid',
        'frozen_files_sha256':{p:sha(ROOT/p) for p in frozen},'magnetic_sphere_example_used':False,
        'fine_reference_status':'FINE_REFERENCE_NOT_AVAILABLE_AT_2MS','cutoff_sensitivity':False,'Benchmark_D':False}
    atomic(EVID/'configuration.json',config);atomic(EVID/'step40_CASE_DATA_integrity.json',audit)
    for name in ['dynamic_history.csv','coarse_vs_fine_latest_exact.json']:
        shutil.copy2(BASE/name,EVID/name)
    shutil.copy2(BASE/'fielddata/robot_0000.vtp',EVID/'fielddata/robot_0000.vtp')
    state('PREPARED',current_step=40,target_step=80,time_s=.001,solver_alive=False,worker_alive=False)
    event('PREPARED',checkpoint40_integrity='PASS',deadline_epoch=deadline)
    print({'status':'PREPARED','CASE_DATA_integrity':'PASS','restart_step':40,'target_step':80,'memory':admission()[1]})

if __name__=='__main__':main()
