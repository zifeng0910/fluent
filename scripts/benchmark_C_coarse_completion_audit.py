"""Final read-only science/checkpoint audit and consolidation; never starts Fluent."""
import csv,json
from datetime import datetime
import numpy as np,psutil,h5py
from benchmark_C_coarse_common import *
from benchmark_C_coarse_commit_audit import system

def main():
    preserve();cont=EVID/'continuation_20h';window=read(cont/'window.json')
    campaign=read(EVID/'state.json');closure=read(cont/'finalization_result.json');dyn=read(EVID/'dynamic.json')
    if read(cont/'state.json')['status']!='COMPLETE' or campaign['status']!='COMPLETE' or closure['status']!='PASS' or native():raise RuntimeError('Completed campaign and scoped engine closure required')
    if psutil.pid_exists(campaign['job_pid']):raise RuntimeError('Dynamics worker is still alive')
    history=EVID/'dynamic_history.csv'; rows=list(csv.DictReader(history.open()))
    if [int(r['step']) for r in rows]!=list(range(1,41)):raise RuntimeError('History has missing/duplicate steps')
    prefix=cont/'resume29_01/dynamic_history.csv'
    if not history.read_bytes().startswith(prefix.read_bytes()):raise RuntimeError('Original1..29 history prefix changed')
    if dyn['completed_steps']!=40 or abs(dyn['time_s']-.001)>1e-12:raise RuntimeError('Full1ms not completed')
    for r in rows:
        if abs(float(r['time_s'])-int(r['step'])*25e-6)>1e-12 or not np.isfinite([float(v) for v in r.values()]).all():raise RuntimeError('Time or finite history gate failed')
        if int(r['orphan_count']) or int(r['receptors_without_donors']) or float(r['minimum_cell_volume_m3'])<=0 or float(r['robot_wall_clearance_m'])<.0001 or abs(float(r['q_norm'])-1)>1e-6:raise RuntimeError('A saved history gate failed')
    stats=[read(EVID/f'fielddata/connectivity_{i:04d}.json') for i in range(1,41)]
    if any(not s or s['total_cells']!=4699301 or s['orphans'] or s['invalid_donors'] or s['cell_type_counts']['-3'] or s['minimum_volume_m3']<=0 for s in stats):raise RuntimeError('Missing/failed official connectivity evidence')
    checkpoint=read(EVID/'checkpoint_0040.json');verify=read(EVID/'step40_checkpoint_verification.json')
    restart_statuses={i:read(EVID/f'benchmark_C_coarse_step{i}_restart_validation.json').get('status') for i in [29,30]}
    if any(s!='PASS' for s in restart_statuses.values()):raise RuntimeError('Native restart continuity gate missing')
    if verify['status']!='PASS' or verify['timesteps_advanced']!=0:raise RuntimeError('Native final checkpoint gate missing')
    for k in ['case','data']:
        if sha(checkpoint[k])!=checkpoint[k+'_sha256'] or checkpoint[k+'_sha256']!=verify[k+'_sha256']:raise RuntimeError('Final checkpoint hash changed')
        with h5py.File(checkpoint[k],'r') as f:
            if not list(f.keys()):raise RuntimeError('Empty native HDF5')
    exact=read(EVID/'coarse_vs_fine_latest_exact.json')
    if exact['interpolation_used'] or not exact['screening_trend_pass']:raise RuntimeError('Exact-time comparison gate failed')
    sources=[EVID/'benchmark_C_coarse_memory_profile.json']+list(cont.glob('*/benchmark_C_coarse_memory_profile.json'))
    samples={};historical_stops={}
    for p in sources:
        profile=read(p)
        for r in profile.get('rows',[]):samples[r['timestamp']]=r
        for stop in [profile.get('abort'),profile.get('historical_resource_stop')]:
            if stop: historical_stops[stop['sample']['timestamp']]=stop
    allrows=sorted(samples.values(),key=lambda r:r['timestamp'])
    current=[r for r in allrows if datetime.fromisoformat(r['timestamp']).timestamp()>=window['started_epoch']]
    stable=all(r['available_gib']>=3 and r['commit_fraction']<=.95 and r['total_project_working_set_gib']<=15 for r in current)
    if not current or not stable:raise RuntimeError('20h window resource guard was not stable')
    keys=['timestamp','stage','available_gib','commit_gib','commit_limit_gib','commit_fraction','total_project_working_set_gib','project_private_bytes_gib','Fluent_working_set_gib','Python_worker_working_set_gib','pagefile_used_gib','pagefile_total_gib']
    memorycsv=cont/'verified_memory_samples.csv'
    with memorycsv.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r.get(k) for k in keys} for r in allrows)
    mem={'timestamp':stamp(),'total_actual_sample_count':len(allrows),'window_actual_sample_count':len(current),
         'peak_project_working_set_gib':max(r['total_project_working_set_gib'] for r in allrows),
         'window_peak_measured_system_commit_gib':max(r['commit_gib'] for r in current),
         'window_peak_measured_system_commit_percent':100*max(r['commit_fraction'] for r in current),
         'window_min_available_physical_gib':min(r['available_gib'] for r in current),
         'window_peak_measured_project_private_bytes_gib':max(r.get('project_private_bytes_gib',0) for r in current),
         'window_resource_guards_stable':stable,'historical_resource_stops_before_this_window':list(historical_stops.values()),
         'system_boot_CommitPeak_is_not_campaign_peak':True,'measurement_csv_sha256':sha(memorycsv),
         'raw_profiles_preserved_locally':{str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sources}}
    atomic(cont/'verified_memory_summary.json',mem)
    rec={'timestamp':stamp(),'status':'BENCHMARK_C_COARSE_SCREENING_PASS','campaign':'BENCHMARK_C_COARSE_SCREENING','next_state':'COARSE_DEVELOPMENT_BASELINE',
         'completed_steps':40,'time_s':dyn['time_s'],'history_sequence_and_original_prefix_verified':True,'original_history_prefix_sha256':sha(prefix),'final_history_sha256':sha(history),
         'peak_orphan':max(int(r['orphan_count']) for r in rows),'peak_invalid_donors':max(int(r['receptors_without_donors']) for r in rows),
         'minimum_cell_volume_m3':min(float(r['minimum_cell_volume_m3']) for r in rows),'minimum_dynamic_clearance_mm':1000*min(float(r['robot_wall_clearance_m']) for r in rows),
         'maximum_q_norm_error':max(abs(float(r['q_norm'])-1) for r in rows),'restart29':restart_statuses[29],'restart30':restart_statuses[30],'native_step40_checkpoint':verify['status'],
         'checkpoint40':checkpoint,'checkpoint40_validation_kind':verify['validation_kind'],'peak_project_working_set_gib':mem['peak_project_working_set_gib'],
         'resource_guards_stable_in_new_window':True,'fine_1ms_reference':exact['one_ms_reference_status'],'latest_exact_fine_comparison_time_s':exact['time_s'],
         'coarse_vs_fine_latest_exact':exact,'owned_engine_closed':True,'unrelated_processes_touched':[],'frozen_files_verified':True,
         'new_solver_launched_by_this_audit':False,'timesteps_advanced_by_this_audit':0,'no_mesh_convergence_claim':True,'deadline_unchanged_epoch':window['deadline_epoch']}
    atomic(EVID/'final_completion_verification.json',rec)
    # Consolidate concise final fields while retaining all historical failures separately.
    from benchmark_C_coarse_report import write_report
    report=write_report();report.update(status='COMPLETE',coarse_screening='PASS',remaining_blocker=None,next_state='COARSE_DEVELOPMENT_BASELINE',
        peak_invalid_donors=rec['peak_invalid_donors'],minimum_cell_volume_m3=rec['minimum_cell_volume_m3'],maximum_q_norm_error=rec['maximum_q_norm_error'],
        final_native_state=verify['native_state'],owned_engine_closed=True,final_system_memory=system(),
        verified_memory_summary=mem,finalization_result=closure,
        resource_stop_events=[{'timestamp':x['sample']['timestamp'],'reason':x['reason'],'commit_percent':100*x['sample']['commit_fraction'],'historical_before_new_window':True} for x in historical_stops.values()])
    atomic(EVID/'final_report.json',report)
    state('COMPLETE',current_step=40,time_steps_advanced=40,time_s=dyn['time_s'],pose=verify['native_state'],native_checkpoint_step=40,solver_alive=False,worker_alive=False,scheduler_enabled=False,review='final_completion_verification.json',next_state='COARSE_DEVELOPMENT_BASELINE')
    print({k:rec[k] for k in ['status','completed_steps','time_s','minimum_dynamic_clearance_mm','peak_project_working_set_gib','fine_1ms_reference','latest_exact_fine_comparison_time_s','owned_engine_closed']})

if __name__=='__main__':main()
