"""Consolidate actual measurements, never label partial dynamics full PASS."""
import csv,numpy as np
from benchmark_C_coarse_common import *

def write_report():
    mesh=read(EVID/'mesh.json');static=read(EVID/'static.json');dyn=read(EVID/'dynamic.json');mem=read(EVID/'benchmark_C_coarse_memory_profile.json');comparison=read(EVID/'coarse_vs_fine_025ms.json')
    first=read(EVID/'first_worker_memory_profile.json')
    if first and mem and not mem.get('combined_same_native_session'):
        current_rows=[{**r,'stage':'before_same_session_reconnect' if r['stage']=='before_Fluent_launch' else r['stage']} for r in mem['rows']]
        previous=read(EVID/'resource_stop_0028/benchmark_C_coarse_memory_profile.json')
        all_rows=first['rows']+previous.get('rows',[])+current_rows
        seen=set();unique=[]
        for r in all_rows:
            key=(r['timestamp'],r['total_project_working_set_gib'])
            if key not in seen:seen.add(key);unique.append(r)
        combined={'timestamp':stamp(),'combined_same_native_session':True,'first_worker_host_pid':12684,'rows':unique,'peak_project_working_set_gib':max(r['total_project_working_set_gib'] for r in unique),'note':'One native Fluent session. All controller memory samples retained; resource-stop archive is immutable.','abort':mem.get('abort'),'historical_resource_stop':previous.get('abort')}
        atomic(EVID/'benchmark_C_coarse_memory_profile_combined.json',combined)
        if mem['rows'] and mem['rows'][-1]['stage']=='after_Fluent_exit':
            atomic(EVID/'second_worker_memory_profile.json',mem);atomic(EVID/'benchmark_C_coarse_memory_profile.json',combined)
        mem=combined
    path=EVID/'dynamic_history.csv';rows=list(csv.DictReader(path.open())) if path.exists() else []
    peak=mem.get('peak_project_working_set_gib');fine_resident=19.48876190185547
    resource=peak is not None and peak<=12
    ok=static.get('status')=='BENCHMARK_C_COARSE_STATIC_PASS' and dyn.get('status')=='BENCHMARK_C_COARSE_SCREENING_NUMERICAL_PASS' and dyn.get('completed_steps')==40 and resource and comparison.get('qualitative_trend_pass')
    staticrows=static.get('rows',[])
    rec={'timestamp':stamp(),'campaign':'BENCHMARK_C_COARSE_SCREENING','fine_mesh_cells':11364822,'coarse_mesh_cells':mesh.get('total_cells'),'cell_reduction_percent':mesh.get('cell_reduction_percent'),'background_cells':mesh.get('background_cells'),'component_cells':mesh.get('component_cells'),'critical_overlap_spacing_mm':mesh.get('critical_spacing_mm'),'far_field_spacing_mm':mesh.get('far_spacing_mm'),'static_poses':{r['pose']:r['status'] for r in staticrows},'peak_orphan':max([r.get('orphans',0) for r in staticrows]+[int(r['orphan_count']) for r in rows],default=None),'minimum_overlap_layers':min([r['background_actual_minimum_layers'] for r in staticrows]+[static.get('component_actual_minimum_layers',float('inf'))]) if staticrows else None,'max_donor_length_ratio':max([r['donor_characteristic_length_ratio']['max'] for r in staticrows]+[float(r['donor_length_ratio_max']) for r in rows],default=None),'peak_project_RAM_GiB':peak,'fine_comparison_resident_RAM_GiB':fine_resident,'RAM_reduction_percent_vs_observed_fine_sample':100*(1-peak/fine_resident) if peak else None,'RAM_comparison_note':'Fine value is an observed resident sample, not a proven fine peak.','dynamic_steps':dyn.get('completed_steps',0),'requested_steps':40,'simulation_time_ms':dyn.get('time_s',0)*1000,'peak_angular_velocity_rad_s':max([float(np.linalg.norm([float(r[f'omega_{a}_rad_s']) for a in 'xyz'])) for r in rows],default=None),'minimum_physical_clearance_mm':min([r['physical_clearance_mm'] for r in staticrows]+[float(r['robot_wall_clearance_m'])*1000 for r in rows],default=None),'coarse_vs_fine_trajectory_difference':comparison,'coarse_screening':'PASS' if ok else 'FAIL','current_blocker':None if ok else 'See failed static/dynamic/resource/trend gate evidence.','recommended_next':'Review this screening baseline before authorizing a 2ms coarse run or medium-grid convergence.' if ok else 'Resolve only the documented failed gate; preserve the fine mesh and frozen physics.','is_mesh_convergence_result':False}
    atomic(EVID/'final_report.json',rec);return rec

if __name__=='__main__':write_report()
