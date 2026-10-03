"""Consolidate real history and pre/post-ramp metrics; no mesh-convergence claim."""
import csv, json
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_2ms_common import *

def vectors(rows,keys):return np.asarray([[float(r[k]) for k in keys] for r in rows])
def rotations(rows):return Rotation.from_quat(vectors(rows,[f'q{i}' for i in [1,2,3,0]]))
def interval(rows):
    t=vectors(rows,['time_s']).ravel();p=vectors(rows,[f'com_{a}_m' for a in 'xyz'])
    v=vectors(rows,[f'v{a}_m_s' for a in 'xyz']);w=vectors(rows,[f'omega_{a}_rad_s' for a in 'xyz'])
    F=vectors(rows,[f'Fmag_{a}_N' for a in 'xyz']);T=vectors(rows,[f'Tmag_{a}_Nm' for a in 'xyz'])
    duration=t[-1]-t[0];r=rotations(rows);mean=lambda x:(np.trapezoid(x,t,axis=0)/duration).tolist()
    return {'start_time_s':float(t[0]),'end_time_s':float(t[-1]),'COM_displacement_m':(p[-1]-p[0]).tolist(),
        'mean_axial_velocity_m_s':float(mean(v)[0]),'mean_lateral_velocity_xyz_m_s':[0.,*mean(v)[1:]],
        'mean_lateral_speed_m_s':float(np.trapezoid(np.linalg.norm(v[:,1:],axis=1),t)/duration),
        'orientation_change_rad':float((r[-1]*r[0].inv()).magnitude()),
        'peak_angular_velocity_rad_s':float(np.linalg.norm(w,axis=1).max()),
        'mean_magnetic_force_xyz_N':mean(F),'peak_magnetic_force_N':float(np.linalg.norm(F,axis=1).max()),
        'mean_magnetic_force_magnitude_N':float(np.trapezoid(np.linalg.norm(F,axis=1),t)/duration),
        'mean_magnetic_torque_xyz_Nm':mean(T),'peak_magnetic_torque_Nm':float(np.linalg.norm(T,axis=1).max()),
        'mean_magnetic_torque_magnitude_Nm':float(np.trapezoid(np.linalg.norm(T,axis=1),t)/duration)}

def write_report():
    rows=history();n=int(rows[-1]['step']);dyn=read(EVID/'dynamic.json');st=read(EVID/'state.json')
    valid=[int(r['step']) for r in rows]==list(range(1,n+1))
    prefix=(EVID/'dynamic_history.csv').read_bytes().startswith((BASE/'dynamic_history.csv').read_bytes())
    finite=all(np.isfinite([float(v) for v in r.values()]).all() for r in rows)
    hard=all(int(r['orphan_count'])==0 and int(r['receptors_without_donors'])==0 and float(r['minimum_cell_volume_m3'])>0
        and float(r['robot_wall_clearance_m'])>=.0001 and float(r['overset_wall_clearance_m'])>0
        and abs(float(r['q_norm'])-1)<=1e-6 and abs(float(r['time_s'])-int(r['step'])*DT)<=1e-12 for r in rows)
    restart=read(EVID/'step40_restart_continuity.json');final=read(EVID/'step80_checkpoint_verification.json')
    gates={'sequence':valid,'original1ms_prefix':prefix,'finite':finite,'all_numerical':hard,
        'step40_restart':restart.get('status')=='BENCHMARK_C_COARSE_STEP40_NATIVE_RESTART_PASS',
        'step80_native_checkpoint':final.get('status')=='PASS' and final.get('timesteps_advanced')==0}
    checkpoints={}
    for step in [50,60,70,80]:
        cp=read(EVID/f'checkpoint_{step:04d}.json')
        if cp:checkpoints[str(step)]=cp
    zero={k:0. for k in rows[0]};zero.update(step=0,time_s=0.,com_x_m=COM0[0],q0=1.,q_norm=1.)
    arr=[zero,*rows];p=vectors(arr,[f'com_{a}_m' for a in 'xyz']);v=vectors(arr,[f'v{a}_m_s' for a in 'xyz']);w=vectors(arr,[f'omega_{a}_rad_s' for a in 'xyz'])
    F=vectors(arr,[f'Fmag_{a}_N' for a in 'xyz']);T=vectors(arr,[f'Tmag_{a}_Nm' for a in 'xyz']);rots=rotations(arr)
    report={'timestamp':stamp(),'campaign':CAMPAIGN,'mesh_cells':4699301,'background_cells':3345695,'component_cells':1353606,
        'completed_steps':n,'additional_steps_completed':n-40,'target_steps':80,'time_s':float(rows[-1]['time_s']),
        'restart_source':'step40 / 1.000ms native CASE+DATA','restart_gate':restart.get('status','NOT_READY'),
        'numerical_gates':gates,'final_COM_m':p[-1].tolist(),'total_COM_displacement_m':(p[-1]-COM0).tolist(),
        'maximum_lateral_displacement_m':float(np.linalg.norm(p[:,1:],axis=1).max()),
        'final_orientation_scalar_first':vectors([rows[-1]],[f'q{i}' for i in range(4)])[0].tolist(),
        'final_orientation_rotvec_rad':rots[-1].as_rotvec().tolist(),'maximum_orientation_change_rad':float(rots.magnitude().max()),
        'final_linear_velocity_m_s':v[-1].tolist(),'peak_linear_velocity_m_s':float(np.linalg.norm(v,axis=1).max()),
        'final_angular_velocity_rad_s':w[-1].tolist(),'peak_angular_velocity_rad_s':float(np.linalg.norm(w,axis=1).max()),
        'minimum_physical_clearance_m':min(float(r['robot_wall_clearance_m']) for r in rows),
        'orphan_peak':max(int(r['orphan_count']) for r in rows),'invalid_donor_peak':max(int(r['receptors_without_donors']) for r in rows),
        'maximum_donor_length_ratio':max(float(r['donor_length_ratio_max']) for r in rows),'donor_length_ratio_policy':'WARNING_ONLY',
        'minimum_cell_volume_m3':min(float(r['minimum_cell_volume_m3']) for r in rows),
        'maximum_q_norm_error':max(abs(float(r['q_norm'])-1) for r in rows),
        'peak_Fmag_N':float(np.linalg.norm(F,axis=1).max()),'peak_Tmag_Nm':float(np.linalg.norm(T,axis=1).max()),
        'pre_ramp_0_to_1ms':interval(arr[:41]),'post_ramp_1_to_current':interval(arr[40:]) if n>40 else None,
        'checkpoint_metadata':checkpoints,'fine_reference_status':'FINE_REFERENCE_NOT_AVAILABLE_AT_2MS',
        'exact_comparison_retained':'coarse_vs_fine_latest_exact.json','exact_comparison_time_s':.000825,
        'mesh_convergence_claim':False,'cutoff_sensitivity_run':False,'Benchmark_D_entered':False,
        'magnetic_sphere_example_used':False,'resource_summary':read(EVID/'memory_summary.json'),
        'engine_exit':read(EVID/'engine_exit.json'),'visualization_status':read(EVID/'visual_review.json').get('status','NOT_READY'),
        'remaining_blocker':st.get('error'),'status':'INCOMPLETE','dynamics_analysis_note':'Time-resolved finite differences and interval summaries; no causal attribution to ramp alone.'}
    if n==80 and dyn.get('status')=='BENCHMARK_C_COARSE_FREE_6DOF_2MS_PASS' and all(gates.values()) and len(checkpoints)==4:
        for cp in checkpoints.values():checkpoint_audit(cp)
        stats=[read(EVID/f'fielddata/connectivity_{i:04d}.json') for i in range(41,81)]
        if not all(s and s['total_cells']==4699301 and s['orphans']==0 and s['invalid_donors']==0 and s['minimum_volume_m3']>0 and s['cell_type_counts']['-3']==0 for s in stats):
            raise RuntimeError('Full40 additional official connectivity gates required')
        if read(EVID/'memory_summary.json').get('abort'):raise RuntimeError('Resource history stopped before full PASS')
        report.update(status='BENCHMARK_C_COARSE_FREE_6DOF_2MS_PASS',remaining_blocker=None)
    t=vectors(arr,['time_s']).ravel();dv=np.diff(v,axis=0)/np.diff(t)[:,None];dw=np.diff(w,axis=0)/np.diff(t)[:,None]
    # Inferred response is explicitly not a Fluent traction report or exact force decomposition.
    mass=8.904428864007322e-6;I=np.diag([7.257810693523445e-13,3.929384741151496e-12,3.92938474115149e-12])
    with (EVID/'dynamics_acceleration_and_response.csv').open('w',newline='') as fp:
        keys=['step','time_s']+[f'acceleration_{a}_m_s2' for a in 'xyz']+[f'angular_acceleration_{a}_rad_s2' for a in 'xyz']+[f'inferred_nonmagnetic_force_{a}_N' for a in 'xyz']+[f'inferred_nonmagnetic_torque_{a}_Nm' for a in 'xyz']
        writer=csv.DictWriter(fp,fieldnames=keys);writer.writeheader()
        for i,r in enumerate(rows,1):
            R=rots[i].as_matrix();J=R@I@R.T;f=mass*dv[i-1]-F[i];torque=J@dw[i-1]+np.cross(w[i],J@w[i])-T[i]
            writer.writerow(dict(zip(keys,[i,t[i],*dv[i-1],*dw[i-1],*f,*torque])))
    report['inferred_response_note']='m*finite_difference(v)-Fmag and rotated-inertia Euler residual minus Tmag. Diagnostic only; not direct fluid pressure/shear forces.'
    if n>40:
        report['peak_post_ramp_angular_acceleration_rad_s2']=float(np.linalg.norm(dw[40:],axis=1).max())
        report['peak_pre_ramp_angular_acceleration_rad_s2']=float(np.linalg.norm(dw[:40],axis=1).max())
    atomic(EVID/'final_report.json',report);return report

if __name__=='__main__':print(json.dumps({'status':write_report()['status']}))
