"""Offline same-C70 repeatability, native angle, endpoint integration and admission audit."""
import csv,json
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark_D_overwrite_common import *
from benchmark_C_recovery_v2_restart import compare
from benchmark_D_validate_impulse import reference

def rotation(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
def angle(q1,q0):return float((rotation(q1)*rotation(q0).inv()).magnitude())
def trace(branch):return [r for r in rows(EVID/branch/'native_contact_callback_trace_node.jsonl') if r['moving_body'] and r['contact_face_count']>0]
def history(branch):return read(EVID/branch/'native_history.json')['records']
def integrity(branch):
    import benchmark_D_lifecycle_review as old_review
    old_review.EVID=EVID
    return old_review.integrity(branch)

def compare_branches(a,b):
    ah=history(a);bh=history(b)
    if len(ah)!=5 or len(bh)!=5:raise RuntimeError('Exactly5 completed diagnostic timesteps required')
    rr=[]
    for x,y in zip(ah,bh):
        if abs(x['time_s']-y['time_s'])>1e-12:raise RuntimeError('Repeats compared at different times')
        aa=x['native_state'];bb=y['native_state']
        rr.append(dict(time_s=x['time_s'],COM_norm_difference_m=float(np.linalg.norm(np.asarray(bb['com'])-aa['com'])),
            COM_max_component_difference_m=float(np.max(abs(np.asarray(bb['com'])-aa['com']))),orientation_difference_rad=angle(bb['q'],aa['q']),
            velocity_norm_difference_m_s=float(np.linalg.norm(np.asarray(bb['velocity'])-aa['velocity'])),omega_norm_difference_rad_s=float(np.linalg.norm(np.asarray(bb['omega'])-aa['omega']))))
    ta=trace(a);tb=trace(b);keys=['CURRENT_TIME','timestep_index','contact_face_count','contact_point','contact_normal']
    callbacks_same=len(ta)==len(tb) and all(all(x[k]==y[k] for k in keys) for x,y in zip(ta,tb))
    peaks={k:max(r[k] for r in rr) for k in rr[0] if k!='time_s'}
    tol=read(EVID/'configuration.json')['reproducibility_tolerance']
    passed=peaks['COM_max_component_difference_m']<=tol['com_m'] and peaks['orientation_difference_rad']<=tol['orientation_rad'] and peaks['velocity_norm_difference_m_s']<=tol['velocity_m_s'] and peaks['omega_norm_difference_rad_s']<=tol['omega_rad_s'] and callbacks_same
    return dict(timestamp=stamp(),branches=[a,b],status='PASS' if passed else 'FAIL',peaks=peaks,each_step=rr,callback_count=len(ta),callback_time_face_geometry_equal=callbacks_same,
                tolerance=tol,bitwise_identity_required=False)

def build_csv():
    records=[]
    for branch in ['A1','A2','B1','B2','controlled']:
        for r in rows(EVID/branch/'theta_lifecycle_node.jsonl'):
            out={k:r[k] for k in ['stage','CURRENT_TIME','property_time','dtime','callback_index','timestep_index']};out['branch']=branch
            for key in ['get_theta','DT_THETA','Q','CG','omega','velocity','Theta_From_Q_native','Euler_From_Q_native']:
                for i,x in enumerate(r[key]):out[f'{key}_{i}']=x
            records.append(out)
    if records:
        with (EVID/'theta_quaternion_semantics.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    return records

def semantics_audit():
    allrows=[r for branch in ['A1','A2','B1','B2'] for r in rows(EVID/branch/'theta_lifecycle_node.jsonl')]
    helpers=dict(DT_THETA_vs_Theta_From_Q_max_abs=max(float(np.max(abs(np.asarray(r['DT_THETA'])-r['Theta_From_Q_native']))) for r in allrows),
        DT_THETA_vs_rotation_vector_max_abs=max(float(np.max(abs(np.asarray(r['DT_THETA'])-rotation(r['Q']).as_rotvec()))) for r in allrows),
        Q_From_DT_THETA_max_orientation_error_rad=max(angle(r['Q_From_DT_THETA_native'],r['Q']) for r in allrows),
        DT_THETA_vs_Euler_From_Q_max_abs=max(float(np.max(abs(np.asarray(r['DT_THETA'])-r['Euler_From_Q_native']))) for r in allrows),
        get_theta_max_abs=max(float(np.max(abs(np.asarray(r['get_theta'])))) for r in allrows),
        omega_dt_nonzero_in_callbacks=all(np.linalg.norm(r['omega'])>1 for r in allrows if r['stage']=='CALLBACK_BEFORE'))
    callback=[r for r in allrows if r['stage']=='CALLBACK_BEFORE']
    helpers['get_theta_not_absolute_native_DT_THETA']=all(np.linalg.norm(np.asarray(r['get_theta'])-r['DT_THETA'])>.01 for r in callback)
    hypotheses=[]
    for branch in ['B1','B2']:
        states=history(branch);starts=[read(EVID/branch/'state_0070.json'),*states[:-1]]
        semantic=rows(EVID/branch/'theta_lifecycle_node.jsonl')
        for event in trace(branch):
            i=next(i for i,s in enumerate(starts) if abs(s['time_s']-event['CURRENT_TIME'])<1e-12)
            prev=starts[i]['native_state'];post=states[i]['native_state'];dt=25e-6
            eCG=np.asarray(prev['com'])+dt*np.asarray(event['get_velocity'])
            eR=Rotation.from_rotvec(np.asarray(event['get_omega'])*dt)*rotation(prev['q'])
            before=next(r for r in semantic if r['stage']=='CALLBACK_BEFORE' and r['callback_index']==event['callback_index'])
            after=next(r for r in semantic if r['stage']=='CALLBACK_AFTER' and r['callback_index']==event['callback_index'])
            hypotheses.append(dict(branch=branch,callback_time_s=event['CURRENT_TIME'],completed_time_s=states[i]['time_s'],
                endpoint_CG_residual_m=float(np.linalg.norm(np.asarray(post['com'])-eCG)),
                endpoint_world_omega_Q_residual_rad=float((rotation(post['q'])*eR.inv()).magnitude()),
                immediate_after_CG_matches_completed_m=float(np.linalg.norm(np.asarray(after['CG'])-post['com'])),
                immediate_after_Q_matches_completed_rad=angle(after['Q'],post['q']),
                overwrite_immediate_CG_change_m=float(np.linalg.norm(np.asarray(after['CG'])-before['CG'])),
                overwrite_immediate_Q_change_rad=angle(after['Q'],before['Q']),
                getter_velocity_unchanged=before['get_velocity']==after['get_velocity'],getter_omega_unchanged=before['get_omega']==after['get_omega'],
                getter_theta_before=before['get_theta'],getter_theta_after=after['get_theta'],DT_THETA_before=before['DT_THETA'],DT_THETA_after=after['DT_THETA']))
    free=[]
    stages=[]
    import pyvista as pv
    for branch in ['A1','A2']:
        states=history(branch);starts=[read(EVID/branch/'state_0070.json'),*states[:-1]]
        for event in trace(branch):
            i=next(i for i,s in enumerate(starts) if abs(s['time_s']-event['CURRENT_TIME'])<1e-12)
            prev=starts[i]['native_state'];post=states[i]['native_state']
            eCG=np.asarray(prev['com'])+.5*25e-6*(np.asarray(prev['velocity'])+post['velocity'])
            free.append(dict(branch=branch,time_s=states[i]['time_s'],trapezoidal_CG_residual_m=float(np.linalg.norm(np.asarray(post['com'])-eCG))))
            def tube_gap(step):
                mesh=pv.read(EVID/branch/'fielddata'/f'state_{step:04d}_robot.vtp')
                return float(.0009-np.sqrt((mesh.points[:,1:]**2).sum(axis=1)).max())
            stages.append(dict(branch=branch,callback_CURRENT_TIME_s=event['CURRENT_TIME'],completed_time_s=states[i]['time_s'],
                callback_CG_vs_completed_m=float(np.linalg.norm(np.asarray(event['DT_CG'])-post['com'])),
                callback_Q_vs_completed_rad=angle(event['DT_Q'],post['q']),
                callback_actual_vertices_ideal_gap_m=event['ideal_tube_gap_from_actual_vertices_m'],
                previous_export_vertices_ideal_gap_m=tube_gap(starts[i]['native_step_index']),
                completed_export_vertices_ideal_gap_m=tube_gap(states[i]['native_step_index']),
                surface_export_precision_tolerance_m=1e-10))
    endpoint_explained=bool(hypotheses) and all(x['endpoint_CG_residual_m']<1e-12 and x['endpoint_world_omega_Q_residual_rad']<1e-9 and x['getter_velocity_unchanged'] and x['getter_omega_unchanged'] for x in hypotheses)
    out=dict(timestamp=stamp(),status='PARTIAL',native_angle_helpers=helpers,
        Get_Motion_theta_semantics='EMPIRICALLY BOUNDED: initialized-zero third array remains0 here while native absolute orientation evolves; not the absolute DT_THETA or omega*dt. Whether Get deliberately omits assigning it in this6DOF path is UNKNOWN.',
        native_DT_THETA='EMPIRICALLY_MEASURED in this trajectory: absolute rotation vector matching nativeTheta_From_Q and an independent quaternion rotation vector. It differs from nativeEuler_From_Q; no converter output is fed to Overwrite.',
        overwrite_pose_reconstruction=dict(classification='EMPIRICALLY_MEASURED',endpoint_model_verified=endpoint_explained,
            visible_CG_Q_unchanged_immediately_after=all(x['overwrite_immediate_CG_change_m']<1e-15 and x['overwrite_immediate_Q_change_rad']<1e-12 for x in hypotheses),
            delayed_effect_interpretation='EMPIRICALLY_INFERRED: overwrite changes subsequent motion construction, not necessarily immediately visible DT_CG/DT_Q. Internal buffer/flag identity UNKNOWN.',
            model='CG_next=CG_start+dt*v_endpoint; Q_next=worldRotation(omega_endpoint*dt)*Q_start',records=hypotheses),
        read_only_translation_model=dict(classification='EMPIRICALLY_MEASURED',model='CG_next=CG_start+0.5*dt*(v_start+v_endpoint)',records=free),
        callback_stage=dict(classification='EMPIRICALLY_INFERRED',statement='Actual contact occurs with endpoint predicted v/omega and pose, while CURRENT_TIME/N_TIME and actual surface geometry remain at the previous completed step; before final mesh relocation. Exact internal predictor/corrector flag UNKNOWN.',records=stages),
        official_guarantees_noop_equals_omission=False,none_of_the_native_conversions_sent_to_overwrite=True)
    atomic(EVID/'theta_pose_semantics_audit.json',out);return out

def artifact_and_admission():
    ref=history('A1');base=read(EVID/'A1/state_0070.json');i=3;prev=ref[i-1]['native_state'];post=ref[i]['native_state'];no=history('B1')[i]['native_state'];dt=25e-6
    cg_floor=float(np.linalg.norm(np.asarray(no['com'])-post['com']));q_floor=angle(no['q'],post['q'])
    normal_cg=float(np.linalg.norm(np.asarray(post['com'])-prev['com']));normal_angle=angle(post['q'],prev['q'])
    event=trace('A1')[0];R=rotation(event['DT_Q']).as_matrix();mass=8.904428864007322e-6
    I=np.array([[7.257810693523445e-13,2.782693607479792e-28,-1.9327591150313555e-29],
                [2.782693607479792e-28,3.929384741151496e-12,-2.4813784672159598e-29],
                [-1.9327591150313555e-29,-2.4813784672159598e-29,3.9293847411514897e-12]])
    V,W,J=reference(mass,R@I@R.T,np.asarray(event['contact_point'])-event['DT_CG'],np.asarray(event['contact_normal']),np.asarray(event['get_velocity']),np.asarray(event['get_omega']))
    dv=float(np.linalg.norm(V-event['get_velocity']));dw=float(np.linalg.norm(W-event['get_omega']))
    a=read(EVID/'read_only_reproducibility.json');b=read(EVID/'noop_reproducibility.json')
    cross=read(EVID/'noop_vs_read_only.json')
    # Conservative admission also includes the accumulated two-callback pose
    # difference at the fifth boundary, although controlled writes only once.
    v_floor=max(cross['peaks']['COM_norm_difference_m']/dt,a['peaks']['velocity_norm_difference_m_s'],b['peaks']['velocity_norm_difference_m_s'],1e-8)
    w_floor=max(cross['peaks']['orientation_difference_rad']/dt,a['peaks']['omega_norm_difference_rad_s'],b['peaks']['omega_norm_difference_rad_s'],1e-6)
    ratios=dict(linear_velocity=dv/v_floor,angular_velocity=dw/w_floor)
    semantics=read(EVID/'theta_pose_semantics_audit.json');explained=semantics['overwrite_pose_reconstruction']['endpoint_model_verified']
    deterministic=a['status']=='PASS' and b['status']=='PASS'
    read_valid=all(not r['dt_NULL'] and not r['motion_all_zero'] and not r['overwrite_called'] and
        np.max(abs(np.asarray(r['get_velocity'])-r['native_current_velocity']))<=1e-8 and
        np.max(abs(np.asarray(r['get_omega'])-r['native_current_omega']))<=1e-6
        for branch in ['A1','A2'] for r in trace(branch))
    noop_exact=all(r['action']=='NOOP_OVERWRITE' and r['overwrite_called'] and
        r['get_velocity']==r['requested_velocity'] and r['get_omega']==r['requested_omega'] and r['get_theta']==[0,0,0]
        for branch in ['B1','B2'] for r in trace(branch))
    passed=deterministic and explained and read_valid and noop_exact and min(ratios.values())>10 and J>0 and semantics['native_angle_helpers']['get_theta_max_abs']==0
    out=dict(timestamp=stamp(),first_contact_completed_time_s=ref[i]['time_s'],
        COM_difference_norm_m=cg_floor,COM_difference_max_component_m=float(np.max(abs(np.asarray(no['com'])-post['com']))),orientation_difference_rad=q_floor,
        orientation_difference_deg=q_floor*180/np.pi,velocity_norm_times_dt_m=float(np.linalg.norm(post['velocity'])*dt),omega_norm_times_dt_rad=float(np.linalg.norm(post['omega'])*dt),
        actual_normal_step_COM_displacement_m=normal_cg,actual_normal_step_orientation_increment_rad=normal_angle,
        relative_COM_artifact_percent=100*cg_floor/normal_cg,relative_orientation_artifact_percent=100*q_floor/normal_angle,
        significant_vs_normal_step=bool(max(cg_floor/normal_cg,q_floor/normal_angle)>.01),
        real_callback_read_PASS=bool(read_valid),exact_same_value_noop_arrays=bool(noop_exact),
        explained_as_endpoint_pose_reconstruction=explained,solver_noise_can_explain_artifact=not deterministic,
        empirically_bounded_theta_same_return_only=True,signal_to_artifact_floor=ratios,
        admission_floor_scope='Maximum accumulated A1/B1 pose difference over all five boundaries, converted by dt; includes two no-op contact calls, conservative for the one-event controlled experiment',
        peak_noop_vs_read_only=cross['peaks'],
        intended_normal_impulse_N_s=J,intended_delta_velocity_norm_m_s=dv,intended_delta_omega_norm_rad_s=dw,
        equivalent_velocity_artifact_m_s=v_floor,equivalent_omega_artifact_rad_s=w_floor,
        status='PASS' if passed else 'STOP',
        decision='One controlled event only to1.875ms; no micro-run' if passed else 'Stop; semantics or signal-to-artifact admission insufficient')
    atomic(EVID/'no_op_artifact_quantification.json',out)
    atomic(EVID/'controlled_write_admission.json',dict(timestamp=stamp(),status='PASS' if passed else 'FAIL',
        reason=out['decision'],quantification_sha256=sha(EVID/'no_op_artifact_quantification.json'),
        semantics_sha256=sha(EVID/'theta_pose_semantics_audit.json'),configuration_sha256=sha(EVID/'configuration.json'),
        minimum_signal_to_floor=10,passed_motion_array_read=bool(read_valid),exact_same_value_noop_arrays=bool(noop_exact),theta_not_manually_converted=True,
        limited_to_one_event=True,end_time_s=.001875))
    return out

def main():
    verify_frozen()
    for branch in ['A1','A2','B1','B2']:integrity(branch)
    a=compare_branches('A1','A2');atomic(EVID/'read_only_reproducibility.json',a)
    b=compare_branches('B1','B2');atomic(EVID/'noop_reproducibility.json',b)
    cross=compare_branches('A1','B1');atomic(EVID/'noop_vs_read_only.json',cross)
    build_csv();semantics_audit();out=artifact_and_admission()
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
