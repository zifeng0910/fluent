"""Offline one-event write and next-step inheritance audit; no Fluent connection."""
import json
import numpy as np
from benchmark_D_overwrite_review import *
from benchmark_D_lifecycle_review import difference

INERTIA=np.array([[7.257810693523445e-13,2.782693607479792e-28,-1.9327591150313555e-29],
 [2.782693607479792e-28,3.929384741151496e-12,-2.4813784672159598e-29],
 [-1.9327591150313555e-29,-2.4813784672159598e-29,3.9293847411514897e-12]])

def main():
    verify_frozen()
    admission=read(EVID/'controlled_write_admission.json')
    if admission.get('status')!='PASS':raise RuntimeError('Reviewed semantics/signal admission required')
    for p,k in [('no_op_artifact_quantification.json','quantification_sha256'),
                ('theta_pose_semantics_audit.json','semantics_sha256'),('configuration.json','configuration_sha256')]:
        if sha(EVID/p)!=admission[k]:raise RuntimeError('Admission evidence changed')
    integrity('controlled')
    events=trace('controlled');applied=[r for r in events if r['action']=='APPLIED']
    if len(applied)!=1:raise RuntimeError('Exactly one compute-node physical impulse required')
    ev=applied[0];R=rotation(ev['DT_Q']).as_matrix()
    V,W,J=reference(8.904428864007322e-6,R@INERTIA@R.T,np.asarray(ev['contact_point'])-ev['DT_CG'],
        np.asarray(ev['contact_normal']),np.asarray(ev['get_velocity']),np.asarray(ev['get_omega']))
    hh=history('controlled');first=read(EVID/'controlled/branch_result.json')['first_contact_completed_iteration']
    post=hh[first-1];nextpost=hh[first];before=read(EVID/'controlled'/f'before_step_{first+1:02d}.json')
    free=next(r for r in history('A1') if abs(r['time_s']-post['time_s'])<1e-12)
    math_errors=dict(velocity=difference(ev['requested_velocity'],V),omega=difference(ev['requested_omega'],W),impulse_abs_N_s=abs(ev['impulse_N_s']-J))
    math_pass=math_errors['velocity']['max_abs']<1e-10 and math_errors['omega']['max_abs']<1e-7 and math_errors['impulse_abs_N_s']<1e-16
    intended_v=V-np.asarray(ev['get_velocity']);intended_w=W-np.asarray(ev['get_omega'])
    observed_v=np.asarray(post['native_state']['velocity'])-free['native_state']['velocity']
    observed_w=np.asarray(post['native_state']['omega'])-free['native_state']['omega']
    changes=dict(velocity=difference(observed_v,intended_v),omega=difference(observed_w,intended_w))
    correct=all(d['relative_error'] is not None and d['relative_error']<=.05 and d['direction_cosine']>=.999 for d in changes.values())
    inherited=compare(before['native_state'],post['native_state'])
    entries=[r for r in rows(EVID/'controlled/theta_lifecycle_node.jsonl') if r['stage']=='PROPERTIES_ENTRY'
        and abs(r['CURRENT_TIME']-post['time_s'])<1e-12 and abs(r['property_time']-nextpost['time_s'])<1e-12]
    entry=entries[0] if entries else None
    entry_pass=bool(entry) and difference(entry['velocity'],post['native_state']['velocity'])['max_abs']<=1e-8 and difference(entry['omega'],post['native_state']['omega'])['max_abs']<=1e-6
    next_pass=all(x['status']=='PASS' for x in inherited.values()) and entry_pass
    one_event=all(not r['overwrite_called'] for r in events if r['action']!='APPLIED')
    host_applied=[r for r in rows(EVID/'controlled/native_contact_callback_trace_host.jsonl') if r['action']=='APPLIED']
    # Host and compute node are copies of the same one-rank distributed body,
    # not two separate impulses. The first event time/body/contact must match.
    host_same=len(host_applied)==1 and all(host_applied[0][k]==ev[k] for k in ['CURRENT_TIME','dynamic_zone_id','opposite_zone_id','contact_point','impulse_N_s'])
    semrows=rows(EVID/'controlled/theta_lifecycle_node.jsonl')
    cb_before=next(r for r in semrows if r['stage']=='CALLBACK_BEFORE' and r['callback_index']==ev['callback_index'])
    cb_after=next(r for r in semrows if r['stage']=='CALLBACK_AFTER' and r['callback_index']==ev['callback_index'])
    pending=dict(classification='EMPIRICALLY_MEASURED',
        getter_after_vs_requested=dict(velocity=difference(cb_after['get_velocity'],V),omega=difference(cb_after['get_omega'],W)),
        visible_native_velocity_change_m_s=float(np.linalg.norm(np.asarray(cb_after['velocity'])-cb_before['velocity'])),
        visible_native_omega_change_rad_s=float(np.linalg.norm(np.asarray(cb_after['omega'])-cb_before['omega'])),
        visible_native_CG_change_m=float(np.linalg.norm(np.asarray(cb_after['CG'])-cb_before['CG'])),
        visible_native_Q_change_rad=angle(cb_after['Q'],cb_before['Q']),
        getter_theta_before=cb_before['get_theta'],getter_theta_after=cb_after['get_theta'],
        interpretation='EMPIRICALLY_INFERRED pending contact-motion storage: getter reflects requested motion while current DT fields remain unchanged immediately; exact internal storage identity UNKNOWN. This observation is not the write PASS gate.')
    passed=math_pass and correct and next_pass and one_event and host_same
    out=dict(timestamp=stamp(),status='PASS' if passed else 'FAIL',
        result='NATIVE_CONTACT_RESPONSE_PATH_VALIDATED' if passed else 'NATIVE_CONTACT_WRITE_NOT_VALIDATED',
        scope='Exactly one e_n=0,mu=0 contact event plus one next step; ending1.875ms. No full collision or1.9/2ms run.',
        actual_callback=ev,predicted_velocity=V.tolist(),predicted_omega=W.tolist(),predicted_impulse_N_s=J,
        independent_impulse_validation=math_errors,impulse_math_PASS=math_pass,
        actual_completed_step=post,completed_vs_requested=dict(velocity=difference(post['native_state']['velocity'],V),omega=difference(post['native_state']['omega'],W)),
        intended_change=dict(velocity=intended_v.tolist(),omega=intended_w.tolist()),
        observed_change=dict(velocity=observed_v.tolist(),omega=observed_w.tolist()),
        expected_vs_observed_change=changes,completed_step_retains_intended_change=bool(correct),
        next_start_native=before,next_start_errors=inherited,actual_next_properties_entry=entry,
        next_properties_entry_matches_completed=entry_pass,next_timestep_inherited=bool(next_pass),
        next_completed_state=nextpost,physical_event_count=len(applied),host_node_same_event=host_same,
        no_second_overwrite=one_event,immediate_callback_reread_used_as_PASS=False,
        immediate_pending_motion_observation=pending,
        change_acceptance_relative_error=.05,minimum_direction_cosine=.999)
    atomic(EVID/'native_controlled_write_validation.json',out)
    build_csv()
    print(json.dumps(dict(status=out['status'],result=out['result'],change_errors=changes,next_timestep_inherited=out['next_timestep_inherited']),indent=2))

if __name__=='__main__':main()
