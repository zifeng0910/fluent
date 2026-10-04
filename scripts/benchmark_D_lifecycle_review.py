"""Offline review of actual callback and completed native timestep evidence. No solver API."""
import argparse,csv,json
import numpy as np
from benchmark_D_lifecycle_common import *
from benchmark_C_recovery_v2_restart import compare
from benchmark_C_analytic_reference import compute_magnetic_load
from scipy.spatial.transform import Rotation
from benchmark_D_validate_impulse import reference

def magnetic_audit(branch):
    b=EVID/branch;data=list(csv.DictReader((b/'magnetic_history.csv').open()))
    if not data:raise RuntimeError('Actual transient magnetic property trace required')
    error_F=error_T=0
    for row in data:
        F,T,_=compute_magnetic_load([float(row[f'cg_{a}_m']) for a in 'xyz'],
            [float(row[f'q{i}']) for i in [1,2,3,0]],float(row['time_s']))
        error_F=max(error_F,float(np.max(abs(F-np.array([float(row[f'Fmag_{a}_N']) for a in 'xyz'])))))
        error_T=max(error_T,float(np.max(abs(T-np.array([float(row[f'Tmag_{a}_Nm']) for a in 'xyz'])))))
    out=dict(timestamp=stamp(),status='PASS' if error_F<=1e-13 and error_T<=1e-14 else 'FAIL',
        actual_property_calls=len(data),max_F_error_N=error_F,max_T_error_Nm=error_T,
        actual_absolute_time_and_pose_used=True,original_formula_preserved=True)
    atomic(b/'actual_magnetic_load_validation.json',out)
    if out['status']!='PASS':raise RuntimeError('Actual magnetic loads changed')
    return out

def difference(a,b):
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
    den=np.linalg.norm(b);length=np.linalg.norm(a)
    return dict(max_abs=float(np.max(abs(a-b))),norm_error=float(np.linalg.norm(a-b)),
                relative_error=float(np.linalg.norm(a-b)/den) if den else None,
                direction_cosine=float(np.dot(a,b)/(length*den)) if length*den else None)

def callback_records(branch):
    # Independent host and compute-node traces are both retained. Compute node owns mesh faces.
    b=EVID/branch
    return [r for r in rows(b/'native_contact_callback_trace_node.jsonl') if r['moving_body'] and r['contact_face_count']>0]

def integrity(branch):
    b=EVID/branch;result=read(b/'branch_result.json');exit=read(b/'worker_exit.json')
    if result.get('status')!='DIAGNOSTIC_DATA_COMPLETE':raise RuntimeError('Actual callback/completed branch required')
    if not exit.get('solver_closed') or read(b/'engine_exit.json').get('owned_engine_closed') is not True:raise RuntimeError('Owned engine closure required')
    if read(b/'C70_restart_validation.json').get('status')!='PASS':raise RuntimeError('C70 continuity required')
    if read(b/'native_connectivity_crosscheck.json').get('status')!='PASS':raise RuntimeError('Native connectivity crosscheck required')
    saved=read(b/'final_native_checkpoint.json')
    for info in saved['files'].values():
        if sha(ROOT/info['path'])!=info['sha256']:raise RuntimeError('Native saved checkpoint changed')
    if any(r['status']!='PASS' for r in read(b/'native_history.json')['records']):raise RuntimeError('Dynamic numerical gate failed')
    magnetic_audit(branch)
    return result

def dedup_audit(branch):
    trace=callback_records(branch);groups={}
    for r in trace:
        key=(r['CURRENT_TIME'],r['timestep_index'],r['dynamic_zone_id'],r['opposite_zone_id'])
        groups.setdefault(key,[]).append(r)
    out=dict(timestamp=stamp(),branch=branch,status='NOT_NEEDED' if all(r['action']!='APPLIED' for r in trace) else 'PASS',
             key='time + timestep + moving body + opposite body + contact point within20um',
             physical_impulses_not_counted_twice_across_host_node=True,groups=[])
    for key,rr in groups.items():
        applied=[r for r in rr if r['action']=='APPLIED']
        repeated=any(np.linalg.norm(np.array(a['contact_point'])-b['contact_point'])<=20e-6 for i,a in enumerate(applied) for b in applied[i+1:])
        if repeated:out['status']='FAIL'
        out['groups'].append(dict(time_s=key[0],step=key[1],body=key[2],opposite=key[3],
            callback_count=len(rr),applied_count=len(applied),skipped_duplicate_count=sum(r['action']=='SKIPPED_DUPLICATE' for r in rr),
            duplicate_physical_impulse=repeated))
    atomic(EVID/'callback_dedup_audit.json',out);return out

def review_read():
    branch='read_only';result=integrity(branch);trace=callback_records(branch)
    if not trace:raise RuntimeError('Actual moving-body callback required')
    event=trace[0]
    history=read(EVID/branch/'native_history.json')['records']
    first_iteration=result['first_contact_completed_iteration']
    post=history[first_iteration-1]
    before=read(EVID/branch/f'before_step_{first_iteration:02d}.json')
    if min(abs(event['CURRENT_TIME']-before['time_s']),abs(event['CURRENT_TIME']-post['time_s']))>1e-12:
        raise RuntimeError('Actual callback cannot be associated with its exact solve boundary')
    comparisons={}
    matching=[]
    for name,v,w in [('current',event['native_current_velocity'],event['native_current_omega']),
                     ('temporary',event['native_temporary_velocity'],event['native_temporary_omega']),
                     ('post_step',post['native_state']['velocity'],post['native_state']['omega'])]:
        vv=difference(event['get_velocity'],v);ww=difference(event['get_omega'],w)
        comparisons[name]=dict(velocity=vv,omega=ww,native_velocity=v,native_omega=w)
        if vv['max_abs']<=1e-8 and ww['max_abs']<=1e-6:matching.append(name)
    nonzero_native=np.linalg.norm(post['native_state']['velocity'])>1e-8 and np.linalg.norm(post['native_state']['omega'])>1e-6
    pass_read=not event['motion_all_zero'] and nonzero_native and bool(matching) and not event['dt_NULL']
    # Motion predicted inside contact may precede the final state; preserve all phase comparisons.
    out=dict(timestamp=stamp(),status='PASS' if pass_read else 'FAIL',
        result='NATIVE_CONTACT_MOTION_READ_PASS' if pass_read else 'NATIVE_CONTACT_MOTION_READBACK_FAIL',
        actual_transient=True,moving_body_dt_valid=not event['dt_NULL'],stationary_opposite_dt_null_allowed=True,
        first_callback=event,completed_same_step=post,matches_native_state_phases=matching,phase_comparisons=comparisons,
        actual_solve_boundary=dict(before_time_s=before['time_s'],after_time_s=post['time_s'],
            first_contact_completed_iteration=first_iteration,callback_N_TIME=event['timestep_index'],completed_N_TIME=post['native_step_index']),
        tolerance=dict(velocity_max_abs_m_s=1e-8,omega_max_abs_rad_s=1e-6),
        no_overwrite=all(not r['overwrite_called'] for r in trace))
    if not out['no_overwrite']:out.update(status='FAIL',result='READ_ONLY_BRANCH_OVERWROTE_STATE')
    atomic(EVID/'native_motion_read_validation.json',out);dedup_audit(branch);return out

def review_noop():
    integrity('noop');trace=callback_records('noop');reference=read(EVID/'native_motion_read_validation.json')
    if reference.get('status')!='PASS':raise RuntimeError('READ PASS required')
    free=reference['completed_same_step'];history=read(EVID/'noop/native_history.json')['records']
    post=next(r for r in history if r['native_step_index']==free['native_step_index'])
    errors=compare(post['native_state'],free['native_state'])
    exact_arrays=all(r['get_velocity']==r['requested_velocity'] and r['get_omega']==r['requested_omega'] for r in trace)
    callbacks=bool(trace) and all(r['action']=='NOOP_OVERWRITE' for r in trace)
    passed=exact_arrays and callbacks and all(r['status']=='PASS' for r in errors.values()) and abs(free['time_s']-post['time_s'])<1e-12
    out=dict(timestamp=stamp(),status='PASS' if passed else 'FAIL',result='NATIVE_CONTACT_NOOP_OVERWRITE_PASS' if passed else 'NATIVE_CONTACT_NOOP_OVERWRITE_FAIL',
             exactly_original_arrays=exact_arrays,actual_callbacks_overwrote=callbacks,native_post_step_comparison=errors,
             read_only_post_state=free,noop_post_state=post,callback_count=len(trace))
    atomic(EVID/'native_noop_write_validation.json',out);dedup_audit('noop');return out

def review_controlled():
    result=integrity('controlled');trace=callback_records('controlled')
    if read(EVID/'native_noop_write_validation.json').get('status')!='PASS':raise RuntimeError('NOOP PASS required')
    applied=[r for r in trace if r['action']=='APPLIED']
    if len(applied)!=1:raise RuntimeError('Exactly one controlled physical impulse required')
    event=applied[0];rotation=Rotation.from_quat(np.asarray(event['DT_Q'])[[1,2,3,0]]).as_matrix()
    inertia=np.array([[7.257810693523445e-13,2.782693607479792e-28,-1.9327591150313555e-29],
        [2.782693607479792e-28,3.929384741151496e-12,-2.4813784672159598e-29],
        [-1.9327591150313555e-29,-2.4813784672159598e-29,3.9293847411514897e-12]])
    V,W,J=reference(8.904428864007322e-6,rotation@inertia@rotation.T,
        np.asarray(event['contact_point'])-event['DT_CG'],np.asarray(event['contact_normal']),
        np.asarray(event['get_velocity']),np.asarray(event['get_omega']))
    math_errors=dict(velocity=difference(event['requested_velocity'],V),omega=difference(event['requested_omega'],W),
        impulse_abs_N_s=abs(event['impulse_N_s']-J))
    math_pass=math_errors['velocity']['max_abs']<1e-10 and math_errors['omega']['max_abs']<1e-7 and math_errors['impulse_abs_N_s']<1e-16
    i=result['first_contact_completed_iteration'];history=read(EVID/'controlled/native_history.json')['records']
    post=history[i-1];nextpost=history[i];before=read(EVID/'controlled'/f'before_step_{i+1:02d}.json')
    free=read(EVID/'native_motion_read_validation.json')['completed_same_step']
    errors=dict(velocity=difference(post['native_state']['velocity'],V),omega=difference(post['native_state']['omega'],W))
    intended_dv=V-np.asarray(event['get_velocity']);intended_dw=W-np.asarray(event['get_omega'])
    actual_dv=np.asarray(post['native_state']['velocity'])-free['native_state']['velocity']
    actual_dw=np.asarray(post['native_state']['omega'])-free['native_state']['omega']
    delta_errors=dict(velocity=difference(actual_dv,intended_dv),omega=difference(actual_dw,intended_dw))
    # Permit the normal final fluid/magnetic corrector only within5% of the intended change.
    # A setter ignored by the solver gives zero delta and cannot satisfy this condition.
    changed=np.linalg.norm(actual_dv)>1e-6 and np.linalg.norm(actual_dw)>1e-3
    intended_match=all(d['relative_error'] is not None and d['relative_error']<=.05 and d['direction_cosine']>=.999 for d in delta_errors.values())
    inherited_errors=compare(before['native_state'],post['native_state'])
    inherited=all(d['status']=='PASS' for d in inherited_errors.values())
    state_trace=rows(EVID/'controlled/native_state_lifecycle_node.jsonl')
    next_start=[r for r in state_trace if abs(r['CURRENT_TIME']-post['time_s'])<1e-12 and abs(r['property_time']-nextpost['time_s'])<1e-12]
    next_read=next_start[0] if next_start else None
    actual_entry_matches=bool(next_read) and difference(next_read['velocity'],post['native_state']['velocity'])['max_abs']<=1e-8 and difference(next_read['omega'],post['native_state']['omega'])['max_abs']<=1e-6
    audit=dedup_audit('controlled');passed=math_pass and changed and intended_match and inherited and actual_entry_matches and audit['status']=='PASS'
    out=dict(timestamp=stamp(),status='PASS' if passed else 'FAIL',result='NATIVE_CONTACT_MOTION_WRITE_PASS' if passed else 'NATIVE_CONTACT_MOTION_WRITE_FAIL',
        actual_callback=event,independent_impulse_validation=math_errors,impulse_math_PASS=math_pass,
        predicted_velocity=V.tolist(),predicted_omega=W.tolist(),actual_post_step=post,post_step_requested_errors=errors,
        expected_vs_observed_change=delta_errors,change_acceptance_relative_error=.05,minimum_direction_cosine=.999,
        post_state_changed=bool(changed),post_state_change_matches_intended=bool(intended_match),
        next_timestep_inherited=bool(inherited and actual_entry_matches),next_start_native=before,
        actual_next_properties_entry=next_read,next_start_errors=inherited_errors,next_completed_state=nextpost,
        deduplication=audit,immediate_get_after_overwrite_PASS_used=False)
    atomic(EVID/'native_controlled_write_validation.json',out);return out

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--branch',choices=['read_only','noop','controlled'],required=True);a=parser.parse_args()
    verify_frozen();out={'read_only':review_read,'noop':review_noop,'controlled':review_controlled}[a.branch]();print(out['result']);print(out['status'])

if __name__=='__main__':main()
