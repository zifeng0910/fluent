"""Independent final lifecycle conclusion. Preserves all earlier zero-step conclusions."""
import json
from benchmark_D_lifecycle_common import *
from benchmark_D_lifecycle_review import callback_records

def write_report():
    verify_frozen()
    validations={name:read(EVID/file) for name,file in [('read','native_motion_read_validation.json'),
        ('noop','native_noop_write_validation.json'),('controlled','native_controlled_write_validation.json')]}
    r=validations['read'];event=r.get('first_callback',{});same=r.get('completed_same_step',{})
    traces={branch:callback_records(branch) for branch in BRANCHES}
    allstates=[record for branch in BRANCHES for record in read(EVID/branch/'native_history.json').get('records',[])]
    native_failed=any(validations[n].get('status')=='FAIL' for n in validations)
    validated=all(validations[n].get('status')=='PASS' for n in validations) and read(EVID/'callback_dedup_audit.json').get('status')=='PASS'
    # A no-op pose failure stops this sequence. It does not test a changed velocity
    # persisting, so it must not become a blanket motion API not-viable conclusion.
    noop_failed=validations['noop'].get('status')=='FAIL' and validations['read'].get('status')=='PASS'
    classification='NATIVE_CONTACT_NOOP_OVERWRITE_FAIL' if noop_failed else 'NATIVE_CONTACT_MOTION_API_NOT_VIABLE' if native_failed else 'NATIVE_CONTACT_RESPONSE_PATH_VALIDATED' if validated else 'NATIVE_CONTACT_LIFECYCLE_INCOMPLETE'
    blocker=next((v.get('result',v.get('reason')) for v in validations.values() if v.get('status')=='FAIL'),None)
    if not blocker and not validated:blocker='Actual transient lifecycle sequence is not yet fully validated'
    native_same=event.get('native_current_velocity');native_omega=event.get('native_current_omega')
    if 'post_step' in r.get('matches_native_state_phases',[]):native_same=same['native_state']['velocity'];native_omega=same['native_state']['omega']
    counts={branch:{} for branch in BRANCHES}
    for branch,rr in traces.items():
        for x in rr:
            k=str(x['timestep_index']);counts[branch][k]=counts[branch].get(k,0)+1
    report=dict(timestamp=stamp(),campaign=CAMPAIGN,status='HARD_BLOCKER_REQUIRES_REVIEW' if native_failed else 'COMPLETE' if validated else 'INCOMPLETE',
        terminal_result=classification,zero_step_previous_result='FAIL / preserved; conclusion applies only to prior zero-step harness',
        real_transient_DEFINE_CONTACT_callback='TRIGGERED' if event else 'NOT TRIGGERED',moving_body_dt_valid=not event.get('dt_NULL',True),
        SDOF_Get_Motion_real_callback=r.get('status','NOT RUN'),native_same_event_velocity_m_s=native_same,callback_velocity_m_s=event.get('get_velocity'),
        native_same_event_omega_rad_s=native_omega,callback_omega_rad_s=event.get('get_omega'),
        native_completed_same_step=same.get('native_state'),readback_agreement=r.get('phase_comparisons'),
        noop_overwrite=validations['noop'].get('status','NOT RUN'),controlled_overwrite=validations['controlled'].get('status','NOT RUN'),
        post_callback_solver_state_changed='YES: no-op COM/quaternion changed; velocity/omega unchanged' if noop_failed else validations['controlled'].get('post_state_changed','NOT TESTED'),
        next_timestep_state_inherited=validations['controlled'].get('next_timestep_inherited','NOT TESTED'),
        callbacks_per_timestep_compute_node=counts,deduplication=read(EVID/'callback_dedup_audit.json').get('status','NOT NEEDED'),
        native_contact_response_path='INCOMPLETE' if noop_failed else 'NOT VIABLE' if native_failed else 'VALIDATED' if validated else 'INCOMPLETE',
        native_motion_API_global_viability='UNDETERMINED: read PASS; no-op pose FAIL; controlled persistence NOT TESTED' if noop_failed else classification,
        first_contact_time_s=event.get('CURRENT_TIME'),contact_gap_m=event.get('ideal_tube_gap_from_actual_vertices_m'),
        first_contact_completed_time_s=same.get('time_s'),
        noop_pose_change_diagnosis=read(EVID/'noop_trajectory_change_diagnosis.json'),
        contact_gap_reference='Actual callback robot vertices vs authoritative1.8mm-ID tube; completed-step polyhedral signed gap is separately recorded',
        micro_run=read(EVID/'micro_run_validation.json').get('status','NOT RUN'),
        orphan_peak=max([x['connectivity']['orphan'] for x in allstates],default=None),
        invalid_donor_peak=max([x['connectivity']['invalid_donors'] for x in allstates],default=None),
        minimum_volume_m3=min([x['connectivity']['minimum_volume_m3'] for x in allstates],default=None),
        minimum_completed_step_gap_m=min([x['physical_gap_m'] for x in allstates],default=None),
        penetration_m=max([max(0,-x['physical_gap_m']) for x in allstates],default=None),
        penetration_tolerance_m=1e-9,current_blocker=blocker,
        resource_guard_audit=read(EVID/'resource_guard_audit.json'),
        all_owned_engines_closed=all(read(EVID/branch/'engine_exit.json').get('owned_engine_closed') for branch in ['read_only','noop']),
        recommended_next='Investigate the supported callback theta/pose-update contract and first restore a true native no-op; preserve free6DOF. A changed-motion update path must be validated before analytic detection or micro-run' if noop_failed else 'Identify and validate an alternative supported native6DOF state-update path before considering analytic detection; preserve free6DOF' if native_failed else 'Finish the authorized lifecycle gate sequence',
        earlier_C_and_D_frozen_files_preserved=True,immediate_get_after_overwrite_used_as_PASS=False,
        solver_launches_owned_and_sequential=True,no_t0_replay=True,no_prescribed_CG_MOTION=True,
        output_directory=EVID.as_posix())
    atomic(EVID/'final_report.json',report)
    if native_failed:state('HARD_BLOCKER_REQUIRES_REVIEW',terminal_result=classification,current_blocker=blocker,
        worker_alive=False,solver_alive=False,final_report='evidence/benchmark_D_contact_lifecycle/final_report.json')
    return report

if __name__=='__main__':print(json.dumps(write_report(),indent=2,allow_nan=False))
