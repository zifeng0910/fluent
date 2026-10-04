"""Consolidate verified zero-dynamics results after owned solver closure. No launch."""
import json
import numpy as np
import psutil
from benchmark_D_common import *
from benchmark_C_coarse_2ms_common import identity

def main():
    config=read(EVID/'configuration.json')
    frozen={path:dict(expected_sha256=digest,actual_sha256=sha(ROOT/path))
            for path,digest in config['frozen_files_sha256'].items()}
    if any(x['expected_sha256']!=x['actual_sha256'] for x in frozen.values()):
        raise RuntimeError('Frozen C/source evidence changed')
    s=read(EVID/'state.json');owned=read(EVID/'owned_engine.json')
    try:
        worker=psutil.Process(s['worker_pid'])
        worker_alive=abs(worker.create_time()-s['worker_created'])<.01 and worker.is_running()
    except psutil.Error:worker_alive=False
    if worker_alive or any(identity(r) for r in owned['processes']):raise RuntimeError('Owned worker/engine remains alive')
    closed=read(EVID/'engine_exit.json')
    if not closed.get('owned_engine_closed') or closed.get('unrelated_processes_touched') or closed.get('terminated'):
        raise RuntimeError('Require verified clean SDK closure')
    restart=read(EVID/'step70_restart_continuity.json')
    restored=read(EVID/'post_probe_C70_restore.json')
    math=read(EVID/'contact_impulse_unit_validation.json')
    if any(x.get('status')!='PASS' for x in [restart,restored,math]):raise RuntimeError('Prior gate missing')
    audit=read(EVID/'native_motion_state_audit.json')
    if len(audit['records'])!=4 or any(x['status']!='PASS' for x in audit['unchanged_checks'].values()):
        raise RuntimeError('Typed read-only motion audit incomplete')
    wall=[x for x in audit['records'] if x['zone']=='robot_wall']
    if any(np.linalg.norm(x['get_motion_velocity']) or np.linalg.norm(x['get_motion_omega'])
           or not x['motion_tmp_cg_null'] or not x['motion_tmp_theta_null'] for x in wall):
        raise RuntimeError('Motion-context diagnosis differs from report')
    if any(np.linalg.norm(x['current_omega'])<100 for x in wall):raise RuntimeError('Native checkpoint angular velocity absent')
    archive=EVID/'diagnostic_archive/native_response_failure_01'
    transcript=archive/'fluent-20261004-081129-33312.trn'
    lines=transcript.read_text(errors='replace').splitlines()
    error_lines=[dict(line=i+1,text=line) for i,line in enumerate(lines) if 'D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED' in line]
    if not error_lines:raise RuntimeError('Actual native error not preserved')
    atomic(EVID/'native_response_failure_diagnosis.json',dict(timestamp=stamp(),status='CONFIRMED_GATE_FAILURE',
        error='D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED',transcript=str(transcript.relative_to(ROOT)).replace('\\','/'),
        actual_error_lines=error_lines,transcript_sha256=sha(transcript),
        native_error_context=lines[max(0,error_lines[-1]['line']-9):error_lines[-1]['line']],
        failed_job='e860d622-87c6-49b2-b3be-c57878d7a477',
        demonstrated_facts=['Native contact macro compiles and callback returns finite point/normal',
          'Restored current and temporary body state retain exact C70 velocity/omega',
          'SDOF_Get_Motion returns zero on host and node outside motion integration',
          'Native motion temporary pointers are null in this zero-dtime harness',
          'Immediate overwrite/readback assertion failed inside synthetic DEFINE_CONTACT callback'],
        inference='The standalone static detection harness lacks the active native motion context needed to verify response. Null-pointer observations support this inference; they do not establish the internal implementation of SDOF_Get_Motion.',
        limitation='This is not proof of general Fluent/overset API incompatibility. Correct response in a supported active motion context remains unverified.',
        decision='STOP before dynamics under user native-baseline gate',timesteps_advanced=0,
        artificial_sanity_values_available=False,real_contact_event_metrics_available=False))
    semantics=read(archive/'contact_detection_semantics.json')
    patterns={};normal_cosines=[];static_points=[]
    for flag in [0,1]:
        patterns[str(flag)]=[]
        for rec in semantics['probe_records']:
            test=next(x for x in rec['tests'] if x['flag']==flag)
            patterns[str(flag)].append(dict(target_gap_mm=1000*rec['target_gap_m'],
                measured_gap_mm=1000*test['measured_gap_m'],callbacks=test['callback_count'],
                contact_faces=[e['number_of_contact_faces'] for e in test['callbacks']]))
            for e in test['callbacks']:
                point=np.array(e['contact_point_m']);normal=np.array(e['normal']);inward=np.array([0.,-point[1],-point[2]])
                cosine=float(normal@inward/np.linalg.norm(inward));normal_cosines.append(cosine)
                if not np.isfinite(np.r_[point,normal]).all() or abs(np.linalg.norm(normal)-1)>1e-8 or cosine<.9:
                    raise RuntimeError('Point/normal geometric sanity failed')
                static_points.append(dict(gap_mm=1000*test['measured_gap_m'],contact_point_m=point.tolist(),
                    normal=normal.tolist(),contact_faces=e['number_of_contact_faces'],radial_inward_cosine=cosine))
    if not normal_cosines:raise RuntimeError('No static native callback evidence')
    semantics.update(status='FAIL',timestamp=stamp(),timesteps_advanced=0,
        observed_trigger_patterns=patterns,contact_point_normal_sanity='PASS',
        normal_impulse_sanity_status='FAIL_NATIVE_OVERWRITE_READBACK',
        actual_native_error='D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED',
        positive_gap_detection_pass=True,native_C70_restore_status='PASS',production_response_started=False,
        static_only=True,synthetic_callback_logs_are_not_dynamic_contact_events=True,
        interpretation='At fixed C70 attitude and threshold 0.100mm, manual Detect_Contact flag0 returned callbacks near 0.100/0.090/0.070mm; flag1 returned none. This does not establish production callback timing or safe dynamics.',
        native_motion_context_audit='evidence/benchmark_D_contact_baseline/native_motion_state_audit.json',
        artificial_inward_velocity_sanity=dict(status='FAIL',error='D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED',
            actual_post_overwrite_velocity_not_logged=True))
    atomic(EVID/'contact_detection_semantics.json',semantics)
    capability=read(EVID/'contact_capability_audit.json')
    capability.update(timestamp=stamp(),supported_in_current_configuration='NO_VERIFIED_BASELINE',
        DEFINE_CONTACT_support='SUPPORTED',point_normal_gate='PASS',native_response_gate='FAIL_UNVERIFIED',
        zero_dynamics_gate='FAIL',proximity_threshold_semantics=semantics['interpretation'],
        general_API_unsupported_claim=False)
    atomic(EVID/'contact_capability_audit.json',capability)
    proposal=dict(timestamp=stamp(),status='PROPOSED_REQUIRES_REVIEW_NOT_IMPLEMENTED',
        scheme='Geometry-based analytic tube contact',preserve=config['source_law'],
        frozen_robot_mesh_and_physics=True,normal_restitution=0,friction=0,
        detector='Compute signed clearance from transformed authoritative robot surface against the 0.9mm-radius stationary tube; use inward wall normal and a physically consistent contact point.',
        response='Reuse validated rigid-body normal impulse math only after a supported native motion-update context is demonstrated. An analytic detector alone does not resolve motion update/readback.',
        gates=['Zero-dynamics known-gap and point/normal checks','Verified motion context and impulse transfer/readback',
            'Exact native C70 restart and pre-contact continuity','At most four initial steps to1.850ms',
            'Near-contact dt25/12.5/6.25us only as permitted by approach-distance evidence',
            'Short1.900ms PASS before2.000ms; no wall penetration/orphan/invalid/nonpositive volume/nonfinite state'],
        alternative_native_diagnostic='A separately reviewed supported zero-dynamics native motion-context harness may establish viability; no unchecked calls to internal integration functions are authorized by this report.',
        automatic_switch_performed=False,new_solver_or_time_budget_authorized=False)
    atomic(EVID/'next_scheme_proposal.json',proposal)
    frozen_check=dict(status='PASS',timestamp=stamp(),files=frozen,
        native_C70_case_sha256=restored['case_sha256'],native_C70_data_sha256=restored['data_sha256'],
        no_original_C_files_written=True)
    atomic(EVID/'C_preservation_final_audit.json',frozen_check)
    report=dict(campaign=CAMPAIGN,timestamp=stamp(),status='NATIVE_CONTACT_BASELINE_NOT_VIABLE',
        result_scope='Current standalone zero-dtime native detector/response harness; general API incompatibility not established',
        Benchmark_C_terminal_state='FREE_MOTION_NEAR_WALL_LIMIT_REACHED',
        restart_source=dict(step=70,time_s=restored['native_time_s'],status='PASS',
            post_probe_restore='PASS',physical_clearance_mm=1000*restored['overset']['physical_clearance_m']),
        native_DEFINE_CONTACT='SUPPORTED',contact_pair=['robot_wall','pipe_wall'],
        contact_model='frictionless rigid-body normal impulse',normal_restitution=0,friction=0,
        impulse_reference_validation='PASS',impulse_reference_evidence='contact_impulse_unit_validation.json',
        detection_only_probes=patterns,static_point_normal_sanity='PASS',static_contact_points=static_points,
        response_sanity='FAIL_NATIVE_OVERWRITE_READBACK',
        pre_contact_C_vs_D=dict(restart_state='PASS',dynamic_trajectory='NOT_RUN'),
        first_real_contact_time_s=None,clearance_at_first_real_contact_mm=None,real_contact_count=None,
        peak_contact_impulse_N_s=None,post_contact_normal_velocity_m_s=None,peak_contact_omega_rad_s=None,
        maximum_contact_tilt_rad=None,minimum_dynamic_clearance_mm=None,dynamic_penetration='NOT_EVALUATED',
        restored_C70_numerical_state=dict(orphans=0,invalid_donors=0,minimum_volume_m3=restored['overset']['minimum_volume_m3'],q_norm=restored['q_norm']),
        initial_dt_us=25,near_contact_timestep_sensitivity='NOT_RUN',timesteps_advanced=0,
        micro_run_to1_850ms='NOT_RUN',short_run_to1_900ms='NOT_RUN',two_ms='NOT_RUN',visualization='NOT_RUN_NO_CONTACT_DYNAMICS',
        resource_summary=read(EVID/'memory_summary.json'),owned_engine_closure=closed,
        current_blocker='Static detection calls return contact geometry, but synthetic impulse overwrite/readback does not validate in the inactive native motion context.',
        recommended_next='Review next_scheme_proposal.json; no automatic native retry or analytic switch',
        evidence_files=['contact_capability_audit.json','contact_detection_semantics.json','native_response_failure_diagnosis.json',
            'native_motion_state_audit.json','post_probe_C70_restore.json','C_preservation_final_audit.json','engine_exit.json','next_scheme_proposal.json'],
        magnetic_formula_unchanged=True,magnetic_sphere_example_used=False,original_C_deadline_unchanged=True)
    atomic(EVID/'final_report.json',report)
    state('HARD_BLOCKER_REQUIRES_REVIEW',terminal_result=report['status'],solver_alive=False,worker_alive=False,
        active_job=None,error='D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED',time_s=restored['native_time_s'],
        timesteps_advanced=0,current_step=70,latest_clean_native_state='C70_RESTORED_PASS',two_ms='NOT_RUN')
    event('NATIVE_CONTACT_BASELINE_GATE_CLOSED',terminal_result=report['status'],timesteps_advanced=0)
    print(json.dumps(dict(status=report['status'],restart='PASS',impulse_math='PASS',native_motion_context='UNVERIFIED',
        two_ms='NOT_RUN',C_preservation='PASS',owned_engine_closed=True)))

if __name__=='__main__':main()
