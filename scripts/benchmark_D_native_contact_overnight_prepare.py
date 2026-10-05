"""Create the authorized immutable window once; never renew it on recovery."""
import json, os, time
from datetime import datetime, timezone
from benchmark_D_native_contact_overnight_common import *

def main():
    EVID.mkdir(parents=True,exist_ok=True);OUT.mkdir(parents=True,exist_ok=True)
    if (EVID/'window.json').exists() or (EVID/'configuration.json').exists():
        raise RuntimeError('Existing overnight window/configuration must be resumed, never recreated')
    if native():raise RuntimeError('Native engine exists; campaign creation requires ownership review')
    previous=ROOT/'evidence/benchmark_D_contact_overwrite_semantics'
    final=read(previous/'final_report.json')
    if final.get('native_response_pathway')!='VALIDATED' or not final.get('owned_solvers_closed'):
        raise RuntimeError('Authoritative one-event milestone not complete')
    frozen=dict(read(previous/'configuration.json')['frozen_files_sha256'])
    for p in [previous/'configuration.json',previous/'state.json',previous/'final_report.json',
        previous/'controlled_write_validation.json',ROOT/'fluent_udf/l2300_contact_overwrite_semantics.c']:
        if p.exists():frozen[p.relative_to(ROOT).as_posix()]=sha(p)
    for p,h in frozen.items():
        if sha(ROOT/p)!=h:raise RuntimeError('Authoritative evidence hash mismatch: '+p)
    start=time.time()
    window=dict(campaign_id=CAMPAIGN,window_id='native_contact_'+str(int(start)),start_epoch=start,
        target_deadline_epoch=start+10*3600,hard_deadline_epoch=start+12*3600,
        start=datetime.fromtimestamp(start,timezone.utc).isoformat(),
        target_deadline=datetime.fromtimestamp(start+10*3600,timezone.utc).isoformat(),
        hard_deadline=datetime.fromtimestamp(start+12*3600,timezone.utc).isoformat(),
        immutable=True,creation_source='Explicit user authorization attachment a6cf2909',
        timezone_for_display='Asia/Shanghai',independent_of_expired_C_windows=True)
    atomic(EVID/'window.json',window)
    config=dict(campaign=CAMPAIGN,timestamp=stamp(),window_sha256=sha(EVID/'window.json'),
        authorization_attachment='a6cf2909-dded-43e0-9ae4-90e0ffd06327',
        authoritative_git=['b669605','04d1965','2121e06'],
        accepted_prior_result='NATIVE_CONTACT_RESPONSE_PATH_VALIDATED',prior_semantics_reopened=False,
        frozen_files_sha256=frozen,source_checkpoint='evidence/benchmark_C_coarse_2ms/checkpoint_0070.json',
        start_time_s=.00175,micro_end_time_s=.0019,end_time_s=.002,
        contact_source_path='fluent_udf/l2300_native_contact_overnight.c',
        mesh_cells=4699301,background_cells=3345695,component_cells=1353606,
        processor_count=1,precision='double',ui_mode='no_gui_or_graphics',iterations_per_step=2,
        source=dict(type='FROZEN_ANALYTIC_ABAQUS_100HZ',frequency_hz=100,ramp_s=.001,absolute_offset_m_s=.006),
        gravity=[0,0,0],cutoff=1e-50,free_6dof=True,restitution=0,friction_coefficient=0,
        native_contact_proximity_m=.0001,ideal_tube_radius_m=.0009,penetration_tolerance_m=5e-6,
        timestep_candidates_s=[25e-6,12.5e-6,6.25e-6],
        admission=dict(available_gib=12,commit_headroom_gib=19,disk_free_gib=35,stable_seconds=60),
        runtime_guards=dict(available_gib=3,commit_fraction=.95,project_working_set_gib=15,disk_free_gib=15),
        max_lifecycle_restarts=2,max_distinct_repairs_per_failure_class=2,
        exclusive_launch_owner='Fluent-Benchmark-D-Native-Contact-Overnight',
        no_pagefile_change=True,no_unrelated_process_termination=True,maximum_sessions=1,
        dt_acceptance_before_results=dict(contact_time_s='coarser dt',contact_position_m=20e-6,
            signed_gap_difference_m=5e-6,impulse_relative=.10,post_velocity_relative=.10,
            velocity_absolute_floor_m_s=.001,omega_absolute_floor_rad_s=1,post_omega_relative=.10,
            tilt_difference_deg=.5,event_count_difference='<=1, or <=20% when >5 events'),
        fallback_authorized_only_after_native_failure=True,
        fallback_order=['SDOF_LOAD_PATH_SINGLE_INTERVAL_VALIDATION','COMPLIANT_LOAD_PATH_ONLY_IF_LOAD_IMPULSE_UNSTABLE'],
        numerical_PASS_requires_actual_2ms=True,COMPLETE_requires_actual_visual_review=True)
    atomic(EVID/'configuration.json',config)
    state('STARTUP',route='NATIVE_OVERWRITE',current_time_s=.00175,solver_alive=False,worker_alive=False)
    atomic(EVID/'supervisor_state.json',dict(status='NOT_STARTED',timestamp=stamp(),supervisor_alive=False))
    event('IMMUTABLE_WINDOW_CREATED',window_id=window['window_id'],hard_deadline=window['hard_deadline'])
    print(json.dumps(dict(status='LOCAL_CONTROL_PREPARED',window=window)))

if __name__=='__main__':main()
