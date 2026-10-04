"""Consolidate verified overwrite diagnostics. No solver launch or mutation."""
import json
from benchmark_D_overwrite_review import *

def main():
    verify_frozen()
    config=read(EVID/'configuration.json')
    a=read(EVID/'read_only_reproducibility.json');b=read(EVID/'noop_reproducibility.json')
    cross=read(EVID/'noop_vs_read_only.json');sem=read(EVID/'theta_pose_semantics_audit.json')
    artifact=read(EVID/'no_op_artifact_quantification.json');admission=read(EVID/'controlled_write_admission.json')
    controlled=read(EVID/'native_controlled_write_validation.json') if (EVID/'native_controlled_write_validation.json').exists() else None
    if admission['status']=='PASS' and controlled is None:raise RuntimeError('Controlled write required by passed admission remains pending')
    branches=['A1','A2','B1','B2']+(['controlled'] if controlled else [])
    results={branch:integrity(branch) for branch in branches}
    if native():raise RuntimeError('Native engine remains active; do not record terminal state')
    import psutil
    for branch in branches:
        info=read(EVID/branch/'worker_identity.json')
        try:
            p=psutil.Process(info['pid'])
            if abs(p.create_time()-info['created'])<.01:raise RuntimeError('Worker remains active')
        except psutil.NoSuchProcess:pass
    hh=[r for branch in branches for r in history(branch)]
    moving=trace('A1');read_pass=all(not e['dt_NULL'] and not e['motion_all_zero'] and
        np.max(abs(np.asarray(e['get_velocity'])-e['native_current_velocity']))<1e-8 and
        np.max(abs(np.asarray(e['get_omega'])-e['native_current_omega']))<1e-6 and not e['overwrite_called'] for e in moving)
    valid=bool(controlled and controlled['status']=='PASS' and read_pass and a['status']=='PASS' and b['status']=='PASS')
    status='COMPLETE' if valid else 'HARD_BLOCKER_REQUIRES_REVIEW'
    result='NATIVE_CONTACT_RESPONSE_PATH_VALIDATED' if valid else ('NATIVE_OVERWRITE_INTEGRATION_ARTIFACT_MATERIAL' if admission['status']!='PASS' else 'NATIVE_CONTACT_WRITE_NOT_VALIDATED')
    memories=[read(EVID/branch/'memory_summary.json') for branch in branches]
    resource_samples=[r for branch in branches for r in rows(EVID/branch/'memory_samples.jsonl')]
    report=dict(timestamp=stamp(),campaign=CAMPAIGN,status=status,result=result,
        scope='C70/1.750ms ->1.875ms: two independent read-only and two same-value overwrite repeats; one conditional physical event and one inherited step. No1.9/2ms/micro rollout.',
        official_local_theta_semantics='PARTIAL',official_usage_pattern=read(EVID/'local_api_audit.json')['official_same_release_online_example'],
        local_official_example='NOT_FOUND; same-release official online example verified; original local file/lines unavailable',
        read_path='PASS' if read_pass else 'FAIL',read_only_reproducibility=a,noop_reproducibility=b,noop_vs_read_only=cross,
        theta_and_pose_semantics=sem,artifact=artifact,
        is_artifact_deterministic=a['status']=='PASS' and b['status']=='PASS',
        is_artifact_material_relative_to_normal_step=artifact['significant_vs_normal_step'],
        artifact_interpretation='Deterministic endpoint-pose reconstruction, significant relative to ordinary step; accepted only for the bounded one-event experiment after>10x signal/floor verification.' if valid else 'See admission and controlled validation; no blanket contact API failure conclusion.',
        controlled_write='NOT_RUN' if controlled is None else controlled['status'],controlled_validation=controlled,
        write_storage_semantics=controlled.get('immediate_pending_motion_observation') if controlled else None,
        native_response_pathway='VALIDATED' if valid else 'INCOMPLETE',
        native_response_scope='One event only; this does not validate a full collision trajectory or contact constitutive accuracy.',
        fallback_audit=read(EVID/'fallback_audit.json'),
        preferred_next_architecture='Native DEFINE_CONTACT + same-returned-theta Overwrite with explicitly accounted endpoint-pose integration semantics' if valid else 'Audit-supported normal contactF/T via existing SDOF_PROPERTIES load path; implementation requires a separate task',
        numerical_gates=dict(orphan_peak=max(r['connectivity']['orphan'] for r in hh),invalid_donor_peak=max(r['connectivity']['invalid_donors'] for r in hh),
            nonpositive_volume_peak=max(r['connectivity']['nonpositive_volume'] for r in hh),minimum_volume_m3=min(r['connectivity']['minimum_volume_m3'] for r in hh),
            minimum_gap_m=min(r['physical_gap_m'] for r in hh),maximum_quaternion_norm_error=max(abs(r['q_norm']-1) for r in hh),all_dynamic_states_PASS=all(r['status']=='PASS' for r in hh)),
        resource_metrics=dict(peak_project_working_set_gib=max(r['peak_project_working_set_gib'] for r in memories),
            peak_project_private_bytes_gib=max(r['peak_project_private_bytes_gib'] for r in memories),peak_system_commit_percent=max(r['peak_system_commit_percent'] for r in memories),
            minimum_available_physical_gib=min(r['available_gib'] for r in resource_samples),
            resource_guard_triggered=any(r['abort'] is not None for r in memories)),
        previous_C_D_source_evidence_sha256_preserved=True,owned_solvers_closed=True,
        blockers=[] if valid else [result],
        remaining_unknowns=['Get_Motion third-array assignment/contract in this6DOF path','Exact internal predictor/corrector flag and implementation body','Persistence vs repeated-invocation contribution after the first no-op was not separately isolated','Behavior for other theta inputs, orientations, constraints or solver architectures'],
        git_verified_prior_milestones=['b669605','04d1965'],
        recommended_next='Retain this verified one-event baseline; separately authorize longer contact validation with nonpenetration and energy checks.' if valid else 'Review the scoped write blocker and audit-only fallbackB; no automatic new implementation.',
        evidence_directory=EVID.relative_to(ROOT).as_posix(),live_case_directory=OUT.relative_to(ROOT).as_posix())
    atomic(EVID/'final_report.json',report)
    aa=a['peaks'];bb=b['peaks'];g=report['numerical_gates']
    txt=f'''# BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS

Result: **{result}**. Native read path: **{report['read_path']}**. Native response pathway: **{report['native_response_pathway']}**, limited to one event.

## Verified experiment

Frozen C70 at1.750ms; 4,699,301 cells;25us;2iterations;1rank/double; unchanged magnetic physics/mass/COM/inertia. A1/A2 are read-only; B1/B2 write the exact Get-returned arrays. All independently restart C70 and end1.875ms. Controlled write: **{report['controlled_write']}**. Previous C/D evidence and source hashes are unchanged. All owned engines closed.

## Repeatability and invocation artifact

| Measurement | COM norm | Orientation |
|---|---:|---:|
| A1 vs A2 peak | {aa['COM_norm_difference_m']:.12g}m | {aa['orientation_difference_rad']:.12g}rad |
| B1 vs B2 peak | {bb['COM_norm_difference_m']:.12g}m | {bb['orientation_difference_rad']:.12g}rad |
| First-contact B1 vs A1 | {artifact['COM_difference_norm_m']:.12g}m | {artifact['orientation_difference_deg']:.12g}deg |
| Five-boundary accumulated B1 vs A1 peak | {cross['peaks']['COM_norm_difference_m']:.12g}m | {cross['peaks']['orientation_difference_rad']:.12g}rad |

COM maximum-component difference: {artifact['COM_difference_max_component_m']:.12g}m. Normal actual step COM motion: {artifact['actual_normal_step_COM_displacement_m']:.12g}m; orientation increment: {artifact['actual_normal_step_orientation_increment_rad']:.12g}rad. Endpoint |v|dt: {artifact['velocity_norm_times_dt_m']:.12g}m; |omega|dt: {artifact['omega_norm_times_dt_rad']:.12g}rad.

Relative artifact: **{artifact['relative_COM_artifact_percent']:.6g}% COM**, **{artifact['relative_orientation_artifact_percent']:.6g}% orientation**. It is material relative to ordinary motion and deterministic, with an empirically verified integration explanation. It is not solver repeat noise. The controlled intended signal/floor ratios are **{artifact['signal_to_artifact_floor']['linear_velocity']:.6g}x linear** and **{artifact['signal_to_artifact_floor']['angular_velocity']:.6g}x angular**; the conservative floor includes pose artifact/dt. This admission is limited to the one-event experiment.

## Theta and lifecycle

Installed prototypes lack argument names and implementation bodies. The rigid-body theta comment says global orientation/Euler angles for6DOF; that does not specify the Get third-array contract. The installed official contact example was **not found**. The [same-release official example](https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html) demonstrates returning Get theta to Overwrite; it does not guarantee no-op equivalence to omission.

In these actual callbacks Get theta remains caller-zero, while DT_THETA is nonzero and matches native Theta_From_Q and the independently observed absolute quaternion rotation vector. It differs from native Euler_From_Q. Get theta is therefore neither the evolving absolute DT_THETA nor omega*dt here. Whether the third output is intentionally unassigned/unused is **UNKNOWN**. Outside contact the Get motion arrays remain zero; those calls are observational only.

Empirical reconstruction: overwrite CG_next=CG_start+dt*v_endpoint; Q_next=worldRotation(omega_endpoint*dt)*Q_start. Read-only translation uses the average start/end velocity. Immediately after the same-value setter, visible CG/Q remain unchanged; the difference appears at the completed boundary. Callback occurs with endpoint native state, old CURRENT_TIME/N_TIME and old surface coordinates, before final mesh relocation. Exact predictor/corrector internals and buffer/flag identity remain **UNKNOWN**. No manual quaternion-to-Euler values were sent to the setter.

## Controlled write and gates

See native_controlled_write_validation.json for independent predicted/actual v,omega and impulse, completed-step response, the next native start and actual PROPERTIES_ENTRY inheritance. Immediate callback reread is not a PASS gate. Exactly one e_n=0,mu=0 event is permitted; no full collision rollout was run.

In the physical event, Get immediately after Overwrite reflects the requested new v/omega, while direct DT_VEL/DT_OMEGA/DT_CG/DT_Q fields still show their prior values. At the completed boundary and next-step entry, native fields retain the requested change. This provides an empirical pending-contact-motion explanation; the exact internal buffer/flag is unknown. The no-op second callback already shows its completed pose before the call, so persistence and a repeated call's separate contribution were not isolated.

Orphan peak={g['orphan_peak']}; invalid donor peak={g['invalid_donor_peak']}; nonpositive volume peak={g['nonpositive_volume_peak']}; minimum V={g['minimum_volume_m3']:.12g}m3; minimum gap={g['minimum_gap_m']:.12g}m.

## Audit-only fallbacks

A: no separately supported alternative state setter found; internal declarations are insufficient. B: documented existing SDOF_PROPERTIES COM force/torque path is the first fallback; event timing, force integral independent of callback count, passivity and frozen-dt adequacy require validation. C: calibrated penalty adds stiffness/damping and timestep requirements. D: external integration is the largest coupling change. No fallback was implemented; analytic detection alone does not solve writeback semantics.

Preferred next architecture: {report['preferred_next_architecture']}.

Recommended next: {report['recommended_next']}

## Evidence

local_api_audit.json distinguishes DOCUMENTED / INFERRED_FROM_HEADER / EMPIRICALLY_MEASURED / UNKNOWN. read_only_reproducibility.json, noop_reproducibility.json, noop_vs_read_only.json and no_op_artifact_quantification.json report each exact time. theta_quaternion_semantics.csv and per-branch host/node JSONL retain callback before/after, completed boundaries and next-step entry. Each branch retains native checkpoint SHA/HDF5 metadata, restart continuity, magnetic oracle, resource summary and clean engine exit. Large native CASE/DATA and surfaces remain local.
'''
    (EVID/'README.md').write_text(txt,encoding='utf-8')
    state(status,result=result,worker_alive=False,solver_alive=False,verified_branches=branches,current_time_s=.001875)
    event('OVERWRITE_SEMANTICS_FINAL_REVIEW',status=status,result=result)
    print(json.dumps(dict(status=status,result=result,gates=g,controlled_write=report['controlled_write']),indent=2))

if __name__=='__main__':main()
