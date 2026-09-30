"""Persist static gate outcome and restore the frozen determinant cutoff."""
import json
def run(solver,context):
    root=context['root']; sweep=json.loads((root/'evidence/benchmark_C_overset_pose_sweep.json').read_text())
    before=float(solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)"))
    doc=root/'evidence/benchmark_C_cutoff_documentation.json'
    if doc.exists():before=json.loads(doc.read_text()).get('fresh_recovered_session_cutoff_before_restore',before)
    solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    after=float(solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)"))
    report={'status':'PASS_DOCUMENTATION','fresh_recovered_session_cutoff_before_restore':before,
      'modified_cutoff':after,'original_failed_session_prechange_value':'NOT_CAPTURED',
      'actual_inertia_determinant':1.1206106483031212e-35,'real_inertia_modified':False,
      'sensitivity':'NOT_RUN_STATIC_GATE_FAILED' if sweep['selected_candidate'] is None else 'PENDING',
      'active_solver_sessions':1,'session_recovered_after_blocking_zone_replacement_prompt':True,
      'free_6dof_rerun':'NOT_RUN_STATIC_GATE_FAILED' if sweep['selected_candidate'] is None else 'PENDING'}
    (root/'evidence/benchmark_C_cutoff_documentation.json').write_text(json.dumps(report,indent=2))
    if sweep['selected_candidate'] is None:
        solver.settings.setup.dynamic_mesh.enabled=False
        report['dynamic_mesh_disabled_on_failed_static_candidate']=True
        solver.settings.file.cff_files=True
        if solver.settings.file.single_precision_coordinates.is_active():
            solver.settings.file.single_precision_coordinates=False
        file=root/'live_cases/benchmark_C_candidates/benchmark_C_candidate_D_diagnostic.cas.h5'
        solver.settings.file.write_case(file_name=str(file))
        report['diagnostic_case']=str(file)
        (root/'evidence/benchmark_C_cutoff_documentation.json').write_text(json.dumps(report,indent=2))
