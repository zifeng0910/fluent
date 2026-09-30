"""Run 1.1 ms with all DOFs frozen and UDF load hook active."""
import json,time,traceback

def run(solver,context):
 root,case=context['root'],context['case_dir'];out=root/'evidence/benchmark_C_loadonly_run.json';r={'status':'RUNNING','requested_steps':44,'dt_s':25e-6,'duration_s':.0011,'all_dofs_frozen':True,'six_dof_enabled':True,'gravity_g_m_s2':[0,0,0],'load_hook':'l2300_magnetic_load_validation::libbenchmark_C'};out.write_text(json.dumps(r,indent=2))
 try:
  start=len(context['messages']);solver.settings.solution.run_calculation.dual_time_iterate(time_step_count=44,max_iter_per_step=5)
  r['transcript']=''.join(context['messages'][start:]);r['transcript_tail']=r['transcript'][-14000:];r['parameters_after']=solver.settings.solution.run_calculation.parameters.get_state();r['wall_state_after']=solver.settings.setup.dynamic_mesh.dynamic_zones[context['c_wall_dynamic_zone']].get_state();r['component_state_after']=solver.settings.setup.dynamic_mesh.dynamic_zones[context['c_component_dynamic_zone']].get_state();r['status']='LOAD_ONLY_SOLVED';r['magnetic_load_csv']=str(root/'evidence/benchmark_C_udf_load_validation.csv');r['solver_time_state']=solver.settings.solution.run_calculation.transient_controls.get_state();context['c_loadonly_run']=r
 except Exception as e:r.update(status='FAIL',error=repr(e));raise
 finally:out.write_text(json.dumps(r,indent=2,default=str))
