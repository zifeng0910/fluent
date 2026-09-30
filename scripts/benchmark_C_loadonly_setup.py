"""Configure the two-zone passive/force-bearing load-only C0 validation."""
import json,math

def run(solver,context):
 root,case=context['root'],context['case_dir'];s=solver.settings;d=s.setup.dynamic_mesh;out=root/'evidence/benchmark_C_loadonly_setup.json';r={'status':'RUNNING','gravity_g_m_s2':[0,0,0],'motion_mode':'all six DOFs frozen','dt_s':25e-6,'duration_s':.0011,'orientation_test_only_deg':17.0,'orientation_test_axis':[.36,.48,.8]}
 try:
  s.setup.general.solver.time='transient';d.options.six_dof.enabled=True;d.options.six_dof.gravity.set_state({'x':0.0,'y':0.0,'z':0.0});s.setup.general.operating_conditions.gravity.enable=False
  d.methods.smoothing.enabled=False
  initial=[0.0012060186937156343,0.0,0.0];ori={'angle':17.0,'axis':[.36,.48,.8]};hook='l2300_magnetic_load_validation::libbenchmark_C'
  for key,name,passive in [('component',context['c_component_dynamic_zone'],True),('wall',context['c_wall_dynamic_zone'],False)]:
   n=d.dynamic_zones[name];n.motion.six_dof.enabled=True;n.motion.six_dof.passive=passive;n.motion.rigid_body_properties.cg_position=initial;n.motion.rigid_body_properties.orientation.set_state(ori);n.motion.motion_def=hook
   r[key+'_state']=n.get_state()
  calc=s.solution.run_calculation;calc.parameters.time_step_size=r['dt_s'];calc.parameters.max_iter_per_time_step=5
  r['time_parameters']=calc.parameters.get_state();r['gravity_sixdof']=d.options.six_dof.get_state();r['gravity_flow']=s.setup.general.operating_conditions.gravity.get_state();r['boundary_conditions']={z:s.setup.boundary_conditions[z].get_state() for z in ['inlet','outlet','pipe_wall','robot_wall']}
  r['status']='PASS';r['wall_hook_allowed']=d.dynamic_zones[context['c_wall_dynamic_zone']].motion.motion_def.allowed_values();context['c_loadonly_setup']=r
 except Exception as e:r.update(status='FAIL',error=repr(e));raise
 finally:out.write_text(json.dumps(r,indent=2,default=str))
