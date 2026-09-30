import json
def run(solver,context):
 s=solver.settings;d=s.setup.dynamic_mesh;r={};
 for zone,name in [('component',context['c_component_dynamic_zone']),('wall',context['c_wall_dynamic_zone'])]:
  n=d.dynamic_zones[name];r[zone]={'motion_def_allowed':n.motion.motion_def.allowed_values(),'motion_def_doc':n.motion.motion_def.__doc__}
 sx=d.options.six_dof.sdof_properties;r['sdof_collection_class']=str(type(sx))
 try:r['sdof_collection_children']=sx.keys()
 except Exception as e:r['sdof_keys_error']=repr(e)
 try:r['sdof_create_args']=sx.create.argument_names;r['sdof_create_doc']=sx.create.__doc__
 except Exception as e:r['sdof_create_error']=repr(e)
 try:r['sdof_state']=sx.get_state()
 except Exception as e:r['sdof_state_error']=repr(e)
 r['gravity_global']=s.setup.general.operating_conditions.gravity.get_state();r['six_dof_gravity']=d.options.six_dof.gravity.get_state();r['time_parameters']=s.solution.run_calculation.parameters.get_state()
 (context['root']/'evidence/benchmark_C_hook_schema.json').write_text(json.dumps(r,indent=2,default=str));print(json.dumps(r,indent=2,default=str))

