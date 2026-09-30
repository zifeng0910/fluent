import json
def run(solver,context):
 d=solver.settings.setup.dynamic_mesh;root=context['root'];r={'status':'RUNNING'};out=root/'evidence/benchmark_C_zone_setup.json'
 try:
  for name in [context['c_component_dynamic_zone'],context['c_wall_dynamic_zone']]:
   node=d.dynamic_zones[name];node.motion.six_dof.enabled=True;r.setdefault('enabled_states',{})[name]=node.motion.six_dof.get_state();r.setdefault('sixdof_child_names',{})[name]=node.motion.six_dof.child_names
   for leaf in node.motion.six_dof.child_names:
    try:r.setdefault('sixdof_allowed',{}).setdefault(name,{})[leaf]=getattr(node.motion.six_dof,leaf).allowed_values()
    except Exception as e:r.setdefault('sixdof_allowed',{}).setdefault(name,{})[leaf]='unavailable '+repr(e)
  r['component_full']=d.dynamic_zones[context['c_component_dynamic_zone']].get_state();r['wall_full']=d.dynamic_zones[context['c_wall_dynamic_zone']].get_state();r['status']='PASS_SIXDOF_SCHEMA';context['c_zone_schema']=r
 except Exception as e:r.update(status='FAIL',error=repr(e));raise
 finally:out.write_text(json.dumps(r,indent=2,default=str))
