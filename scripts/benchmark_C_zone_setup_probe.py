"""Configure only the isolated C case; inspect Fluent's supported 6DOF fields."""
import json

def run(solver,context):
 root= context['root'];s=solver.settings;d=s.setup.dynamic_mesh;out=root/'evidence/benchmark_C_zone_setup.json';r={'status':'RUNNING'}
 def save():out.write_text(json.dumps(r,indent=2,default=str))
 save()
 try:
  s.setup.general.solver.time='transient';d.enabled=True;d.methods.smoothing.enabled=False;d.options.six_dof.enabled=True
  r['gravity_sixdof_before']=d.options.six_dof.gravity.get_state();r['sixdof_state_enabled']=d.options.six_dof.get_state();r['gravity_flow_before']=s.setup.general.operating_conditions.gravity.get_state()
  # C9: passive overset component and non-passive physical force wall.
  names=[]
  for zone in ['robot_component_fluid','robot_wall']:
   d.dynamic_zones.create(zone=zone);name=next(n for n in d.dynamic_zones.keys() if d.dynamic_zones[n].zone.get_state()==zone);node=d.dynamic_zones[name];node.type='rigid-body';names.append(name)
   r.setdefault('zone_nodes',{})[zone]={'name':name,'children':node.child_names,'state':node.get_state(),'motion_children':node.motion.child_names,'motion_state':node.motion.get_state()};save()
  context['c_component_dynamic_zone']=names[0];context['c_wall_dynamic_zone']=names[1]
  # Probe leaf schemas and supported enumerations after active zone creation.
  wall=d.dynamic_zones[names[1]];r['wall_motion_child_names']=wall.motion.child_names;r['wall_motion_state']=wall.motion.get_state()
  r['component_motion_child_names']=d.dynamic_zones[names[0]].motion.child_names
  for attr in ['six_dof','six_dof_enabled','passive','udf','sdof_properties','properties','motion_def','cg_position','orientation']:
   try:r.setdefault('attribute_probe',{})[attr]=str(getattr(wall.motion,attr).get_state())
   except Exception as e:r.setdefault('attribute_probe',{})[attr]='UNAVAILABLE '+repr(e)
  r['wall_zone_state']=wall.get_state();r['dynamic_mesh_state']=d.get_state();r['status']='PASS_SCHEMA_DISCOVERED';save()
 except Exception as e:r.update(status='FAIL',error=repr(e));save();raise
