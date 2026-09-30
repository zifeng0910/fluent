"""Reload frozen B checkpoint, build diagnostic UDF v4, and sample the SDOF hook."""
import csv,json,math
from ansys.fluent.core.fields.field_data_interfaces import SurfaceDataType,SurfaceFieldDataRequest

def run(solver,context):
 root,case=context['root'],context['case_dir'];s=solver.settings;out=root/'evidence/benchmark_C_loadonly_run.json';r={'status':'RUNNING','campaign':'BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF','load_only':True,'requested_steps':44,'dt_s':25e-6,'duration_s':.0011,'orientation_test_deg':17.0,'gravity_g_m_s2':[0,0,0],'six_dof_all_frozen':True}
 out.write_text(json.dumps(r,indent=2))
 try:
  startmsg=len(context['messages']);s.file.read_case(file_name=str(case/'benchmark_C_start.cas.h5'));s.file.read_data(file_name=str(case/'benchmark_C_start.dat.h5'))
  user=s.setup.user_defined;compilemsg=len(context['messages']);user.compiled_udf(library_name='libbenchmark_C_v4',source_files=[str(root/'fluent_udf/l2300_abaqus100hz_6dof.c'),str(root/'fluent_udf/benchmark_C_overset_statistics.c')],header_files=[],use_built_in_compiler=True);user.load(udf_library_name='libbenchmark_C_v4')
  (root/'evidence/benchmark_C_udf_compile.log').write_text(''.join(context['messages'][compilemsg:]),encoding='utf-8');r['compile']='PASS';r['library']='libbenchmark_C_v4'
  d=s.setup.dynamic_mesh;d.enabled=True;d.methods.smoothing.enabled=False;d.options.six_dof.enabled=True;d.options.six_dof.gravity.set_state({'x':0.0,'y':0.0,'z':0.0});s.setup.general.operating_conditions.gravity.enable=False;s.setup.general.solver.time='transient'
  zones={}
  for zone in ['robot_component_fluid','robot_wall']:
   d.dynamic_zones.create(zone=zone);name=next(n for n in d.dynamic_zones.keys() if d.dynamic_zones[n].zone.get_state()==zone);zones[zone]=name;n=d.dynamic_zones[name];n.type='rigid-body';n.motion.six_dof.enabled=True;n.motion.six_dof.passive=(zone=='robot_component_fluid');n.motion.rigid_body_properties.cg_position=[.0012060186937156343,0,0];n.motion.rigid_body_properties.orientation.set_state({'angle':math.radians(17.0),'axis':[.36,.48,.8]});n.motion.motion_def='l2300_magnetic_load_validation::libbenchmark_C_v4'
  context['c_component_dynamic_zone']=zones['robot_component_fluid'];context['c_wall_dynamic_zone']=zones['robot_wall']
  calc=s.solution.run_calculation;calc.parameters.time_step_size=25e-6;calc.parameters.max_iter_per_time_step=1;r['parameters_before']=calc.parameters.get_state();r['zone_states_before']={z:d.dynamic_zones[n].get_state() for z,n in zones.items()};r['gravity_flow']=s.setup.general.operating_conditions.gravity.get_state();r['gravity_sixdof']=d.options.six_dof.gravity.get_state()
  # Prove Fluent's own quaternion conversion and exact source loads at the requested checkpoints.
  user.execute_on_demand(lib_name='benchmark_C_analytic_load_probe::libbenchmark_C_v4');user.execute_on_demand(lib_name='benchmark_C_overset_statistics::libbenchmark_C_v4')
  stats_path=root/'evidence/benchmark_C_overset_live.json';initial=json.loads(stats_path.read_text());r['overset_initial']=initial
  field=solver.fields.field_data
  def bounds(name):
   x=field.get_field_data(SurfaceFieldDataRequest(surfaces=[name],data_types=[SurfaceDataType.Vertices]))[name]
   import numpy as np
   a=np.asarray(x.vertices,float);return [a.min(axis=0).tolist(),a.max(axis=0).tolist(),a.mean(axis=0).tolist()]
  r['initial_robot_wall_bbox_centroid']=bounds('robot_wall');r['initial_robot_component_bbox_centroid']=bounds('overset_component')
  src_count=len(context['messages']);sampled=[]
  for step in range(1,45):
   calc.dual_time_iterate(time_step_count=1,max_iter_per_step=1)
   user.execute_on_demand(lib_name='benchmark_C_overset_statistics::libbenchmark_C_v4')
   item=json.loads(stats_path.read_text());orphans=sum(z['orphan'] for z in item['zones']);missing=sum(z['receptors_without_donors'] for z in item['zones'])
   sampled.append({'step':step,'time_s':item['time_s'],'orphan_count':orphans,'receptors_without_donors':missing,'zones':item['zones']})
   if orphans or missing:raise RuntimeError('C load-only overset connectivity failure: '+json.dumps(sampled[-1]))
  r['overset_samples']=sampled;r['overset_peak_orphans']=max(x['orphan_count'] for x in sampled);r['overset_end']=sampled[-1]
  r['final_robot_wall_bbox_centroid']=bounds('robot_wall');r['final_robot_component_bbox_centroid']=bounds('overset_component');r['time_state_after']=calc.transient_controls.get_state();r['load_history_csv']=str(root/'evidence/benchmark_C_udf_load_validation.csv');r['solver_transcript']=''.join(context['messages'][src_count:]);(root/'evidence/benchmark_C_loadonly_solver_transcript.txt').write_text(r['solver_transcript'],encoding='utf-8')
  r['status']='LOAD_ONLY_SOLVED';context['c_loadonly_run']=r
 except Exception as e:r.update(status='FAIL',error=repr(e));raise
 finally:out.write_text(json.dumps(r,indent=2,default=str))
