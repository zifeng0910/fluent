"""Load frozen B1 state into an isolated C case and compile the 6DOF UDF."""
import json

def run(solver,context):
 root,case=context['root'],context['case_dir'];out=root/'evidence/benchmark_C_setup_inventory.json';r={'stage':'read_B_static_checkpoint','status':'RUNNING','input_case':'benchmark_B_static_initialized.cas.h5'};out.write_text(json.dumps(r,indent=2))
 try:
  s=solver.settings;f=s.file;f.read_case(file_name=str(case/'benchmark_C_start.cas.h5'));f.read_data(file_name=str(case/'benchmark_C_start.dat.h5'))
  r['zones']=solver.fields.solution_variable_info.get_zones_info().zone_names;r['mesh_check_start']=len(context['messages']);s.mesh.check()
  r['gravity']=s.setup.general.operating_conditions.gravity.get_state();r['dynamic_mesh_before']=s.setup.dynamic_mesh.get_state();s.setup.dynamic_mesh.enabled=True;r['dynamic_mesh_enabled_for_C']=True
  dz=s.setup.dynamic_mesh.dynamic_zones;r['dynamic_zone_create_children']=dz.create.argument_names;r['dynamic_zone_create_state']=dz.create.__doc__
  r['six_dof_options_children']=s.setup.dynamic_mesh.options.six_dof.child_names;r['six_dof_options_state']=s.setup.dynamic_mesh.options.six_dof.get_state()
  r['surface_names']=s.results.surfaces.plane_surface.keys()
  compile_start=len(context['messages']);lib=s.setup.user_defined.compiled_udf(library_name='libbenchmark_C',source_files=[str(root/'fluent_udf/l2300_abaqus100hz_6dof.c')],header_files=[],use_built_in_compiler=True)
  r['compile_return']=str(lib);s.setup.user_defined.load(udf_library_name='libbenchmark_C');(root/'evidence/benchmark_C_udf_compile.log').write_text(''.join(context['messages'][compile_start:]),encoding='utf-8');r['compiled_and_loaded']=True
  r['create_api_signature']=str(dz.create.argument_names);r['component_zone_allowed']=dz.create.zone.allowed_values();r['component_type_allowed']=dz.create.get_child_names() if hasattr(dz.create,'get_child_names') else dz.create.argument_names
  r['hook_enum_after_compile']={}
  # A dynamic-zone template exposes Six DOF fields after type is set. Record permitted values via settings objects.
  r['dynamic_mesh_after_load']=s.setup.dynamic_mesh.get_state();r['status']='PASS_UDF_COMPILED_LOADED';context['c_inventory']=r
 except Exception as e:r.update(status='FAIL',error=repr(e));raise
 finally:out.write_text(json.dumps(r,indent=2,default=str))





