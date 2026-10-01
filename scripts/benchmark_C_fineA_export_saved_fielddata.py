"""One headless postprocessing session; native saved states, no time advancement."""
import json,sys
from pathlib import Path
import numpy as np
from ansys.fluent.core.fields.field_data_interfaces import ScalarFieldDataRequest
def run(solver,context):
    root=context['root'];sys.path.insert(0,str(root/'scripts'))
    from benchmark_C_fineA_free_6dof_run import surface_mesh
    report_path=root/'evidence/benchmark_C_fineA_free_6dof.json';rec=json.loads(report_path.read_text());source=Path(rec['fielddata_directory'])
    if rec['status'] not in ['FREE_6DOF_SOLVED','FAIL']:raise RuntimeError('Solver run must be finished before scalar FieldData export')
    s=solver.settings
    for frame in rec.get('frames',[]):
        s.file.read_case(file_name=frame['checkpoint_case']);s.file.read_data(file_name=frame['checkpoint_data'])
        time=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
        if abs(time-frame['time_s'])>1e-10:raise RuntimeError('Saved snapshot time mismatch')
        name='benchmark_c_midplane';planes=s.results.surfaces.plane_surface
        if name not in planes.keys():planes.create(name=name)
        planes[name].method='xy-plane';planes[name].z=0.
        fd=solver.fields.field_data;fluid=surface_mesh(fd,name)
        vel=np.asarray(fd.get_field_data(ScalarFieldDataRequest(field_name='velocity-magnitude',surfaces=[name],node_value=True,boundary_value=False))[name],dtype=float)
        if len(vel)!=fluid.n_points or not np.isfinite(vel).all():raise RuntimeError('Invalid saved-state FieldData')
        fluid.point_data['velocity_m_s']=vel;fluid.save(source/f"midplane_{frame['step']:04d}.vtp")
        frame['velocity_max_m_s']=float(vel.max());frame['fielddata_export_complete']=True
        rec['postprocessing_completed_frames']=sum(f.get('fielddata_export_complete',False) for f in rec['frames'])
        report_path.write_text(json.dumps(rec,indent=2))
    rec['stage']='FieldData_export_complete';rec['time_steps_advanced_during_postprocessing']=0
    report_path.write_text(json.dumps(rec,indent=2))
