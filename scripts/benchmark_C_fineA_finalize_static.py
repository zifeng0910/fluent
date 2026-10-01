"""Save reset t=0 checkpoint after the short sweep passes every required pose."""
import json
def run(solver,context):
    import numpy as np,pyvista as pv
    from ansys.fluent.core.fields.field_data_interfaces import SurfaceFieldDataRequest,SurfaceDataType
    root=context['root'];case=root/'live_cases/benchmark_C_fineA/bg020'
    rec=json.loads((root/'evidence/benchmark_C_fineA_short_sweep.json').read_text())
    rec.pop('theta_safety_allowed_rad',None)
    if rec['status']!='BENCHMARK_C_OVERSET_RESOLUTION_PASS' or len(rec['rows'])!=10 or any(r['status']!='PASS' for r in rec['rows']):raise RuntimeError('All 10 prescribed static poses must pass')
    layers=json.loads((root/'evidence/benchmark_C_fineA_actual_layers.json').read_text())
    component_edge=max(r['max_crossed_cell_edge_mm'] for r in layers['component']['rows'])
    for row in rec['rows']:
        bg=layers['background'] if row['theta_rad']==0 else json.loads((root/f"evidence/benchmark_C_fineA_layers_{row['axis']}_{row['theta_rad']:.2f}.json").read_text())
        row['background_actual_max_crossed_cell_edge_mm']=max(r['max_crossed_cell_edge_mm'] for r in bg['rows'])
        row['component_actual_max_crossed_cell_edge_mm']=component_edge
        row['component_actual_minimum_layers']=layers['component']['minimum_crossed_cells']
        row['background_nominal_boi_mm']=.020;row['component_nominal_size_mm']=[.010,.0125]
    s=solver.settings
    # Sweep returns both axes to zero. Initialization advances no physical time.
    s.solution.initialization.standard_initialize()
    solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    startcase=case/'fineA_static_start.cas.h5';startdata=case/'fineA_static_start.dat.h5'
    s.file.write_case(file_name=str(startcase));s.file.write_data(file_name=str(startdata))
    fields=root/'evidence/benchmark_C_fineA_static_fielddata';fields.mkdir(exist_ok=True)
    for name in ['pipe_wall','robot_wall','overset_component']:
        data=solver.fields.field_data.get_field_data(SurfaceFieldDataRequest(surfaces=[name],data_types=[SurfaceDataType.Vertices,SurfaceDataType.FacesConnectivity]))[name]
        faces=np.concatenate([np.r_[len(a),a] for a in data.connectivity]).astype(np.int64)
        pv.PolyData(np.asarray(data.vertices),faces).save(fields/f'{name}.vtp')
    rec.update(start_case=str(startcase),start_data=str(startdata),theta_allowed_rad=.35,determinant_cutoff=1e-50)
    (root/'evidence/benchmark_C_fineA_final_static_gate.json').write_text(json.dumps(rec,indent=2))
    (root/'evidence/benchmark_C_fineA_short_sweep.json').write_text(json.dumps(rec,indent=2))
    metadata={'status':'BENCHMARK_C_OVERSET_RESOLUTION_PASS','geometry':json.loads((root/'evidence/benchmark_C_fineA_geometry.json').read_text()),'mesh':json.loads((root/'evidence/benchmark_C_fineA_prime_bg020.json').read_text()),'refinement':json.loads((root/'evidence/benchmark_C_background_refinement_region.json').read_text()),'theta0':json.loads((root/'evidence/benchmark_C_fineA_theta0_connectivity.json').read_text()),'static_sweep':rec}
    (root/'evidence/benchmark_C_fineA_final_mesh_metadata.json').write_text(json.dumps(metadata,indent=2))
