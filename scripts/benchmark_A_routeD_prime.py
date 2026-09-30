import os, json, traceback
import ansys.meshing.prime as prime
from ansys.meshing.prime import lucid
STEP=r'H:/fluent/geometry/benchmark_A_fluid_domain/BenchmarkA_FluidDomain.step'
OUT=r'H:/fluent/live_cases/benchmark_A_stationary/prime'; os.makedirs(OUT,exist_ok=True)
E=r'H:/fluent/evidence'; os.makedirs(E,exist_ok=True)
res={"route":"BENCHMARK_A_ROUTE_D_PYPRIME_NATIVE","prime_version":prime.__version__,"step":STEP}
with prime.launch_prime() as session:
    model=session.model
    io=prime.FileIO(model)
    ir=io.import_cad(STEP, params=prime.ImportCadParams(model=model))
    model._sync_up_model()
    parts=list(model.parts)
    part=parts[0] if parts else None
    summ=part.get_summary(prime.PartSummaryParams(model)) if part else None
    res["import_result"]=str(ir); res["parts"]=len(parts)
    res["topology"]={"part_count":len(parts),"topo_volume_count":len(list(part.get_topo_volumes())) if part else 0,"topo_face_count":len(list(part.get_topo_faces())) if part else 0,"bbox":str(summ).split('Mesh Summary:')[0] if summ else None,"summary":str(summ) if summ else None}
    if not part or len(list(part.get_topo_volumes()))!=1: raise RuntimeError('TopoVolume hard gate failed')
    mu=lucid.Mesh(model)
    mu.surface_mesh(min_size=0.05,max_size=0.10)
    model._sync_up_model()
    faces=list(part.get_topo_faces()); zones=model.topo_data.get_mesh_zonelets_of_topo_faces(faces)
    su=prime.SurfaceUtilities(model)
    boxes={str(f):str(su.get_bounding_box_of_zonelets([z])) for f,z in zip(faces,zones)}
    # Face IDs are classified from their actual Prime mesh bounding boxes.
    labels={"inlet":[],"outlet":[],"pipe_wall":[],"robot_wall":[]}
    for f,z in zip(faces,zones):
        b=su.get_bounding_box_of_zonelets([z]); s=str(b)
        if 'xmin :  -5' in s and 'xmax :  -5' in s: labels['inlet'].append(f)
        elif 'xmin :  5' in s and 'xmax :  5' in s: labels['outlet'].append(f)
        elif 'xmin :  0' in s or 'xmax :  2.3' in s or 'xmin :  0.261495' in s or 'xmax :  2.3' in s: labels['robot_wall'].append(f)
        else: labels['pipe_wall'].append(f)
    for name,fs in labels.items():
        if fs: part.add_labels_on_topo_entities([name],fs)
    model._sync_up_model(); mu.create_zones_from_labels('inlet,outlet,pipe_wall,robot_wall'); model._sync_up_model()
    res['surface']={"topo_faces":len(faces),"face_zonelets":len(list(part.get_face_zonelets())),"labels":labels,"boxes":boxes,"summary":str(part.get_summary(prime.PartSummaryParams(model)))}
    vzones=list(part.get_volume_zones()); vz=[model.get_zone_name(v) for v in vzones]
    res['volume_zone_names']=vz
    mu.volume_mesh(volume_fill_type=prime.VolumeFillType.TET,scope=lucid.VolumeScope(part_expression=part.name,entity_expression='*'))
    model._sync_up_model(); vs=part.get_summary(prime.PartSummaryParams(model)); res['volume_mesh_summary']=str(vs)
    c=io.export_fluent_case(OUT+r'/benchmark_A_prime.cas.h5',prime.ExportFluentCaseParams(model,cff_format=True)); res['cas_export']=str(c)
    mm=io.export_fluent_meshing_mesh(OUT+r'/benchmark_A_prime.msh.h5',prime.ExportFluentMeshingMeshParams(model,cff_format=True)); res['msh_export']=str(mm)
json.dump(res,open(E+r'/benchmark_A_routeD_prime_topology.json','w'),indent=2)
json.dump({"prime_version":prime.__version__,"import_result":res.get('import_result'),"launch":"PASS"},open(E+r'/benchmark_A_routeD_prime_preflight.json','w'),indent=2)
print(json.dumps({k:res[k] for k in ['prime_version','parts','topology','surface','volume_zone_names','cas_export','msh_export']},indent=2))
