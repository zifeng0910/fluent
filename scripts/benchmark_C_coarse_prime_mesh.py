"""Rebuild only the background; append the extracted, unchanged component mesh."""
import argparse,json,re,time,traceback
from pathlib import Path
import ansys.meshing.prime as p
from ansys.meshing.prime import lucid
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'live_cases/benchmark_C_coarse_A';EVID=ROOT/'evidence/benchmark_C_coarse_A'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--critical-size',type=float,default=.022);args=ap.parse_args()
    if not .018<=args.critical_size<=.022:raise RuntimeError('Critical background size outside authorized range')
    rec={'status':'RUNNING','stage':'launch','critical_spacing_mm':args.critical_size,'buffer_spacing_mm':.040,'transition_spacing_mm':.080,'far_spacing_mm':.15,'component_remeshed':False}
    def save():
        tmp=EVID/'mesh.json.tmp';tmp.write_text(json.dumps(rec,indent=2));tmp.replace(EVID/'mesh.json')
    save();start=time.time()
    try:
        with p.launch_prime(timeout=120) as session:
            model=session.model;io=p.FileIO(model)
            geometry=json.loads((EVID/'geometry.json').read_text())
            boinames=[item['name'] for item in geometry['parts']]
            for name in ['background_water']+boinames:
                r=io.import_cad(str(ROOT/f'geometry/benchmark_C_coarse_A/{name}.step'),p.ImportCadParams(model,append=True))
                if r.error_code.value:raise RuntimeError(str(r))
                model._sync_up_model();part=model.get_part_by_name(name)
                if len(part.get_volume_zones())!=1:raise RuntimeError('BOI must be single closed volume: '+name)
                model.set_suggested_zone_name(part.get_volume_zones()[0],name)
            mesh=lucid.Mesh(model);parts={x.name:x for x in model.parts};controls=[]
            for name in boinames:
                rec['stage']=name+'_surface';save()
                mesh.surface_mesh(min_size=.08,max_size=.12,scope=lucid.SurfaceScope(part_expression=name))
            model.set_global_sizing_params(p.GlobalSizingParams(model,min=args.critical_size,max=.15,growth_rate=1.2))
            ctrl=model.control_data.create_size_control(p.SizingType.CURVATURE)
            ctrl.set_curvature_sizing_params(p.CurvatureSizingParams(model,min=.12,max=.15,growth_rate=1.2,normal_angle=18,use_cad_curvature=True))
            ctrl.set_scope(p.ScopeDefinition(model,part_expression='background_water',label_expression='*',entity_type=p.ScopeEntity.FACEANDEDGEZONELETS));controls.append(ctrl.id)
            for item in geometry['parts']:
                name=item['name'];h=args.critical_size if item['group']=='critical' else item['spacing_mm']
                ctrl=model.control_data.create_size_control(p.SizingType.BOI);ctrl.set_boi_sizing_params(p.BoiSizingParams(model,max=h,growth_rate=1.2))
                ctrl.set_scope(p.ScopeDefinition(model,part_expression=name,label_expression='*',entity_type=p.ScopeEntity.FACEANDEDGEZONELETS));controls.append(ctrl.id)
            rec['stage']='size_field';save()
            r=p.SizeField(model).compute_volumetric(controls,p.VolumetricSizeFieldComputeParams(model))
            if r.error_code.value:raise RuntimeError(str(r))
            bg=parts['background_water'];rec['stage']='background_surface';save()
            r=p.Surfer(model).mesh_topo_faces(bg.id,bg.get_topo_faces(),p.SurferParams(model,size_field_type=p.SizeFieldType.VOLUMETRIC,min_size=args.critical_size,max_size=.15,growth_rate=1.2))
            if r.error_code.value:raise RuntimeError(str(r))
            model._sync_up_model();util=p.SurfaceUtilities(model);classes={}
            for face,z in zip(bg.get_topo_faces(),model.topo_data.get_mesh_zonelets_of_topo_faces(bg.get_topo_faces())):
                b=[float(v) for v in re.findall(r'(?:xmin|xmax|ymin|ymax|zmin|zmax)\s*:\s*([-+\d.eE]+)',str(util.get_bounding_box_of_zonelets([z])))]
                if len(b)!=6:raise RuntimeError('Invalid surface bounding box')
                label='inlet' if b[3]<-4.99 else 'outlet' if b[0]>4.99 else 'pipe_wall';classes.setdefault(label,[]).append(face)
            if set(classes)!=set(['inlet','outlet','pipe_wall']):raise RuntimeError('Background face labels incomplete')
            for label,faces in classes.items():bg.add_labels_on_topo_entities([label],faces)
            mesh.create_zones_from_labels('inlet,outlet,pipe_wall');model._sync_up_model()
            vc=model.control_data.create_volume_control();vc.set_params(p.VolumeControlParams(model,cell_zonelet_type=p.CellZoneletType.FLUID))
            vc.set_scope(p.ScopeDefinition(model,part_expression='background_water',zone_expression='*',entity_type=p.ScopeEntity.VOLUME,evaluation_type=p.ScopeEvaluationType.ZONES))
            rec['stage']='background_volume';save()
            r=p.AutoMesh(model).mesh(bg.id,p.AutoMeshParams(model,size_field_type=p.SizeFieldType.VOLUMETRIC,volume_fill_type=p.VolumeFillType.TET,volume_control_ids=[vc.id]))
            if r.error_code.value:raise RuntimeError(str(r))
            model._sync_up_model();summary=bg.get_summary(p.PartSummaryParams(model));rec['background_cells']=int(summary.n_tet_cells);save()
            if rec['background_cells']<=0:raise RuntimeError('Empty background')
            model.delete_parts([parts[n].id for n in boinames]);model._sync_up_model()
            rec['stage']='append_validated_component';save()
            r=io.import_fluent_case(str(OUT/'validated_component.cas.h5'),p.ImportFluentCaseParams(model,append=True))
            if r.error_code.value:raise RuntimeError(str(r))
            model._sync_up_model();rec['component_cells']=sum(int(x.get_summary(p.PartSummaryParams(model)).n_tet_cells) for x in model.parts)-rec['background_cells']
            if rec['component_cells']!=1353606:raise RuntimeError('Component cell count changed')
            rec['total_cells']=rec['background_cells']+rec['component_cells'];rec['cell_reduction_percent']=100*(1-rec['total_cells']/11364822)
            rec['stage']='export';save()
            r=io.export_fluent_case(str(OUT/'coarse_raw.cas.h5'),p.ExportFluentCaseParams(model,cff_format=True))
            if r.error_code.value:raise RuntimeError(str(r))
            rec.update(status='MESH_GENERATED_STATIC_PENDING',elapsed_s=time.time()-start,cell_count_target_pass=rec['total_cells']<=5000000);save()
    except Exception as e:
        rec.update(status='FAIL',error=repr(e),traceback=traceback.format_exc());save();raise

if __name__=='__main__':main()
