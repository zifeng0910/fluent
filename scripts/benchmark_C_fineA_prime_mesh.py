"""C-only close-fitting shell and local BOI tetra mesh; frozen physics untouched."""
import json,re,time,traceback,argparse
from pathlib import Path
import ansys.meshing.prime as p
from ansys.meshing.prime import lucid
ROOT=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--background-size',type=float,default=.020);args=ap.parse_args()
    h=args.background_size; tag=f'bg{round(h*1000):03d}'
    out=ROOT/'live_cases/benchmark_C_fineA'/tag;out.mkdir(parents=True,exist_ok=True)
    report=ROOT/f'evidence/benchmark_C_fineA_prime_{tag}.json'
    rec={'status':'RUNNING','background_local_size_mm':h,'component_size_mm':[.010,.0125],'growth_rate':1.2,'stage':'launch'}
    def save():report.write_text(json.dumps(rec,indent=2))
    save();start=time.time()
    try:
        with p.launch_prime(timeout=120) as session:
            model=session.model;io=p.FileIO(model)
            for name in ['background_water','robot_component_fluid','overlap_boi']:
                result=io.import_cad(str(ROOT/f'geometry/benchmark_C_fineA/{name}.step'),p.ImportCadParams(model,append=True))
                if result.error_code.value:raise RuntimeError(str(result))
                model._sync_up_model();part=model.get_part_by_name(name)
                if len(part.get_volume_zones())!=1:raise RuntimeError(f'Not single volume: {name}')
                model.set_suggested_zone_name(part.get_volume_zones()[0],name)
            model._sync_up_model();parts={a.name:a for a in model.parts};mesh=lucid.Mesh(model)
            # Closed BOI surface provides the sizing volume; it is never a solver cell zone.
            rec['stage']='boi_surface';save()
            mesh.surface_mesh(min_size=.08,max_size=.10,scope=lucid.SurfaceScope(part_expression='overlap_boi'))
            model._sync_up_model()
            model.set_global_sizing_params(p.GlobalSizingParams(model,min=.010,max=.12,growth_rate=1.2))
            controls=[]
            for name,lo,hi in [('background_water',.06,.12),('robot_component_fluid',.010,.0125)]:
                ctrl=model.control_data.create_size_control(p.SizingType.CURVATURE)
                ctrl.set_curvature_sizing_params(p.CurvatureSizingParams(model,min=lo,max=hi,growth_rate=1.2,normal_angle=18,use_cad_curvature=True))
                ctrl.set_scope(p.ScopeDefinition(model,part_expression=name,label_expression='*',entity_type=p.ScopeEntity.FACEANDEDGEZONELETS))
                controls.append(ctrl.id)
            boi=model.control_data.create_size_control(p.SizingType.BOI)
            boi.set_boi_sizing_params(p.BoiSizingParams(model,max=h,growth_rate=1.2))
            boi.set_scope(p.ScopeDefinition(model,part_expression='overlap_boi',label_expression='*',entity_type=p.ScopeEntity.FACEANDEDGEZONELETS))
            controls.append(boi.id)
            rec['stage']='size_field';save()
            sf=p.SizeField(model); result=sf.compute_volumetric(controls,p.VolumetricSizeFieldComputeParams(model))
            rec['size_field_result']=str(result);save()
            for name in ['background_water','robot_component_fluid']:
                part=parts[name];rec['stage']=name+'_surface';save()
                result=p.Surfer(model).mesh_topo_faces(part.id,part.get_topo_faces(),p.SurferParams(model,size_field_type=p.SizeFieldType.VOLUMETRIC,min_size=.010,max_size=.12,growth_rate=1.2))
                if result.error_code.value:raise RuntimeError(str(result))
                model._sync_up_model()
            util=p.SurfaceUtilities(model);rec['face_classes']={}
            for name in ['background_water','robot_component_fluid']:
                part=parts[name];faces=list(part.get_topo_faces());zs=model.topo_data.get_mesh_zonelets_of_topo_faces(faces);classes={}
                for face,z in zip(faces,zs):
                    b=[float(v) for v in re.findall(r'(?:xmin|xmax|ymin|ymax|zmin|zmax)\s*:\s*([-+\d.eE]+)',str(util.get_bounding_box_of_zonelets([z])))]
                    if len(b)!=6:raise RuntimeError(str(b))
                    xmin,ymin,zmin,xmax,ymax,zmax=b
                    if name=='background_water':label='inlet' if xmax<-4.99 else 'outlet' if xmin>4.99 else 'pipe_wall'
                    else:label='overset_component' if xmin<-.001 or xmax>2.301 or max(abs(ymin),abs(ymax),abs(zmin),abs(zmax))>.425 else 'robot_wall'
                    classes.setdefault(label,[]).append(face)
                expected=3 if name=='background_water' else 2
                if len(classes)!=expected:raise RuntimeError(f'Wrong face classes {classes}')
                for label,faces0 in classes.items():part.add_labels_on_topo_entities([label],faces0)
                rec['face_classes'][name]=classes
            model._sync_up_model();mesh.create_zones_from_labels('inlet,outlet,pipe_wall,robot_wall,overset_component');model._sync_up_model()
            for name in ['background_water','robot_component_fluid']:
                rec['stage']=name+'_volume';save();part=parts[name]
                vc=model.control_data.create_volume_control()
                vc.set_params(p.VolumeControlParams(model,cell_zonelet_type=p.CellZoneletType.FLUID))
                vc.set_scope(p.ScopeDefinition(model,part_expression=name,zone_expression='*',entity_type=p.ScopeEntity.VOLUME,evaluation_type=p.ScopeEvaluationType.ZONES))
                result=p.AutoMesh(model).mesh(part.id,p.AutoMeshParams(model,size_field_type=p.SizeFieldType.VOLUMETRIC,volume_fill_type=p.VolumeFillType.TET,volume_control_ids=[vc.id]))
                rec[name+'_mesh_result']=str(result);model._sync_up_model()
                summary=part.get_summary(p.PartSummaryParams(model));rec[name+'_summary']=str(summary)
                if summary.n_tet_cells<=0:raise RuntimeError(f'Zero tetrahedra: {name}')
                rec[name+'_cells']=int(summary.n_tet_cells);save()
            model.delete_parts([parts['overlap_boi'].id]);model._sync_up_model()
            file=out/'benchmark_C_fineA.cas.h5';rec['stage']='export';save()
            result=io.export_fluent_case(str(file),p.ExportFluentCaseParams(model,cff_format=True))
            if result.error_code.value:raise RuntimeError(str(result))
            rec.update(status='MESH_GENERATED_NOT_CONNECTIVITY_VALIDATED',stage='complete',case_file=str(file),elapsed_seconds=time.time()-start)
            save()
    except Exception as e:
        rec.update(status='FAIL',error=repr(e),traceback=traceback.format_exc(),elapsed_seconds=time.time()-start);save();raise
if __name__=='__main__':main()
