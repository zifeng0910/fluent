"""Extract the validated component through Prime; never load fine solver DATA."""
import json, time, traceback
from pathlib import Path
import ansys.meshing.prime as p
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'live_cases/benchmark_C_coarse_A'
EVID=ROOT/'evidence/benchmark_C_coarse_A'

def main():
    OUT.mkdir(parents=True,exist_ok=True);EVID.mkdir(parents=True,exist_ok=True)
    rec={'status':'RUNNING','stage':'import_fine_raw_mesh','source':'live_cases/benchmark_C_fineA/bg020/benchmark_C_fineA.cas.h5','fine_solver_DATA_loaded':False}
    def save():
        tmp=EVID/'component_extraction.json.tmp';tmp.write_text(json.dumps(rec,indent=2));tmp.replace(EVID/'component_extraction.json')
    save();start=time.time()
    try:
        with p.launch_prime(timeout=120) as session:
            model=session.model;io=p.FileIO(model)
            r=io.import_fluent_case(str(ROOT/rec['source']),p.ImportFluentCaseParams(model,append=False))
            if r.error_code.value:raise RuntimeError(str(r))
            model._sync_up_model()
            rec['parts']=[{'name':part.name,'id':part.id,'summary':str(part.get_summary(p.PartSummaryParams(model)))} for part in model.parts];save()
            component=model.get_part_by_name('robot_component_fluid')
            if component is None and len(model.parts)==1:
                component=model.parts[0]
                info=p.MeshInfo(model);background=[]
                rec['cell_zonelet_statistics']={}
                for cell in component.get_cell_zonelets():
                    stats=info.get_statistics_of_cell_zonelets([cell],p.CellStatisticsParams(model,get_volume=True))
                    rec['cell_zonelet_statistics'][int(cell)]=str(stats)
                    if 24<stats.volume<26:background.append(cell)
                save()
                if len(background)!=1:raise RuntimeError('Cannot uniquely identify validated background cell zonelet')
                result=component.delete_zonelets(background)
                rec['delete_background_cells_result']=str(result)
                volumes=component.get_volumes_of_zone_name_pattern('background_water',p.NamePatternParams(model))
                if len(volumes)!=1:raise RuntimeError(f'Background volume selection failed: {volumes}')
                r=component.delete_volumes(volumes,p.DeleteVolumesParams(model,delete_small_volumes=False))
                rec['delete_background_result']=str(r)
                model._sync_up_model();component.set_suggested_name('robot_component_fluid');model._sync_up_model()
            if component is None:raise RuntimeError('Validated component missing from Prime import')
            summary=component.get_summary(p.PartSummaryParams(model))
            if summary.n_tet_cells!=1353606:raise RuntimeError(f'Component count mismatch: {summary.n_tet_cells}')
            others=[part.id for part in model.parts if part.id!=component.id]
            if others:model.delete_parts(others)
            model._sync_up_model()
            rec['stage']='export_component';save()
            target=OUT/'validated_component.cas.h5'
            r=io.export_fluent_case(str(target),p.ExportFluentCaseParams(model,cff_format=True))
            if r.error_code.value:raise RuntimeError(str(r))
            rec.update(status='EXTRACTED_TOPOLOGY_VERIFICATION_PENDING',component_cells=int(summary.n_tet_cells),output=str(target),elapsed_s=time.time()-start);save()
    except Exception as e:
        rec.update(status='FAIL',error=repr(e),traceback=traceback.format_exc());save();raise

if __name__=='__main__':main()
