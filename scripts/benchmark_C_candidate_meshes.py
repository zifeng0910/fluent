"""Build four component-only Prime meshes; frozen B background is never meshed."""
import json, re, traceback, subprocess
from pathlib import Path
import ansys.meshing.prime as prime
from ansys.meshing.prime import lucid

ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble.step'
OUT = ROOT/'live_cases/benchmark_C_candidates'
GEOM = ROOT/'geometry/benchmark_C_candidates'
REPORT = ROOT/'evidence/benchmark_C_candidate_meshes.json'

def main():
    OUT.mkdir(exist_ok=True); GEOM.mkdir(exist_ok=True)
    records = []
    for cid, shell in zip('ABCD', [.05,.0625,.075,.10]):
        rec = dict(candidate_id=cid, shell_radial_mm=shell, axial_extension_mm=.10,
                   radius_mm=.4075+shell, x_bounds_mm=[-.10,2.40], status='RUNNING')
        records.append(rec); REPORT.write_text(json.dumps(records,indent=2))
        try:
            step = GEOM/f'component_{cid}.step'
            subprocess.run([str(ROOT/'.venv/Scripts/python.exe'),str(ROOT/'scripts/benchmark_C_candidate_geometry.py'),cid,str(shell)],check=True)
            with prime.launch_prime(timeout=120) as session:
                model=session.model; io=prime.FileIO(model)
                imp=io.import_cad(str(step),prime.ImportCadParams(model,append=True))
                if imp.error_code.value: raise RuntimeError(str(imp))
                model._sync_up_model(); part=model.parts[0]
                model.set_suggested_zone_name(part.get_volume_zones()[0],'robot_component_fluid')
                mesh=lucid.Mesh(model)
                mesh.surface_mesh(min_size=.01,max_size=.02)
                model._sync_up_model(); utility=prime.SurfaceUtilities(model)
                faces=list(part.get_topo_faces()); zonelets=model.topo_data.get_mesh_zonelets_of_topo_faces(faces)
                labels={'robot_wall':[],'overset_component':[]}
                for face,z in zip(faces,zonelets):
                    vals=[float(x) for x in re.findall(r'(?:xmin|xmax|ymin|ymax|zmin|zmax)\s*:\s*([-+\d.eE]+)',str(utility.get_bounding_box_of_zonelets([z])))]
                    xmin,ymin,zmin,xmax,ymax,zmax=vals
                    outer=xmin<-.0999 or xmax>2.3999 or max(abs(ymin),abs(ymax),abs(zmin),abs(zmax))>.425
                    labels['overset_component' if outer else 'robot_wall'].append(face)
                if any(not x for x in labels.values()): raise RuntimeError(str(labels))
                for name,ids in labels.items(): part.add_labels_on_topo_entities([name],ids)
                model._sync_up_model(); mesh.create_zones_from_labels('robot_wall,overset_component')
                mesh.volume_mesh(volume_fill_type=prime.VolumeFillType.TET,scope=lucid.VolumeScope(part_expression=part.name,entity_expression='*'))
                model._sync_up_model()
                summary=part.get_summary(prime.PartSummaryParams(model))
                rec['summary']=str(summary)
                if summary.n_tet_cells <= 0: raise RuntimeError('Zero component tetrahedra')
                rec['tetra_cells']=int(summary.n_tet_cells)
                file=OUT/f'component_{cid}.cas.h5'
                exp=io.export_fluent_case(str(file),prime.ExportFluentCaseParams(model,cff_format=True))
                if exp.error_code.value: raise RuntimeError(str(exp))
                rec.update(status='PASS',case_file=str(file),surface_size_mm=[.01,.02])
        except Exception as e: rec.update(status='FAIL',error=repr(e),traceback=traceback.format_exc())
        REPORT.write_text(json.dumps(records,indent=2)); print(json.dumps(rec),flush=True)

if __name__=='__main__': main()
