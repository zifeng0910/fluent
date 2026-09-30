"""Mesh separate background and moving component CAD parts with Prime Route D."""

from __future__ import annotations

import json
import re
import traceback
from pathlib import Path

import ansys.meshing.prime as prime
from ansys.meshing.prime import lucid


ROOT = Path(__file__).resolve().parents[1]
GEOM = ROOT / "geometry" / "benchmark_B_overset"
OUT = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_prime_mesh.json"


def save(data):
    EVIDENCE.write_text(json.dumps(data, indent=2), encoding="utf-8")


def bounds(box):
    text = str(box)
    vals = [float(x) for x in re.findall(r"(?:xmin|xmax|ymin|ymax|zmin|zmax)\s*:\s*([-+\d.eE]+)", text)]
    if len(vals) != 6:
        raise RuntimeError(f"Cannot parse Prime bounding box: {text}")
    return vals  # xmin, ymin, zmin, xmax, ymax, zmax


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"status": "RUNNING", "stage": "launch", "prime_version": prime.__version__}
    save(report)
    try:
        with prime.launch_prime(timeout=120) as session:
            model = session.model
            io = prime.FileIO(model)
            for filename in ["background_water.step", "robot_component_fluid.step"]:
                result = io.import_cad(str(GEOM / filename), prime.ImportCadParams(model=model, append=True))
                if result.error_code.value != 0:
                    raise RuntimeError(f"Prime CAD import failed: {filename}: {result}")
                model._sync_up_model()
                imported_part = model.get_part_by_name(Path(filename).stem)
                imported_zones = list(imported_part.get_volume_zones())
                if len(imported_zones) != 1:
                    raise RuntimeError(f"Imported part has unexpected zones: {filename}: {imported_zones}")
                model.set_suggested_zone_name(imported_zones[0], Path(filename).stem)
                model._sync_up_model()
            parts = {part.name: part for part in model.parts}
            if set(parts) != {"background_water", "robot_component_fluid"}:
                raise RuntimeError(f"Expected two independent Prime parts, got {list(parts)}")
            report["stage"] = "surface_mesh"
            save(report)
            mesh = lucid.Mesh(model)
            mesh.surface_mesh(min_size=0.06, max_size=0.12)
            model._sync_up_model()
            surfaces = {}
            utility = prime.SurfaceUtilities(model)
            for part_name, part in parts.items():
                faces = list(part.get_topo_faces())
                zonelets = model.topo_data.get_mesh_zonelets_of_topo_faces(faces)
                classes = {name: [] for name in (["inlet", "outlet", "pipe_wall"] if part_name == "background_water" else ["robot_wall", "overset_component"])}
                face_bounds = {}
                for face, zonelet in zip(faces, zonelets):
                    b = bounds(utility.get_bounding_box_of_zonelets([zonelet]))
                    face_bounds[str(face)] = b
                    xmin, ymin, zmin, xmax, ymax, zmax = b
                    if part_name == "background_water":
                        label = "inlet" if xmax < -4.99 else "outlet" if xmin > 4.99 else "pipe_wall"
                    else:
                        is_outer = (xmin < -0.49 or xmax > 2.79 or
                                    max(abs(ymin), abs(ymax), abs(zmin), abs(zmax)) > 0.55)
                        label = "overset_component" if is_outer else "robot_wall"
                    classes[label].append(face)
                if any(not ids for ids in classes.values()):
                    raise RuntimeError(f"Missing face class on {part_name}: {classes}")
                for name, ids in classes.items():
                    part.add_labels_on_topo_entities([name], ids)
                surfaces[part_name] = {"labels": {key: list(map(int, ids)) for key, ids in classes.items()},
                                       "bounds": face_bounds}
            model._sync_up_model()
            mesh.create_zones_from_labels("inlet,outlet,pipe_wall,robot_wall,overset_component")
            model._sync_up_model()
            report["surface"] = surfaces
            report["stage"] = "volume_mesh"
            save(report)
            for part_name, part in parts.items():
                mesh.volume_mesh(volume_fill_type=prime.VolumeFillType.TET,
                                 scope=lucid.VolumeScope(part_expression=part.name,
                                                         entity_expression="*"))
                model._sync_up_model()
                zones = list(part.get_volume_zones())
                if len(zones) != 1:
                    raise RuntimeError(f"Expected one volume zone for {part_name}: {zones}")
                if model.get_zone_name(zones[0]) != part_name:
                    raise RuntimeError(f"Wrong volume zone for {part_name}: {model.get_zone_name(zones[0])}")
            model._sync_up_model()
            report["mesh_summaries"] = {name: str(part.get_summary(prime.PartSummaryParams(model)))
                                        for name, part in parts.items()}
            report["volume_zone_names"] = {name: [model.get_zone_name(z) for z in part.get_volume_zones()]
                                           for name, part in parts.items()}
            report["stage"] = "export"
            save(report)
            case = OUT / "benchmark_B_overset.cas.h5"
            msh = OUT / "benchmark_B_overset.msh.h5"
            report["case_export"] = str(io.export_fluent_case(str(case),
                prime.ExportFluentCaseParams(model, cff_format=True)))
            report["mesh_export"] = str(io.export_fluent_meshing_mesh(str(msh),
                prime.ExportFluentMeshingMeshParams(model, cff_format=True)))
            report["status"] = "PASS"
            report["stage"] = "complete"
            save(report)
            print(json.dumps({k: report[k] for k in ["status", "volume_zone_names", "case_export", "mesh_export"]}, indent=2))
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = repr(exc)
        report["traceback"] = traceback.format_exc()
        save(report)
        raise


if __name__ == "__main__":
    main()
