"""Probe Fluent 2026 R1 generated overset workflow on the Prime mesh."""

import json
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_overset_workflow_probe.json"


def save(data):
    EVIDENCE.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")


def main():
    result = {"status": "RUNNING", "stage": "launch"}
    save(result)
    meshing = None
    try:
        meshing = pyfluent.launch_fluent(
            mode="meshing", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE_DIR),
        )
        result["stage"] = "read_prime_mesh"
        save(result)
        meshing.meshing.File.ReadMesh(FileName=str(CASE_DIR / "benchmark_B_overset.msh.h5"))
        result["stage"] = "inspect_overset_task"
        save(result)
        boundary_args = meshing.meshing_workflow.application.update_boundaries.create_instance()
        result["update_boundaries_state"] = boundary_args.get_state() if boundary_args else None
        region_args = meshing.meshing_workflow.application.update_region_settings.create_instance()
        result["update_region_state"] = region_args.get_state() if region_args else None
        save(result)
        result["update_region_result"] = meshing.meshing_workflow.application.update_region_settings(
            main_fluid_region="background_water",
            region_name_list=["background_water", "robot_component_fluid"],
            region_overset_componen_list=["no", "yes"],
            old_region_overset_componen_list=["no", "no"],
        )
        save(result)
        result["update_boundaries_result"] = meshing.meshing_workflow.application.update_boundaries(
            selection_type="zone",
            boundary_zone_list=["overset_component"],
            boundary_zone_type_list=["overset"],
            old_boundary_zone_list=["overset_component"],
            old_boundary_zone_type_list=["wall"],
        )
        task = meshing.meshing_workflow.application.create_overset_mesh
        args = task.create_instance()
        result["task_argument_state"] = args.get_state() if args is not None else None
        result["task_object_selection_allowed"] = (
            args.object_selection_list.allowed_values() if args is not None else None
        )
        result["stage"] = "create_overset_interface"
        save(result)
        result["create_overset_result"] = task(
            overset_interfaces_name="benchmark_B_overset",
            object_selection_list=["background_water", "robot_component_fluid"],
        )
        save(result)
        result["stage"] = "write_overset_case"
        save(result)
        overset_case = CASE_DIR / "benchmark_B_overset_workflow.cas.h5"
        meshing.meshing.File.WriteCase(FileName=str(overset_case))
        result["overset_case"] = str(overset_case)
        result["status"] = "PASS"
        result["stage"] = "complete"
        save(result)
        print(json.dumps(result, indent=2, default=str))
    except Exception as exc:
        result["status"] = "FAIL"
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        save(result)
        raise
    finally:
        if meshing is not None:
            meshing.exit()


if __name__ == "__main__":
    main()
