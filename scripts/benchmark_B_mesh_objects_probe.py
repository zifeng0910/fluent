"""Test Fluent 2026 R1 generated mesh-object and overset commands on Prime mesh."""

import json
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_mesh_objects_probe.json"


def save(report):
    EVIDENCE.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")


def main():
    report = {"status": "RUNNING", "stage": "launch"}
    save(report)
    session = None
    try:
        session = pyfluent.launch_fluent(
            mode="meshing", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE_DIR),
        )
        report["stage"] = "read_mesh"
        save(report)
        session.meshing.File.ReadMesh(FileName=str(CASE_DIR / "benchmark_B_overset.msh.h5"))
        report["objects_before"] = session.meshing_utilities.get_all_objects()
        report["stage"] = "create_mesh_objects"
        save(report)
        report["mesh_object_result"] = session.meshing.CreateMeshObjects(
            MergeZonesBasedOnLabels=False, CreateAFaceZonePerBody=False
        )
        report["objects_after"] = session.meshing_utilities.get_all_objects()
        report["stage"] = "create_overset_interfaces"
        save(report)
        command = session.meshing.CreateOversetInterfaces
        args = command.create_instance()
        report["object_selection_allowed"] = args.ObjectSelectionList.allowed_values()
        save(report)
        report["interface_result"] = command(
            OversetInterfacesName="benchmark_B_overset_objects",
            ObjectSelectionList=["background_water", "robot_component_fluid"],
        )
        report["stage"] = "write_case"
        save(report)
        session.meshing.File.WriteCase(FileName=str(CASE_DIR / "benchmark_B_mesh_objects.cas.h5"))
        report["status"] = "COMMAND_ACCEPTED_UNVERIFIED" if report["interface_result"] else "FAIL"
        report["stage"] = "complete"
        save(report)
        print(json.dumps(report, indent=2, default=str))
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = repr(exc)
        report["traceback"] = traceback.format_exc()
        save(report)
        raise
    finally:
        if session is not None:
            session.exit()


if __name__ == "__main__":
    main()
