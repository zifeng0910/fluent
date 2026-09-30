"""Try generated overset interface creation after the solver set its boundary type."""

import json
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_overset_from_boundary.json"


def save(data):
    EVIDENCE.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")


def main():
    report = {"status": "RUNNING", "stage": "launch"}
    save(report)
    meshing = None
    try:
        meshing = pyfluent.launch_fluent(
            mode="meshing", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE_DIR),
        )
        report["stage"] = "read_boundary_case"
        save(report)
        meshing.meshing.File.ReadCase(FileName=str(CASE_DIR / "benchmark_B_overset_boundary_SI.cas.h5"))
        report["stage"] = "inspect_existing_overset_task"
        save(report)
        tasks = meshing.meshing_workflow.task_object.create_overset_mesh
        report["task_names"] = [str(name) for name in tasks.get_state()]
        report["task_before"] = {
            name: tasks[name].get_state() for name in report["task_names"]
        }
        save(report)
        task = tasks["benchmark_B_overset"]
        task.arguments.set_state({
            "overset_interfaces_name": "benchmark_B_overset",
            "object_selection_list": ["background_water", "robot_component_fluid"],
        })
        report["stage"] = "execute_overset_task"
        save(report)
        report["result"] = task.execute()
        report["task_after"] = task.get_state()
        output = CASE_DIR / "benchmark_B_overset_interface_SI.cas.h5"
        meshing.meshing.File.WriteCase(FileName=str(output))
        report["output"] = str(output)
        report["status"] = "COMMAND_ACCEPTED_UNVERIFIED" if report["result"] else "FAIL"
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
        if meshing is not None:
            meshing.exit()


if __name__ == "__main__":
    main()
