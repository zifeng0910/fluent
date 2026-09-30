"""Verify overset boundary conversion through Fluent settings API only."""

import json
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_boundary_probe.json"


def save(data):
    EVIDENCE.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")


def main():
    report = {"status": "RUNNING"}
    save(report)
    solver = None
    try:
        solver = pyfluent.launch_fluent(
            mode="solver", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE_DIR),
        )
        solver.settings.file.read_case(file_name=str(CASE_DIR / "benchmark_B_overset_workflow_SI.cas.h5"))
        report["before"] = {
            "overset_boundaries": list(solver.settings.setup.boundary_conditions.overset.keys()),
            "mesh_interfaces": list(solver.settings.setup.mesh_interfaces.interface.keys()),
        }
        save(report)
        solver.settings.setup.boundary_conditions.set_zone_type(
            zone_list=["overset_component"], new_type="overset"
        )
        report["after"] = {
            "overset_boundaries": list(solver.settings.setup.boundary_conditions.overset.keys()),
            "mesh_interfaces": list(solver.settings.setup.mesh_interfaces.interface.keys()),
            "overset_boundary_state": solver.settings.setup.boundary_conditions.overset["overset_component"].get_state(),
        }
        report["status"] = "PASS" if "overset_component" in report["after"]["overset_boundaries"] else "FAIL"
        solver.settings.file.write_case(file_name=str(CASE_DIR / "benchmark_B_overset_boundary_SI.cas.h5"))
        save(report)
        print(json.dumps(report, indent=2, default=str))
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = repr(exc)
        report["traceback"] = traceback.format_exc()
        save(report)
        raise
    finally:
        if solver is not None:
            solver.exit()


if __name__ == "__main__":
    main()
