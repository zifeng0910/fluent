"""Inspect the exported two-zone overset candidate through direct headless PyFluent."""

import json
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_solver_probe.json"


def save(data):
    EVIDENCE.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")


def main():
    result = {"status": "RUNNING", "stage": "launch"}
    save(result)
    solver = None
    try:
        solver = pyfluent.launch_fluent(
            mode="solver", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE_DIR),
        )
        case = CASE_DIR / "benchmark_B_overset_workflow.cas.h5"
        solver.settings.file.read_case(file_name=str(case))
        result["stage"] = "scale_to_SI"
        save(result)
        solver.settings.mesh.scale(x_scale=0.001, y_scale=0.001, z_scale=0.001)
        result["mesh_check"] = str(solver.settings.mesh.check())
        result["cell_zone_names"] = list(solver.settings.setup.cell_zone_conditions.fluid.keys())
        result["boundary_names"] = {
            name: list(getattr(solver.settings.setup.boundary_conditions, name).keys())
            for name in ["wall", "velocity_inlet", "pressure_outlet", "overset"]
        }
        result["overset_surface_name"] = "overset_component"
        result["cell_zone_states"] = {
            name: solver.settings.setup.cell_zone_conditions.fluid[name].get_state()
            for name in result["cell_zone_names"]
        }
        result["dynamic_mesh_children"] = list(solver.settings.setup.dynamic_mesh.child_names)
        result["setup_overset_attributes"] = [name for name in dir(solver.settings.setup) if "overset" in name.lower()]
        result["root_overset_attributes"] = [name for name in dir(solver.settings) if "overset" in name.lower()]
        result["setup_interfaces_attributes"] = [name for name in dir(solver.settings.setup) if "interface" in name.lower()]
        result["root_interfaces_attributes"] = [name for name in dir(solver.settings) if "interface" in name.lower()]
        result["stage"] = "write_scaled_case"
        save(result)
        scaled = CASE_DIR / "benchmark_B_overset_workflow_SI.cas.h5"
        solver.settings.file.write_case(file_name=str(scaled))
        result["scaled_case"] = str(scaled)
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
        if solver is not None:
            solver.exit()


if __name__ == "__main__":
    main()
