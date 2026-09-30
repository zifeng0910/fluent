"""Read the headless overset case back in the solver and record actual interface state."""

import json
from pathlib import Path

import ansys.fluent.core as pyfluent


ROOT = Path(__file__).resolve().parents[1]
CASE_DIR = ROOT / "live_cases" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_interface_verify.json"


def main():
    solver = pyfluent.launch_fluent(
        mode="solver", dimension=3, precision="double", processor_count=1,
        ui_mode="no_gui_or_graphics", start_timeout=120,
        fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
        cwd=str(CASE_DIR),
    )
    try:
        solver.settings.file.read_case(file_name=str(CASE_DIR / "benchmark_B_overset_interface_SI.cas.h5"))
        bc = solver.settings.setup.boundary_conditions
        interfaces = solver.settings.setup.mesh_interfaces.interface
        report = {
            "overset_boundaries": list(bc.overset.get_state()),
            "mesh_interfaces": list(interfaces.get_state()),
            "mesh_interface_state": interfaces.get_state(),
            "cell_zones": list(solver.settings.setup.cell_zone_conditions.fluid.get_state()),
            "mesh_check": str(solver.settings.mesh.check()),
        }
        report["status"] = "PASS" if report["mesh_interfaces"] else "FAIL"
        EVIDENCE.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
        print(json.dumps(report, indent=2, default=str))
    finally:
        solver.exit()


if __name__ == "__main__":
    main()
