"""Inspect initialized solver connectivity and persist it without TUI."""
import json
from pathlib import Path


def run(solver, context):
    root, case = context["root"], context["case_dir"]
    context["messages"] = []
    solver.transcript.register_callback(context["messages"].append, keep_new_lines=True)
    svi = solver.fields.solution_variable_info
    zones = svi.get_zones_info()
    report = {"zones": zones.zone_names, "zone_details": {name: str(zones[name]) for name in zones.zone_names}}
    report["cell_solution_variables"] = svi.get_variables_info(
        zone_names=["background_water", "robot_component_fluid"]
    ).solution_variables
    dynamic = solver.settings.setup.dynamic_mesh
    report["dynamic_mesh_state_before"] = dynamic.get_state()
    report["dynamic_zone_create_doc"] = dynamic.dynamic_zones.create.__doc__
    report["dynamic_zone_create_arguments"] = dynamic.dynamic_zones.create.argument_names
    report["scalar_fields"] = solver.fields.field_data.scalar_fields.allowed_values()
    report["surfaces"] = solver.fields.field_data.surfaces.allowed_values()
    output = case / "benchmark_B_static_initialized.cas.h5"
    solver.settings.file.write_case(file_name=str(output))
    solver.settings.file.write_data(file_name=str(case / "benchmark_B_static_initialized.dat.h5"))
    report["initialized_case"] = str(output)
    (root / "evidence/benchmark_B_connectivity_inventory.json").write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8"
    )
    context["inventory"] = report
    print(json.dumps({"initialized_case": str(output), "solution_variables": report["cell_solution_variables"]}, default=str), flush=True)
