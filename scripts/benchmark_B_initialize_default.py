"""B1: validate topology, initialize the existing mesh, then check connectivity."""
from __future__ import annotations

import json
import re
import time


def run(solver, context):
    root, case = context["root"], context["case_dir"]
    evidence = root / "evidence/benchmark_B_default_overset_interface.json"
    report = {"status": "RUNNING", "stage": "read_existing_case",
              "source_case": str(case / "benchmark_B_overset_boundary_SI.cas.h5"),
              "official_default_interface_reference": "https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_ug/flu_ug_sec_overset.html#flu_ug_sec_overset_setup"}

    def save():
        evidence.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    save()
    try:
        solver.settings.file.read_case(file_name=report["source_case"])
        settings = solver.settings
        bc = settings.setup.boundary_conditions
        fluids = settings.setup.cell_zone_conditions.fluid
        report["cell_zones"] = list(fluids.keys())
        report["face_zones"] = {}
        for kind in ["wall", "interior", "overset", "pressure_inlet", "pressure_outlet"]:
            family = getattr(bc, kind)
            for name in family.keys():
                node = family[name]
                report["face_zones"][name] = {
                    "type": kind,
                    "adjacent_cell_zone": node.adjacent_cell_zone() if hasattr(node, "adjacent_cell_zone") else None,
                }
        assert set(report["cell_zones"]) == {"background_water", "robot_component_fluid"}
        assert report["face_zones"]["overset_component"]["type"] == "overset"
        assert report["face_zones"]["robot_wall"]["type"] == "wall"
        report["topology_gate"] = "PASS" if (
            report["face_zones"]["overset_component"]["adjacent_cell_zone"] in
            ["robot_component_fluid", 2, [2], ["robot_component_fluid"]]
        ) else "UNRESOLVED"
        # Boundary adjacency is queried from Fluent, not inferred from names.
        save()
        if report["topology_gate"] != "PASS":
            raise RuntimeError("Cannot establish component-only overset boundary adjacency")
        report["stage"] = "minimal_water_physics"
        save()
        settings.setup.models.viscous.model = "laminar"
        materials = settings.setup.materials.fluid
        if "water" not in materials.keys():
            materials.create(name="water")
        materials["water"].density.set_state({"option": "constant", "value": 998.2})
        materials["water"].viscosity.set_state({"option": "constant", "value": 0.001003})
        for name in report["cell_zones"]:
            fluids[name].general.material = "water"
        bc.set_zone_type(zone_list=["inlet"], new_type="pressure-inlet")
        bc.set_zone_type(zone_list=["outlet"], new_type="pressure-outlet")
        report["physics_readback"] = {
            "viscous_model": settings.setup.models.viscous.model.get_state(),
            "water": materials["water"].get_state(),
            "inlet": bc.pressure_inlet["inlet"].get_state(),
            "outlet": bc.pressure_outlet["outlet"].get_state(),
            "robot_wall": bc.wall["robot_wall"].get_state(),
            "pipe_wall": bc.wall["pipe_wall"].get_state(),
            "dynamic_mesh": settings.setup.dynamic_mesh.enabled.get_state(),
        }
        save()
        report["stage"] = "hybrid_initialization"
        save()
        start = len(context["messages"])
        settings.solution.initialization.hybrid_initialize()
        time.sleep(0.5)
        init_text = "".join(context["messages"][start:])
        (root / "evidence/benchmark_B_initialization_transcript.txt").write_text(init_text, encoding="utf-8")
        report["initialization_transcript"] = str(root / "evidence/benchmark_B_initialization_transcript.txt")
        report["mesh_interfaces_settings_state"] = settings.setup.mesh_interfaces.get_state()
        report["default_interface_created"] = "default-overset-interface" in init_text or "Auto create default overset interface" in init_text
        report["interface_name"] = "default-overset-interface" if report["default_interface_created"] else None
        report["background_cell_zone"] = "background_water"
        report["component_cell_zone"] = "robot_component_fluid"
        report["overset_face_zones"] = ["overset_component"]
        report["stage"] = "post_initialization_mesh_check"
        save()
        check_start = len(context["messages"])
        settings.mesh.check()
        time.sleep(0.5)
        check_text = "".join(context["messages"][check_start:])
        check_report = {
            "status": "FAIL" if re.search(r"mesh check failed|does not belong|negative volume|invalid cell", check_text, re.I) else "PASS",
            "transcript": check_text,
            "performed_after_initialization": True,
        }
        (root / "evidence/benchmark_B_post_init_mesh_check.json").write_text(json.dumps(check_report, indent=2), encoding="utf-8")
        report["post_init_mesh_check"] = check_report["status"]
        report["initialization_connectivity_lines"] = [line for line in init_text.splitlines()
            if re.search(r"overset|orphan|receptor|donor|dead|solve cells|hole|connect", line, re.I)]
        report["available_overset_fields"] = [name for name in solver.fields.field_data.scalar_fields.allowed_values()
                                              if "overset" in name.lower()]
        report["status"] = "CONNECTIVITY_PENDING_STATISTICS" if report["default_interface_created"] and check_report["status"] == "PASS" else "BLOCKED"
        report["stage"] = "initialization_complete"
        context["b1"] = report
        save()
        print(json.dumps(report, indent=2, default=str), flush=True)
    except Exception as exc:
        report.update(status="BLOCKED", error=repr(exc))
        context["b1"] = report
        save()
        raise
