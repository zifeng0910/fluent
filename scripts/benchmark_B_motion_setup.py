"""Configure one rigid component translation using introspected 261 settings."""
import json


def run(solver, context):
    assert context["b1"]["status"] == "PASS", "B1 must pass before motion setup"
    settings = solver.settings
    report = {"status": "RUNNING"}
    output = context["root"] / "evidence/benchmark_B_prescribed_motion_setup.json"

    def save():
        output.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    try:
        general = settings.setup.general
        report["time_allowed"] = general.solver.time.allowed_values()
        general.solver.time = "transient"
        dynamic = settings.setup.dynamic_mesh
        dynamic.enabled = True
        dynamic.methods.smoothing.enabled = False
        report["dynamic_mesh_state"] = dynamic.get_state()
        report["physics_regions_allowed"] = dynamic.dynamic_zones.create.physics_region.allowed_values()
        report["zones_allowed"] = dynamic.dynamic_zones.create.zone.allowed_values()
        report["general_state"] = general.get_state()
        save()
        physics = report["physics_regions_allowed"]
        assert len(physics) <= 1, f"Ambiguous physics regions: {physics}"
        assert "robot_component_fluid" in report["zones_allowed"]
        if not dynamic.dynamic_zones.keys():
            if physics:
                dynamic.dynamic_zones.create(physics_region=physics[0], zone="robot_component_fluid")
            else:
                dynamic.dynamic_zones.create(zone="robot_component_fluid")
        report["dynamic_zone_names"] = list(dynamic.dynamic_zones.keys())
        name = next(name for name in report["dynamic_zone_names"]
                    if dynamic.dynamic_zones[name].zone.get_state() == "robot_component_fluid")
        node = dynamic.dynamic_zones[name]
        report["component_type_allowed"] = node.type.allowed_values()
        node.type = "rigid-body"
        report["motion_def_allowed"] = node.motion.motion_def.allowed_values()
        report["component_state_before_hook"] = node.get_state()
        save()
        hook = "benchmark_B_translate_x::libbenchmark_B"
        assert hook in report["motion_def_allowed"]
        node.motion.motion_def = hook
        node.motion.rigid_body_properties.cg_position = [0.0012060186937156343, 0.0, 0.0]
        node.motion.rigid_body_properties.orientation.set_state({"angle": 0.0, "axis": [1.0, 0.0, 0.0]})
        report["component_state"] = node.get_state()
        report["dynamic_mesh_state_after"] = dynamic.get_state()
        context["component_dynamic_zone_name"] = name
        report["status"] = "PASS"
        save()
        print(json.dumps(report, indent=2, default=str), flush=True)
    except Exception as exc:
        report.update(status="BLOCKED", error=repr(exc))
        save()
        raise
