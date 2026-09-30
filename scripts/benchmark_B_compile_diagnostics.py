"""Compile supported diagnostics, count actual overset cells, and finish B1."""
import json
import re
import time

import h5py


def run(solver, context):
    root, case = context["root"], context["case_dir"]
    user = solver.settings.setup.user_defined
    report = {"status": "RUNNING", "compiled_udf_arguments": user.compiled_udf.argument_names,
              "load_arguments": user.load.argument_names,
              "execute_on_demand_arguments": user.execute_on_demand.argument_names}
    output = root / "evidence/benchmark_B_compiled_diagnostics.json"

    def save():
        output.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    save()
    try:
        source = root / "fluent_udf/benchmark_B_prescribed_motion.c"
        if "benchmark_B_overset_statistics::libbenchmark_B" not in user.execute_on_demand.lib_name.allowed_values():
            user.compiled_udf(library_name="libbenchmark_B", source_files=[str(source)],
                              header_files=[], use_built_in_compiler=True)
            user.load(udf_library_name="libbenchmark_B")
        report["on_demand_allowed_values"] = user.execute_on_demand.lib_name.allowed_values()
        user.execute_on_demand(lib_name="benchmark_B_overset_statistics::libbenchmark_B")
        stats_path = root / "fluent_udf/benchmark_B_connectivity_live.json"
        context["statistics_path"] = stats_path
        stats = json.loads(stats_path.read_text())
        report["statistics"] = stats
        (root / "evidence/benchmark_B_initial_overset_statistics.json").write_text(
            json.dumps(stats, indent=2), encoding="utf-8")
        with h5py.File(case / "benchmark_B_static_initialized.cas.h5") as stream:
            raw = stream["settings/Rampant Variables"][()].tobytes().decode("utf-8", errors="replace")
            line = next(line for line in raw.splitlines() if line.startswith("(overset/interfaces "))
        report["case_file_interface_record"] = line
        gate = json.loads((root / "evidence/benchmark_B_default_overset_interface.json").read_text())
        gate["default_interface_created"] = "default-overset-interface" in line
        gate["interface_name"] = "default-overset-interface"
        gate["case_file_interface_record"] = line
        gate["statistics"] = stats
        valid = gate["default_interface_created"] and gate["post_init_mesh_check"] == "PASS"
        valid = valid and {zone["name"] for zone in stats["zones"]} == {"background_water", "robot_component_fluid"}
        valid = valid and all(zone["orphan"] == 0 and zone["receptors_without_donors"] == 0
                             and zone["min_volume_m3"] > 0 and zone["solve"] > 0 for zone in stats["zones"])
        valid = valid and sum(zone["receptor"] for zone in stats["zones"]) > 0
        gate["status"] = "PASS" if valid else "BLOCKED_CONNECTIVITY"
        gate["stage"] = "B1_static_connectivity_verified"
        (root / "evidence/benchmark_B_default_overset_interface.json").write_text(
            json.dumps(gate, indent=2, default=str), encoding="utf-8")
        context["b1"] = gate
        report["status"] = "PASS" if valid else "BLOCKED"
        save()
        print(json.dumps(report, indent=2, default=str), flush=True)
    except Exception as exc:
        report.update(status="BLOCKED", error=repr(exc))
        save()
        raise
