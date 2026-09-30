"""Record coordinates for the first orphan cell at the stopped C pose."""
import json


def run(solver, context):
    root = context["root"]
    user = solver.settings.setup.user_defined
    user.compiled_udf(library_name="libbenchmark_C_stats_v1",
                      source_files=[str(root / "fluent_udf/benchmark_C_overset_statistics.c")],
                      header_files=[], use_built_in_compiler=True)
    user.load(udf_library_name="libbenchmark_C_stats_v1")
    user.execute_on_demand(lib_name="benchmark_C_overset_statistics::libbenchmark_C_stats_v1")
    src = root / "evidence/benchmark_C_overset_live.json"
    audit = json.loads(src.read_text())
    audit["pose_time_s"] = audit["time_s"]
    (root / "evidence/benchmark_C_orphan_location.json").write_text(
        json.dumps(audit, indent=2), encoding="utf-8")
