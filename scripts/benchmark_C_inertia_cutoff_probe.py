"""Lower Fluent's determinant cutoff without changing the L2300 inertia tensor."""
import json
from pathlib import Path


def run(solver, context):
    root = context["root"]
    before = solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)")
    set_result = solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    after = solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)")
    result = {
        "status": "PASS" if float(after) <= 1e-50 else "FAIL",
        "setting": "dynamesh/sdof/minimum-cutoff-moments",
        "before": str(before),
        "set_result": str(set_result),
        "after": str(after),
        "body_inertia_kg_m2": [7.257810693523445e-13, 3.929384741151496e-12, 3.92938474115149e-12],
        "inertia_determinant_kg3_m6": 7.257810693523445e-13 * 3.929384741151496e-12 * 3.92938474115149e-12,
        "physical_inertia_modified": False,
    }
    (root / "evidence/benchmark_C_inertia_cutoff.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    context["c_inertia_cutoff"] = result
