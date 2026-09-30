"""Precompute the normalized straight-tube field/gradient basis once."""
from __future__ import annotations

import json
import time

import numpy as np

from table_model import CONFIG_PATH, OUT, SERVER_PATH, build_live_model, config, gradient_unit


def main():
    cfg = config()
    spec = cfg["table"]
    started = time.perf_counter()
    phases = np.arange(spec["phase_min_deg"], spec["phase_max_deg"] + 0.5 * spec["dphase_deg"], spec["dphase_deg"])
    stations = np.arange(spec["s_eff_min_mm"], spec["s_eff_max_mm"] + 0.5 * spec["ds_mm"], spec["ds_mm"])
    # Sample the production implementation itself.  Its polarity-fixed local
    # tangent is intentionally not replaced by a hand-inferred sign convention.
    live = build_live_model(cfg)
    position = np.asarray(cfg["initial_center_aba_mm"], dtype=float)
    field_rows = []
    for value in phases:
        live.magnetic_model.reset_robot_arc_continuity()
        result = live.evaluate(value / (360.0 * cfg["frequency_Hz"]), position, np.zeros(3))
        field_rows.append(np.asarray(result["B_aba_vec_T"], dtype=float) / cfg["B0_T"])
    phase_rows = np.column_stack((phases, field_rows))
    spatial_rows = np.column_stack((stations, [gradient_unit(value, cfg).reshape(9) for value in stations]))
    output = OUT / spec["path"]
    with output.open("w", encoding="ascii", newline="\n") as handle:
        handle.write("{} {} {:.17g} {:.17g} {:.17g} {:.17g}\n".format(
            len(stations), len(phases), stations[0], spec["ds_mm"], phases[0], spec["dphase_deg"]))
        np.savetxt(handle, phase_rows, fmt="%.17e")
        np.savetxt(handle, spatial_rows, fmt="%.17e")
    elapsed = time.perf_counter() - started
    metadata = {
        "representation": "FIELD_GRADIENT_TABLE",
        "factorization": "B_unit(phase) x grad_unit(s_eff); exact tensor-product equivalent",
        "source": str(SERVER_PATH),
        "production_branch": "analytic ROBOT_LOCAL_ELLIPTIC_ROCKING point-dipole equivalent",
        "normalization": {"B_unit": "multiply by B0_T", "grad_unit_per_m": "multiply by gradient_G_T"},
        "runtime_coordinates": {"phase": "phase0 + 2*pi*f*t", "s_eff_mm": "robot_s - driver_s(t)"},
        "dimensions": {"phase_stations": len(phases), "s_eff_stations": len(stations),
                       "logical_tensor_shape": [len(stations), len(phases)]},
        "resolution": {"dphase_deg": spec["dphase_deg"], "ds_mm": spec["ds_mm"]},
        "ranges": {"phase_deg": [float(phases[0]), float(phases[-1])],
                   "s_eff_mm": [float(stations[0]), float(stations[-1])]},
        "table_bytes": output.stat().st_size,
        "precomputation_wallclock_s": elapsed,
        "config": str(CONFIG_PATH.name),
    }
    (OUT / spec["metadata_path"]).write_text(json.dumps(metadata, indent=2) + "\n", encoding="ascii")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
