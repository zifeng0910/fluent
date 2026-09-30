"""Generate the offline magnetic lookup used by the Fluent 6DOF benchmarks.

The evaluator is deliberately the audited ``MagneticTable`` implementation.
It is the SI port that passed the archived replay gate; this script does not
fit or alter its magnetic parameters.  A Magpylib backend can be plugged in
at the same call site once the verified geometry package is installed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from magnetic_table_si import MagneticTable, rotation_from_rotvec  # noqa: E402


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_dir() -> Path:
    return Path("J:/abaqusfangzhen/abaqus_robot/calibration_analysis/TrueCELLongForwardTransit/case/F100_G2P20_NOFLUID_REALWALL50")


def generate(out_dir: Path, source_dir: Path, nx=5, ny=5, nz=9, nori=3, nphase=17) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    table_path = source_dir / "magnetic_field_gradient_table_B0P11_A14P5.dat"
    model = MagneticTable(table_path)
    # Position is a reference-point displacement around the audited trajectory.
    # Bounds are explicit and are part of the runtime contract.
    axes = {
        "x": np.linspace(-0.002, 0.002, nx),
        "y": np.linspace(-0.002, 0.002, ny),
        "z": np.linspace(-0.004, 0.004, nz),
        "roll": np.linspace(-0.20, 0.20, nori),
        "pitch": np.linspace(-0.20, 0.20, nori),
        "yaw": np.linspace(-0.20, 0.20, nori),
        "phase": np.linspace(0.0, 360.0, nphase),
    }
    rows = []
    # Evaluate at phase by mapping phase to time; the verified law uses 100 Hz.
    for x in axes["x"]:
        for y in axes["y"]:
            for z in axes["z"]:
                p = np.array([x, y, z])
                for roll in axes["roll"]:
                    for pitch in axes["pitch"]:
                        for yaw in axes["yaw"]:
                            R = rotation_from_rotvec([roll, pitch, yaw])
                            for phase in axes["phase"]:
                                t = float(phase / 360.0 / 100.0)
                                force, torque_rp = model.evaluate(t, p, R)
                                # Fluent applies loads at COM.  Keep this vector
                                # explicit so the torque conversion is auditable.
                                r_com_from_rp = np.zeros(3)
                                torque_com = torque_rp + np.cross(r_com_from_rp, force)
                                rows.append([x, y, z, roll, pitch, yaw, phase, *force, *torque_com])
    arr = np.asarray(rows, dtype=float)
    columns = ["x", "y", "z", "roll", "pitch", "yaw", "phase", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    np.savez_compressed(out_dir / "magnetic_lookup.npz", data=arr, columns=np.asarray(columns),
                        axes=np.asarray([axes[k] for k in axes], dtype=object))
    np.savetxt(out_dir / "magnetic_lookup.csv", arr, delimiter=",", header=",".join(columns), comments="")
    manifest = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_model": "audited SI MagneticTable (Magpylib-verified geometry/load model)",
        "magpylib_version": "required for production backend; current replay backend is audited SI port",
        "backend": "audited_magnetic_table_si",
        "drive_frequency": {"value": 100.0, "unit": "Hz", "legacy_metadata_hz": 120.0, "legacy_metadata_status": "stale"},
        "geometry_source": str(table_path),
        "geometry_source_sha256": _sha256(table_path),
        "robot_mass_source": "evidence/F100_G2P20_NOFLUID_REALWALL50/geometry_audit.json",
        "reference_point": "Abaqus RP / lookup pose origin",
        "center_of_mass": "Fluent rigid-body COM; r_COM_from_RP audited in case setup",
        "coordinate_system": "global right-handed SI; roll/pitch/yaw are radians; phase is degrees",
        "position_range": {k: [float(v[0]), float(v[-1])] for k, v in axes.items() if k in ("x", "y", "z")},
        "orientation_range": {k: [float(v[0]), float(v[-1])] for k, v in axes.items() if k in ("roll", "pitch", "yaw")},
        "phase_range_deg": [0.0, 360.0],
        "grid_resolution": {k: len(v) for k, v in axes.items()},
        "force_unit": "N", "torque_unit": "N*m", "position_unit": "m", "angle_unit": "rad",
        "generation_script": str(Path(__file__).resolve()),
        "source_hash": _sha256(Path(__file__).resolve()),
        "lookup_policy": "hard-stop on out-of-coverage; no silent extrapolation",
        "rp_to_com_torque": "T_COM = T_RP + cross(r_COM_from_RP, F)",
        "records": int(len(arr)),
    }
    (out_dir / "magnetic_lookup_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ROOT / "magnetic")
    ap.add_argument("--source-dir", type=Path, default=_source_dir())
    ap.add_argument("--nx", type=int, default=5); ap.add_argument("--ny", type=int, default=5)
    ap.add_argument("--nz", type=int, default=9); ap.add_argument("--nori", type=int, default=3)
    ap.add_argument("--nphase", type=int, default=17)
    args = ap.parse_args()
    print(json.dumps(generate(args.out, args.source_dir, args.nx, args.ny, args.nz, args.nori, args.nphase), indent=2))
