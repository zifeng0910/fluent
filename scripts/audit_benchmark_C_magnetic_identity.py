"""Audit the existing magnetic lookup against the current authoritative robot."""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
MAG = ROOT / "magnetic"
IDENTITY = ROOT / "freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_magnetic_identity.json"
MASS = ROOT / "freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_mass_properties.json"
OUTPUT = ROOT / "evidence/benchmark_C_magnetic_identity_audit.json"


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    manifest_path = MAG / "magnetic_lookup_manifest.yaml"
    csv_path = MAG / "magnetic_lookup.csv"
    npz_path = MAG / "magnetic_lookup.npz"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    identity = json.loads(IDENTITY.read_text(encoding="utf-8"))
    mass = json.loads(MASS.read_text(encoding="utf-8"))
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        columns = next(reader)
        first = next(reader)
    with np.load(npz_path, allow_pickle=False) as archive:
        npz_data_shape = list(archive["data"].shape)
        npz_columns = archive["columns"].tolist()
    source = (ROOT / "scripts/magnetic_table_si.py").read_text(encoding="utf-8")
    udf = (ROOT / "fluent_udf/magnetic_lookup_6dof.c").read_text(encoding="utf-8")
    legacy_moment = np.array([-1.061824299255672e-3, 6.72499168508204e-5, -2.25676452654195e-4])
    transverse = mass["principal_mass_moments"]["value_mg_mm2"][:2]
    longitudinal = mass["principal_mass_moments"]["value_mg_mm2"][2]
    report = {
        "campaign": "BENCHMARK_BC_OVERSET_MAGPYLIB_6DOF",
        "status": "FAIL",
        "lookup_disposition": "REGENERATE_REQUIRED; existing table must not be hooked to current 6DOF",
        "operating_frequency_hz": 100.0,
        "legacy_metadata_frequency_hz": manifest["drive_frequency"]["legacy_metadata_hz"],
        "frequency_audit": "SI port phase=36000*time_s proves 100 Hz; 120 Hz metadata is stale",
        "authoritative": {
            "robot": "Robot_L2300_D0815_WallWobble",
            "axis_global": "+X from nose tip to flat tail",
            "mass_kg": mass["mass"]["mg"] * 1e-6,
            "com_m": [v * 1e-3 for v in mass["center_of_mass"]["value"]],
            "inertia_kg_m2_global_xyz": [longitudinal * 1e-12, transverse[0] * 1e-12, transverse[1] * 1e-12],
            "magnetic_moment_Am2": identity["physical_magnetic_moment_target_Am2"],
        },
        "existing_lookup": {
            "backend": manifest["backend"],
            "magpylib_runtime": False,
            "geometry_source": manifest["geometry_source"],
            "robot_mass_source": manifest["robot_mass_source"],
            "pose_reference": manifest["reference_point"],
            "torque_reference": manifest["rp_to_com_torque"],
            "force_unit": manifest["force_unit"],
            "torque_unit": manifest["torque_unit"],
            "position_unit": manifest["position_unit"],
            "angle_unit": manifest["angle_unit"],
            "position_range_m": manifest["position_range"],
            "orientation_range_rad": manifest["orientation_range"],
            "phase_range_deg": manifest["phase_range_deg"],
            "records_manifest": manifest["records"],
            "csv_columns": columns,
            "csv_first_record": [float(x) for x in first],
            "npz_data_shape": npz_data_shape,
            "npz_columns": npz_columns,
            "legacy_moment_vector_Am2": legacy_moment.tolist(),
            "legacy_moment_magnitude_Am2": float(np.linalg.norm(legacy_moment)),
            "sha256": {"manifest": digest(manifest_path), "csv": digest(csv_path), "npz": digest(npz_path)},
        },
        "identity_findings": [
            "Existing lookup references an Abaqus RP/old load model, not the current +X CAD robot pose.",
            "Legacy moment vector magnitude differs from current volume-derived authoritative moment.",
            "External field source provenance and pose transform for the current robot are not established.",
            "Manifest torque reference formula has the opposite sign to the requested T_COM = T_RP + (r_RP-r_COM) x F when r_COM_from_RP means COM minus RP.",
            "Mass mismatch alone would not require magnetic table regeneration; magnetic model identity mismatch does.",
        ],
        "udf_findings": {
            "nearest_neighbor_only": "Nearest-neighbour runtime hook" in udf,
            "orientation_and_phase_interpolation": False,
            "hard_coverage_gate": False,
            "current_com_torque_transform": False,
            "safe_to_compile_and_hook": False,
        },
        "source_confirms_100hz": "phase=(36000*time_s)%360" in source,
        "conversion_gate": {
            "mg_to_kg": 1e-6,
            "mg_mm2_to_kg_m2": 1e-12,
            "mm_to_m": 1e-3,
            "force": "N", "torque": "N m", "angles": "rad", "angular_velocity": "rad/s",
        },
        "coverage_policy": "HARD STOP on any out-of-domain pose, without clamp/extrapolation/nearest substitution",
        "required_next_step": "Build a versioned Magpylib table for the current +X robot and verify field-source identity and RP/COM transform before UDF coupling.",
    }
    if columns != npz_columns or npz_data_shape != [manifest["records"], len(columns)]:
        report["identity_findings"].append("CSV/NPZ schema or record count mismatch.")
    OUTPUT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(OUTPUT)}, indent=2))


if __name__ == "__main__":
    main()
