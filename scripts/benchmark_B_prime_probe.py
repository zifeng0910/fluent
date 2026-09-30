"""Inspect the two Route D CAD volumes inside one Prime session."""

import json
from pathlib import Path

import ansys.meshing.prime as prime
from ansys.meshing.prime import lucid


ROOT = Path(__file__).resolve().parents[1]
GEOM = ROOT / "geometry" / "benchmark_B_overset"
EVIDENCE = ROOT / "evidence" / "benchmark_B_prime_topology_probe.json"


def main():
    report = {"status": "RUNNING", "prime_version": prime.__version__, "imports": []}
    EVIDENCE.write_text(json.dumps(report, indent=2), encoding="utf-8")
    with prime.launch_prime(timeout=120) as session:
        model = session.model
        io = prime.FileIO(model)
        for filename in ["background_water.step", "robot_component_fluid.step"]:
            imported = io.import_cad(str(GEOM / filename), params=prime.ImportCadParams(model=model, append=True))
            model._sync_up_model()
            report["imports"].append({"file": filename, "result": str(imported),
                                      "parts": [p.name for p in model.parts]})
            EVIDENCE.write_text(json.dumps(report, indent=2), encoding="utf-8")
        details = []
        for part in model.parts:
            faces = list(part.get_topo_faces())
            details.append({"name": part.name, "volume_count": len(list(part.get_topo_volumes())),
                            "face_count": len(faces), "faces": [str(f) for f in faces],
                            "summary": str(part.get_summary(prime.PartSummaryParams(model)))})
        report["parts"] = details
        report["status"] = "PASS" if len(details) == 2 and all(d["volume_count"] == 1 for d in details) else "FAIL"
    EVIDENCE.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
