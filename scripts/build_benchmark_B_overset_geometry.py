"""Build overlapping CAD fluid volumes for the first overset motion gate (mm)."""

from __future__ import annotations

import json
from pathlib import Path

import gmsh


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "geometry" / "benchmark_B_overset"
ROBOT = ROOT / "freecad_parametric_robot" / "variants" / "L2300_D0815_wallwobble" / "Robot_L2300_D0815_WallWobble.step"


def model(name: str) -> None:
    gmsh.clear()
    gmsh.model.add(name)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    gmsh.initialize()
    try:
        model("background_water")
        background = gmsh.model.occ.addCylinder(-5, 0, 0, 10, 0, 0, 0.9)
        gmsh.model.occ.synchronize()
        gmsh.write(str(OUT / "background_water.step"))
        bg_volume = gmsh.model.occ.getMass(3, background)

        model("robot_component")
        # The component fluid follows the rigid robot. Its outer cylindrical
        # surface stays clear of the pipe wall throughout the small B motion.
        outer = gmsh.model.occ.addCylinder(-0.5, 0, 0, 3.3, 0, 0, 0.65)
        robot = gmsh.model.occ.importShapes(str(ROBOT))
        cut, _ = gmsh.model.occ.cut([(3, outer)], robot, removeObject=True, removeTool=False)
        gmsh.model.occ.synchronize()
        volumes = gmsh.model.getEntities(3)
        if len(cut) != 1 or len(volumes) != 2:
            raise RuntimeError(f"Unexpected component Boolean result: cut={cut}, volumes={volumes}")
        # Remove the solid tool. Only the surrounding fluid enters the export.
        gmsh.model.occ.remove(robot, recursive=True)
        gmsh.model.occ.synchronize()
        volumes = gmsh.model.getEntities(3)
        if len(volumes) != 1:
            raise RuntimeError(f"Robot solid was not removed: {volumes}")
        component_volume = gmsh.model.occ.getMass(3, volumes[0][1])
        gmsh.write(str(OUT / "robot_component_fluid.step"))
        result = {
            "status": "PASS", "unit": "mm", "axis": "+X",
            "background": {"name": "background_water", "fluid_volume_mm3": bg_volume,
                           "radius_mm": 0.9, "x_bounds_mm": [-5, 5],
                           "step": str(OUT / "background_water.step")},
            "component": {"name": "robot_component", "fluid_volume_mm3": component_volume,
                          "outer_radius_mm": 0.65, "x_bounds_mm": [-0.5, 2.8],
                          "robot_solid_volume_included": False,
                          "step": str(OUT / "robot_component_fluid.step")},
            "robot_source": str(ROBOT),
            "nominal_outer_overlap_margin_mm": 0.9 - 0.65,
            "requested_translation_mm": 0.10,
            "radial_clearance_to_pipe_mm": 0.9 - 0.4075,
        }
        (OUT / "geometry_manifest.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    main()
