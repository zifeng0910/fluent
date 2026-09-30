"""Build explicit Benchmark A water domain: cylinder minus authoritative robot."""
from pathlib import Path
import hashlib, json, math, yaml, gmsh

ROOT = Path(__file__).resolve().parents[1]
src = ROOT / "freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble.step"
all_cfg = yaml.safe_load((ROOT / "geometry/tube_fixture.yaml").read_text())
cfg = all_cfg["tube"]
out = ROOT / "geometry/benchmark_A_fluid_domain"; out.mkdir(parents=True, exist_ok=True)
R, L, x0 = cfg["radius_mm"], cfg["length_mm"], cfg["inlet_x_mm"]
gmsh.initialize(); gmsh.model.add("BenchmarkA_FluidDomain")
robot = gmsh.model.occ.importShapes(str(src), highestDimOnly=True)
gmsh.model.occ.synchronize()
cyl = gmsh.model.occ.addCylinder(x0, 0, 0, L, 0, 0, R)
gmsh.model.occ.synchronize()
fluid, _ = gmsh.model.occ.cut([(3, cyl)], robot, removeObject=True, removeTool=True)
gmsh.model.occ.synchronize()
vols = [tag for dim, tag in fluid if dim == 3]
if len(vols) != 1: raise RuntimeError(f"expected one fluid solid, got {fluid}")
v_cyl = math.pi * R * R * L; v_robot = 1.140268022399
v = gmsh.model.occ.getMass(3, vols[0])
gmsh.write(str(out / "BenchmarkA_FluidDomain.step")); gmsh.write(str(out / "BenchmarkA_FluidDomain.brep"))
manifest = {"source_model":"Gmsh OCC boolean using authoritative FreeCAD STEP",
 "authoritative_robot_source":str(src.relative_to(ROOT)).replace("\\","/"),
 "robot_sha256":hashlib.sha256(src.read_bytes()).hexdigest(), "tube_fixture_source":"geometry/tube_fixture.yaml",
 "robot_pose":all_cfg.get("robot_pose"), "cylinder":{"radius_mm":R,"length_mm":L,"xmin_mm":x0,"xmax_mm":cfg["outlet_x_mm"],"axis":"X"},
 "V_cylinder_mm3":v_cyl,"V_robot_mm3":v_robot,"V_fluid_mm3":v,"expected_V_fluid_mm3":v_cyl-v_robot,
 "bbox_mm":gmsh.model.getBoundingBox(3,vols[0]), "solid_count":len(vols),
 "boolean_operation":"water cylinder MINUS fixed robot", "valid_brep":abs(v-(v_cyl-v_robot))<0.01,
 "freecad_version":"unavailable; OCC backend gmsh "+gmsh.__version__,
 "note":"Cylinder represents WATER, not physical tube material. Fixed Benchmark A only."}
(out / "BenchmarkA_FluidDomain_manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
gmsh.finalize(); print(json.dumps(manifest,indent=2))
