"""Facet the frozen FreeCAD STEP through Gmsh OCC for Fluent Meshing import."""
from pathlib import Path
import gmsh

root=Path(__file__).resolve().parents[1]
src=root/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble.step'
out=root/'geometry/robot_L2300_D0815_wallwobble.stl'
out.parent.mkdir(exist_ok=True)
gmsh.initialize(); gmsh.model.add('robot_L2300_D0815_wallwobble')
gmsh.model.occ.importShapes(str(src), highestDimOnly=True); gmsh.model.occ.synchronize()
gmsh.option.setNumber('Mesh.CharacteristicLengthMin', 8.0e-5)
gmsh.option.setNumber('Mesh.CharacteristicLengthMax', 1.2e-4)
gmsh.option.setNumber('Mesh.RandomFactor', 1e-5)
gmsh.option.setNumber('Mesh.Algorithm', 5)
gmsh.model.occ.removeAllDuplicates(); gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2); gmsh.write(str(out)); gmsh.finalize()
print(out)
