"""Component-only CAD generation in the established gmsh virtual environment."""
import sys
from pathlib import Path
import gmsh
ROOT=Path(__file__).resolve().parents[1]
cid,shell=sys.argv[1],float(sys.argv[2])
gmsh.initialize()
try:
    gmsh.model.add(cid)
    outer=gmsh.model.occ.addCylinder(-.1,0,0,2.5,0,0,.4075+shell)
    robot=gmsh.model.occ.importShapes(str(ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble.step'))
    cut,_=gmsh.model.occ.cut([(3,outer)],robot,removeTool=True)
    gmsh.model.occ.synchronize()
    if len(cut)!=1: raise RuntimeError('Non-single component volume')
    gmsh.write(str(ROOT/f'geometry/benchmark_C_candidates/component_{cid}.step'))
finally: gmsh.finalize()
