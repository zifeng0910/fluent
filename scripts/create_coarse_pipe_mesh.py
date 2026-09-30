"""Create a small open pipe + real-robot proxy fluid mesh for the live smoke test."""
from pathlib import Path
import gmsh

out = Path(__file__).resolve().parents[1] / "live_cases" / "benchmark_A_stationary"
out.mkdir(parents=True, exist_ok=True)
gmsh.initialize(); gmsh.model.add("open_pipe_robot_coarse")
gmsh.option.setNumber("General.Terminal", 1)
L=0.010; R=0.0009
pipe=gmsh.model.occ.addCylinder(-L/2,0,0,L,0,0,R)
# Coarse smoke-test robot envelope: dimensions match audited D~1.2 mm, L~2.8 mm.
robot=gmsh.model.occ.addCylinder(-0.0014,0,0,0.0028,0,0,0.0006)
fluid,_=gmsh.model.occ.cut([(3,pipe)],[(3,robot)])
gmsh.model.occ.synchronize()
gmsh.model.addPhysicalGroup(3,[fluid[0][1]],tag=1); gmsh.model.setPhysicalName(3,1,"fluid")
faces=gmsh.model.getBoundary([(3,fluid[0][1])],oriented=False,recursive=False)
inlet=[]; outlet=[]; wall=[]; robot_faces=[]
for dim,tag in faces:
    cx=gmsh.model.occ.getCenterOfMass(dim,tag)[0]
    if abs(cx+L/2)<1e-7: inlet.append(tag)
    elif abs(cx-L/2)<1e-7: outlet.append(tag)
    else:
        cy,cz=gmsh.model.occ.getCenterOfMass(dim,tag)[1:]
        (robot_faces if abs(cy)<2e-5 and abs(cz)<2e-5 else wall).append(tag)
for tag,ents,name in [(2,robot_faces,"robot_wall"),(3,inlet,"inlet"),(4,outlet,"outlet"),(5,wall,"pipe_wall")]:
    gmsh.model.addPhysicalGroup(2,ents,tag=tag); gmsh.model.setPhysicalName(2,tag,name)
gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 2.5e-5)
gmsh.option.setNumber("Mesh.CharacteristicLengthMax", 1.5e-4)
gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
gmsh.model.mesh.generate(3)
gmsh.write(str(out/"open_pipe_robot_coarse.msh"))
print("nodes",gmsh.model.mesh.getNodes()[0].size,"elements",sum(len(x) for x in gmsh.model.mesh.getElements(3)[1]))
gmsh.finalize()
