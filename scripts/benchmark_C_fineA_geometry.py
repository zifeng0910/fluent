"""Close-fitting normal-offset profile revolved about frozen robot +X axis."""
import json,math
from pathlib import Path
import gmsh,numpy as np
from scipy.special import comb
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'geometry/benchmark_C_fineA';OUT.mkdir(exist_ok=True)
identity=json.loads((ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_geometry.json').read_text())
poles=np.array(identity['head']['poles_mm']);ts=np.linspace(0,1,257)
def bez(p,t):
    n=len(p)-1;return np.array([sum(comb(n,i)*t0**i*(1-t0)**(n-i)*p[i] for i in range(n+1)) for t0 in t])
profile=bez(poles,ts);derivative=bez(5*np.diff(poles,axis=0),ts)
normal=np.column_stack([-derivative[:,1],derivative[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None]
offset=profile+.05*normal
gmsh.initialize()
try:
    gmsh.model.add('C_FINE_A')
    pt=lambda x,r:gmsh.model.occ.addPoint(float(x),float(r),0)
    points=[pt(*v) for v in offset];curve=gmsh.model.occ.addSpline(points)
    cylend=pt(2.3,.4575);center=pt(2.3,.4075);roundend=pt(2.35,.4075);axisend=pt(2.35,0)
    curves=[curve,gmsh.model.occ.addLine(points[-1],cylend),gmsh.model.occ.addCircleArc(cylend,center,roundend),gmsh.model.occ.addLine(roundend,axisend),gmsh.model.occ.addLine(axisend,points[0])]
    face=gmsh.model.occ.addPlaneSurface([gmsh.model.occ.addWire(curves)])
    outer=[v for v in gmsh.model.occ.revolve([(2,face)],0,0,0,1,0,0,2*math.pi) if v[0]==3]
    source=ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble.step'
    robot=gmsh.model.occ.importShapes(str(source));shell,_=gmsh.model.occ.cut(outer,robot,removeTool=True)
    gmsh.model.occ.synchronize()
    if len(shell)!=1:raise RuntimeError('Component Boolean not a single fluid volume')
    boundary=set(gmsh.model.getBoundary(shell,oriented=False))
    leftovers=[v for v in gmsh.model.getEntities(2) if v not in boundary]
    if leftovers:
        gmsh.model.occ.remove(leftovers,recursive=False)
        gmsh.model.occ.synchronize()
    vol=gmsh.model.occ.getMass(*shell[0]);bounds=gmsh.model.occ.getBoundingBox(*shell[0])
    if not (.05<vol<.8):raise RuntimeError(f'Bad shell volume {vol}')
    gmsh.write(str(OUT/'robot_component_fluid.step'))
    rec={'status':'PASS','normal_offset_mm':.05,'axial_extension_mm':.05,'method':'257-point normal offset of authoritative degree-5 Bezier head; exact cylinder offset and rounded tail offset; revolve then subtract unchanged authoritative STEP','robot_source':str(source),'component_volume_mm3':vol,'bbox_mm':bounds,'normal_offset_sample_error_mm':float(np.max(np.abs(np.linalg.norm(offset-profile,axis=1)-.05))),'faces':[{'tag':t,'bbox':gmsh.model.occ.getBoundingBox(d,t),'area':gmsh.model.occ.getMass(d,t)} for d,t in gmsh.model.getEntities(2)]}
    (ROOT/'evidence/benchmark_C_fineA_geometry.json').write_text(json.dumps(rec,indent=2))
    r=.4575;phi=np.linspace(0,2*math.pi,181)
    samples=np.concatenate([np.column_stack([np.full(len(phi),x),r*np.cos(phi),r*np.sin(phi)]) for x in [-.05,.26149477,2.3,2.35]])
    com=np.array([1.2060186937156343,0,0]);axis=np.array([.04129325489475344,-.3684752565834513,-.928720007529695]);orth=np.array([0,-axis[2],axis[1]]);orth/=np.linalg.norm(orth)
    poses=[{'axis':a.tolist(),'theta_rad':float(t)} for a in [axis,orth] for t in [0,.1,.2,.25,.3,.35]]
    swept=np.concatenate([Rotation.from_rotvec(np.array(p['axis'])*p['theta_rad']).apply(samples-com)+com for p in poses])
    lo=swept.min(axis=0)-.225;hi=swept.max(axis=0)+.225
    gmsh.clear();gmsh.model.add('background');gmsh.model.occ.addCylinder(-5,0,0,10,0,0,.9);gmsh.model.occ.synchronize();gmsh.write(str(OUT/'background_water.step'))
    gmsh.clear();gmsh.model.add('refinement');box=gmsh.model.occ.addBox(*lo,*(hi-lo));pipe=gmsh.model.occ.addCylinder(-5,0,0,10,0,0,.9);region,_=gmsh.model.occ.intersect([(3,box)],[(3,pipe)]);gmsh.model.occ.synchronize();volume=sum(gmsh.model.occ.getMass(*v) for v in region);gmsh.write(str(OUT/'overlap_boi.step'))
    (ROOT/'evidence/benchmark_C_background_refinement_region.json').write_text(json.dumps({'bbox_mm':[lo.tolist(),hi.tolist()],'pose_samples':poses,'buffer_mm':.125,'translation_margin_mm':.1,'expected_refined_volume_mm3':volume,'shape':'swept conservative bbox clipped to pipe; not whole pipe','conservative_bbox_note':'Uses outer radius bound at axial samples; contains true close-fitting surface'},indent=2))
finally:gmsh.finalize()
