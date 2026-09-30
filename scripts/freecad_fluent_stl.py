"""Run using the FreeCAD bundled Python. BREP remains authoritative; STL coordinates are mm.

Fluent's STL importer ignores mesh-unit conversion, so the derived STL must carry
millimetre-valued coordinates even when the CFD model later uses SI settings.
"""
from pathlib import Path
import json, hashlib, math
import FreeCAD as App
import Part, MeshPart
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble'
manifest=json.loads((SRC/'cad_identity_manifest.json').read_text())
for name,digest in manifest['files'].items():
    assert hashlib.sha256((SRC/name).read_bytes()).hexdigest()==digest, name
s=Part.Shape(); s.read(str(SRC/'Robot_L2300_D0815_WallWobble.brep'))
assert s.isValid() and s.isClosed() and len(s.Solids)==1
m=MeshPart.meshFromShape(Shape=s,LinearDeflection=0.001,AngularDeflection=math.radians(3),Relative=False)
points,faces=m.Topology
edges={}; volume=0.; area=0.; maxdev=0.
for face in faces:
    a,b,c=[points[i] for i in face]
    volume+=a.dot(b.cross(c))/6
    area+=(b-a).cross(c-a).Length/2
    for i,j in zip(face,face[1:]+face[:1]):
        key=tuple(sorted((i,j))); edges.setdefault(key,[]).append(1 if i<j else -1)
    for p in ((a+b+c)/3,(a+b)/2,(b+c)/2,(c+a)/2):
        maxdev=max(maxdev,Part.Vertex(p).distToShape(s)[0])
b=m.BoundBox; sb=s.BoundBox
audit={'authoritative_sha256':manifest['files'],'source':'BREP','triangle_count':len(faces),
 'linear_deflection_mm':0.001,'angular_deflection_rad':math.radians(3),
 'bbox_mm':[b.XLength,b.YLength,b.ZLength],
 'bbox_difference_mm':[b.XLength-sb.XLength,b.YLength-sb.YLength,b.ZLength-sb.ZLength],
 'surface_area_mm2':area,'area_relative_difference':area/s.Area-1,
 'signed_volume_mm3':volume,'volume_relative_difference':volume/s.Volume-1,
 'watertight':all(len(v)==2 for v in edges.values()),
 'manifold':all(len(v)==2 for v in edges.values()),
 'consistent_outward_normals':volume>0 and all(sum(v)==0 for v in edges.values()),
 'sampled_max_surface_deviation_mm':maxdev,'deviation_note':'facet centroid and edge midpoint samples; not a certified Hausdorff bound',
 'stl_unit':'mm','freecad_version':App.Version()}
audit['status']='PASS' if (audit['watertight'] and audit['consistent_outward_normals'] and abs(volume/s.Volume-1)<0.01 and maxdev<=0.005 and max(abs(v) for v in audit['bbox_difference_mm'])<0.005) else 'FAIL'
assert audit['status']=='PASS',audit
out=ROOT/'geometry/Robot_L2300_D0815_WallWobble_fluent.stl';out.parent.mkdir(exist_ok=True);m.write(str(out))
audit['stl_sha256']=hashlib.sha256(out.read_bytes()).hexdigest()
(ROOT/'evidence/robot_stl_geometry_audit.json').write_text(json.dumps(audit,indent=2),encoding='utf-8')
print(json.dumps(audit,indent=2))
