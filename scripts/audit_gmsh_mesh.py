"""Audit Gmsh topology before any Fluent import attempt."""
from pathlib import Path
import json, sys, hashlib
import numpy as np
import meshio

def audit(path: Path):
    m=meshio.read(path)
    points=np.asarray(m.points)[:,:3]
    cells={c.type: np.asarray(c.data) for c in m.cells}
    tet=cells.get('tetra', np.empty((0,4),dtype=int)); tri=cells.get('triangle',np.empty((0,3),dtype=int))
    a=points[tet[:,1]]-points[tet[:,0]]; b=points[tet[:,2]]-points[tet[:,0]]; c=points[tet[:,3]]-points[tet[:,0]]
    vols=np.abs(np.einsum('ij,ij->i',np.cross(a,b),c))/6 if len(tet) else np.array([])
    faces={}
    for i,t in enumerate(tet):
        for f in [(t[0],t[1],t[2]),(t[0],t[1],t[3]),(t[0],t[2],t[3]),(t[1],t[2],t[3])]: faces.setdefault(tuple(sorted(f)),[]).append(i)
    boundary={k:v for k,v in faces.items() if len(v)==1}
    tri_set={tuple(sorted(f)) for f in tri}
    isolated=tri_set-set(boundary)
    phys={}
    for name,arr in (m.cell_sets or {}).items(): phys[name]={k:int(len(v)) for k,v in arr.items()}
    physical_ids={k:{str(i):int(np.sum(vals==i)) for i in np.unique(vals)} for k,vals in m.cell_data_dict.get('gmsh:physical',{}).items()}
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'format':'Gmsh via meshio','points':int(len(points)),'cell_types':{k:int(len(v)) for k,v in cells.items()},'tetra_positive_volume':bool(len(vols) and np.all(vols>0)),'tetra_min_volume':float(vols.min()) if len(vols) else None,'tetra_max_volume':float(vols.max()) if len(vols) else None,'duplicate_points':int(len(points)-len(np.unique(points,axis=0))),'boundary_faces_from_tetra':len(boundary),'triangle_faces':len(tri),'triangle_faces_not_owned_by_tetra':len(isolated),'closed_fluid_boundary':bool(len(tri_set)==len(boundary) and not isolated),'physical_groups':physical_ids,'mesh_field_data':{k:list(map(int,v[:2])) for k,v in m.field_data.items()}}

if __name__=='__main__':
    paths=[Path(x) for x in sys.argv[1:]] or [Path('live_cases/benchmark_A_stationary/open_pipe_robot_coarse.msh'),Path('live_cases/benchmark_A_stationary/box.msh')]
    out=[audit(p) for p in paths]
    Path('evidence').mkdir(exist_ok=True)
    Path('evidence/gmsh_mesh_audit.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps(out,indent=2))
