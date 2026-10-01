"""Count actual tetrahedra crossed by sampled physical wall normal rays.

Reads CFF faces in chunks and retains only cells close to audit rays.
No thickness / nominal-size estimate is used as a layer count.
"""
import json,math
from pathlib import Path
import numpy as np,h5py
from scipy.spatial import cKDTree
from scipy.special import comb
ROOT=Path(__file__).resolve().parents[1]

def rays():
    geo=json.loads((ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_geometry.json').read_text())
    p=np.array(geo['head']['poles_mm']);out=[]
    def add(label,xr,n,phi):
        x,r=xr;nx,nr=n;u=np.array([x,r*math.cos(phi),r*math.sin(phi)]);v=np.array([nx,nr*math.cos(phi),nr*math.sin(phi)])
        out.append({'label':label,'start_mm':u.tolist(),'normal':v.tolist(),'length_mm':.05})
    for t in [.02,.1,.25,.5,.75,.95]:
        v=sum(comb(5,i)*t**i*(1-t)**(5-i)*p[i] for i in range(6));d=sum(comb(4,i)*t**i*(1-t)**(4-i)*5*(p[i+1]-p[i]) for i in range(5))
        n=np.array([-d[1],d[0]]);n/=np.linalg.norm(n)
        for phi in np.arange(8)*math.pi/4:add('head',v,n,phi)
    for x in [.3,.6,1.2,1.8,2.25]:
        for phi in np.arange(8)*math.pi/4:add('cylinder',[x,.4075],[0,1],phi)
    for r in [0,.1,.25,.38]:
        for phi in np.arange(8)*math.pi/4:add('tail',[2.3,r],[1,0],phi)
    return out

def audit(path,zone,raydefs,scale=1):
    samples=np.array([np.array(r['start_mm'])+np.linspace(.00005,.04995,501)[:,None]*r['normal'] for r in raydefs])
    with h5py.File(path) as f:
        m=f['meshes/1'];top=m['cells/zoneTopology'];names=top['name'][0].decode().split(';');k=names.index(zone);lo,hi=int(top['minId'][k]),int(top['maxId'][k])
        coords=np.concatenate([d[:] for d in m['nodes/coords'].values()])*scale
        near=np.zeros(len(coords),dtype=bool);ntree=cKDTree(coords)
        for ids in ntree.query_ball_point(samples[:,::100,:].reshape(-1,3),.035):near[ids]=True
        del ntree
        for fg in m['faces/nodes'].values():
            if not np.all(fg['nnodes'][:]==3):raise ValueError('Non tetra mesh')
        selected=set();vertices={}
        for phase in [0,1]:
            for side in ['c0','c1']:
                for key,dataset in m['faces/'+side].items():
                    fg=m['faces/nodes/'+key]
                    for start in range(0,len(dataset),200000):
                        cells=dataset[start:start+200000].astype(np.int64)
                        fn=fg['nodes'][start*3:(start+len(cells))*3].reshape(-1,3).astype(np.int64)-1
                        mask=(cells>=lo)&(cells<=hi)
                        if phase==0:
                            mask&=near[fn].any(axis=1);selected.update(cells[mask].tolist())
                        else:
                            mask&=np.isin(cells,list(selected))
                            for c,n in zip(cells[mask],fn[mask]):vertices.setdefault(int(c),set()).update(n.tolist())
        ids=np.array(sorted(vertices));tet=np.array([coords[sorted(vertices[c])] for c in ids]);del coords
    if tet.shape[1:]!=(4,3):raise ValueError('Invalid tetra candidate topology')
    centers=tet.mean(axis=1);tree=cKDTree(centers);inv=np.linalg.inv(np.stack([tet[:,i]-tet[:,0] for i in [1,2,3]],axis=2));edge=np.max(np.stack([np.linalg.norm(tet[:,i]-tet[:,j],axis=1) for i in range(4) for j in range(i)]),axis=0)
    rows=[]
    for ray,points in zip(raydefs,samples):
        _,idx=tree.query(points,k=min(32,len(tet)));b=np.einsum('...ij,...j->...i',inv[idx],points[:,None,:]-tet[idx,0]);inside=(b>=-1e-8).all(axis=2)&(b.sum(axis=2)<=1+1e-8)
        covered=inside.any(axis=1);chosen=idx[np.arange(len(points)),np.argmax(inside,axis=1)][covered]
        good=np.flatnonzero(covered)
        first,last=(int(good[0]),int(good[-1])) if len(good) else (0,500)
        gaps=int((~covered[first:last+1]).sum())
        rows.append({**ray,'crossed_cells':len(np.unique(chosen)),'uncovered_sample_count':int((~covered).sum()),'uncovered_interior_samples':gaps,'first_covered_distance_mm':float(.00005+first*.0499/500),'last_covered_distance_mm':float(.00005+last*.0499/500),'actual_covered_span_mm':float((last-first)*.0499/500),'max_crossed_cell_edge_mm':float(edge[chosen].max()) if len(chosen) else None})
    return {'zone':zone,'method':'501 samples along each 0.05 mm CAD-normal ray; count distinct tetrahedra in continuous actual mesh intersection; retain endpoint tessellation deviations explicitly','ray_count':len(rows),'minimum_crossed_cells':min(r['crossed_cells'] for r in rows),'uncovered_samples':sum(r['uncovered_sample_count'] for r in rows),'uncovered_interior_samples':sum(r['uncovered_interior_samples'] for r in rows),'minimum_actual_covered_span_mm':min(r['actual_covered_span_mm'] for r in rows),'candidate_cells':len(tet),'rows':rows}

if __name__=='__main__':
    defs=rays();path=ROOT/'live_cases/benchmark_C_fineA/bg020/benchmark_C_fineA.cas.h5'
    rec={'component':audit(path,'robot_component_fluid',defs),'background':audit(path,'background_water',defs)}
    rec['status']='PASS' if all(v['minimum_crossed_cells']>=4 and v['uncovered_interior_samples']==0 and v['minimum_actual_covered_span_mm']>=.048 for v in rec.values() if isinstance(v,dict)) else 'FAIL'
    (ROOT/'evidence/benchmark_C_fineA_actual_layers.json').write_text(json.dumps(rec,indent=2))
