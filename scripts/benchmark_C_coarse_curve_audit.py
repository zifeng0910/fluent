"""Fast BOI curve audit using native evaluation and conservative sample coverage."""
import json,math
import gmsh,numpy as np
from scipy.special import comb
from scipy.spatial import cKDTree
from benchmark_C_coarse_common import ROOT,EVID,atomic,stamp

def curve_audit(poles,offset,curve):
    t=np.linspace(0,1,262145)
    p=np.asarray(poles);dc=5*np.diff(p,axis=0);dd=4*np.diff(dc,axis=0)
    def bez(ctrl):
        n=len(ctrl)-1;result=np.zeros((len(t),2))
        for i in range(n+1):result+=comb(n,i)*(t**i*(1-t)**(n-i))[:,None]*ctrl[i]
        return result
    profile=bez(p);d=bez(dc);normal=np.column_stack([-d[:,1],d[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None]
    reference=profile+offset*normal
    lo,hi=gmsh.model.getParametrizationBounds(1,curve);pars=np.linspace(float(lo[0]),float(hi[0]),65537)
    values=np.asarray(gmsh.model.getValue(1,curve,pars)).reshape(-1,3)[:,:2]
    error=max(float(cKDTree(values).query(reference)[0].max()),float(cKDTree(reference).query(values)[0].max()))
    if np.any(dc<0):raise RuntimeError('Monotonic derivative bound requires nonnegative Bernstein derivative controls')
    speed_bound=float(np.linalg.norm(dc,axis=1).max()+offset*np.linalg.norm(dd,axis=1).max()/(dc.sum(axis=1).min()/math.sqrt(2)))
    reference_cover_bound=speed_bound/(2*(len(t)-1))
    deriv=np.asarray(gmsh.model.getDerivative(1,curve,pars)).reshape(-1,3);second=np.asarray(gmsh.model.getSecondDerivative(1,curve,pars)).reshape(-1,3)
    # Native C2 spline is sampled on 65536 parameter intervals. Include a full
    # interval times twice the largest measured second derivative in speed bound.
    step=(float(hi[0])-float(lo[0]))/(len(pars)-1)
    native_speed_bound=float(np.linalg.norm(deriv,axis=1).max()+2*step*np.linalg.norm(second,axis=1).max())
    native_cover_bound=native_speed_bound*step/2
    return {'method':'Native OCC spline evaluation, bidirectional KD distances, Bernstein reference-speed bound and dense native derivative interval allowance','reference_samples':len(t),'native_samples':len(pars),'sampled_bidirectional_distance_mm':error,'reference_cover_bound_mm':reference_cover_bound,'native_cover_allowance_mm':native_cover_bound,'conservative_audited_distance_mm':error+reference_cover_bound+native_cover_bound}

def main():
    geo=json.loads((ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_geometry.json').read_text());p=np.array(geo['head']['poles_mm']);t=np.linspace(0,1,33)
    bez=lambda ctrl:np.array([sum(comb(len(ctrl)-1,i)*x**i*(1-x)**(len(ctrl)-1-i)*ctrl[i] for i in range(len(ctrl))) for x in t])
    profile=bez(p);d=bez(5*np.diff(p,axis=0));normal=np.column_stack([-d[:,1],d[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None];rec={'timestamp':stamp(),'rows':[]}
    gmsh.initialize()
    try:
        for offset in [.16,.26,.46]:
            gmsh.clear();gmsh.model.add('audit');points=[gmsh.model.occ.addPoint(*v,0) for v in profile+offset*normal];c=gmsh.model.occ.addSpline(points);gmsh.model.occ.synchronize()
            row=curve_audit(p,offset,c);row['offset_mm']=offset;rec['rows'].append(row);atomic(EVID/'BOI_curve_preflight.json',rec)
        rec['status']='PASS' if all(r['conservative_audited_distance_mm']<=.0005 for r in rec['rows']) else 'FAIL';atomic(EVID/'BOI_curve_preflight.json',rec)
    finally:gmsh.finalize()

if __name__=='__main__':main()
