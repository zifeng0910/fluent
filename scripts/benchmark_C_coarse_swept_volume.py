"""Reproducible numerical union volume estimate clipped to the physical pipe."""
import json,math
import numpy as np
from scipy.special import comb
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_common import *

def main():
    rng=np.random.default_rng(20261002);n=500000;lo=np.array([-.5,-.9,-.9]);hi=np.array([2.8,.9,.9]);points=rng.uniform(lo,hi,(n,3));pipe=np.hypot(points[:,1],points[:,2])<=.9
    geo=read(ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_geometry.json');p=np.array(geo['head']['poles_mm']);t=np.linspace(0,1,4097)
    bez=lambda c:sum(comb(len(c)-1,i)*(t**i*(1-t)**(len(c)-1-i))[:,None]*c[i] for i in range(len(c)))
    profile=bez(p);d=bez(5*np.diff(p,axis=0));normal=np.column_stack([-d[:,1],d[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None];head=profile+.16*normal
    if np.any(np.diff(head[:,0])<0):raise RuntimeError('Nonmonotone head offset')
    mask=np.zeros(n,dtype=bool);poses=read(EVID/'geometry.json')['parts'];com=np.array(COM)*1000
    for pose in [p for p in poses if p['group']=='critical']:
        local=(points-com)@Rotation.from_rotvec(np.array(pose['axis'])*pose['theta_rad']).as_matrix()+com
        x=local[:,0];radius=np.interp(x,head[:,0],head[:,1],left=-1,right=.5675)
        tail=x>2.3;radius[tail]=.4075+np.sqrt(np.maximum(0,.16**2-(x[tail]-2.3)**2));inside=(x<=2.46)&(x>=-.16)&(np.hypot(local[:,1],local[:,2])<=radius);mask|=inside
    mask&=pipe;fraction=mask.mean();volume=float(np.prod(hi-lo)*fraction);sigma=float(np.prod(hi-lo)*math.sqrt(fraction*(1-fraction)/n))
    atomic(EVID/'swept_volume_estimate.json',{'timestamp':stamp(),'method':'Seeded Monte Carlo of union membership in offset CAD profile bodies, clipped to unchanged radius 0.9 mm pipe; sizing pieces overlap, so individual volumes are not summed.','samples':n,'seed':20261002,'critical_union_volume_estimate_mm3':volume,'sampling_95pct_interval_mm3':[volume-1.96*sigma,volume+1.96*sigma],'old_fine_BOI_volume_mm3':7.680493078609159,'critical_volume_reduction_estimate_percent':100*(1-volume/7.680493078609159),'BOI_curve_approximation_bound_evidence':'BOI_curve_preflight.json','is_actual_cell_count':False})

if __name__=='__main__':main()
