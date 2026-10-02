"""Closed swept BOI pieces; independent controls form the sizing-field union."""
import json,math
import gmsh,numpy as np
from scipy.special import comb
from benchmark_C_coarse_curve_audit import curve_audit
from benchmark_C_coarse_common import ROOT,EVID,atomic,stamp,COM,AXIS,ORTH
OUT=ROOT/'geometry/benchmark_C_coarse_A'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    geo=json.loads((ROOT/'freecad_parametric_robot/variants/L2300_D0815_wallwobble/Robot_L2300_D0815_WallWobble_geometry.json').read_text());p=np.array(geo['head']['poles_mm']);t=np.linspace(0,1,33)
    bez=lambda c:np.array([sum(comb(len(c)-1,i)*x**i*(1-x)**(len(c)-1-i)*c[i] for i in range(len(c))) for x in t])
    profile=bez(p);d=bez(5*np.diff(p,axis=0));normal=np.column_stack([-d[:,1],d[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None]
    poses=[(AXIS,0)]+[(a,float(theta)) for a in [AXIS,ORTH] for theta in np.linspace(.025,.35,14)]
    rec={'status':'RUNNING','timestamp':stamp(),'region_type':'Independent closed offset-body BOI controls; pointwise minimum sizing forms union without CAD boolean fuse','component_offset_mm':.05,'critical_safety_inflation_mm':.11,'translation_domain_radius_mm':.005,'theta_sample_spacing_rad':.025,'theta_bounds_rad':[0,.35],'pose_axes':[AXIS,ORTH],'parts':[],'regions':[],'far_spacing_mm':.15}
    atomic(EVID/'geometry.json',rec);gmsh.initialize()
    try:
        for group,extra,h in [('critical',.11,.020),('buffer',.21,.040),('transition',.41,.080)]:
            offset=.05+extra;check=None
            for i,(axis,theta) in enumerate(poses):
                name=f'{group}_boi_{i:02d}';gmsh.clear();gmsh.model.add(name)
                pt=lambda x,r:gmsh.model.occ.addPoint(float(x),float(r),0)
                points=[pt(*v) for v in profile+offset*normal];curve=gmsh.model.occ.addSpline(points)
                if check is None:
                    gmsh.model.occ.synchronize();check=curve_audit(p,offset,curve)
                    if check['conservative_audited_distance_mm']>.0005:raise RuntimeError('BOI approximation audit failed')
                end=pt(2.3,.4075+offset);center=pt(2.3,.4075);rnd=pt(2.3+offset,.4075);axend=pt(2.3+offset,0)
                curves=[curve,gmsh.model.occ.addLine(points[-1],end),gmsh.model.occ.addCircleArc(end,center,rnd),gmsh.model.occ.addLine(rnd,axend),gmsh.model.occ.addLine(axend,points[0])]
                face=gmsh.model.occ.addPlaneSurface([gmsh.model.occ.addWire(curves)])
                volumes=[v for v in gmsh.model.occ.revolve([(2,face)],0,0,0,1,0,0,2*math.pi) if v[0]==3]
                if len(volumes)!=1:raise RuntimeError('BOI piece must have one volume')
                if theta:gmsh.model.occ.rotate(volumes,*(np.array(COM)*1000),*axis,theta)
                gmsh.model.occ.synchronize();boundary=set(gmsh.model.getBoundary(volumes,oriented=False));left=[v for v in gmsh.model.getEntities(2) if v not in boundary]
                if left:gmsh.model.occ.remove(left,recursive=False);gmsh.model.occ.synchronize()
                gmsh.write(str(OUT/(name+'.step')))
                rec['parts'].append({'name':name,'group':group,'spacing_mm':h,'axis':axis,'theta_rad':theta,'volume_mm3':float(gmsh.model.occ.getMass(*volumes[0]))})
                atomic(EVID/'geometry.json',rec)
            rec['regions'].append({'name':group,'spacing_mm':h,'piece_count':29,'curve_audit':check});atomic(EVID/'geometry.json',rec)
        gmsh.clear();gmsh.model.add('background_water');gmsh.model.occ.addCylinder(-5,0,0,10,0,0,.9);gmsh.model.occ.synchronize();gmsh.write(str(OUT/'background_water.step'))
        radius=math.hypot(max(COM[0]*1000+.05,2.35-COM[0]*1000),.4575);bound=radius*2*math.sin(.025/4)
        rec.update(status='GENERATED',timestamp=stamp(),continuous_rotation_sampling_displacement_bound_mm=bound,minimum_safety_after_sampling_and_translation_mm=.11-bound-.005-max(r['curve_audit']['conservative_audited_distance_mm'] for r in rec['regions']),old_fine_BOI_volume_mm3=7.680493078609159,
            outside_pipe_note='BOI pieces can extend outside the pipe; only background_water is volume meshed, all BOI parts are removed before export. No extra fluid cells are added outside the pipe.')
        atomic(EVID/'geometry.json',rec)
    finally:gmsh.finalize()

if __name__=='__main__':main()
