"""Static C component family audit in one solver, with frozen B background."""
import csv,json,hashlib,sys
from pathlib import Path
import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from ansys.fluent.core.fields.field_data_interfaces import SurfaceDataType,SurfaceFieldDataRequest

COM=np.array([.0012060186937156343,0,0])

def run(solver,context):
    root=context['root'];sys.path.insert(0,str(root/'scripts'))
    from benchmark_C_mesh_metrics import tetra_metrics,local_size
    s=solver.settings;out=root/'evidence/benchmark_C_overset_pose_sweep.json';csvout=out.with_suffix('.csv')
    transcript_start=len(context['messages'])
    bg=tetra_metrics(root/'live_cases/benchmark_B_overset/benchmark_B_static_initialized.cas.h5','background_water')
    hist=list(csv.DictReader((root/'evidence/benchmark_C_analytic_6dof_history.csv').open()))
    failure=[r for r in hist if r['mode']=='FREE_6DOF'][-1]
    q=np.array([float(failure[f'q{i}']) for i in range(4)])
    failure_axis=q[1:]/np.linalg.norm(q[1:])
    # Orthogonal direction in the pitch/yaw plane.
    orthogonal=np.array([0.,-failure_axis[2],failure_axis[1]]);orthogonal/=np.linalg.norm(orthogonal)
    report={'status':'RUNNING','physics_frozen':True,'no_flow_iterations':True,'no_magnetic_load_hooks':True,
      'background_source':str(root/'live_cases/benchmark_B_overset/benchmark_B_overset_boundary_SI.cas.h5'),
      'background_cells':bg['count'],'rotation_angle_api_unit':'radian (verified by transformed vertices)',
      'overlap_method':'minimum shell/end thickness divided by largest nearest-cell actual tetra edge; conservative geometric estimate, not a direct donor stencil count',
      'safety_margin_rad':.05,'rows':[],'selected_candidate':None,
      'frozen_udf_sha256':hashlib.sha256((root/'fluent_udf/l2300_abaqus100hz_6dof.c').read_bytes()).hexdigest()}
    def save():
        out.write_text(json.dumps(report,indent=2,default=str))
        if report['rows']:
            with csvout.open('w',newline='',encoding='utf-8') as f:
                w=csv.DictWriter(f,fieldnames=list(report['rows'][0]));w.writeheader();w.writerows(report['rows'])
    def surface(name):
        a=solver.fields.field_data.get_field_data(SurfaceFieldDataRequest(surfaces=[name],data_types=[SurfaceDataType.Vertices,SurfaceDataType.FacesConnectivity]))[name]
        faces=np.concatenate([np.r_[len(f),f] for f in a.connectivity]).astype(np.int64)
        return pv.PolyData(np.asarray(a.vertices),faces)
    save()
    for cid,shell in zip('ABCD',[.05,.0625,.075,.1]):
        comp=tetra_metrics(root/f'live_cases/benchmark_C_candidates/component_{cid}_SI.cas.h5','robot_component_fluid')
        for axis_name,axis in [('failure_trajectory',failure_axis),('orthogonal_pitch_yaw',orthogonal)]:
            s.file.read_case(file_name=report['background_source'])
            pipe0=surface('pipe_wall'); bg_before=bg['count']
            inside_sign=float(np.sign(pv.PolyData(COM[None,:]).compute_implicit_distance(pipe0)['implicit_distance'][0]))
            s.mesh.modify_zones.replace_zone(file_name=str(root/f'live_cases/benchmark_C_candidates/component_{cid}_SI.cas'),zone_1_name='robot_component_fluid',zone_2_name='robot_component_fluid',interpolate=False)
            if not np.array_equal(pipe0.points,surface('pipe_wall').points):raise RuntimeError('Frozen pipe wall coordinates changed')
            s.setup.boundary_conditions.set_zone_type(zone_list=['inlet'],new_type='pressure-inlet')
            s.setup.boundary_conditions.set_zone_type(zone_list=['outlet'],new_type='pressure-outlet')
            s.setup.models.viscous.model='laminar'
            mat=s.setup.materials.fluid
            if 'water' not in mat.keys():mat.create(name='water')
            mat['water'].density.set_state({'option':'constant','value':998.2})
            mat['water'].viscosity.set_state({'option':'constant','value':.001003})
            for z in ['background_water','robot_component_fluid']:s.setup.cell_zone_conditions.fluid[z].general.material='water'
            s.setup.user_defined.load(udf_library_name=str(root/'fluent_udf/libbenchmark_C_pose'))
            robot0=surface('robot_wall').points.copy();env0=surface('overset_component').points.copy()
            prev=0.
            for theta in np.arange(8)*.05:
                if theta:
                    s.mesh.modify_zones.rotate_zone(zone_names=['robot_component_fluid'],rotation_angle=float(theta-prev),origin=COM.tolist(),axis=axis.tolist())
                prev=theta;robot=surface('robot_wall');env=surface('overset_component')
                R=Rotation.from_rotvec(axis*theta)
                predicted=R.apply(robot0-COM)+COM
                err=float(np.max(cKDTree(robot.points).query(predicted)[0]))
                if err>1e-8:raise RuntimeError(f'Native rotation does not match radians: {err}')
                rc=float(np.min(robot.compute_implicit_distance(pipe0)['implicit_distance']*inside_sign)*1000)
                ec=float(np.min(env.compute_implicit_distance(pipe0)['implicit_distance']*inside_sign)*1000)
                s.solution.initialization.hybrid_initialize()
                s.setup.user_defined.execute_on_demand(lib_name='benchmark_C_pose_statistics::libbenchmark_C_pose')
                stats=json.loads((root/'evidence/benchmark_C_pose_live.json').read_text())
                if stats['zones'][0]['total']!=bg_before:raise RuntimeError('Frozen background cell count changed')
                hbg=local_size(bg,np.vstack([robot.points,env.points]))*1000
                hc=local_size(comp,np.vstack([robot0,env0]))*1000
                layers=min(shell,.1)/max(hbg,hc)
                get=lambda k:sum(z[k] for z in stats['zones'])
                reasons=[]
                if rc<.1:reasons.append('PHYSICAL_CLEARANCE')
                if ec<=0:reasons.append('ENVELOPE_OUTSIDE_PIPE')
                if get('orphan'):reasons.append('ORPHAN')
                if get('missing'):reasons.append('MISSING_DONOR')
                if layers<4:reasons.append('OVERLAP_BELOW_4')
                minv=min(z['min_volume_m3'] for z in stats['zones'])
                if minv<=0:reasons.append('NONPOSITIVE_VOLUME')
                row={'candidate_id':cid,'shell_radial_mm':shell,'axial_extension_mm':.1,'rotation_axis':axis_name,'axis_xyz':json.dumps(axis.tolist()),'theta_rad':float(theta),
                  'robot_wall_clearance_mm':rc,'overset_wall_clearance_mm':ec,'receptors':get('receptor'),'donors':get('donor'),'receptors_with_donors':get('receptor')-get('missing'),
                  'missing_donors':get('missing'),'orphans':get('orphan'),'dead_cells':get('dead'),
                  'minimum_geometric_overlap_thickness_mm':min(shell,.1),'background_local_cell_size_mm':hbg,'component_local_cell_size_mm':hc,'estimated_overlap_layers':layers,
                  'minimum_donor_receptor_size_ratio':'NOT_EXPOSED','minimum_volume_m3':minv,'pose_transform_error_m':err,
                  'status':'FAIL:'+','.join(reasons) if reasons else 'PASS'}
                report['rows'].append(row);save()
                (root/f'evidence/benchmark_C_pose_{cid}_{axis_name}_{theta:.2f}.json').write_text(json.dumps(stats,indent=2))
                if rc<.1:break
    passes=[cid for cid in 'ABCD' if all(r['status']=='PASS' for r in report['rows'] if r['candidate_id']==cid)]
    report['selected_candidate']=passes[-1] if passes else None
    report['status']='PASS' if passes else 'FAIL_NO_ROBUST_CANDIDATE'
    report['theta_allowed_rad']=None if not passes else .30
    report['remaining_blocker']='No candidate meets connectivity and >=4 conservative effective layers with frozen B background.' if not passes else None
    report['original_failure_state']=failure
    report['frozen_udf_unchanged']=report['frozen_udf_sha256']==hashlib.sha256((root/'fluent_udf/l2300_abaqus100hz_6dof.c').read_bytes()).hexdigest()
    save()
    (root/'evidence/benchmark_C_pose_sweep_solver_transcript.txt').write_text(''.join(context['messages'][transcript_start:]),encoding='utf-8')
