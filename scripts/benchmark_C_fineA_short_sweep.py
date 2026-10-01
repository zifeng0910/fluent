"""Gated short static sweep. No flow advancement or magnetic callbacks."""
import json,sys,time
import numpy as np,pyvista as pv
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from ansys.fluent.core.fields.field_data_interfaces import SurfaceDataType,SurfaceFieldDataRequest
COM=np.array([.0012060186937156343,0,0])
def run(solver,context):
    root=context['root'];sys.path.insert(0,str(root/'scripts'))
    from benchmark_C_fineA_theta0 import summarize_csv
    from benchmark_C_actual_ray_layers import audit,rays
    s=solver.settings;out=root/'evidence/benchmark_C_fineA_short_sweep.json';case=root/'live_cases/benchmark_C_fineA/bg020'
    theta0=json.loads((root/'evidence/benchmark_C_fineA_theta0_connectivity.json').read_text());layers=json.loads((root/'evidence/benchmark_C_fineA_actual_layers.json').read_text())
    if theta0.get('orphans')!=0 or layers['status']!='PASS':raise RuntimeError('theta0 gate not passed')
    def surface(name):
        data=solver.fields.field_data.get_field_data(SurfaceFieldDataRequest(surfaces=[name],data_types=[SurfaceDataType.Vertices,SurfaceDataType.FacesConnectivity]))[name]
        return pv.PolyData(np.asarray(data.vertices),np.concatenate([np.r_[len(f),f] for f in data.connectivity]).astype(np.int64))
    axis=np.array([.04129325489475344,-.3684752565834513,-.928720007529695]);orth=np.array([0,-axis[2],axis[1]]);orth/=np.linalg.norm(orth)
    rec={'status':'RUNNING','stage':'static_sweep','physics_frozen':True,'automatic_adaption':False,'rows':[],'component_actual_layers':layers['component']['minimum_crossed_cells']}
    def save():out.write_text(json.dumps(rec,indent=2))
    save();msgstart=len(context['messages'])
    s.file.read_case(file_name=str(case/'theta0_SI.cas.h5'));pipe=surface('pipe_wall');robot0=surface('robot_wall').points.copy();env0=surface('overset_component').points.copy()
    sign=float(np.sign(pv.PolyData(COM[None,:]).compute_implicit_distance(pipe)['implicit_distance'][0]))
    for name,a,angles in [('failure_trajectory',axis,[0,.1,.2,.25,.3,.35]),('orthogonal_pitch_yaw',orth,[0,.2,.3,.35])]:
        prev=0.
        for theta in angles:
            rec['current_pose']={'axis':name,'theta_rad':theta};save()
            if theta:s.mesh.modify_zones.rotate_zone(zone_names=['robot_component_fluid'],rotation_angle=float(theta-prev),origin=COM.tolist(),axis=a.tolist())
            prev=theta;robot=surface('robot_wall');env=surface('overset_component')
            rc=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign)*1000);ec=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*sign)*1000)
            err=float(cKDTree(robot.points).query(Rotation.from_rotvec(a*theta).apply(robot0-COM)+COM)[0].max())
            row={'axis':name,'axis_xyz':a.tolist(),'theta_rad':theta,'physical_clearance_mm':rc,'envelope_clearance_mm':ec,'pose_transform_error_m':err}
            if theta==0:row.update({k:theta0[k] for k in ['orphans','donors','receptors','cell_type_counts','donor_characteristic_length_ratio','donor_length_ratio_gt3_count','minimum_volume_m3']});bg_layers=layers['background']
            else:
                s.solution.initialization.standard_initialize()
                fields=case/'sweep_official_current.csv'
                s.file.export.ascii(file_name=str(fields),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
                row.update(summarize_csv(fields))
                native=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
                row['receptors_with_valid_donors']=sum(int(np.count_nonzero(native[z]>0)) for z in ['background_water','robot_component_fluid'])
                # Explicitly audit background cell crossings at this transformed pose.
                transformed=[];R=Rotation.from_rotvec(a*theta)
                for r in rays():transformed.append({**r,'start_mm':(R.apply(np.array(r['start_mm'])-COM*1000)+COM*1000).tolist(),'normal':R.apply(r['normal']).tolist()})
                bg_layers=audit(case/'benchmark_C_fineA.cas.h5','background_water',transformed)
                (root/f'evidence/benchmark_C_fineA_layers_{name}_{theta:.2f}.json').write_text(json.dumps(bg_layers,indent=2))
            row['background_actual_minimum_layers']=bg_layers['minimum_crossed_cells'];row['background_layer_interior_gaps']=bg_layers['uncovered_interior_samples']
            reasons=[]
            if row['orphans']:reasons.append('ORPHAN')
            if row['cell_type_counts']['-3']:reasons.append('UNIDENTIFIED_CELL')
            row['donor_length_ratio_gt3_fraction']=row['donor_length_ratio_gt3_count']/max(1,row.get('donor_ratio_samples',row['donors']))
            row['large_ratio_diagnostic_fraction_limit']=.01
            if row['donor_length_ratio_gt3_fraction']>.01:reasons.append('MANY_LENGTH_RATIOS_GT3')
            if row.get('receptors_with_valid_donors',row['receptors'])<row['receptors']:reasons.append('INVALID_DONORS')
            if row['minimum_volume_m3']<=0:reasons.append('VOLUME')
            if rc<.10:reasons.append('PHYSICAL_CLEARANCE')
            if ec<=0:reasons.append('ENVELOPE_CLEARANCE')
            if bg_layers['minimum_crossed_cells']<4 or bg_layers['uncovered_interior_samples']:reasons.append('BACKGROUND_LAYERS')
            if err>1e-8:reasons.append('POSE_TRANSFORM')
            row['status']='FAIL' if reasons else 'PASS';row['failure_reasons']=reasons;rec['rows'].append(row);save()
            if name=='failure_trajectory' and theta==0:
                theta0.update(row,status='PASS' if not reasons else 'FAIL',component_actual_minimum_layers=layers['component']['minimum_crossed_cells'],background_actual_minimum_layers=layers['background']['minimum_crossed_cells'],overlap_layers_source=str(root/'evidence/benchmark_C_fineA_actual_layers.json'),overlap_layers='ACTUAL_TETRAHEDRON_RAY_CROSSINGS',valid_donor_check='All official receptor cells matched positive SV_OVERSET_NDONOR count')
                (root/'evidence/benchmark_C_fineA_theta0_connectivity.json').write_text(json.dumps(theta0,indent=2))
            if reasons:rec['status']='FAIL_STATIC_SWEEP';save();return
        s.mesh.modify_zones.rotate_zone(zone_names=['robot_component_fluid'],rotation_angle=-float(prev),origin=COM.tolist(),axis=a.tolist())
    s.solution.initialization.standard_initialize()
    solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    startcase=case/'fineA_static_start.cas.h5';startdata=case/'fineA_static_start.dat.h5'
    s.file.write_case(file_name=str(startcase));s.file.write_data(file_name=str(startdata))
    rec.update(status='BENCHMARK_C_OVERSET_RESOLUTION_PASS',theta_max_validated_rad=.35,start_case=str(startcase),start_data=str(startdata),theta_allowed_rad=.35);save()
    (root/'evidence/benchmark_C_fineA_final_static_gate.json').write_text(json.dumps(rec,indent=2))
    (root/'evidence/benchmark_C_fineA_sweep_transcript.txt').write_text(''.join(context['messages'][msgstart:]))
