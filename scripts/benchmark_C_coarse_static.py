"""Four-pose official connectivity and actual tetra crossing gate, no dynamics."""
import numpy as np,pyvista as pv
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from benchmark_C_coarse_common import *
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_fineA_theta0 import summarize_csv
from benchmark_C_actual_ray_layers import audit,rays

def run(solver,profile,already_initialized=False):
    s=solver.settings;rec={'status':'RUNNING','rows':[],'time_steps_advanced':0,'automatic_adaption':False,'component_identity':read(EVID/'component_identity.json')}
    if rec['component_identity'].get('status')!='PASS':raise RuntimeError('Unchanged component identity is not verified')
    if not read(EVID/'mesh.json').get('cell_count_target_pass'):raise RuntimeError('Total cell target 5M not achieved')
    atomic(EVID/'static.json',rec);state('CASE_LOAD')
    profile.check('CASE_LOAD')
    if not already_initialized:
        s.file.read_case(file_name=str(OUT/'coarse_raw.cas.h5'));profile.sample('after_CASE_load');s.mesh.scale(x_scale=.001,y_scale=.001,z_scale=.001)
        for name,typ in [('overset_component','overset'),('inlet','pressure-inlet'),('outlet','pressure-outlet')]:s.setup.boundary_conditions.set_zone_type(zone_list=[name],new_type=typ)
        s.setup.models.viscous.model='laminar';mat=s.setup.materials.fluid
        if 'water' not in mat.keys():mat.create(name='water')
        mat['water'].density.set_state({'option':'constant','value':998.2});mat['water'].viscosity.set_state({'option':'constant','value':.001003})
        for z in ['background_water','robot_component_fluid']:s.setup.cell_zone_conditions.fluid[z].general.material='water'
        s.solution.initialization.standard_initialize();profile.sample('after_DATA_or_init');profile.sample('after_overset_initialization')
    else:
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)")))>1e-12 or not solver.fields.field_data.is_data_valid():raise RuntimeError('Static reconnection is not initialized at t=0')
    pipe=surface_mesh(solver.fields.field_data,'pipe_wall');robot0=surface_mesh(solver.fields.field_data,'robot_wall').points.copy()
    if abs(pipe.bounds[1]-.005)>1e-7 or abs(pipe.bounds[0]+.005)>1e-7:raise RuntimeError('Native units/pipe bounds not SI')
    sign=float(np.sign(pv.PolyData(np.array(COM)[None,:]).compute_implicit_distance(pipe)['implicit_distance'][0]))
    component_layers=audit(OUT/'coarse_raw.cas.h5','robot_component_fluid',rays());atomic(EVID/'layers_component.json',component_layers)
    rec['component_actual_minimum_layers']=component_layers['minimum_crossed_cells']
    for name,axis,theta in [('theta0',AXIS,0),('failure_025',AXIS,.25),('failure_035',AXIS,.35),('orthogonal_035',ORTH,.35)]:
        state('STATIC_AUDIT',pose=name);profile.check('static_'+name)
        if theta:s.mesh.modify_zones.rotate_zone(zone_names=['robot_component_fluid'],rotation_angle=theta,origin=COM,axis=axis);s.solution.initialization.standard_initialize()
        robot=surface_mesh(solver.fields.field_data,'robot_wall');env=surface_mesh(solver.fields.field_data,'overset_component')
        robot.save(EVID/(name+'_robot.vtp'));env.save(EVID/(name+'_component.vtp'))
        csvpath=OUT/(name+'_official_cells.csv')
        s.file.export.ascii(file_name=str(csvpath),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
        row={**summarize_csv(csvpath),'pose':name,'axis':axis,'theta_rad':theta,'official_fields':str(csvpath)}
        donor=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
        row['receptors_with_valid_donors']=sum(int(np.count_nonzero(donor[z]>0)) for z in ['background_water','robot_component_fluid'])
        row['invalid_donors']=max(0,row['receptors']-row['receptors_with_valid_donors'])
        row['physical_clearance_mm']=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign)*1000)
        row['envelope_clearance_mm']=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*sign)*1000)
        R=Rotation.from_rotvec(np.array(axis)*theta);row['transform_error_m']=float(cKDTree(robot.points).query(R.apply(robot0-np.array(COM))+COM)[0].max())
        defs=[{**r,'start_mm':(R.apply(np.array(r['start_mm'])-np.array(COM)*1000)+np.array(COM)*1000).tolist(),'normal':R.apply(r['normal']).tolist()} for r in rays()]
        layers=audit(OUT/'coarse_raw.cas.h5','background_water',defs);atomic(EVID/('layers_'+name+'.json'),layers)
        row.update(background_actual_minimum_layers=layers['minimum_crossed_cells'],background_interior_gaps=layers['uncovered_interior_samples'],minimum_actual_covered_span_mm=layers['minimum_actual_covered_span_mm'])
        reasons=[]
        for failed,reason in [(row['orphans']>0,'ORPHANS'),(row['invalid_donors']>0,'INVALID_DONORS'),(row['cell_type_counts']['-3']>0,'UNIDENTIFIED'),(row['minimum_volume_m3']<=0,'VOLUME'),(row['physical_clearance_mm']<.10,'PHYSICAL_CLEARANCE'),(row['envelope_clearance_mm']<=0,'ENVELOPE_CLEARANCE'),(row['transform_error_m']>1e-8,'POSE_TRANSFORM'),(layers['minimum_crossed_cells']<4,'BACKGROUND_LAYERS'),(layers['uncovered_interior_samples']>0,'BACKGROUND_GAPS'),(component_layers['minimum_crossed_cells']<4,'COMPONENT_LAYERS'),(component_layers['uncovered_interior_samples']>0,'COMPONENT_GAPS'),(layers['minimum_actual_covered_span_mm']<.048,'BACKGROUND_SPAN')]:
            if failed:reasons.append(reason)
        row['donor_length_ratio_gt3_fraction']=row['donor_length_ratio_gt3_count']/max(1,row['donor_ratio_samples'])
        # Preserve the existing fine static donor acceptance rule.
        if row['donor_length_ratio_gt3_fraction']>.01:reasons.append('MANY_DONOR_LENGTH_RATIOS_GT3')
        row.update(status='FAIL' if reasons else 'PASS',failure_reasons=reasons);rec['rows'].append(row);atomic(EVID/'static.json',rec)
        profile.sample('after_static_'+name)
        if reasons:
            rec['status']='FAIL_STATIC_GATE';atomic(EVID/'static.json',rec);raise RuntimeError(f'{name}: {reasons}')
        if theta:s.mesh.modify_zones.rotate_zone(zone_names=['robot_component_fluid'],rotation_angle=-theta,origin=COM,axis=axis);s.solution.initialization.standard_initialize()
    s.solution.initialization.standard_initialize();solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    s.file.write_case(file_name=str(OUT/'coarse_static_start.cas.h5'));s.file.write_data(file_name=str(OUT/'coarse_static_start.dat.h5'))
    rec.update(status='BENCHMARK_C_COARSE_STATIC_PASS',start_case=str(OUT/'coarse_static_start.cas.h5'),start_data=str(OUT/'coarse_static_start.dat.h5'));atomic(EVID/'static.json',rec);state('BENCHMARK_C_COARSE_STATIC_PASS')
    return rec
