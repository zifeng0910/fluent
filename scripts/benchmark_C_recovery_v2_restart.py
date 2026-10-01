"""Zero-timestep native restart gates, using the unchanged production UDF."""
import csv, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from benchmark_C_recovery_v2_common import ROOT, OUT, atomic, stamp, archive_history, read
from benchmark_C_analytic_reference import compute_magnetic_load

def archived(step):
    rows=[r for r in archive_history() if int(r['step'])==step]
    if rows:return rows[-1]
    candidates=[read(p) for p in (OUT/'checkpoints').glob('*.json')]
    match=[c for c in candidates if c.get('step')==step and c.get('numerical_gate')=='PASS' and c.get('row')]
    if match:return sorted(match,key=lambda x:x['timestamp'])[-1]['row']
    raise RuntimeError('No verified native checkpoint history row exists')
def state_vector(row):
    return dict(com=[float(row[f'com_{a}_m']) for a in 'xyz'],
      q=[float(row[f'q{i}']) for i in range(4)],
      velocity=[float(row[f'v{a}_m_s']) for a in 'xyz'],
      omega=[float(row[f'omega_{a}_rad_s']) for a in 'xyz'])
def compare(actual,expected):
    tolerances={'com':1e-10,'q':1e-9,'velocity':1e-8,'omega':1e-6}
    errors={}
    for k,tol in tolerances.items():
        a=np.asarray(actual[k]);b=np.asarray(expected[k])
        err=float(min(np.max(abs(a-b)),np.max(abs(a+b)))) if k=='q' else float(np.max(abs(a-b)))
        errors[k]={'error':err,'tolerance':tol,'status':'PASS' if np.isfinite(a).all() and err<=tol else 'FAIL'}
    return errors
def native_state(solver,zone_id):
    props={}
    for key in ['origin','orient','velo','omega']:
        props[key]=np.asarray(solver.scheme.eval(f"(cadr (assq '{key} (cdr (assq {zone_id} (rpgetvar 'dynamesh/dynamic-zones)))))"),dtype=float)
        if props[key].shape!=(3,) or not np.isfinite(props[key]).all():raise RuntimeError('Invalid native dynamic-zone state')
    xyzw=Rotation.from_rotvec(props['orient']).as_quat()
    return dict(com=props['origin'].tolist(),q=xyzw[[3,0,1,2]].tolist(),velocity=props['velo'].tolist(),omega=props['omega'].tolist(),native_theta_rad=props['orient'].tolist())
def validate(solver,context,step):
    from benchmark_C_fineA_free_6dof_run import surface_mesh
    from benchmark_C_fineA_theta0 import summarize_csv
    expected=archived(step);want=state_vector(expected)
    rec={'status':'RUNNING','timestamp':stamp(),'step':step,'timesteps_advanced':0,'pose_reset_performed':False,
         'quaternion_access':'Native persisted dynamesh/dynamic-zones orient -> Q_From_Theta equivalent rotation-vector conversion. No normalization/reset of Fluent state. Verified against archived DT_Q and actual surface.',
         'expected':want}
    path=OUT/f'restart_v2_step{step}_state_validation.json';atomic(path,rec)
    try:
        t=float(solver.scheme.eval("(rpgetvar 'flow-time)"));rec['native_time_s']=t
        if abs(t-step*25e-6)>1e-10:raise RuntimeError('Native restart CURRENT_TIME mismatch; no time reset allowed')
        wall=native_state(solver,7);component=native_state(solver,2)
        rec['restored']=wall;rec['errors']=compare(wall,want);rec['component_errors']=compare(component,want)
        qerr=abs(np.linalg.norm(wall['q'])-1);rec['q_norm']=float(np.linalg.norm(wall['q']))
        if qerr>1e-6 or any(x['status']!='PASS' for group in [rec['errors'],rec['component_errors']] for x in group.values()):raise RuntimeError('Native restart pose/velocity mismatch')
        fd=solver.fields.field_data;robot=surface_mesh(fd,'robot_wall');pipe=surface_mesh(fd,'pipe_wall');env=surface_mesh(fd,'overset_component')
        import pyvista as pv
        field=Path(read(ROOT/'evidence/benchmark_C_fineA_free_6dof.json')['fielddata_directory'])
        expected_mesh=pv.read(field/f'robot_{step:04d}.vtp')
        surface_error=float(cKDTree(expected_mesh.points).query(robot.points)[0].max())
        base=pv.read(field/'robot_0000.vtp');r=Rotation.from_quat(np.asarray(wall['q'])[[1,2,3,0]])
        transformed=r.apply(base.points-np.array([.0012060186937156343,0,0]))+np.asarray(wall['com'])
        rigid_error=float(cKDTree(robot.points).query(transformed)[0].max())
        rec.update(surface_error_m=surface_error,native_quaternion_surface_error_m=rigid_error)
        if surface_error>1e-10 or rigid_error>1e-9:raise RuntimeError('Actual surface disagrees with native reconstructed orientation')
        F,T,meta=compute_magnetic_load(wall['com'],np.asarray(wall['q'])[[1,2,3,0]],t)
        archived_F=np.array([float(expected[f'Fmag_{a}_N']) for a in 'xyz']);archived_T=np.array([float(expected[f'Tmag_{a}_Nm']) for a in 'xyz'])
        mag={'timestamp':stamp(),'time_s':t,'source':'Unchanged validated benchmark_C_analytic_reference.py evaluated at native restored state','Fmag_N':F.tolist(),'Tmag_COM_Nm':T.tolist(),'archived_Fmag_N':archived_F.tolist(),'archived_Tmag_Nm':archived_T.tolist(),'F_error_N':float(max(abs(F-archived_F))),'T_error_Nm':float(max(abs(T-archived_T))),'phase_rad':meta['phase_rad'],'ramp':meta['ramp'],'source_s_eff_mm':meta['s_eff_mm']}
        mag['status']='BENCHMARK_C_RESTART_MAGNETIC_CONTINUITY_PASS' if mag['F_error_N']<=1e-13 and mag['T_error_Nm']<=1e-14 else 'FAIL'
        atomic(OUT/f'restart_v2_step{step}_magnetic_validation.json',mag)
        if mag['status']=='FAIL':raise RuntimeError('Magnetic absolute-time/pose continuity failure')
        s=solver.settings
        for expression,value,tol in [("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)",1e-50,1e-60),("(rpgetvar 'physical-time-step)",25e-6,1e-12)]:
            measured=float(solver.scheme.eval(expression))
            if abs(measured-value)>tol:raise RuntimeError('Frozen solver setting differs at native restart')
        if not s.setup.dynamic_mesh.options.six_dof.enabled.get_state():raise RuntimeError('Native 6DOF disabled')
        if s.setup.general.operating_conditions.gravity.enable.get_state():raise RuntimeError('Frozen gravity changed')
        export=ROOT/'live_cases/benchmark_C_fineA/bg020/recovery_v2_restart_cells.csv'
        s.file.export.ascii(file_name=str(export),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
        stats=summarize_csv(export)
        donors=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
        valid=sum(int(np.count_nonzero(donors[z]>0)) for z in ['background_water','robot_component_fluid'])
        inside=float(np.sign(pv.PolyData(np.array([[.0012060186937156343,0,0]])).compute_implicit_distance(pipe)['implicit_distance'][0]))
        gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*inside))
        envgap=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*inside))
        stats.update(receptors_with_valid_donors=valid,physical_clearance_m=gap,envelope_clearance_m=envgap)
        acceptable=(stats['orphans']==0 and valid>=stats['receptors'] and not stats['cell_type_counts']['-3'] and stats['minimum_volume_m3']>0 and stats['donor_characteristic_length_ratio']['max']<=2.76814+1e-3 and gap>=.0001 and envgap>0 and abs(gap-float(expected['robot_wall_clearance_m']))<=1e-9)
        stats['status']='PASS' if acceptable else 'RESTART_OVERSET_FAIL';atomic(OUT/f'restart_v2_step{step}_overset_validation.json',stats)
        if not acceptable:raise RuntimeError('Restart overset/donor/volume/clearance failure')
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t)>1e-12:raise RuntimeError('Validation unexpectedly advanced time')
        rec.update(status='BENCHMARK_C_STEP32_NATIVE_RESTART_PASS' if step==32 else 'NATIVE_RESTART_PASS',magnetic='PASS',overset='PASS',zero_new_timesteps=True)
        atomic(path,rec);return rec
    except Exception as exc:
        rec.update(status='FAIL',error=repr(exc));atomic(path,rec);raise
def compare_step33(row):
    old=archived(33);errors=compare(state_vector(row),state_vector(old))
    for prefix,unit,tol in [('Fmag','N',1e-13),('Tmag','Nm',1e-14)]:
        error=max(abs(float(row[f'{prefix}_{a}_{unit}'])-float(old[f'{prefix}_{a}_{unit}'])) for a in 'xyz')
        errors[prefix]={'error':error,'tolerance':tol,'status':'PASS' if error<=tol else 'FAIL'}
    errors['clearance']={'error':abs(float(row['robot_wall_clearance_m'])-float(old['robot_wall_clearance_m'])),'tolerance':1e-9}
    ok=all(v.get('status','PASS')=='PASS' for v in errors.values()) and errors['clearance']['error']<=1e-9 and int(row['orphan_count'])==int(old['orphan_count'])==0
    rec={'status':'PASS' if ok else 'FAIL','timestamp':stamp(),'step':33,'errors':errors};atomic(OUT/'step33_recomputed_validation.json',rec)
    if not ok:raise RuntimeError('Recomputed step33 differs from archived trajectory')
    return rec
