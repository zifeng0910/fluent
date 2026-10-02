"""Native 6DOF 10/20/40-step gates in the same session after the static pass."""
import csv,io,math
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_common import *
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_fineA_theta0 import summarize_csv

def compare(row):
    fine=next(r for r in csv.DictReader((ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv').open()) if int(r['step'])==10)
    groups={'COM_displacement':([f'com_{a}_m' for a in 'xyz'],np.array(COM)),
        'linear_velocity':([f'v{a}_m_s' for a in 'xyz'],np.zeros(3)),
        'angular_velocity':([f'omega_{a}_rad_s' for a in 'xyz'],np.zeros(3))}
    rec={'time_s':float(row['time_s']),'comparison_purpose':'Development screening, not mesh convergence','fine_source':'evidence/benchmark_C_fineA_free_6dof_history.csv','groups':{}}
    for name,(keys,origin) in groups.items():
        a=np.array([float(row[k]) for k in keys])-origin;b=np.array([float(fine[k]) for k in keys])-origin
        den=float(np.linalg.norm(b));cos=float(np.dot(a,b)/(np.linalg.norm(a)*den)) if den*np.linalg.norm(a)>0 else None
        rec['groups'][name]={'coarse':a.tolist(),'fine':b.tolist(),'absolute_difference_norm':float(np.linalg.norm(a-b)),'relative_difference':float(np.linalg.norm(a-b)/den) if den>1e-30 else None,'direction_cosine':cos}
    qa=np.array([float(row[f'q{i}']) for i in range(4)]);qb=np.array([float(fine[f'q{i}']) for i in range(4)])
    ra=Rotation.from_quat(qa[[1,2,3,0]]);rb=Rotation.from_quat(qb[[1,2,3,0]])
    rec['orientation']={'coarse_angle_rad':float(ra.magnitude()),'fine_angle_rad':float(rb.magnitude()),'geodesic_difference_rad':float((ra*rb.inv()).magnitude()),'relative_geodesic_difference':float((ra*rb.inv()).magnitude()/max(float(rb.magnitude()),1e-30))}
    # An opposite trend or >100% velocity/rotation error cannot be a screening baseline.
    rec['trend_gate_rule']='Both velocity direction cosines > 0 and relative differences <= 1; orientation geodesic difference <= fine angle. No accuracy/convergence claim.'
    rec['qualitative_trend_pass']=all(rec['groups'][name]['direction_cosine'] is not None and rec['groups'][name]['direction_cosine']>0 and rec['groups'][name]['relative_difference']<=1 for name in ['linear_velocity','angular_velocity']) and rec['orientation']['relative_geodesic_difference']<=1
    atomic(EVID/'coarse_vs_fine_025ms.json',rec);return rec

def run(solver,profile,resume_step=0):
    if read(EVID/'static.json').get('status')!='BENCHMARK_C_COARSE_STATIC_PASS':raise RuntimeError('Four static poses must pass before dynamics')
    s=solver.settings;solver.tui.define.overset_interfaces.adapt.set.automatic('no')
    solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    if float(solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)"))>1e-50:raise RuntimeError('Inertia cutoff not preserved')
    if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-resume_step*25e-6)>1e-12:raise RuntimeError('Native start time mismatch; reset forbidden')
    dyn=s.setup.dynamic_mesh
    if not resume_step:
        s.setup.user_defined.load(udf_library_name=str(ROOT/'fluent_udf/libbenchmark_C_v4'))
        dyn.enabled=True;dyn.methods.smoothing.enabled=False;dyn.options.six_dof.enabled=True
        dyn.options.six_dof.gravity.set_state({'x':0.,'y':0.,'z':0.});s.setup.general.operating_conditions.gravity.enable=False;s.setup.general.solver.time='transient'
    else:
        validation=read(EVID/f'checkpoint_{resume_step:04d}.json')
        if validation.get('status')!='NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS' or validation.get('timesteps_advanced')!=0:raise RuntimeError('Verified zero-step resource checkpoint required')
    zones={}
    for zone in ['robot_component_fluid','robot_wall']:
        if not resume_step:dyn.dynamic_zones.create(zone=zone)
        name=next(n for n in dyn.dynamic_zones.keys() if dyn.dynamic_zones[n].zone.get_state()==zone);zones[zone]=name;node=dyn.dynamic_zones[name]
        if not resume_step:
            node.type='rigid-body';node.motion.six_dof.enabled=True;node.motion.six_dof.passive=(zone=='robot_component_fluid')
            node.motion.rigid_body_properties.cg_position=COM;node.motion.rigid_body_properties.orientation.set_state({'angle':0.,'axis':[1.,0.,0.]});node.motion.motion_def='l2300_magnetic_6dof::libbenchmark_C_v4'
    calc=s.solution.run_calculation;calc.parameters.time_step_size=25e-6;calc.parameters.max_iter_per_time_step=2
    field=solver.fields.field_data;pipe=surface_mesh(field,'pipe_wall');robot0=surface_mesh(field,'robot_wall');env0=surface_mesh(field,'overset_component')
    sign=float(np.sign(robot0.compute_implicit_distance(pipe)['implicit_distance'][0]));fd=EVID/'fielddata';fd.mkdir(exist_ok=True)
    if not resume_step:
        robot0.save(fd/'robot_0000.vtp');env0.save(fd/'component_0000.vtp');pipe.save(fd/'pipe.vtp')
    max_r=read(EVID/'dynamic.json')['maximum_component_radius_m'] if resume_step else float(np.linalg.norm(env0.points-np.array(COM),axis=1).max())
    sampled=[Rotation.from_rotvec(np.array(a)*t) for a in [AXIS,ORTH] for t in np.linspace(0,.35,15)]
    callback_path=ROOT/'evidence/benchmark_C_analytic_6dof_history.csv';header=callback_path.open().readline();offset=callback_path.stat().st_size
    rec={'status':'RUNNING','steps_requested':40,'completed_steps':0,'time_s':0.,'dt_s':25e-6,'maximum_iterations_per_step':2,'cutoff':1e-50,'frozen_UDF_sha256':FROZEN_SHA,'stages':[],'frames':[],'gravity':[0,0,0],'automatic_overset_adaption':False,'original_callback_file_bytes':offset,'maximum_component_radius_m':max_r}
    if resume_step:
        rec=read(EVID/'dynamic.json');rec.update(status='RUNNING',resumed_from_step=resume_step,resume_callback_offset=offset)
        rec.pop('stop_reason',None)
    atomic(EVID/'dynamic.json',rec)
    native_columns=[f'{p}_{a}_{u}' for p,u in [('omega','rad_s'),('Fmag','N'),('Tmag','Nm')] for a in 'xyz']+[f'v{a}_m_s' for a in 'xyz']+[f'theta_{a}_rad' for a in 'xyz']
    with (EVID/'dynamic_history.csv').open('a' if resume_step else 'w',newline='') as stream:
        writer=None
        for step in range(resume_step+1,41):
            profile.check('during_timestep_solve');state('DYNAMIC_SOLVE',current_step=step-1,target_step=10 if step<=10 else 20 if step<=20 else 40)
            calc.dual_time_iterate(time_step_count=1,max_iter_per_step=2)
            profile.sample('after_timestep_'+str(step));t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
            if abs(t-step*25e-6)>1e-10:raise RuntimeError('Native time mismatch')
            current=OUT/'dynamic_official_current.csv';s.file.export.ascii(file_name=str(current),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
            stats=summarize_csv(current);donors=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
            positive=sum(int(np.count_nonzero(donors[z]>0)) for z in ['background_water','robot_component_fluid']);missing=max(0,stats['receptors']-positive)
            stats.update(time_s=t,receptors_with_valid_donors=positive,invalid_donors=missing)
            atomic(fd/f'connectivity_{step:04d}.json',stats)
            with callback_path.open('rb') as fp:fp.seek(offset);newtext=fp.read().decode('utf-8')
            callbacks=list(csv.DictReader(io.StringIO(header+newtext)))
            native_row=next(r for r in reversed(callbacks) if r['mode']=='FREE_6DOF' and r['zone_name']=='robot_component_fluid' and abs(float(r['time_s'])-t)<1e-10)
            com=np.array(dyn.dynamic_zones[zones['robot_wall']].motion.rigid_body_properties.cg_position.get_state());q=np.array([float(native_row[f'q{i}']) for i in range(4)]);qnorm=float(np.linalg.norm(q))
            robot=surface_mesh(field,'robot_wall');env=surface_mesh(field,'overset_component')
            clearance=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign));envgap=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*sign))
            R=Rotation.from_quat(q[[1,2,3,0]])
            # Bound every surface point's displacement from the actual sampled BOI poses.
            distance=min(2*math.sin(float((R*rot.inv()).magnitude())/2)*max_r for rot in sampled)+float(np.linalg.norm(com-np.array(COM)))
            row={'step':step,'time_s':t,**{f'com_{a}_m':float(v) for a,v in zip('xyz',com)},**{f'q{i}':float(q[i]) for i in range(4)},'q_norm':qnorm,**{k:float(native_row[k]) for k in native_columns},'orphan_count':stats['orphans'],'receptors_without_donors':missing,'official_donor_count':stats['donors'],'official_receptor_count':stats['receptors'],'minimum_cell_volume_m3':stats['minimum_volume_m3'],'robot_wall_clearance_m':clearance,'overset_wall_clearance_m':envgap,**{f'donor_length_ratio_{k}':stats['donor_characteristic_length_ratio'][k] for k in ['min','median','p95','max']},'BOI_pose_displacement_bound_m':distance}
            if writer is None:
                writer=csv.DictWriter(stream,fieldnames=list(row))
                if not resume_step:writer.writeheader()
            writer.writerow(row);stream.flush();rec.update(completed_steps=step,time_s=t,latest_row=row);atomic(EVID/'dynamic.json',rec)
            reasons=[]
            if stats['orphans']:reasons.append('ORPHAN')
            if missing:reasons.append('INVALID_DONORS')
            if stats['cell_type_counts']['-3']:reasons.append('UNIDENTIFIED')
            if stats['minimum_volume_m3']<=0:reasons.append('VOLUME')
            if not np.isfinite(list(row.values())).all():reasons.append('NONFINITE')
            if abs(qnorm-1)>1e-6:reasons.append('QUATERNION_NORM')
            if clearance<.0001 or envgap<=0:reasons.append('CLEARANCE')
            if np.linalg.norm(com-np.array(COM))>5e-6 or R.magnitude()>.35 or distance>.00011:reasons.append('SWEEP_COVERAGE')
            if reasons:
                robot.save(fd/f'robot_failure_{step:04d}.vtp');env.save(fd/f'component_failure_{step:04d}.vtp');rec.update(status='FAIL',failure_reasons=reasons);atomic(EVID/'dynamic.json',rec);raise RuntimeError(f'Step {step}: {reasons}')
            if step%10==0:
                profile.check('after_'+str(step)+'_steps');profile.sample('after_'+str(step)+'_steps')
                c=OUT/f'checkpoint_{step:04d}.cas.h5';d=OUT/f'checkpoint_{step:04d}.dat.h5'
                if c.exists() or d.exists():raise RuntimeError('Refusing checkpoint overwrite')
                s.file.write_case(file_name=str(c));s.file.write_data(file_name=str(d));robot.save(fd/f'robot_{step:04d}.vtp');env.save(fd/f'component_{step:04d}.vtp')
                checkpoint={'step':step,'time_s':t,'case':str(c),'data':str(d),'case_sha256':sha(c),'data_sha256':sha(d),'row':row,'status':'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS'};atomic(EVID/f'checkpoint_{step:04d}.json',checkpoint)
                rec['stages'].append({'steps':step,'status':'PASS','time_s':t});rec['frames'].append({'step':step,'time_s':t});atomic(EVID/'dynamic.json',rec)
                if step==10:
                    comparison=compare(row)
                    if not comparison['qualitative_trend_pass']:raise RuntimeError('Coarse-vs-fine trend gate failed at 0.25ms')
            print(f'COARSE step {step}/40 t={t:.8f} orphan={stats["orphans"]} qnorm={qnorm:.15g}',flush=True)
    rec.update(status='BENCHMARK_C_COARSE_SCREENING_NUMERICAL_PASS');atomic(EVID/'dynamic.json',rec);return rec
