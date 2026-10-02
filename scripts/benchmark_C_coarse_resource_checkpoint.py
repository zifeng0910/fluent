"""Preserve the idle original engine after a resource stop; advance zero steps."""
import csv,shutil
import numpy as np,pyvista as pv
import psutil
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_common import *
from benchmark_C_recovery_v2_restart import compare,state_vector
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_analytic_reference import compute_magnetic_load

def native_state(solver,zone):
    nodes=solver.settings.setup.dynamic_mesh.dynamic_zones
    name=next(n for n in nodes.keys() if nodes[n].zone.get_state()==zone)
    props=nodes[name].motion.rigid_body_properties.get_state()
    o=props['orientation'];xyzw=Rotation.from_rotvec(np.array(o['axis'])*o['angle']).as_quat()
    return {'com':props['cg_position'],'q':xyzw[[3,0,1,2]].tolist(),
            'velocity':props['cg_velocity'],'omega':props['angular_velocity'],
            'native_orientation_angle_axis':o}

def capture():
    preserve();old=read(EVID/'state.json')
    if old['status']!='RESOURCE_BLOCKER' or psutil.pid_exists(old['pid']):
        raise RuntimeError('Resource-stopped controller must be gone before capture')
    archive=EVID/'resource_stop_0028';fresh=not archive.exists();archive.mkdir(exist_ok=True)
    names=['state.json','failure.json','dynamic.json','dynamic_history.csv',
           'benchmark_C_coarse_memory_profile.json','final_report.json',
           'scheduler_stderr.log','scheduler_stdout.log','solver.trn']
    if fresh:
        for name in names:
            if (EVID/name).exists():shutil.copy2(EVID/name,archive/name)
        atomic(archive/'manifest.json',{'timestamp':stamp(),'files':{p.name:sha(p) for p in archive.iterdir() if p.is_file()}})
    else:
        if any(sha(archive/n)!=h for n,h in read(archive/'manifest.json')['files'].items()):raise RuntimeError('Archive changed')
    cp=read(OUT/'private_session_connection.json');host=psutil.Process(cp['fluent_host_pid'])
    if abs(host.create_time()-cp['host_created'])>.01:raise RuntimeError('Original host changed')
    import ansys.fluent.core as pyfluent
    solver=pyfluent.connect_to_fluent(ip=cp['ip'],port=cp['port'],password=cp['password'],
        cleanup_on_exit=False,start_transcript=False,start_watchdog=False)
    try:
        if int(solver.connection_properties.fluent_host_pid)!=cp['fluent_host_pid']:raise RuntimeError('Wrong engine')
        dyn=read(EVID/'dynamic.json');step=dyn['completed_steps']
        if step!=28:raise RuntimeError('Expected the resource stop at step 28')
        row=list(csv.DictReader((EVID/'dynamic_history.csv').open()))[-1]
        t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
        if abs(t-step*25e-6)>1e-12:raise RuntimeError('Native stop time mismatch')
        actual=native_state(solver,'robot_wall');component=native_state(solver,'robot_component_fluid');want=state_vector(row)
        errors=compare(actual,want);component_errors=compare(component,want)
        if any(v['status']!='PASS' for group in [errors,component_errors] for v in group.values()):raise RuntimeError('Native state mismatch')
        robot=surface_mesh(solver.fields.field_data,'robot_wall');env=surface_mesh(solver.fields.field_data,'overset_component')
        base=pv.read(EVID/'fielddata/robot_0000.vtp');r=Rotation.from_quat(np.array(actual['q'])[[1,2,3,0]])
        target=r.apply(base.points-np.array(COM))+np.array(actual['com'])
        err=float(cKDTree(robot.points).query(target)[0].max())
        if err>1e-9:raise RuntimeError('Actual surface/native quaternion mismatch')
        F,T,meta=compute_magnetic_load(actual['com'],np.array(actual['q'])[[1,2,3,0]],t)
        ferr=float(np.max(abs(F-np.array([float(row[f'Fmag_{a}_N']) for a in 'xyz']))))
        terr=float(np.max(abs(T-np.array([float(row[f'Tmag_{a}_Nm']) for a in 'xyz']))))
        if ferr>1e-13 or terr>1e-14:raise RuntimeError('Magnetic absolute-time continuity mismatch')
        for expr,w,tol in [("(rpgetvar 'physical-time-step)",25e-6,1e-12),("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)",1e-50,1e-60)]:
            if abs(float(solver.scheme.eval(expr))-w)>tol:raise RuntimeError('Frozen setting mismatch')
        c=OUT/'checkpoint_0028.cas.h5';d=OUT/'checkpoint_0028.dat.h5'
        if c.exists() or d.exists():raise RuntimeError('Checkpoint overwrite refused')
        solver.settings.file.write_case(file_name=str(c));solver.settings.file.write_data(file_name=str(d))
        robot.save(EVID/'fielddata/robot_0028.vtp');env.save(EVID/'fielddata/component_0028.vtp')
        rec={'timestamp':stamp(),'step':step,'time_s':t,'status':'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS',
             'case':str(c),'data':str(d),'case_sha256':sha(c),'data_sha256':sha(d),'row':row,
             'native_state':actual,'native_errors':errors,'component_errors':component_errors,
             'native_quaternion_surface_error_m':err,'F_error_N':ferr,'T_error_Nm':terr,
             'timesteps_advanced':0,'same_native_host_pid':host.pid,'source_ramp':meta['ramp']}
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t)>1e-12:raise RuntimeError('Capture advanced time')
        atomic(EVID/'checkpoint_0028.json',rec)
        dyn.update(status='STOPPED_RESOURCE_BLOCKER',stop_reason='System commit crossed unchanged 95% guard')
        atomic(EVID/'dynamic.json',dyn)
        print({'status':'NATIVE_RESOURCE_CHECKPOINT_PASS','step':step,'timesteps_advanced':0,'host_pid':host.pid},flush=True)
    finally:solver.exit()

if __name__=='__main__':capture()
