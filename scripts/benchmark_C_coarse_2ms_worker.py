"""One native step40 cold restart, absolute-time 6DOF steps41..80 and FieldData."""
import csv, io, json, math, msvcrt, os, sys, threading, time, traceback
import numpy as np
import psutil, pyvista as pv
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_2ms_common import *
from benchmark_C_coarse_dynamic import select_native_callback
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_recovery_v2_restart import compare, state_vector
from benchmark_C_analytic_reference import compute_magnetic_load
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_fineA_theta0 import summarize_csv
from benchmark_C_coarse_common import AXIS, ORTH

class Profile:
    def __init__(self):
        self.solver=None;self.owned={};self.rows=[];self.abort=None;self.stage='before_launch'
        self.stop=threading.Event();self.lock=threading.RLock()
        self.thread=threading.Thread(target=self.monitor,daemon=True)
    def register(self):
        cp=self.solver.connection_properties
        hosts=[]
        for pid in [cp.fluent_host_pid,cp.cortex_pid]:
            p=psutil.Process(pid)
            self.owned[p.pid]={'pid':p.pid,'created':p.create_time(),'name':p.name(),'ownership':'SDK registered host/cortex'}
            if p.name().lower()=='fl2610.exe':hosts.append(p)
        for item in mpi_node_associations(hosts,native()):
            p=psutil.Process(item['pid']);self.owned[p.pid]={**item,'name':p.name(),'ownership':item['ownership_evidence']}
        atomic(EVID/'owned_engine.json',{'timestamp':stamp(),'worker_pid':os.getpid(),'worker_created':psutil.Process().create_time(),
            'processes':list(self.owned.values()),'processor_count':1,'ui_mode':'no_gui_or_graphics'})
    def sample(self,stage=None):
        with self.lock:
            if stage:self.stage=stage
            if self.solver is not None:self.register()
            processes=[]
            for rec in [{'pid':os.getpid(),'created':psutil.Process().create_time(),'name':psutil.Process().name()},*self.owned.values()]:
                p=identity(rec)
                if p is None:continue
                m=p.memory_info();processes.append({**rec,'working_set_gib':m.rss/2**30,'private_bytes_gib':m.private/2**30,'process_commit_gib':m.pagefile/2**30})
            m=system();m.update(stage=self.stage,processes=processes,disk_free_gib=psutil.disk_usage(str(ROOT)).free/2**30,
                project_working_set_gib=sum(p['working_set_gib'] for p in processes),
                project_private_bytes_gib=sum(p['private_bytes_gib'] for p in processes))
            self.rows.append(m)
            if m['available_gib']<3 or m['commit_fraction']>=.95 or m['project_working_set_gib']>=15 or m['disk_free_gib']<15:
                self.abort={'reason':'RESOURCE_HARD_STOP','sample':m}
            if time.time()>=read(EVID/'configuration.json')['deadline_epoch'] and self.stage not in ['final_checkpoint_and_exit','after_owned_exit']:
                self.abort={'reason':'TIME_BUDGET_EXHAUSTED','sample':m}
            with (EVID/'memory_samples.jsonl').open('a') as fp:fp.write(json.dumps(m,allow_nan=False)+'\n')
            atomic(EVID/'memory_latest.json',m)
            return m
    def monitor(self):
        while not self.stop.wait(5):
            try:self.sample()
            except Exception as e:
                self.abort={'reason':'RESOURCE_MONITOR_FAILURE','error':repr(e)}
                atomic(EVID/'memory_sampler_failure.json',self.abort)
    def start(self):self.sample();self.thread.start()
    def check(self,stage):
        self.stage=stage
        if self.abort:raise RuntimeError(self.abort['reason'])
    def close(self):
        self.stop.set();self.thread.join(timeout=10)
        self.sample('after_owned_exit')
        atomic(EVID/'memory_summary.json',{'timestamp':stamp(),'sample_count':len(self.rows),
            'peak_project_working_set_gib':max(r['project_working_set_gib'] for r in self.rows),
            'peak_project_private_bytes_gib':max(r['project_private_bytes_gib'] for r in self.rows),
            'peak_measured_system_commit_gib':max(r['commit_gib'] for r in self.rows),
            'peak_measured_system_commit_percent':100*max(r['commit_fraction'] for r in self.rows),
            'minimum_available_gib':min(r['available_gib'] for r in self.rows),'abort':self.abort,
            'boot_CommitPeak_is_not_campaign_peak':True,'unrelated_processes_touched':[]})

def snapshot(solver,step,t):
    from ansys.fluent.core.fields.field_data_interfaces import ScalarFieldDataRequest
    fd=solver.fields.field_data;dest=EVID/'fielddata';name='benchmark_c_coarse_2ms_midplane'
    planes=solver.settings.results.surfaces.plane_surface
    if name not in planes.keys():
        planes.create(name=name);planes[name].method='xy-plane';planes[name].z=0.
    fluid=surface_mesh(fd,name)
    vel=np.asarray(fd.get_field_data(ScalarFieldDataRequest(field_name='velocity-magnitude',surfaces=[name],node_value=True,boundary_value=False))[name],dtype=float)
    if len(vel)!=fluid.n_points or not np.isfinite(vel).all() or len(vel)==0:raise RuntimeError('Invalid actual velocity FieldData')
    fluid.point_data['velocity_m_s']=vel;fluid.save(dest/f'midplane_{step:04d}.vtp')
    for surf,label in [('robot_wall','robot'),('overset_component','component')]:
        surface_mesh(fd,surf).save(dest/f'{label}_{step:04d}.vtp')
    if step==40:surface_mesh(fd,'pipe_wall').save(dest/'pipe.vtp')
    rec=read(EVID/'snapshots.json',{'frames':[]});rec['frames'].append({'step':step,'time_s':t,
        'velocity_max_m_s':float(vel.max()),'velocity_min_m_s':float(vel.min()),
        'velocity_points':int(fluid.n_points),'actual_FieldData':True,'fielddata_export_complete':True})
    atomic(EVID/'snapshots.json',rec)
    if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t)>1e-12:raise RuntimeError('FieldData advanced time')

def save_checkpoint(solver,step,row,stop=False):
    name=('stop_' if stop else 'checkpoint_')+f'{step:04d}'
    c=OUT/(name+'.cas.h5');d=OUT/(name+'.dat.h5')
    if c.exists() or d.exists():raise RuntimeError('Checkpoint overwrite refused')
    t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    actual=native_state(solver,'robot_wall')
    if abs(t-step*DT)>1e-12:raise RuntimeError('Save native time mismatch')
    if row is not None:
        errs=compare(actual,state_vector(row))
        if any(e['status']!='PASS' for e in errs.values()):raise RuntimeError('Native save state differs from history')
    solver.settings.file.write_case(file_name=str(c));solver.settings.file.write_data(file_name=str(d))
    F,T,meta=compute_magnetic_load(actual['com'],np.asarray(actual['q'])[[1,2,3,0]],t)
    rec={'timestamp':stamp(),'step':step,'time_s':t,'case':str(c),'data':str(d),
        'case_sha256':sha(c),'data_sha256':sha(d),'case_size_bytes':c.stat().st_size,'data_size_bytes':d.stat().st_size,
        'row':row,'native_state':actual,'Fmag_N':F.tolist(),'Tmag_COM_Nm':T.tolist(),'source_state':meta,
        'status':'SAVED_HASHED_STOP_PENDING_REVIEW' if stop else 'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS'}
    rec['integrity']=checkpoint_audit(rec,check_hash=False)
    atomic(EVID/(name+'.json'),rec)
    event('CHECKPOINT_SAVED',step=step,time_s=t,kind='STOP' if stop else 'VALID')
    return rec

def collect_step(solver,step,callback_offset,header,pipe,sign,max_radius):
    s=solver.settings;t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    if abs(t-step*DT)>1e-12:raise RuntimeError('Absolute native time mismatch')
    current=OUT/'dynamic_official_current.csv'
    s.file.export.ascii(file_name=str(current),surface_name_list=[],delimiter='comma',
        quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
    stats=summarize_csv(current)
    donors=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
    valid=sum(int(np.count_nonzero(donors[z]>0)) for z in ['background_water','robot_component_fluid'])
    missing=max(0,stats['receptors']-valid);stats.update(time_s=t,receptors_with_valid_donors=valid,invalid_donors=missing)
    atomic(EVID/f'fielddata/connectivity_{step:04d}.json',stats)
    callback=ROOT/'evidence/benchmark_C_analytic_6dof_history.csv'
    with callback.open('rb') as fp:fp.seek(callback_offset);text=fp.read().decode('utf-8');new_offset=fp.tell()
    callbacks=list(csv.DictReader(io.StringIO(header+text)))
    actual=native_state(solver,'robot_wall');component=native_state(solver,'robot_component_fluid')
    if any(e['status']!='PASS' for e in compare(component,actual).values()):raise RuntimeError('Active/passive native state differs')
    rowcb,gate=select_native_callback(callbacks,t,actual)
    atomic(EVID/f'fielddata/callback_native_gate_{step:04d}.json',gate)
    robot=surface_mesh(solver.fields.field_data,'robot_wall');env=surface_mesh(solver.fields.field_data,'overset_component')
    gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign))
    envgap=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*sign))
    q=np.asarray(actual['q']);R=Rotation.from_quat(q[[1,2,3,0]]);com=np.asarray(actual['com'])
    sampled=[Rotation.from_rotvec(np.asarray(a)*v) for a in [AXIS,ORTH] for v in np.linspace(0,.35,15)]
    distance=min(2*math.sin(float((R*r.inv()).magnitude())/2)*max_radius for r in sampled)+float(np.linalg.norm(com-COM0))
    cols=[f'{p}_{a}_{u}' for p,u in [('omega','rad_s'),('Fmag','N'),('Tmag','Nm')] for a in 'xyz']+[f'v{a}_m_s' for a in 'xyz']+[f'theta_{a}_rad' for a in 'xyz']
    row={'step':step,'time_s':t,**{f'com_{a}_m':float(v) for a,v in zip('xyz',com)},
        **{f'q{i}':float(q[i]) for i in range(4)},'q_norm':float(np.linalg.norm(q)),
        **{k:float(rowcb[k]) for k in cols},'orphan_count':stats['orphans'],'receptors_without_donors':missing,
        'official_donor_count':stats['donors'],'official_receptor_count':stats['receptors'],
        'minimum_cell_volume_m3':stats['minimum_volume_m3'],'robot_wall_clearance_m':gap,'overset_wall_clearance_m':envgap,
        **{f'donor_length_ratio_{k}':stats['donor_characteristic_length_ratio'][k] for k in ['min','median','p95','max']},
        'BOI_pose_displacement_bound_m':distance}
    outside=np.linalg.norm(com-COM0)>5e-6 or R.magnitude()>.35 or distance>.00011
    reasons=[]
    if stats['orphans']:reasons.append('ORPHAN')
    if missing:reasons.append('INVALID_DONORS')
    if stats['cell_type_counts']['-3']:reasons.append('UNIDENTIFIED')
    if stats['total_cells']!=4699301:reasons.append('CELL_COUNT')
    if stats['minimum_volume_m3']<=0:reasons.append('VOLUME')
    if not np.isfinite(list(row.values())).all():reasons.append('NONFINITE')
    if abs(row['q_norm']-1)>1e-6:reasons.append('QUATERNION_NORM')
    if gap<.0001 or envgap<=0:reasons.append('CLEARANCE')
    diagnostic={'step':step,'time_s':t,'native_state':actual,'physical_clearance_m':gap,
        'static_envelope_exceeded':bool(outside),'orientation_magnitude_rad':float(R.magnitude()),
        'classification':'STATIC_ENVELOPE_EXCEEDED_BUT_DYNAMIC_CONNECTIVITY_VALID' if outside and not reasons else 'WITHIN_STATIC_ENVELOPE' if not outside else 'OUTSIDE_STATIC_ENVELOPE_WITH_HARD_GATE_FAILURE',
        'hard_failures':reasons,'donor_length_ratio_policy':'WARNING_ONLY'}
    atomic(EVID/f'fielddata/pose_envelope_{step:04d}.json',diagnostic)
    if reasons:
        robot.save(EVID/f'fielddata/robot_failure_{step:04d}.vtp');env.save(EVID/f'fielddata/component_failure_{step:04d}.vtp')
        # Preserve exact official coordinates/ratios/volumes for subsequent diagnosis.
        import shutil
        shutil.copy2(current,OUT/f'failure_official_cells_{step:04d}.csv')
        atomic(EVID/'failure_pose.json',{**diagnostic,'row':row,'connectivity':stats,
            'official_export':str(OUT/f'failure_official_cells_{step:04d}.csv'),
            'local_spacing_status':'PENDING_EXACT_POSE_DIAGNOSIS; not inferred from nominal mesh',
            'classification_pending':'OUTSIDE_STATIC_ENVELOPE / LOCAL_RESOLUTION_FAILURE / TRANSIENT_OVERSET_FAILURE / OTHER'})
    return row,reasons,new_offset

def exit_owned(solver,profile):
    """Only exact registered, idle, owned leftovers after public SDK exit may close."""
    with profile.lock:
        profile.register();profile.solver=None
    solver.exit()
    for _ in range(15):
        if not any(identity(x) for x in profile.owned.values()):break
        time.sleep(2)
    remaining=[(r,identity(r)) for r in profile.owned.values()];remaining=[(r,p) for r,p in remaining if p]
    proof={'timestamp':stamp(),'SDK_exit_requested':True,'unrelated_processes_touched':[],'terminated':[]}
    if remaining:
        before={p.pid:sum(p.cpu_times()[:2]) for _,p in remaining};time.sleep(5)
        idle={p.pid:sum(p.cpu_times()[:2])-before[p.pid] for r,p in remaining if identity(r)}
        proof['CPU_seconds_over5s']=idle
        if any(v>.05 for v in idle.values()):raise RuntimeError('Owned engine not idle after SDK exit; no forced cleanup')
        # Node closure permits its host to exit; process identities are checked again.
        order=sorted(remaining,key=lambda pair:0 if pair[0]['name'].lower()=='fl_mpi2610.exe' else 1)
        for r,_ in order:
            p=identity(r)
            if p is not None:p.terminate();proof['terminated'].append(r)
        psutil.wait_procs([p for _,p in remaining],timeout=15)
    proof['owned_engine_closed']=not any(identity(x) for x in profile.owned.values())
    atomic(EVID/'engine_exit.json',proof)
    if not proof['owned_engine_closed']:raise RuntimeError('Owned engine closure incomplete')

def main():
    sys.stdout=(EVID/'worker_stdout.log').open('a',buffering=1);sys.stderr=(EVID/'worker_stderr.log').open('a',buffering=1)
    lock=(BASE/'pipeline.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    solver=None;profile=Profile();step=40;last_row=None
    try:
        verify_frozen()
        if native():raise RuntimeError('Existing native engine: duplicate launch refused')
        checkpoint=read(BASE/'checkpoint_0040.json');checkpoint_audit(checkpoint)
        if not admission()[0]:raise RuntimeError('Restart admission changed')
        if (EVID/'step40_restart_continuity.json').exists():raise RuntimeError('Previous attempt must be reviewed before another launch')
        if len(history())!=40:raise RuntimeError('Only original40-row prefix may begin this run')
        profile.start();config=read(EVID/'configuration.json')
        state('NATIVE_RESTART',worker_pid=os.getpid(),worker_created=psutil.Process().create_time(),worker_alive=True,solver_alive=False)
        import ansys.fluent.core as pyfluent
        solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,
            ui_mode='no_gui_or_graphics',start_timeout=180,start_watchdog=False,
            fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(OUT))
        profile.solver=solver;profile.sample('after_launch');solver.transcript.start(str(EVID/'solver.trn'))
        solver.settings.file.read_case(file_name=checkpoint['case']);solver.settings.file.read_data(file_name=checkpoint['data'])
        profile.sample('after_native_CASE_DATA');profile.check('zero_step_restart_gate')
        # Use proven validator with output roots explicitly scoped to the new campaign.
        import benchmark_C_coarse_native_resume as restart
        restart.EVID=EVID;restart.OUT=OUT
        rec=restart.validation(solver,checkpoint,40)
        meta=rec['source_state']
        if abs(meta['phase_rad']-2*math.pi*100*.001)>1e-12 or abs(meta['ramp']-1)>1e-12:
            raise RuntimeError('Ramp endpoint/absolute phase continuity failed')
        expected_s=31.699921242759284+(rec['native_state']['com'][0]-COM0[0])*1000-6*.001
        if abs(meta['s_eff_mm']-expected_s)>1e-12:raise RuntimeError('Absolute-time6mm/s offset reset')
        rec.update(status='BENCHMARK_C_COARSE_STEP40_NATIVE_RESTART_PASS',phase_hz=100,
            ramp_endpoint_pass=True,absolute_time_offset_mm=6*.001,source_offset_pass=True)
        atomic(EVID/'step40_restart_continuity.json',rec)
        event('STEP40_NATIVE_RESTART_PASS',timesteps_advanced=0,time_s=rec['native_time_s'])
        snapshot(solver,40,rec['native_time_s'])
        pipe=surface_mesh(solver.fields.field_data,'pipe_wall');robot=surface_mesh(solver.fields.field_data,'robot_wall')
        sign=float(np.sign(robot.compute_implicit_distance(pipe)['implicit_distance'][0]))
        radius=read(BASE/'dynamic.json')['maximum_component_radius_m']
        callback=ROOT/'evidence/benchmark_C_analytic_6dof_history.csv';header=callback.open().readline();offset=callback.stat().st_size
        calc=solver.settings.solution.run_calculation
        if abs(float(calc.parameters.time_step_size.get_state())-DT)>1e-12 or int(calc.parameters.max_iter_per_time_step.get_state())!=2:
            raise RuntimeError('Frozen calculation parameters differ')
        with (EVID/'dynamic_history.csv').open('a',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(history()[-1]))
            for step in range(41,81):
                profile.check('during_timestep_solve');state('WORKER_RUNNING',current_step=step-1,target_step=80,solver_alive=True,worker_alive=True)
                calc.dual_time_iterate(time_step_count=1,max_iter_per_step=2)
                profile.sample('after_timestep_'+str(step))
                row,reasons,offset=collect_step(solver,step,offset,header,pipe,sign,radius);last_row=row
                writer.writerow(row);stream.flush();os.fsync(stream.fileno())
                atomic(EVID/'dynamic.json',{'status':'FAIL' if reasons else 'RUNNING','completed_steps':step,
                    'time_s':row['time_s'],'latest_row':row,'failure_reasons':reasons})
                state('WORKER_RUNNING',current_step=step,time_s=row['time_s'],latest_row=row)
                if reasons:raise RuntimeError('DYNAMIC_HARD_GATE: '+','.join(reasons))
                profile.check('after_step_'+str(step))
                if step in config['checkpoint_steps']:
                    save_checkpoint(solver,step,row);verify_frozen()
                if step in config['fielddata_snapshot_steps']:
                    try:snapshot(solver,step,row['time_s'])
                    except Exception as export_error:
                        # Postprocessing error does not invalidate a healthy native solve.
                        event('FIELDDATA_EXPORT_FAILED',step=step,error=repr(export_error))
                        atomic(EVID/f'fielddata/export_failure_{step:04d}.json',{'step':step,'error':repr(export_error)})
                print(f'COARSE_2MS {step}/80 time={row["time_s"]:.9g} orphan={row["orphan_count"]}',flush=True)
        profile.stage='final_checkpoint_and_exit'
        end=read(EVID/'checkpoint_0080.json');verify=restart.validation(solver,end,80)
        verify['validation_kind']='ZERO_STEP_NATIVE_SAVE_STATE_AND_HDF5_INTEGRITY; not a second cold reload'
        atomic(EVID/'step80_checkpoint_verification.json',verify)
        atomic(EVID/'dynamic.json',{'status':'BENCHMARK_C_COARSE_FREE_6DOF_2MS_PASS','completed_steps':80,'time_s':last_row['time_s'],'latest_row':last_row})
        state('NUMERICAL_PASS',current_step=80,time_s=last_row['time_s'])
        exit_owned(solver,profile);solver=None;profile.close()
        from benchmark_C_coarse_2ms_report import write_report
        write_report()
        state('AWAITING_RENDER',worker_alive=False,solver_alive=False)
        event('NUMERICAL_2MS_PASS',completed_steps=80)
    except Exception as e:
        failure={'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc(),
            'recorded_history_step':int(history()[-1]['step']),'profile_abort':profile.abort}
        if solver is not None:
            try:
                t=float(solver.scheme.eval("(rpgetvar 'flow-time)"));actual_step=int(round(t/DT))
                row=last_row if last_row is not None and int(last_row['step'])==actual_step else None
                failure['stop_checkpoint']=save_checkpoint(solver,actual_step,row,stop=True)
            except Exception as err:failure['checkpoint_save_error']=repr(err)
        atomic(EVID/'failure.json',failure)
        state('TIME_BUDGET_EXHAUSTED' if str(e)=='TIME_BUDGET_EXHAUSTED' else 'STOPPED_RESOURCE_BLOCKER' if profile.abort else 'AWAITING_REPAIR_REVIEW',error=repr(e),failure_file=str(EVID/'failure.json'))
        event('WORKER_FAILURE',error=repr(e))
    finally:
        if solver is not None:
            try:exit_owned(solver,profile)
            except Exception as e:atomic(EVID/'engine_exit_failure.json',{'timestamp':stamp(),'error':repr(e)})
        if profile.thread.is_alive():profile.close()
        old=read(EVID/'state.json');state(old['status'],worker_alive=False,solver_alive=bool(native()))
        from benchmark_C_coarse_2ms_report import write_report
        write_report()

if __name__=='__main__':main()
