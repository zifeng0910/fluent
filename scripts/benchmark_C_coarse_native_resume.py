"""One cold native checkpoint restart, zero-step continuity gate, then <=40 steps."""
import argparse, csv, json, msvcrt, os, shutil, sys, time, traceback
from pathlib import Path
import numpy as np
import psutil, pyvista as pv
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_common import *
from benchmark_C_coarse_commit_audit import system, redact
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_recovery_v2_restart import compare, state_vector
from benchmark_C_analytic_reference import compute_magnetic_load
from benchmark_C_fineA_free_6dof_run import surface_mesh
from benchmark_C_fineA_theta0 import summarize_csv

CONT=EVID/'continuation_20h'

class CommitProfile(MemoryProfile):
    def _sample(self,stage=None):
        row=super()._sample(stage)
        row.update(system())
        for item in row['processes']:
            item['command']=redact(item['command'])
            try:
                m=psutil.Process(item['pid']).memory_info()
                item['private_bytes_gib']=m.private/2**30
                item['commit_gib']=m.pagefile/2**30
            except psutil.Error: pass
        row['project_private_bytes_gib']=sum(p.get('private_bytes_gib',0) for p in row['processes'])
        if time.time()>=read(CONT/'window.json')['deadline_epoch']:
            self.abort={'reason':'TIME_BUDGET_EXHAUSTED','sample':row}
        atomic(EVID/'benchmark_C_coarse_memory_profile.json',{'timestamp':stamp(),
            'rows':self.rows,'peak_project_working_set_gib':max(r['total_project_working_set_gib'] for r in self.rows),
            'abort':self.abort,'sample_interval_s':5,'contains_private_bytes_and_commit':True})
        return row

def validation(solver,checkpoint,step):
    rows=list(csv.DictReader((EVID/'dynamic_history.csv').open()))
    row=next(r for r in reversed(rows) if int(r['step'])==step)
    rec={'timestamp':stamp(),'step':step,'status':'RUNNING','timesteps_advanced':0,
         'pose_reset_performed':False,'velocity_reset_performed':False,'row':row}
    path=EVID/f'benchmark_C_coarse_step{step}_restart_validation.json'
    atomic(path,rec)
    try:
        for k in ['case','data']:
            p=Path(checkpoint[k])
            if sha(p)!=checkpoint[k+'_sha256']: raise RuntimeError('Checkpoint hash changed')
            import h5py
            with h5py.File(p,'r') as f:
                if not list(f.keys()): raise RuntimeError('Empty HDF5 checkpoint')
        t=float(solver.scheme.eval("(rpgetvar 'flow-time)")); rec['native_time_s']=t
        if abs(t-step*25e-6)>1e-12: raise RuntimeError('Native restored time mismatch')
        want=state_vector(row)
        wall=native_state(solver,'robot_wall'); component=native_state(solver,'robot_component_fluid')
        rec.update(native_state=wall,native_errors=compare(wall,want),component_errors=compare(component,want),q_norm=float(np.linalg.norm(wall['q'])))
        if abs(rec['q_norm']-1)>1e-6 or any(v['status']!='PASS' for group in [rec['native_errors'],rec['component_errors']] for v in group.values()):
            raise RuntimeError('Native pose/velocity continuity failure')
        s=solver.settings
        for expr,value,tol in [("(rpgetvar 'physical-time-step)",25e-6,1e-12),("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)",1e-50,1e-60)]:
            if abs(float(solver.scheme.eval(expr))-value)>tol: raise RuntimeError('Frozen time step/cutoff mismatch')
        if not s.setup.dynamic_mesh.options.six_dof.enabled.get_state() or s.setup.general.operating_conditions.gravity.enable.get_state():
            raise RuntimeError('Frozen six-DOF/gravity changed')
        gravity=s.setup.dynamic_mesh.options.six_dof.gravity.get_state()
        if any(float(gravity[k])!=0 for k in 'xyz'): raise RuntimeError('Six-DOF gravity changed')
        nodes=s.setup.dynamic_mesh.dynamic_zones
        for zone in ['robot_wall','robot_component_fluid']:
            name=next(n for n in nodes.keys() if nodes[n].zone.get_state()==zone)
            node=nodes[name]
            if not node.motion.six_dof.enabled.get_state() or bool(node.motion.six_dof.passive.get_state())!=(zone=='robot_component_fluid'):
                raise RuntimeError('Native six-DOF active/passive mismatch')
            if node.motion.motion_def.get_state()!='l2300_magnetic_6dof::libbenchmark_C_v4': raise RuntimeError('Frozen UDF hook mismatch')
        field=solver.fields.field_data
        robot=surface_mesh(field,'robot_wall'); pipe=surface_mesh(field,'pipe_wall'); env=surface_mesh(field,'overset_component')
        base=pv.read(EVID/'fielddata/robot_0000.vtp'); r=Rotation.from_quat(np.asarray(wall['q'])[[1,2,3,0]])
        target=r.apply(base.points-np.asarray(COM))+np.asarray(wall['com'])
        rec['native_quaternion_surface_error_m']=float(cKDTree(robot.points).query(target)[0].max())
        if rec['native_quaternion_surface_error_m']>1e-9: raise RuntimeError('Native surface orientation mismatch')
        F,T,meta=compute_magnetic_load(wall['com'],np.asarray(wall['q'])[[1,2,3,0]],t)
        rec.update(Fmag_N=F.tolist(),Tmag_COM_Nm=T.tolist(),source_state=meta,
                   F_error_N=float(np.max(abs(F-np.array([float(row[f'Fmag_{a}_N']) for a in 'xyz'])))),
                   T_error_Nm=float(np.max(abs(T-np.array([float(row[f'Tmag_{a}_Nm']) for a in 'xyz'])))))
        if rec['F_error_N']>1e-13 or rec['T_error_Nm']>1e-14: raise RuntimeError('Magnetic absolute time/state continuity failure')
        export=OUT/f'restart_{step:04d}_official_cells.csv'
        s.file.export.ascii(file_name=str(export),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
        stats=summarize_csv(export)
        donors=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
        valid=sum(int(np.count_nonzero(donors[z]>0)) for z in ['background_water','robot_component_fluid'])
        inside=float(np.sign(pv.PolyData(np.array([COM])).compute_implicit_distance(pipe)['implicit_distance'][0]))
        gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*inside)); envgap=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*inside))
        stats.update(invalid_donors=max(0,stats['receptors']-valid),physical_clearance_m=gap,envelope_clearance_m=envgap,
            donor_length_ratio_policy='WARNING_ONLY; no refinement or stop solely for ratio >3')
        rec['overset']=stats
        if stats['total_cells']!=4699301 or stats['orphans'] or stats['invalid_donors'] or stats['cell_type_counts']['-3'] or stats['minimum_volume_m3']<=0 or gap<.0001 or envgap<=0:
            raise RuntimeError('Native restart connectivity/volume/clearance failure')
        if abs(gap-float(row['robot_wall_clearance_m']))>1e-9: raise RuntimeError('Archived clearance differs')
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t)>1e-12: raise RuntimeError('Validation advanced time')
        rec.update(status='PASS',case=checkpoint['case'],data=checkpoint['data'],case_sha256=checkpoint['case_sha256'],data_sha256=checkpoint['data_sha256'])
        atomic(path,rec)
        return rec
    except Exception as e:
        rec.update(status='FAIL',error=repr(e));atomic(path,rec);raise

def matched_comparison():
    fine=list(csv.DictReader((ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv').open()))
    coarse=list(csv.DictReader((EVID/'dynamic_history.csv').open()))
    validfine=[r for r in fine if int(r.get('orphan_count',0))==0 and int(r.get('receptors_without_donors',0))==0 and float(r.get('minimum_cell_volume_m3',1))>0]
    matches=[(a,b) for a in coarse for b in validfine if abs(float(a['time_s'])-float(b['time_s']))<1e-12]
    if not matches: raise RuntimeError('No exact valid fine/coarse matched time')
    a,b=max(matches,key=lambda p:float(p[0]['time_s']))
    out={'timestamp':stamp(),'requested_time_s':.001,'time_s':float(a['time_s']),
         'one_ms_reference_status':'AVAILABLE' if abs(float(a['time_s'])-.001)<1e-12 else 'FINE_REFERENCE_NOT_AVAILABLE_AT_1MS',
         'fine_reference_sha256':sha(ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv'),
         'interpolation_used':False,'mesh_convergence_claim':False,'coarse_COM_xyz_m':[float(a[f'com_{k}_m']) for k in 'xyz'],'fine_COM_xyz_m':[float(b[f'com_{k}_m']) for k in 'xyz'],'groups':{}}
    for name,keys,origin in [('COM_displacement',[f'com_{k}_m' for k in 'xyz'],np.array(COM)),('linear_velocity',[f'v{k}_m_s' for k in 'xyz'],np.zeros(3)),('angular_velocity',[f'omega_{k}_rad_s' for k in 'xyz'],np.zeros(3))]:
        x=np.array([float(a[k]) for k in keys])-origin;y=np.array([float(b[k]) for k in keys])-origin; den=float(np.linalg.norm(y));err=float(np.linalg.norm(x-y))
        out['groups'][name]={'coarse':x.tolist(),'fine':y.tolist(),'absolute_difference_norm':err,'relative_difference':err/den if den>1e-20 else None,'direction_cosine':float(np.dot(x,y)/(np.linalg.norm(x)*den)) if np.linalg.norm(x)*den>0 else None}
    ra=Rotation.from_quat([float(a[f'q{i}']) for i in [1,2,3,0]]); rb=Rotation.from_quat([float(b[f'q{i}']) for i in [1,2,3,0]])
    out['orientation_geodesic_difference_rad']=float((ra*rb.inv()).magnitude())
    out['orientation_relative_difference']=out['orientation_geodesic_difference_rad']/max(float(rb.magnitude()),1e-20)
    out['screening_trend_pass']=all(v['relative_difference'] is not None and v['relative_difference']<=1 and v['direction_cosine'] is not None and v['direction_cosine']>0 for v in out['groups'].values()) and out['orientation_relative_difference']<=1
    out['acceptance_note']='Same qualitative development trend gate as the existing 0.25ms milestone; no mesh convergence claim.'
    atomic(EVID/'coarse_vs_fine_latest_exact.json',out);return out

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);args=ap.parse_args()
    config=read(Path(args.config)); attempt=config['attempt']; logdir=CONT/attempt;logdir.mkdir(exist_ok=True)
    sys.stdout=(logdir/'worker_stdout.log').open('a',buffering=1);sys.stderr=(logdir/'worker_stderr.log').open('a',buffering=1)
    lock=(EVID/'pipeline.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    solver=None;profile=None
    old=read(EVID/'state.json');preserve()
    if native(): raise RuntimeError('Existing Fluent host/node: refuse a second solver')
    window=read(CONT/'window.json')
    if time.time()>=window['deadline_epoch']: raise RuntimeError('TIME_BUDGET_EXHAUSTED')
    # Preserve complete previous sample history and failure before changing state.
    for name in ['state.json','dynamic.json','dynamic_history.csv','benchmark_C_coarse_memory_profile.json','final_report.json']:
        if (logdir/name).exists(): raise RuntimeError('Attempt archive already exists')
        shutil.copy2(EVID/name,logdir/name)
    atomic(logdir/'archive_manifest.json',{'timestamp':stamp(),'files':{p.name:sha(p) for p in logdir.glob('*.json') if p.name!='archive_manifest.json'},'dynamic_history_sha256':sha(logdir/'dynamic_history.csv')})
    checkpoint=read(ROOT/config['checkpoint_metadata']);step=int(checkpoint['step'])
    if step!=int(read(EVID/'dynamic.json')['completed_steps']) or step>=40: raise RuntimeError('Native checkpoint/current step mismatch')
    for k in ['case','data']:
        if sha(Path(checkpoint[k]))!=checkpoint[k+'_sha256']: raise RuntimeError('Checkpoint hash changed before launch')
    # Admission reserved ~19 GiB commit headroom from observed historical load increase plus margin.
    m=system()
    if m['available_gib']<12 or m['commit_headroom_gib']<19 or psutil.disk_usage(str(ROOT)).free/2**30<35:
        state('RESOURCE_BLOCKER',error='Cold restart admission changed; zero new steps');return
    state('NATIVE_RESTART',current_step=step,target_step=40,job='scripts/benchmark_C_coarse_native_resume.py',job_created=psutil.Process().create_time(),worker_alive=True,solver_alive=False,attempt=attempt,error=None,execution_owner='20h local supervisor / direct pythonw worker')
    try:
        import ansys.fluent.core as pyfluent
        profile=CommitProfile();profile.start()
        solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,ui_mode='no_gui_or_graphics',start_timeout=180,start_watchdog=False,fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(OUT))
        profile.solver=solver;profile.sample('after_Fluent_launch')
        cp=solver.connection_properties
        atomic(logdir/'owned_engine.json',{'timestamp':stamp(),'host_pid':cp.fluent_host_pid,'host_created':psutil.Process(cp.fluent_host_pid).create_time(),'cortex_pid':cp.cortex_pid,'requested_processor_count':1,'ui_mode':'no_gui_or_graphics'})
        solver.transcript.start(str(logdir/'solver.trn'))
        solver.settings.file.read_case(file_name=checkpoint['case']);solver.settings.file.read_data(file_name=checkpoint['data'])
        profile.sample('after_native_CASE_DATA_read');profile.check('zero_step_restart_gate')
        rec=validation(solver,checkpoint,step)
        atomic(EVID/f'checkpoint_{step:04d}.json',{**rec,'status':'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS'})
        from benchmark_C_coarse_dynamic import run
        run(solver,profile,resume_step=step)
        profile.check('final_checkpoint_validation');preserve()
        end=read(EVID/'checkpoint_0040.json');verify=validation(solver,end,40)
        verify['validation_kind']='ZERO_STEP_NATIVE_SAVE_STATE_AND_HDF5_INTEGRITY; not a second cold reload'
        atomic(EVID/'step40_checkpoint_verification.json',verify)
        matched_comparison(); solver.settings.mesh.check()
        solver.exit();solver=None;profile.close()
        from benchmark_C_coarse_report import write_report
        report=write_report()
        if report['coarse_screening']!='PASS': raise RuntimeError('Final screening gate failed')
        state('COMPLETE',current_step=40,target_step=40,time_s=.001,worker_alive=False,solver_alive=False,next_state='COARSE_DEVELOPMENT_BASELINE',final_report=str(EVID/'final_report.json'))
        write_report()
    except Exception as e:
        dyn=read(EVID/'dynamic.json');step=int(dyn.get('completed_steps',step));status='RESOURCE_BLOCKER' if str(e)=='COARSE_RESOURCE_LIMIT' else 'TIME_BUDGET_EXHAUSTED' if str(e)=='TIME_BUDGET_EXHAUSTED' else 'AWAITING_REPAIR_REVIEW'
        atomic(logdir/'failure.json',{'timestamp':stamp(),'error':repr(e),'traceback':traceback.format_exc(),'step':step,'dynamic':dyn})
        if solver is not None:
            try:
                t=float(solver.scheme.eval("(rpgetvar 'flow-time)"));c=OUT/f'{attempt}_stop_{step:04d}.cas.h5';d=OUT/f'{attempt}_stop_{step:04d}.dat.h5'
                if c.exists() or d.exists(): raise RuntimeError('Refuse stop checkpoint overwrite')
                solver.settings.file.write_case(file_name=str(c));solver.settings.file.write_data(file_name=str(d))
                saved={'timestamp':stamp(),'step':step,'time_s':t,'case':str(c),'data':str(d),'case_sha256':sha(c),'data_sha256':sha(d),'native_state':native_state(solver,'robot_wall'),'status':'SAVED_HASHED_PENDING_RESTART_VALIDATION'}
                atomic(logdir/'stop_checkpoint.json',saved)
            except Exception as save_error: atomic(logdir/'stop_checkpoint_error.json',{'timestamp':stamp(),'error':repr(save_error)})
        state(status,current_step=step,time_s=dyn.get('time_s'),error=repr(e),failure_file=str(logdir/'failure.json'))
    finally:
        if solver is not None:
            try:solver.exit()
            except Exception as e:atomic(logdir/'cleanup_error.json',{'timestamp':stamp(),'error':repr(e)})
        if profile and profile.thread.is_alive(): profile.close()
        prior=read(EVID/'state.json');state(prior['status'],worker_alive=False,solver_alive=bool(native()))
        from benchmark_C_coarse_report import write_report
        write_report()

if __name__=='__main__': main()
