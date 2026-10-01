"""Durable file/process supervisor. Fluent is owned by one existing session worker.

No Fluent RPC polling, no physics edits, no automatic whole-region remeshing.
Repair plans are reviewed by the continuing Codex chat; repeated configurations
are rejected. Task Scheduler owns this process independently of the chat shell.
"""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, shutil, subprocess, sys, time
from datetime import datetime, timezone, timedelta
from pathlib import Path
import h5py
import psutil
import msvcrt

ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'evidence'; LOG=E/'benchmark_C_longrun'; STATE=E/'benchmark_C_longrun_state.json'
CASE=ROOT/'live_cases/benchmark_C_analytic_6dof'
QUEUE=CASE/'session_command.json'; SESSION=E/'benchmark_C_single_session.json'
FREE=E/'benchmark_C_fineA_free_6dof.json'; HIST=E/'benchmark_C_fineA_free_6dof_history.csv'
MESH=ROOT/'live_cases/benchmark_C_fineA/bg020'
CAMPAIGN='BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF'
FROZEN_SHA='0ae61d4e84207ced13dad7f29517212f86ca0aa0281fb423166637aec881dd6e'
TERMINAL={'COMPLETE','HARD_BLOCKER_REQUIRES_REVIEW','TIME_BUDGET_EXHAUSTED','STOPPED_RESOURCE_BLOCKER'}

def utc():return datetime.now(timezone.utc)
def stamp():return utc().isoformat()
def sim_status(report):return report.get('simulation_status',report.get('status'))
def read(path,default=None):
    try:return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError,ValueError):return {} if default is None else default
def atomic(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(obj,indent=2,allow_nan=False),encoding='utf-8');tmp.replace(path)
def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as fp:
        for chunk in iter(lambda:fp.read(8*1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()
def rows():
    try:
        with HIST.open(encoding='utf-8') as fp:return [r for r in csv.DictReader(fp) if r.get('q_norm')]
    except (OSError,csv.Error):return []
def norm(row,prefix,suffix):return math.sqrt(sum(float(row[f'{prefix}{a}{suffix}'])**2 for a in 'xyz'))
def native():
    found=[]
    for p in psutil.process_iter(['pid','name','create_time','cmdline','memory_info']):
        try:
            if p.name().lower() in ('fl2610.exe','fl_mpi2610.exe'):
                found.append({'pid':p.pid,'name':p.name(),'created':p.create_time(),
                              'cpu':sum(p.cpu_times()[:2]),'rss':p.memory_info().rss,
                              'command':' '.join(p.cmdline())})
        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
    return found
def worker(info):
    try:
        p=psutil.Process(int(info['pid']))
        if info.get('process_created') and abs(p.create_time()-info['process_created'])>.01:return None
        cmd=' '.join(p.cmdline()).replace('\\','/').lower()
        return p if 'h:/fluent/scripts/benchmark_c_single_session.py' in cmd else None
    except (KeyError,ValueError,psutil.Error):return None
def integrity(case,data):
    if not case.exists() or not data.exists():return False
    if utc().timestamp()-max(case.stat().st_mtime,data.stat().st_mtime)<30:return False
    try:
        with h5py.File(case,'r') as f:
            if 'meshes/1/cells/zoneTopology' not in f:return False
        with h5py.File(data,'r') as f:
            if 'results' not in f or 'settings' not in f:return False
        return case.stat().st_size>1_000_000 and data.stat().st_size>1_000_000
    except (OSError,ValueError):return False

class Supervisor:
    def __init__(self,args):
        self.args=args;LOG.mkdir(parents=True,exist_ok=True)
        self.lock=(LOG/'supervisor.lock').open('a+b')
        try:
            self.lock.seek(0)
            if self.lock.read(1)==b'':self.lock.write(b'0');self.lock.flush()
            self.lock.seek(0);msvcrt.locking(self.lock.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:
            self.lock.close();raise SystemExit('Existing supervisor holds the campaign lock; duplicate refused')
        self.s=read(STATE)
        if not self.s:
            self.s={'campaign':CAMPAIGN,'campaign_start_time':stamp(),
                    'target_hours':8,'hard_maximum_hours':12,'minimum_useful_hours':5,
                    'campaign_status':'ACTIVE','retry_count':0,'repairs_attempted':[],
                    'fluent_sessions_launched':0,'maximum_simultaneous_fluent_sessions':0,
                    'milestones':{},'checkpoint_inventory':[],
                    'physics_frozen_sha256':FROZEN_SHA,'production_processor_count':1,
                    'prior_parallel_stalls':'Two archived 4-core step-5 stalls; do not retry 4-core',
                    'next_action':'Adopt a progressing solver or restore the latest verified checkpoint'}
        self.start=datetime.fromisoformat(self.s['campaign_start_time'])
        self.s.update(supervisor_pid=os.getpid(),supervisor_created=psutil.Process().create_time(),
                      monitoring_interval_seconds=args.interval,execution_owner='Windows Task Scheduler')
        self.handles=[];self.last_cpu=None;self.last_signature=None;self.stall_since=None
        self.last_step=int(self.s.get('current_step',0));self.resource_low_cycles=0
        self.save();self.event('SUPERVISOR_STARTED',pid=os.getpid())

    def save(self):
        self.s.update(last_update_time=stamp(),elapsed_hours=(utc()-self.start).total_seconds()/3600,
                      hard_deadline=(self.start+timedelta(hours=12)).isoformat(),
                      target_deadline=(self.start+timedelta(hours=8)).isoformat())
        atomic(STATE,self.s)
    def event(self,kind,**detail):
        rec={'timestamp':stamp(),'event':kind,**detail}
        with (LOG/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(rec,allow_nan=False)+'\n')
        with (LOG/'supervisor.log').open('a',encoding='utf-8') as f:f.write(f'{rec["timestamp"]} {kind} {json.dumps(detail)}\n')
        print(json.dumps(rec),flush=True)
    def queue(self,job=None,config=None,action=None):
        if QUEUE.exists():return False
        atomic(QUEUE,{'action':action} if action else {'job':job,**({'config':config} if config else {})})
        return True
    def inventory(self):
        hist=rows();safe={int(r['step']):r for r in hist if r['mesh_motion_status']=='PASS'
                         and int(r['orphan_count'])==0 and int(r['receptors_without_donors'])==0
                         and float(r['robot_wall_clearance_m'])>=.0001
                         and float(r['minimum_cell_volume_m3'])>0 and abs(float(r['q_norm'])-1)<=1e-6
                         and all(math.isfinite(float(r[f'{p}_{a}_{u}'])) for p,u in [('Fmag','N'),('Tmag','Nm'),('omega','rad_s')] for a in 'xyz')}
        known={x['case']:x for x in self.s.get('checkpoint_inventory',[])}
        for item in known.values():
            item['verification']='Closed HDF5 headers + SHA256 + complete per-step gate'
            item.setdefault('native_restore_verification','PENDING_NATIVE_TIME_AND_POSE_CHECK')
        gate=read(E/'benchmark_C_fineA_final_static_gate.json')
        candidates=[(0,Path(gate['start_case']),Path(gate['start_data']))]
        for step in sorted(safe):
            if step%4==0:candidates.append((step,MESH/f'fielddata_checkpoint_{step:04d}.cas.h5',MESH/f'fielddata_checkpoint_{step:04d}.dat.h5'))
        report=read(FREE)
        if report.get('stop_checkpoint_case'):
            candidates.append((int(report['completed_time_steps']),Path(report['stop_checkpoint_case']),Path(report['stop_checkpoint_data'])))
        for step,case,data in candidates:
            if str(case) in known:continue
            if integrity(case,data):
                entry={'step':step,'time_s':step*25e-6,'case':str(case),'data':str(data),
                       'case_sha256':sha(case),'data_sha256':sha(data),'verified_at':stamp(),
                       'verification':'Closed HDF5 headers + SHA256 + complete per-step gate',
                       'native_restore_verification':'PENDING_NATIVE_TIME_AND_POSE_CHECK'}
                known[str(case)]=entry;self.event('CHECKPOINT_WRITTEN',step=step,case=str(case))
        items=sorted(known.values(),key=lambda x:x['step']);self.s['checkpoint_inventory']=items
        if items:self.s['last_successful_checkpoint']=items[-1]
        atomic(LOG/'checkpoint_inventory.json',items)
        return safe
    def snapshot(self,label):
        folder=LOG/'snapshots'/f'{utc():%Y%m%dT%H%M%SZ}_{label}';folder.mkdir(parents=True,exist_ok=True)
        for p in [FREE,HIST,SESSION,E/'benchmark_C_fineA_dynamic_failure.json']:
            if p.exists():shutil.copy2(p,folder/p.name)
        # Copy light actual FieldData/images before any reviewed repair can replace them.
        field=Path(read(FREE).get('fielddata_directory',''))
        if field.is_dir() and field.is_relative_to(E):
            saved=folder/'fielddata';saved.mkdir(exist_ok=True)
            for p in field.iterdir():
                if p.is_file() and p.suffix in ('.vtp','.json'):shutil.copy2(p,saved/p.name)
        renders=E/'benchmark_C_fineA_pyvista'
        if renders.exists():
            saved=folder/'renders';saved.mkdir(exist_ok=True)
            for p in renders.iterdir():
                if p.is_file() and p.suffix in ('.gif','.png'):shutil.copy2(p,saved/p.name)
        trans=Path(read(SESSION).get('transcript',''))
        if trans.is_file():shutil.copy2(trans,folder/trans.name)
        atomic(folder/'supervisor_state.json',self.s)
        self.s['latest_evidence_files']=str(folder);return folder
    def milestone(self,key,message,paths):
        if key in self.s['milestones']:return
        # Fixed task files only. Never stage unrelated changes or ignored native states.
        paths=[str(p.relative_to(ROOT)) for p in paths if p.is_file()]
        for p in paths:
            ignored=subprocess.run(['git','check-ignore','--quiet',p],cwd=ROOT).returncode==0
            if ignored:raise RuntimeError(f'Attempt to stage ignored artifact: {p}')
        if paths:subprocess.run(['git','add','--',*paths],cwd=ROOT,check=True)
        staged=subprocess.run(['git','diff','--cached','--name-only'],cwd=ROOT,capture_output=True,text=True,check=True).stdout.splitlines()
        if set(staged)-set(paths):raise RuntimeError('Unrelated staged changes exist; milestone commit refused')
        if staged:subprocess.run(['git','commit','-m',message],cwd=ROOT,check=True)
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        self.s['milestones'][key]={'commit':commit,'timestamp':stamp()};self.save()
        self.event('GATE_PASS',gate=key,commit=commit)
        self.push()
    def push(self):
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()
        if branch!='main':raise RuntimeError('Milestone push requires existing main branch')
        self.s['last_push_attempt']=stamp()
        try:
            p=subprocess.run(['git','-c','core.sshCommand=ssh -o ConnectTimeout=15','push','origin','main'],cwd=ROOT,capture_output=True,text=True,timeout=90)
        except subprocess.TimeoutExpired:
            self.s['push_pending']=True;self.event('PUSH_FAILED',error='SSH push timeout');return
        self.s['push_pending']=p.returncode!=0
        if p.returncode:self.event('PUSH_FAILED',error=p.stderr[-1000:])
        else:self.s['remote_head']=subprocess.check_output(['git','rev-parse','origin/main'],cwd=ROOT,text=True).strip()
    def resources_ready(self):
        resources={'available_ram_gib':psutil.virtual_memory().available/2**30,'disk_free_gib':shutil.disk_usage(ROOT).free/2**30}
        self.s['resource_check']=resources
        if resources['available_ram_gib']<self.args.minimum_ram_gib or resources['disk_free_gib']<35:
            self.s.update(campaign_status='RESOURCE_WAIT',next_action='Wait for safe available RAM/disk; no process will be killed')
            return False
        return True
    def launch(self,config=None,job='scripts/benchmark_C_fineA_free_6dof_run.py'):
        if native() or worker(read(SESSION)):raise RuntimeError('Existing Fluent/worker prevents another launch')
        if not self.resources_ready():self.save();return False
        if sha(ROOT/'fluent_udf/l2300_abaqus100hz_6dof.c')!=FROZEN_SHA:raise RuntimeError('Frozen main UDF changed')
        if QUEUE.exists():shutil.move(str(QUEUE),str(LOG/f'preserved_queue_{utc():%Y%m%dT%H%M%SZ}.json'))
        self.queue(job,config)
        index=self.s['fluent_sessions_launched']+1
        out=(LOG/f'worker_{index:02d}_stdout.log').open('a',encoding='utf-8');err=(LOG/f'worker_{index:02d}_stderr.log').open('a',encoding='utf-8')
        self.handles.extend([out,err])
        p=subprocess.Popen([sys.executable,'-u',str(ROOT/'scripts/benchmark_C_single_session.py'),'--processors','1'],cwd=ROOT,stdout=out,stderr=err,creationflags=subprocess.CREATE_NO_WINDOW)
        self.s.update(fluent_sessions_launched=index,worker_pid=p.pid,launch_pending_at=stamp(),campaign_status='ACTIVE')
        self.event('SOLVER_STARTED',worker_pid=p.pid,processors=1,resume_step=(config or {}).get('resume_step',0),job=job);self.save();return True
    def recovery(self,safe):
        # Resource waiting must never truncate history or count as a repair attempt.
        if not self.s.get('interrupted_run_archive'):
            folder=self.snapshot('interrupted_serial_run')
            self.s['interrupted_run_archive']=str(folder)
            self.s['last_failure']={'class':'WORKER_DISAPPEARED','timestamp':stamp(),'evidence':str(folder),
                                   'reason':'Worker and native Fluent disappeared after step 33, without a fatal transcript; cause unconfirmed'}
            self.event('SOLVER_CRASH',failure=self.s['last_failure'])
        if not self.resources_ready():return
        checkpoint=self.s.get('last_successful_checkpoint')
        if not checkpoint:raise RuntimeError('No verified checkpoint can be restored')
        step=checkpoint['step'];report=read(FREE)
        hist=rows();selected=[r for r in hist if int(r['step'])<=step]
        attempts=[r for r in self.s['repairs_attempted'] if r['failure_class']=='WORKER_DISAPPEARED']
        if attempts:
            self.request_review('WORKER_DISAPPEARED','Repeated disappearance after detached ownership; a distinct evidence-based repair is needed')
            return
        for key in ['case','data']:
            path=Path(checkpoint[key])
            if not integrity(Path(checkpoint['case']),Path(checkpoint['data'])) or sha(path)!=checkpoint[key+'_sha256']:
                raise RuntimeError('Checkpoint integrity/hash changed before restoration')
        folder=self.snapshot('before_checkpoint_restore')
        if len(hist)>len(selected):
            with HIST.open('w',newline='',encoding='utf-8') as f:
                w=csv.DictWriter(f,fieldnames=list(hist[0]));w.writeheader();w.writerows(selected)
        frames=[]
        field=Path(report['fielddata_directory'])
        for r in selected:
            n=int(r['step'])
            if n%4:continue
            frames.append({'step':n,'time_s':float(r['time_s']),'com_m':[float(r[f'com_{a}_m']) for a in 'xyz'],
                           'radial_displacement_m':float(r['radial_displacement_m']),'orphan_count':int(r['orphan_count']),
                           'robot_wall_dx_m':float(r['robot_wall_dx_m']),'component_bounds_delta_m':float(r['component_bounds_delta_m']),
                           'velocity_max_m_s':None,'checkpoint_case':str(MESH/f'fielddata_checkpoint_{n:04d}.cas.h5'),
                           'checkpoint_data':str(MESH/f'fielddata_checkpoint_{n:04d}.dat.h5')})
        cfg={'resume_step':step,'resume_case':checkpoint['case'],'resume_data':checkpoint['data'],
             'prior_report':report,'prior_frames':frames,
             'expected_com_m':[float(selected[-1][f'com_{a}_m']) for a in 'xyz'] if selected else [.0012060186937156343,0,0]}
        self.s['last_failure']={'class':'WORKER_DISAPPEARED','evidence':str(folder),
                               'root_cause':'Worker and native solver exited without final gate/fatal transcript; host termination suspected, unconfirmed',
                               'concrete_change':'Task Scheduler now owns the supervisor and worker independently of the chat executor'}
        if self.launch(cfg):
            self.s['repairs_attempted'].append({'failure_class':'WORKER_DISAPPEARED','change':'detached_scheduler_ownership',
                                              'evidence':str(folder),'checkpoint':checkpoint,'timestamp':stamp()})
            self.s['retry_count']+=1;self.event('REPAIR_APPLIED',repair='detached_scheduler_ownership');self.event('RESTARTED',step=step)
    def audit(self,hist):
        if not hist:return
        r=hist[-1];finite=all(math.isfinite(float(r[k])) for k in
             ['q0','q1','q2','q3','q_norm','com_x_m','com_y_m','com_z_m','vx_m_s','vy_m_s','vz_m_s']+
             [f'{p}_{a}_{u}' for p,u in [('omega','rad_s'),('Fmag','N'),('Tmag','Nm')] for a in 'xyz'])
        self.s.update(current_step=int(r['step']),current_simulation_time=float(r['time_s']),
                      latest_orphan_count=int(r['orphan_count']),latest_receptor_count=int(r['official_receptor_count']),
                      latest_donor_connectivity={'receptors_without_donors':int(r['receptors_without_donors']),'donors':int(r['official_donor_count'])},
                      latest_donor_length_ratio={k:float(r[f'donor_length_ratio_{k}']) for k in ['min','median','p95','max']},
                      latest_minimum_cell_volume=float(r['minimum_cell_volume_m3']),latest_physical_wall_clearance=float(r['robot_wall_clearance_m']),
                      latest_COM=[float(r[f'com_{a}_m']) for a in 'xyz'],latest_quaternion=[float(r[f'q{i}']) for i in range(4)],
                      latest_quaternion_norm=float(r['q_norm']),latest_linear_velocity=[float(r[f'v{a}_m_s']) for a in 'xyz'],
                      latest_angular_velocity=[float(r[f'omega_{a}_rad_s']) for a in 'xyz'])
        if not finite:self.s['last_failure']={'class':'NONFINITE_STATE'}
        if int(r['step'])>self.last_step:
            self.event('STEP_COMPLETE',step=int(r['step']),time_s=float(r['time_s']),orphans=int(r['orphan_count']))
            self.last_step=int(r['step'])
    def controlled_stop(self,reason):
        atomic(LOG/'stop_request.json',{'timestamp':stamp(),'reason':reason})
        if QUEUE.exists():shutil.move(str(QUEUE),str(LOG/f'cancelled_queue_{utc():%Y%m%dT%H%M%SZ}.json'))
        p=worker(read(SESSION))
        self.s['owned_stop_processes']=[{'pid':x.pid,'created':x.create_time()} for x in ([p]+p.children(recursive=True) if p else [])]
        self.queue(action='exit');self.s.update(campaign_status='STOPPING',stop_reason=reason,next_action='Finish current timestep, write distinct safe checkpoint, close owned solver')
        self.event('STOP_REQUESTED',reason=reason)
    def request_review(self,failure_class,reason):
        # The thread heartbeat performs diagnosis and can submit a distinct repair.
        if self.s.get('campaign_status')!='AWAITING_REPAIR_REVIEW':
            folder=self.snapshot('failure_'+failure_class.lower())
            self.s['last_failure']={'class':failure_class,'reason':reason,'evidence':str(folder),'timestamp':stamp()}
            self.event('GATE_FAIL',failure=self.s['last_failure'])
        self.s.update(campaign_status='AWAITING_REPAIR_REVIEW',next_action='Continuation agent: diagnose evidence, submit repair_plan.json or a documented hard blocker')
    def reviewed_action(self):
        plan_path=LOG/'repair_plan.json'
        if not plan_path.exists():return
        plan=read(plan_path)
        failure=self.s.get('last_failure',{})
        if plan.get('failure_timestamp')!=failure.get('timestamp'):
            raise RuntimeError('Repair plan does not match the current archived failure')
        if plan.get('decision')=='HARD_BLOCKER':
            if not plan.get('reason'):raise RuntimeError('Hard blocker requires a reason')
            self.s.update(campaign_status='HARD_BLOCKER_REQUIRES_REVIEW',hard_blocker_decision=plan);return
        if plan.get('decision') not in ('DIAGNOSE','REPAIR'):raise RuntimeError('Invalid continuation action')
        job=(ROOT/plan.get('job','')).resolve()
        if not job.is_relative_to(ROOT/'scripts') or not job.is_file():raise RuntimeError('Reviewed job must be an existing project script')
        if sha(job)!=plan.get('job_sha256'):raise RuntimeError('Reviewed job hash differs')
        if not plan.get('root_cause') or not plan.get('evidence_files'):raise RuntimeError('Action requires cause and archived evidence')
        for path in plan['evidence_files']:
            if not (ROOT/path).is_file():raise RuntimeError('Missing repair evidence')
        repairs=[r for r in self.s['repairs_attempted'] if r['failure_class']==failure['class']]
        if plan['decision']=='REPAIR':
            change=plan.get('configuration_sha256')
            if not change or not plan.get('concrete_change'):raise RuntimeError('Repair requires concrete change and configuration fingerprint')
            if len(repairs)>=2 or any(r.get('configuration_sha256')==change for r in repairs):
                self.s.update(campaign_status='HARD_BLOCKER_REQUIRES_REVIEW',next_action='Two repairs exhausted or repeated configuration refused');return
        info=read(SESSION);p=worker(info)
        if p:
            if info.get('status')!='READY' or QUEUE.exists():return
            if not self.queue(str(job.relative_to(ROOT)),plan.get('config')):return
        else:
            if native() or not self.launch(plan.get('config'),str(job.relative_to(ROOT))):return
        if plan['decision']=='REPAIR':
            for old in [E/'benchmark_C_fineA_finish_workflow.json',LOG/'visual_review.json']:
                if old.exists():shutil.move(str(old),str(LOG/f'previous_{time.time_ns()}_{old.name}'))
            self.s['repairs_attempted'].append({**plan,'failure_class':failure['class'],'timestamp':stamp()})
            self.s['retry_count']+=1;self.event('REPAIR_APPLIED',change=plan['concrete_change'])
        else:self.event('DIAGNOSTIC_STARTED',job=str(job))
        shutil.move(str(plan_path),str(LOG/f'action_{time.time_ns()}.json'))
        self.s.update(campaign_status='REVIEWED_ACTION_RUNNING',reviewed_job=str(job),reviewed_decision=plan['decision'])
    def tick(self):
        self.audit(rows());safe=self.inventory();info=read(SESSION);p=worker(info);processes=native();free=read(FREE);finish=read(E/'benchmark_C_fineA_finish_workflow.json')
        self.s.update(solver_status=info.get('status','UNKNOWN') if p else ('UNREGISTERED_LIVE_SOLVER' if processes else 'NOT_RUNNING'),Fluent_PID=[x['pid'] for x in processes],worker_pid=p.pid if p else None,
                      current_phase=finish.get('stage') if finish.get('status')=='RUNNING' else free.get('stage'),
                      available_ram_gib=psutil.virtual_memory().available/2**30)
        self.s['latest_git_commit']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        self.s['maximum_simultaneous_fluent_sessions']=max(self.s['maximum_simultaneous_fluent_sessions'],int(bool(processes)))
        if len([x for x in processes if x['name']=='fl2610.exe'])>1:raise RuntimeError('More than one solver host detected')
        if self.s['elapsed_hours']>=11.9 and self.s['campaign_status']!='STOPPING':self.controlled_stop('12-hour deadline')
        if self.s['campaign_status']=='STOPPING':
            if not p and not processes:
                status='TIME_BUDGET_EXHAUSTED' if self.s.get('stop_reason')=='12-hour deadline' else 'STOPPED_RESOURCE_BLOCKER'
                self.s.update(campaign_status=status,next_action='Preserved safe checkpoint; continuation requires resolving the recorded stop condition');return
            if self.s['elapsed_hours']>=12.25:
                # Only identities captured from our worker's tree; never unknown Fluent/apps.
                for identity in reversed(self.s.get('owned_stop_processes',[])):
                    try:
                        target=psutil.Process(identity['pid'])
                        if abs(target.create_time()-identity['created'])<.01:target.terminate()
                    except psutil.Error:pass
                self.event('ATOMIC_STOP_GRACE_EXHAUSTED',checkpoint=self.s.get('last_successful_checkpoint'),clean_shutdown=False)
                self.s.update(campaign_status='TIME_BUDGET_EXHAUSTED',next_action='Last previously verified checkpoint preserved; forced shutdown recorded');return
            return
        if self.s.get('push_pending') and (utc()-datetime.fromisoformat(self.s.get('last_push_attempt',stamp()))).total_seconds()>1800:self.push()
        if self.s['campaign_status']=='AWAITING_REPAIR_REVIEW':self.reviewed_action();return
        if self.s['campaign_status']=='REVIEWED_ACTION_RUNNING':
            if p and info.get('status')=='RUNNING':return
            if p and info.get('status')=='READY' and self.s.get('reviewed_decision')=='REPAIR' and sim_status(free)=='FREE_6DOF_SOLVED':
                self.s['campaign_status']='ACTIVE';return
            self.request_review(self.s['last_failure']['class'],'Reviewed diagnostic/repair finished; verify its output before advancing');return
        self.s['disk_free_gib']=shutil.disk_usage(ROOT).free/2**30
        self.resource_low_cycles=self.resource_low_cycles+1 if self.s['available_ram_gib']<1.5 else 0
        if p and self.resource_low_cycles>=2:self.controlled_stop('Unsafe available RAM persisted');return
        if p and self.s['disk_free_gib']<8:self.controlled_stop('Unsafe disk space');return
        sim=free.get('simulation_status',free.get('status'))
        if sim=='FREE_6DOF_SOLVED' and free.get('completed_time_steps')==80:
            self.milestone('BENCHMARK_C_FREE_6DOF_2MS_PASS','Pass Benchmark C free magnetic 6DOF',[FREE,HIST,E/'benchmark_C_fineA_free_6dof_solver_transcript.txt'])
        cutoff=read(E/'benchmark_C_fineA_cutoff_sensitivity.json')
        if cutoff.get('status')=='PASS':
            self.milestone('INERTIA_CUTOFF_NUMERICALLY_INSENSITIVE','Validate Benchmark C inertia cutoff sensitivity',[E/'benchmark_C_fineA_cutoff_sensitivity.json',E/'benchmark_C_fineA_free_6dof_history_cutoff51.csv'])
        if finish.get('status')=='FILES_VERIFIED_VISUAL_INSPECTION_PENDING':
            if not (LOG/'visual_review.json').exists():
                self.s.update(campaign_status='AWAITING_VISUAL_REVIEW',next_action='Codex continuation must inspect actual final PNG/GIF and write visual_review.json');return
            review=read(LOG/'visual_review.json')
            if review.get('status')!='PASS':self.s.update(campaign_status='HARD_BLOCKER_REQUIRES_REVIEW',last_failure=review);return
            self.milestone('FINAL_EVIDENCE','Add final Benchmark C off-screen evidence',[E/'benchmark_C_fineA_final_report.json',E/'benchmark_C_fineA_free_6dof.json',E/'benchmark_C_fineA_finish_workflow.json',*list((E/'benchmark_C_fineA_pyvista').glob('*.gif')),*list((E/'benchmark_C_fineA_pyvista').glob('*.png'))])
            if sim=='FREE_6DOF_SOLVED' and cutoff.get('status')=='PASS':
                if self.s.get('push_pending'):
                    self.s.update(campaign_status='FINALIZING',next_action='Verified milestones committed; retry pending push before completion')
                    if p:self.queue(action='exit')
                    return
                if p or processes:
                    self.s.update(campaign_status='FINALIZING',next_action='Verified milestones pushed; close owned solver before final summary')
                    self.queue(action='exit');return
                self.s.update(campaign_status='COMPLETE',next_action='Benchmark C, sensitivity, actual visualization and evidence complete; no Benchmark D')
                self.event('CAMPAIGN_COMPLETE');self.queue(action='exit');return
            self.request_review('DYNAMIC_GATE','Failed dynamic gate; partial actual images preserved. Classify exact pose/local resolution/transient connectivity')
            return
        if p:
            if self.s.get('last_adopted_worker_pid')!=p.pid:
                self.event('SOLVER_FOUND',worker_pid=p.pid,Fluent_PID=self.s['Fluent_PID'])
                self.s['last_adopted_worker_pid']=p.pid
            self.s['campaign_status']='ACTIVE'
            if info.get('status')=='READY' and sim in ('FREE_6DOF_SOLVED','FAIL') and not finish:
                self.queue('scripts/benchmark_C_fineA_finish_in_same_session.py')
            elif info.get('status')=='RUNNING' and info.get('job','').endswith('benchmark_C_fineA_free_6dof_run.py') and not QUEUE.exists():
                self.queue('scripts/benchmark_C_fineA_finish_in_same_session.py')
            elif info.get('status')=='READY' and finish.get('status')=='FAIL':
                self.request_review('POSTPROCESSING',finish.get('error','Finish workflow failed'));return
            elif info.get('status')=='READY' and info.get('last_job')=='ERROR' and sim not in ('FAIL','FREE_6DOF_SOLVED'):
                self.request_review('WORKER_JOB_ERROR',info.get('error','Worker job failed'));return
            trans=Path(info.get('transcript',''));sig=(self.s.get('current_step'),trans.stat().st_size if trans.is_file() else 0)
            cpu=sum(x['cpu'] for x in processes)
            # Sustained busy CPU is not evidence of a stall. Three indicators must agree.
            unchanged=sig==self.last_signature;idle=self.last_cpu is not None and cpu-self.last_cpu<1
            if unchanged and idle:self.stall_since=self.stall_since or time.monotonic()
            else:self.stall_since=None
            self.last_signature,self.last_cpu=sig,cpu
            self.s['progress_indicators']={'signature':sig,'native_cpu_seconds':cpu,'CPU_idle':idle,'unchanged':unchanged}
            if self.stall_since and time.monotonic()-self.stall_since>1800:
                folder=self.snapshot('suspected_stall');self.event('STALL_DETECTED',evidence=str(folder))
                self.request_review('SOLVER_STALL','No history/transcript progress and idle native CPU for >30 minutes; inspect before stop/restart')
            return
        if processes:
            self.s.update(campaign_status='HARD_BLOCKER_REQUIRES_REVIEW',next_action='Live solver without registered worker; do not start another solver');return
        pending=self.s.get('launch_pending_at')
        if pending and (utc()-datetime.fromisoformat(pending)).total_seconds()<240:return
        if sim=='FAIL':
            self.request_review('DYNAMIC_GATE',free.get('error','Dynamic gate failed'));return
        if sim=='FREE_6DOF_SOLVED':
            self.launch(job='scripts/benchmark_C_fineA_finish_in_same_session.py');return
        self.recovery(safe)
    def final(self):
        hist=rows();last=hist[-1] if hist else {};free=read(FREE)
        def peak(p,u):return max((norm(r,p,u) for r in hist),default=None)
        summary={'campaign':CAMPAIGN,'supervisor_status':self.s['campaign_status'],'start':self.s['campaign_start_time'],'end':stamp(),
                 'elapsed_hours':self.s['elapsed_hours'],'fluent_sessions_launched':self.s['fluent_sessions_launched'],
                 'maximum_simultaneous_fluent_sessions':self.s['maximum_simultaneous_fluent_sessions'],
                 'static_overset_gate':'BENCHMARK_C_OVERSET_RESOLUTION_PASS','production_mesh':str(MESH),
                 'dynamic_steps_completed':int(last.get('step',0)),'simulation_time_s':float(last.get('time_s',0)),
                 'free_6dof_2ms':self.s['milestones'].get('BENCHMARK_C_FREE_6DOF_2MS_PASS'),
                 'dynamic_orphan_peak':max((int(r['orphan_count']) for r in hist),default=None),
                 'minimum_physical_wall_clearance_m':min((float(r['robot_wall_clearance_m']) for r in hist),default=None),
                 'peak_donor_length_ratio':max((float(r['donor_length_ratio_max']) for r in hist),default=None),
                 'quaternion_norm_max_error':max((abs(float(r['q_norm'])-1) for r in hist),default=None),
                 'final_COM':self.s.get('latest_COM'),'final_orientation':self.s.get('latest_quaternion'),
                 'peak_angular_velocity_rad_s':peak('omega_','_rad_s'),'peak_Fmag_N':peak('Fmag_','_N'),'peak_Tmag_Nm':peak('Tmag_','_Nm'),
                 'cutoff_sensitivity':read(E/'benchmark_C_fineA_cutoff_sensitivity.json').get('status','NOT RUN'),
                 'gif':free.get('gif'),'velocity_gif':free.get('velocity_gif'),'final_frame':free.get('preview'),
                 'repairs_attempted':self.s['repairs_attempted'],'hard_blocker':self.s.get('last_failure'),
                 'evidence_root':str(LOG),'git_commits':self.s['milestones'],'remote_head':self.s.get('remote_head')}
        atomic(E/'benchmark_C_longrun_final.json',summary);atomic(LOG/'final_summary.json',summary)
    def run(self):
        try:
            while self.s['campaign_status'] not in TERMINAL:
                self.save();self.tick();self.save()
                atomic(LOG/'state_snapshots'/'latest.json',self.s)
                if self.args.once:break
                if self.s['campaign_status'] not in TERMINAL:time.sleep(self.args.interval)
        except Exception as exc:
            self.snapshot('supervisor_exception');self.s.update(campaign_status='HARD_BLOCKER_REQUIRES_REVIEW',last_failure={'class':'SUPERVISOR_EXCEPTION','error':repr(exc)},next_action='Inspect preserved supervisor exception; do not launch another Fluent');self.event('SUPERVISOR_ERROR',error=repr(exc));self.save();raise
        finally:
            self.save()
            if self.s['campaign_status'] in TERMINAL:
                if read(SESSION).get('status')=='READY' and worker(read(SESSION)):self.queue(action='exit')
                self.final()
            self.lock.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--interval',type=int,default=180)
    parser.add_argument('--minimum-ram-gib',type=float,default=22);parser.add_argument('--once',action='store_true')
    args=parser.parse_args()
    if not 120<=args.interval<=300:parser.error('Monitoring interval must be 2–5 minutes')
    Supervisor(args).run()
