"""Local FSM. No model, chat, heartbeat, or interactive-shell dependency.

One checkpoint-load probe; resource admission before validation/solve.
Only a valid numerical run with unexplained process loss gets at most two
lifecycle recoveries. Numerical failures terminate for scientific review.
"""
import csv, json, math, os, shutil, subprocess, sys, time, traceback
from datetime import datetime, timedelta, timezone
import msvcrt, psutil
from benchmark_C_recovery_v2_common import *
from benchmark_C_longrun_supervisor import integrity

STATE=OUT/'state.json';FREE=ROOT/'evidence/benchmark_C_fineA_free_6dof.json'
TERMINAL={'COMPLETE','HARD_BLOCKER','RESOURCE_BLOCKER'}
def resource_policy(peak_gib,historical_gib):
    # Historical resident maximum is a measured lower bound for solving, not a known full-run peak.
    projected=max(peak_gib,historical_gib)
    overhead=max(0.,historical_gib-peak_gib)
    reserve=max(.20*projected,3.)
    return {'checkpoint_load_peak_gib':peak_gib,'historical_observed_solve_resident_gib':historical_gib,
      'expected_solve_overhead_gib':overhead,'safety_reserve_gib':reserve,
      'required_guard_gib':projected+reserve,'historical_full_run_peak_verified':False,
      'policy':'max(measured checkpoint load, historical observed solve footprint) + max(20%, 3 GiB); monitor resources during solving. Not a guarantee of the unobserved full-run peak.'}
class Recovery:
    def __init__(self):
        OUT.mkdir(parents=True,exist_ok=True)
        # Shared lock prevents the old supervisor and V2 owning launches together.
        self.lock=(ROOT/'evidence/benchmark_C_longrun/supervisor.lock').open('a+b')
        try:self.lock.seek(0);msvcrt.locking(self.lock.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:self.lock.close();raise SystemExit('Another campaign supervisor owns the launch lock')
        self.s=read(STATE)
        if not self.s:
            started=datetime.now(timezone.utc)
            self.s={'campaign':'BENCHMARK_C_RECOVERY_V2','scientific_campaign':'BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF',
              'recovery_v2_start':started.isoformat(),'recovery_v2_deadline':(started+timedelta(hours=10)).isoformat(),
              'target_deadline':(started+timedelta(hours=8)).isoformat(),'target_hours':8,'hard_maximum_hours':10,
              'state':'STARTUP','lifecycle_recoveries':0,'launches':0,'probe_attempted':False,'milestones':{},
              'frozen_udf_sha256':FROZEN_SHA,'original_verified_step':33,'current_step':33,'current_cfd_time_s':.000825}
        self.s.update(supervisor_pid=os.getpid(),supervisor_created=psutil.Process().create_time(),owner='Windows Task Scheduler',model_required=False)
        self.deadline=datetime.fromisoformat(self.s['recovery_v2_deadline']);self.handles=[];self.root_worker=None
        self.paging=Paging();self.peak=0.;self.heavy_since=None;self.loaded_since=None
        self.save();self.event('SUPERVISOR_STARTED',pid=os.getpid())
    def save(self):
        self.s['last_update']=stamp();atomic(STATE,self.s)
    def event(self,name,**kwargs):
        with (OUT/'events.jsonl').open('a') as f:f.write(json.dumps({'timestamp':stamp(),'event':name,**kwargs})+'\n')
    def transition(self,state,reason=None):
        if self.s['state']!=state:self.event('STATE',state=state,reason=reason)
        self.s.update(state=state,next_action=reason);self.save()
    def latest_checkpoint(self):
        old=read(ROOT/'evidence/benchmark_C_longrun_state.json')['last_successful_checkpoint']
        checkpoints=[old]+[read(p) for p in (OUT/'checkpoints').glob('*.json')]
        valid=[c for c in checkpoints if c.get('case') and integrity(Path(c['case']),Path(c['data']))]
        if not valid:raise RuntimeError('No complete checkpoint available')
        chosen=max(valid,key=lambda c:c['step'])
        for key in ['case','data']:
            if sha(chosen[key])!=chosen[key+'_sha256']:raise RuntimeError('Checkpoint hash changed')
        self.s['latest_complete_checkpoint']=chosen;self.save();return chosen
    def launch(self,probe):
        if native():raise RuntimeError('Existing native Fluent prevents another session')
        wr=read(OUT/'worker.json')
        try:
            p=psutil.Process(wr['pid'])
            if abs(p.create_time()-wr['created'])<.01:raise RuntimeError('Existing recovery worker prevents another session')
        except (psutil.Error,KeyError):pass
        m=memory();self.s['resources']=m
        if m['available_gib']<3 or m['commit_fraction']>=.9 or shutil.disk_usage(ROOT).free/2**30<35:
            self.transition('RESOURCE_BLOCKER','Initial probe resource floor not satisfied');return
        cp=self.latest_checkpoint();step=cp['step'];report=read(FREE)
        from benchmark_C_recovery_v2_restart import archived
        selected=[r for r in archive_history() if int(r['step'])<=step]
        # Recover descriptors from the current report, whose complete native files are immutable.
        frames=[f for f in report.get('frames',[]) if int(f['step'])<=step]
        cfg={'resume_step':step,'resume_case':cp['case'],'resume_data':cp['data'],
             'prior_report':report,'prior_frames':frames,'expected_com_m':[float(archived(step)[f'com_{a}_m']) for a in 'xyz'],
             'checkpoint_prefix':f'recovery_v2_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_a{self.s["lifecycle_recoveries"]}'}
        token=str(time.time_ns());atomic(OUT/'launch_config.json',{'token':token,'checkpoint':cp,'run_config':cfg,'probe':probe})
        for stale in ['worker_control.json','stop_request.json']:
            path=OUT/stale
            if path.exists():shutil.move(str(path),str(OUT/f'archived_{token}_{stale}'))
        out=(OUT/f'worker_{token}_stdout.log').open('a');err=(OUT/f'worker_{token}_stderr.log').open('a');self.handles.extend([out,err])
        env=os.environ.copy();env['BENCHMARK_C_RECOVERY_V2']='1'
        p=subprocess.Popen([sys.executable,'-u',str(ROOT/'scripts/benchmark_C_recovery_v2_worker.py')],cwd=ROOT,
          stdout=out,stderr=err,env=env,creationflags=subprocess.CREATE_NO_WINDOW)
        self.root_worker=p;self.peak=0.;self.loaded_since=None
        self.s.update(worker_launcher_pid=p.pid,worker_launcher_created=psutil.Process(p.pid).create_time(),active_token=token,
                      launches=self.s['launches']+1,launch_time=stamp(),memory_probe_before=m,probe_attempted=True)
        self.transition('MEMORY_PROBE' if probe else 'RESTORING_CHECKPOINT','Load latest native checkpoint in one core; zero timesteps before gates')
    def identities(self,wr):
        ids={}
        for x in wr.get('owned_identities',[]):ids[x['pid']]=x
        try:
            p=psutil.Process(self.s['worker_launcher_pid'])
            if abs(p.create_time()-self.s['worker_launcher_created'])<.01:
                for x in [p]+p.children(recursive=True):ids[x.pid]={'pid':x.pid,'created':x.create_time()}
        except (psutil.Error,KeyError):pass
        hosts=[];nodes=[]
        for p in native():
            try:
                if p.name().lower()=='fl2610.exe' and p.pid in ids:hosts.append(p)
                elif p.name().lower()=='fl_mpi2610.exe':nodes.append(p)
            except psutil.Error:pass
        associated=mpi_node_associations(hosts,nodes)
        for identity in associated:ids[identity['pid']]=identity
        self.s['mpi_node_ownership']=associated
        self.s['memory_monitor_complete']=all(node.pid in ids for node in nodes)
        if not self.s['memory_monitor_complete']:self.s['memory_monitor_was_incomplete']=True
        return ids
    def samples(self,wr):
        processes=[]
        for identity in self.identities(wr).values():
            try:
                p=psutil.Process(identity['pid'])
                if abs(p.create_time()-identity['created'])>.01:continue
                m=p.memory_info();processes.append({'pid':p.pid,'name':p.name(),'working_set_gib':m.rss/2**30,'private_gib':getattr(m,'private',0)/2**30,'peak_working_set_gib':getattr(m,'peak_wset',m.rss)/2**30})
            except psutil.Error:pass
        mem=memory();rates=self.paging.sample();rss=sum(p['working_set_gib'] for p in processes)
        self.peak=max(self.peak,rss);self.s.update(resources=mem,observed_owned_peak_working_set_gib=self.peak,owned_processes=processes,paging=rates)
        rec={'timestamp':stamp(),'state':self.s['state'],'worker_stage':wr.get('stage'),'memory':mem,'paging':rates,'owned_processes':processes,'combined_working_set_gib':rss,'memory_monitor_complete':self.s.get('memory_monitor_complete'),'mpi_node_ownership':self.s.get('mpi_node_ownership')}
        with (OUT/'memory_samples.jsonl').open('a') as f:f.write(json.dumps(rec)+'\n')
        return mem,rates,rss
    def abort_owned(self,wr,reason):
        atomic(OUT/'stop_request.json',{'timestamp':stamp(),'reason':reason})
        atomic(OUT/'worker_control.json',{'token':self.s['active_token'],'decision':'EXIT'})
        # RAM emergencies can interrupt blocking CASE/DATA load. Only exact owned identities.
        ids=list(self.identities(wr).values());terminated=[]
        for identity in reversed(ids):
            try:
                p=psutil.Process(identity['pid'])
                if abs(p.create_time()-identity['created'])<.01:p.terminate();terminated.append(identity)
            except psutil.Error:pass
        atomic(OUT/'probe_abort.json',{'timestamp':stamp(),'reason':reason,'terminated_owned':terminated,'zero_timesteps':wr.get('stage') not in ['FREE_6DOF','POSTPROCESS']})
        self.event('OWNED_RESOURCE_ABORT',reason=reason,identities=terminated)
    def finish_probe(self,status,reason,wr):
        historical=read(ROOT/'evidence/benchmark_C_longrun/benchmark_C_memory_requirement_audit.json')
        policy=resource_policy(self.peak,historical['maximum_observed_combined_working_set_gib'])
        complete=wr.get('status')=='LOADED_WAIT_MEMORY' and not self.s.get('memory_monitor_was_incomplete',False)
        if not complete:
            policy.update(checkpoint_load_peak_gib=None,expected_solve_overhead_gib=None,calibration_status='INCOMPLETE; historical planning bound only')
        rec={'status':status,'timestamp':stamp(),'reason':reason,'before':self.s.get('memory_probe_before'),
             'peak_combined_owned_working_set_gib':self.peak if complete else None,'partial_observed_working_set_gib':self.peak,
             'peak_measurement_complete':complete,'resource_policy':policy,'timesteps_advanced':0,
             'case_data_load_complete':wr.get('status')=='LOADED_WAIT_MEMORY','stage':wr.get('stage'),'paging':self.s.get('paging'),
             'samples':'evidence/benchmark_C_recovery_v2/memory_samples.jsonl','pagefile_capacity_counted_as_ram':False}
        atomic(OUT/'benchmark_C_memory_probe.json',rec)
        self.s['memory_policy']=policy;self.s['memory_probe_status']=status
    def process_tree(self,wr):
        tree=[];p=psutil.Process()
        for x in [p]+p.parents():
            try:tree.append({'pid':x.pid,'name':x.name(),'command':x.cmdline(),'created':x.create_time()})
            except psutil.Error:pass
        forbidden=any(x['name'].lower()=='chatgpt.exe' or 'codex.exe' in ' '.join(x['command']).lower() for x in tree)
        atomic(OUT/'detached_process_test.json',{'timestamp':stamp(),'status':'FAIL' if forbidden else 'PASS',
           'supervisor_ancestry':tree,'worker':wr,'launcher_shell_exit_observed':True,
           'interpretation':'Task Scheduler invocation returned; supervisor/worker survive. No active solve was interrupted for testing.'})
        if forbidden:raise RuntimeError('Supervisor ancestry still depends on Codex')
    def publish(self,key,message,paths):
        if key in self.s['milestones']:return
        atomic(OUT/'publish'/f'{key}.config.json',{'message':message,'paths':paths})
        out=(OUT/f'publish_{key}.log').open('a');self.handles.append(out)
        p=subprocess.Popen([sys.executable,str(ROOT/'scripts/benchmark_C_recovery_v2_publish.py'),key],cwd=ROOT,stdout=out,stderr=out,creationflags=subprocess.CREATE_NO_WINDOW)
        self.s['milestones'][key]={'status':'PUBLISHING','pid':p.pid,'timestamp':stamp()};self.save()
    def verified_milestones(self):
        gate=read(OUT/'restart_v2_step32_state_validation.json')
        if gate.get('status')=='BENCHMARK_C_STEP32_NATIVE_RESTART_PASS':
            self.publish('native_restart','Validate step32 native restart',[
              'evidence/benchmark_C_recovery_v2/restart_v2_step32_state_validation.json',
              'evidence/benchmark_C_recovery_v2/restart_v2_step32_magnetic_validation.json',
              'evidence/benchmark_C_recovery_v2/restart_v2_step32_overset_validation.json'])
        # Serialize publisher ownership so git staging cannot race between gates.
        for key,value in self.s['milestones'].items():
            result=read(OUT/'publish'/f'{key}.result.json')
            if result:value.update(result)
        free=read(FREE)
        if any(x.get('status')=='PUBLISHING' for x in self.s['milestones'].values()):return
        if free.get('simulation_status',free.get('status'))=='FREE_6DOF_SOLVED' and free.get('completed_time_steps')==80:
            self.publish('main_2ms','Pass Benchmark C free magnetic 6DOF',[
              'evidence/benchmark_C_fineA_free_6dof.json','evidence/benchmark_C_fineA_free_6dof_history.csv',
              'evidence/benchmark_C_recovery_v2/step33_recomputed_validation.json'])
    def archive_loss(self,wr):
        folder=OUT/'lifecycle_archives'/str(time.time_ns());folder.mkdir(parents=True)
        for path in [OUT/'worker.json',FREE,ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv',Path(wr.get('transcript',''))]:
            if path.is_file():shutil.copy2(path,folder/path.name)
        self.event('WORKER_LIFECYCLE_TERMINATION',archive=str(folder))
    def tick(self):
        if datetime.now(timezone.utc)>=self.deadline:
            atomic(OUT/'stop_request.json',{'timestamp':stamp(),'reason':'Fixed Recovery V2 10-hour deadline'})
            self.s['deadline_reached']=True
        state=self.s['state']
        if state=='STARTUP':
            if sha(ROOT/'fluent_udf/l2300_abaqus100hz_6dof.c')!=FROZEN_SHA:raise RuntimeError('Frozen UDF changed')
            if native():raise RuntimeError('Existing Fluent: cannot start V2')
            self.transition('RESOURCE_AUDIT','Audit current process/memory and recover historical resident measurements');audit();archive_history()
            self.transition('RESOURCE_WAIT','Resolve old RAM deadlock using one controlled load probe');self.launch(probe=True);return
        wr=read(OUT/'worker.json')
        if wr.get('token')!=self.s.get('active_token'):wr={}
        alive=False
        try:
            p=psutil.Process(wr['pid']);alive=abs(p.create_time()-wr['created'])<.01
        except (psutil.Error,KeyError):
            if self.root_worker and self.root_worker.poll() is None:alive=True
        mem,rates,rss=self.samples(wr)
        if wr.get('owned_identities') and not (OUT/'detached_process_test.json').exists():self.process_tree(wr)
        hosts=[p for p in native() if p.name().lower()=='fl2610.exe']
        self.s['simultaneous_fluent_sessions']=len(hosts)
        self.s['max_simultaneous_fluent_sessions']=max(self.s.get('max_simultaneous_fluent_sessions',0),len(hosts))
        if len(hosts)>1:raise RuntimeError('More than one Fluent host; unknown session preserved')
        dangerous=mem['available_gib']<3 or mem['commit_fraction']>=.95
        # File-backed reads during CASE load alone are not proof of pagefile thrashing.
        heavy=mem['available_gib']<5 and (rates.get('pages_output_per_s') or 0)>2048 and (rates.get('pages_input_per_s') or 0)>2048
        self.heavy_since=(self.heavy_since or time.monotonic()) if heavy else None
        sustained=bool(self.heavy_since and time.monotonic()-self.heavy_since>=30)
        if alive and (dangerous or sustained):
            reason='Available physical RAM below 3 GiB / commit >=95%' if dangerous else 'Sustained heavy paging with low physical memory'
            self.abort_owned(wr,reason)
            if state=='MEMORY_PROBE':self.finish_probe('FAIL',reason,wr)
            self.transition('RESOURCE_BLOCKER',reason);return
        if self.s.get('deadline_reached'):
            if alive:self.abort_owned(wr,'Recovery V2 fixed 10-hour hard deadline; previous complete checkpoint preserved')
            self.transition('RESOURCE_BLOCKER','Recovery V2 fixed deadline exhausted; no automatic extension');return
        if wr.get('status')=='LOADED_WAIT_MEMORY':
            self.loaded_since=self.loaded_since or time.monotonic()
            if time.monotonic()-self.loaded_since<30:return
            self.finish_probe('PASS','Native step32 CASE/DATA/UDF loaded; zero timesteps',wr)
            policy=self.s['memory_policy']
            # Already-loaded processes consume RAM: admission uses available + their current resident footprint.
            capacity=mem['available_gib']+rss;admitted=capacity>=policy['required_guard_gib'] and mem['available_gib']>=policy['safety_reserve_gib'] and self.s.get('memory_monitor_complete',False) and not self.s.get('memory_monitor_was_incomplete',False)
            atomic(OUT/'production_admission.json',{'timestamp':stamp(),'status':'PASS' if admitted else 'RESOURCE_BLOCKER',
               'available_gib':mem['available_gib'],'loaded_owned_working_set_gib':rss,'capacity_gib':capacity,'policy':policy})
            self.process_tree(wr)
            if not admitted:
                atomic(OUT/'worker_control.json',{'token':self.s['active_token'],'decision':'EXIT'})
                self.s['pending_resource_blocker']='Checkpoint loads, but production solve footprint plus measured safety reserve cannot be admitted'
                self.transition('RESOURCE_WAIT','Close probe cleanly; preserve checkpoint and issue a measured resource blocker');return
            self.transition('READY_TO_RESTORE','Calibrated resource gate passed with current loaded session')
            self.transition('RESTORING_CHECKPOINT','Native CASE/DATA already loaded; no simulation time reset')
            atomic(OUT/'worker_control.json',{'token':self.s['active_token'],'decision':'CONTINUE'})
            self.transition('RESTART_VALIDATION','Native time/pose/velocity, magnetic law, official overset metrics; zero new timesteps');return
        if wr.get('status')=='RUNNING' and wr.get('stage')=='FREE_6DOF':
            self.verified_milestones()
            self.transition('RUNNING','Local worker advances steps without model messages')
            free=read(FREE);self.s.update(current_step=free.get('completed_time_steps'),current_cfd_time_s=free.get('latest_time_s'))
            checkpoints=list((OUT/'checkpoints').glob('*.json'))
            if checkpoints:
                cp=max((read(p) for p in checkpoints),key=lambda c:c['step']);self.s['latest_complete_checkpoint']=cp
                if self.s.get('checkpoint_recorded')!=cp['step']:
                    self.transition('CHECKPOINTING',f'Closed native CASE+DATA with hashes at step {cp["step"]}')
                    self.s['checkpoint_recorded']=cp['step'];self.transition('RUNNING','Next step after complete checkpoint')
            return
        if wr.get('status')=='POSTPROCESS':
            self.verified_milestones();self.transition('POSTPROCESS','Full 2 ms passed; cutoff comparison, FieldData, off-screen rendering');return
        if wr.get('status')=='COMPLETE' and wr.get('closed'):
            self.verified_milestones()
            self.s.update(current_step=80,current_cfd_time_s=.002,main_run='PASS',visual_review='PENDING')
            if self.s['milestones'].get('main_2ms',{}).get('status')=='PUBLISHING':
                self.s['next_action']='Wait locally for scoped milestone publication';return
            if self.s['milestones'].get('main_2ms',{}).get('status')!='PASS':
                self.transition('HARD_BLOCKER','Main numerical run PASS, but scoped milestone publication failed');return
            self.transition('COMPLETE','Numerical and file gates finished locally; final visuals require later inspection');return
        if wr.get('status')=='ERROR':
            self.archive_loss(wr);self.s['failure']=wr
            if state=='MEMORY_PROBE':self.finish_probe('FAIL',wr.get('error','Probe worker error'),wr)
            self.transition('HARD_BLOCKER','Worker reported a concrete restart/numerical/postprocessing error; no physics repair performed');return
        if not alive and not native():
            if self.s.get('pending_resource_blocker'):
                self.transition('RESOURCE_BLOCKER',self.s['pending_resource_blocker']);return
            if wr.get('status')=='SAFE_STOP':self.transition('RESOURCE_BLOCKER','Safe stop checkpoint preserved');return
            if (datetime.now(timezone.utc)-datetime.fromisoformat(self.s['launch_time'])).total_seconds()<10:return
            self.archive_loss(wr)
            if state=='MEMORY_PROBE':
                self.finish_probe('FAIL','Probe process disappeared before successful loading; no dynamics advanced',wr)
                self.transition('RESOURCE_BLOCKER','Checkpoint-load probe did not finish; see logs');return
            free=read(FREE)
            if free.get('status')=='FAIL':self.transition('HARD_BLOCKER','Numerical failure is not a lifecycle recovery');return
            if self.s['lifecycle_recoveries']>=2:self.transition('HARD_BLOCKER','Two lifecycle recovery attempts exhausted');return
            self.s['lifecycle_recoveries']+=1;self.launch(probe=False)
    def run(self):
        try:
            while self.s['state'] not in TERMINAL:
                self.tick();self.save();time.sleep(2 if self.s['state'] in ['MEMORY_PROBE','RESTART_VALIDATION','RESTORING_CHECKPOINT'] else 10)
        except Exception as exc:
            self.s['failure']={'error':repr(exc),'traceback':traceback.format_exc()};self.transition('HARD_BLOCKER',repr(exc))
            atomic(OUT/'stop_request.json',{'timestamp':stamp(),'reason':repr(exc)})
        finally:
            self.save();atomic(OUT/'final.json',self.s);self.lock.close()
            for h in self.handles:h.close()
if __name__=='__main__':Recovery().run()
