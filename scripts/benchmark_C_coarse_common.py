"""Scoped coarse campaign state, resource samples and frozen-file provenance."""
import os,threading,time
from pathlib import Path
import psutil
from benchmark_C_recovery_v2_common import atomic,read,sha,stamp,memory,native,mpi_node_associations,FROZEN_SHA
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'live_cases/benchmark_C_coarse_A';EVID=ROOT/'evidence/benchmark_C_coarse_A'
COM=[.0012060186937156343,0,0]
AXIS=[.04129325489475344,-.3684752565834513,-.928720007529695]
ORTH=[0,.9295128170778901,-.3687898085467175]

def preserve():
    target=EVID/'frozen_provenance.json'
    files=['fluent_udf/l2300_abaqus100hz_6dof.c','evidence/benchmark_C_fineA_final_static_gate.json',
        'evidence/benchmark_C_fineA_free_6dof_history.csv',
        'live_cases/benchmark_C_fineA/bg020/fielddata_checkpoint_0032.cas.h5',
        'live_cases/benchmark_C_fineA/bg020/fielddata_checkpoint_0032.dat.h5']
    if not target.exists():atomic(target,{'timestamp':stamp(),'campaign':'BENCHMARK_C_COARSE_SCREENING','files':{f:sha(ROOT/f) for f in files},'fine_recovery_scheduler_disabled':True,'old_time_budget_reset':False})
    recorded=read(target)
    if any(sha(ROOT/f)!=h for f,h in recorded['files'].items()):raise RuntimeError('Frozen fine/UDF evidence changed')
    if recorded['files'][files[0]]!=FROZEN_SHA:raise RuntimeError('Frozen UDF hash changed')

class MemoryProfile:
    def __init__(self):
        self.stage='before_Fluent_launch';self.solver=None;self.rows=[];self.stop=threading.Event();self.abort=None
        self.lock=threading.RLock()
        self.thread=threading.Thread(target=self.monitor,daemon=True)
    def sample(self,stage=None):
        with self.lock:return self._sample(stage)
    def _sample(self,stage=None):
        if stage:self.stage=stage
        worker=psutil.Process();owned={p.pid:p for p in [worker]+worker.children(recursive=True)}
        hosts=[]
        if self.solver is not None:
            cp=self.solver.connection_properties
            for pid in [cp.fluent_host_pid,cp.cortex_pid]:
                try:
                    p=psutil.Process(pid);owned[p.pid]=p
                    for c in p.children(recursive=True):owned[c.pid]=c
                    if p.name().lower()=='fl2610.exe':hosts.append(p)
                except (psutil.Error,TypeError):pass
            for x in mpi_node_associations(hosts,native()):
                try:owned[x['pid']]=psutil.Process(x['pid'])
                except psutil.Error:pass
        processes=[]
        for p in owned.values():
            try:
                processes.append({'pid':p.pid,'created':p.create_time(),'name':p.name(),'working_set_gib':p.memory_info().rss/2**30,'command':p.cmdline()})
            except psutil.Error:pass
        row={**memory(),'stage':self.stage,'processes':processes,
            'Fluent_working_set_gib':sum(p['working_set_gib'] for p in processes if p['name'].lower() in ['fl2610.exe','fl_mpi2610.exe','cx2610.exe']),
            'Python_worker_working_set_gib':worker.memory_info().rss/2**30,
            'total_project_working_set_gib':sum(p['working_set_gib'] for p in processes)}
        self.rows.append(row)
        peak=max(r['total_project_working_set_gib'] for r in self.rows)
        if row['available_gib']<3 or row['commit_fraction']>.95 or row['total_project_working_set_gib']>15:
            self.abort={'reason':'COARSE_RESOURCE_LIMIT','sample':row}
        atomic(EVID/'benchmark_C_coarse_memory_profile.json',{'timestamp':stamp(),'peak_project_working_set_gib':peak,'sample_interval_s':5,'rows':self.rows,'abort':self.abort,'resident_RAM_note':'Sum of measured owned working sets; shared pages may be counted more than once.'})
        return row
    def monitor(self):
        while not self.stop.wait(5):
            try:self.sample()
            except Exception as e:atomic(EVID/'memory_sampler_error.json',{'timestamp':stamp(),'error':repr(e)})
    def start(self):self.sample();self.thread.start()
    def check(self,stage):
        self.stage=stage
        if self.abort:raise RuntimeError(str(self.abort['reason']))
    def close(self):self.stop.set();self.thread.join(timeout=10);self.sample('after_Fluent_exit')

def state(status,**data):
    prior=read(EVID/'state.json');prior.update(campaign='BENCHMARK_C_COARSE_SCREENING',status=status,timestamp=stamp(),pid=os.getpid(),**data);atomic(EVID/'state.json',prior)
