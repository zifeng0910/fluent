"""Local recovery evidence and Windows resource measurements; no Fluent launch."""
from __future__ import annotations
import csv, ctypes, hashlib, json, os, re, time
from ctypes import wintypes
from datetime import datetime, timezone
from pathlib import Path
import psutil

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'evidence/benchmark_C_recovery_v2'
FROZEN_SHA='0ae61d4e84207ced13dad7f29517212f86ca0aa0281fb423166637aec881dd6e'
def stamp():return datetime.now(timezone.utc).isoformat()
def read(p,default=None):
    try:return json.loads(Path(p).read_text(encoding='utf-8-sig'))
    except (OSError,ValueError):return {} if default is None else default
def atomic(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    t=p.with_name(p.name+f'.{os.getpid()}.tmp')
    t.write_text(json.dumps(x,indent=2,allow_nan=False,default=str),encoding='utf-8');t.replace(p)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024**2),b''):h.update(chunk)
    return h.hexdigest()
class Performance(ctypes.Structure):
    _fields_=[('cb',wintypes.DWORD)]+[(n,ctypes.c_size_t) for n in
      ['CommitTotal','CommitLimit','CommitPeak','PhysicalTotal','PhysicalAvailable','SystemCache','KernelTotal','KernelPaged','KernelNonpaged','PageSize']]+[(n,wintypes.DWORD) for n in ['HandleCount','ProcessCount','ThreadCount']]
def memory():
    x=Performance();x.cb=ctypes.sizeof(x)
    if not ctypes.windll.psapi.GetPerformanceInfo(ctypes.byref(x),x.cb):raise ctypes.WinError()
    vm=psutil.virtual_memory();sw=psutil.swap_memory()
    return dict(timestamp=stamp(),physical_total_gib=vm.total/2**30,available_gib=vm.available/2**30,
      used_gib=vm.used/2**30,commit_gib=x.CommitTotal*x.PageSize/2**30,
      commit_limit_gib=x.CommitLimit*x.PageSize/2**30,commit_fraction=x.CommitTotal/x.CommitLimit,
      pagefile_used_gib=sw.used/2**30,pagefile_total_gib=sw.total/2**30,
      kernel_nonpaged_gib=x.KernelNonpaged*x.PageSize/2**30)
class Paging:
    """English PDH counters, even on a localized Windows installation."""
    def __init__(self):
        self.api=ctypes.WinDLL('pdh');self.query=wintypes.HANDLE();self.counters={}
        self.api.PdhOpenQueryW.argtypes=[wintypes.LPCWSTR,ctypes.c_size_t,ctypes.POINTER(wintypes.HANDLE)]
        self.api.PdhAddEnglishCounterW.argtypes=[wintypes.HANDLE,wintypes.LPCWSTR,ctypes.c_size_t,ctypes.POINTER(wintypes.HANDLE)]
        self.api.PdhCollectQueryData.argtypes=[wintypes.HANDLE]
        if self.api.PdhOpenQueryW(None,0,ctypes.byref(self.query)):return
        for name,path in {'page_reads_per_s':r'\Memory\Page Reads/sec','pages_input_per_s':r'\Memory\Pages Input/sec','pages_output_per_s':r'\Memory\Pages Output/sec'}.items():
            c=wintypes.HANDLE()
            if not self.api.PdhAddEnglishCounterW(self.query,path,0,ctypes.byref(c)):self.counters[name]=c
        self.api.PdhCollectQueryData(self.query)
    def sample(self):
        class Value(ctypes.Structure):_fields_=[('status',wintypes.DWORD),('value',ctypes.c_double)]
        self.api.PdhGetFormattedCounterValue.argtypes=[wintypes.HANDLE,wintypes.DWORD,ctypes.POINTER(wintypes.DWORD),ctypes.POINTER(Value)]
        self.api.PdhCollectQueryData(self.query);out={}
        for name,c in self.counters.items():
            v=Value();kind=wintypes.DWORD();rc=self.api.PdhGetFormattedCounterValue(c,0x200,ctypes.byref(kind),ctypes.byref(v))
            out[name]=v.value if rc==0 and v.status in (0,1) else None
        return out
def native():
    return [p for p in psutil.process_iter(['pid','name']) if (p.info['name'] or '').lower() in ('fl2610.exe','fl_mpi2610.exe')]
def mpi_node_associations(hosts,nodes):
    """Intel MPI may spawn through a Windows service, outside the worker tree.

    Associate only an mport connected to a registered Fluent host's TCP port.
    Temporal proximity alone is not process ownership evidence.
    """
    host_ports={}
    for host in hosts:
        try:
            host_ports[host.pid]={c.laddr.port for c in host.net_connections(kind='tcp') if c.laddr}
        except psutil.Error:pass
    matches=[]
    for node in nodes:
        try:
            cmd=node.cmdline()
            if '-mport' not in cmd:continue
            endpoint=cmd[cmd.index('-mport')+1];port=int(endpoint.split(':')[-2])
            owners=[pid for pid,ports in host_ports.items() if port in ports]
            if len(owners)==1:matches.append({'pid':node.pid,'created':node.create_time(),'registered_host_pid':owners[0],'mport':endpoint,'ownership_evidence':'Node -mport matches registered Fluent host local TCP port'})
        except (psutil.Error,ValueError,IndexError):pass
    return matches
def audit():
    before=memory();records=[]
    required=set();old=read(ROOT/'evidence/benchmark_C_longrun_state.json')
    for pid in [os.getpid(),old.get('supervisor_pid')]:
        try:
            p=psutil.Process(pid);required.add(p.pid);required.update(a.pid for a in p.parents());required.update(a.pid for a in p.children(recursive=True))
        except (psutil.Error,TypeError):pass
    for p in psutil.process_iter():
        try:
            m=p.memory_info();cmd=' '.join(p.cmdline());name=p.name()
            classification='CURRENT_CAMPAIGN_REQUIRED' if p.pid in required else 'UNKNOWN' if ('h:\\fluent' in cmd.lower() or name.lower() in ('cx2610.exe','fl2610.exe','fl_mpi2610.exe')) else 'UNRELATED'
            records.append(dict(pid=p.pid,name=name,working_set_bytes=m.rss,private_bytes=getattr(m,'private',None),peak_working_set_bytes=getattr(m,'peak_wset',None),command_line=cmd,parent_pid=p.ppid(),start_time=datetime.fromtimestamp(p.create_time(),timezone.utc).isoformat(),classification=classification))
        except psutil.Error:
            records.append(dict(pid=p.pid,name=p.info.get('name','') if hasattr(p,'info') else '',working_set_bytes=0,private_bytes=None,peak_working_set_bytes=None,command_line=None,parent_pid=None,start_time=None,classification='UNKNOWN'))
    records.sort(key=lambda r:r['working_set_bytes'],reverse=True)
    rec={'timestamp':stamp(),'memory':before,'processes':records,'top_process_count':min(30,len(records)),'cleanup':{'proven_stale':[],'terminated':[],'before':before,'after':memory()},'interpretation':'No stale campaign process proven. Private bytes are commit, not resident RAM; working sets may share pages.'}
    atomic(OUT/'memory_audit_v2.json',rec)
    with (OUT/'memory_audit_v2.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    return rec
def archive_history():
    source=ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv'
    with source.open() as f:rows=list(csv.DictReader(f))
    # Immutable authoritative 32/33 values before truncation or recomputation.
    target=OUT/'archived_verified_history.json'
    if not target.exists():atomic(target,{'timestamp':stamp(),'source':str(source),'source_sha256':sha(source),'rows':rows})
    return read(target)['rows']
