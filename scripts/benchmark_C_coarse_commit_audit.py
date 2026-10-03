"""Read-only Windows commit audit. Command credentials are redacted before storage."""
import csv, ctypes, json, os, subprocess
from datetime import datetime, timezone
import psutil
from benchmark_C_coarse_common import ROOT, EVID, atomic, stamp, memory
from benchmark_C_recovery_v2_common import Performance

def redact(args):
    args=list(args)
    if any('watchdog' in str(a).lower() for a in args):
        return args[:2]+['[watchdog arguments redacted: may contain Fluent password]']
    out=[]; secret=False
    for a in args:
        if secret: out.append('[REDACTED]'); secret=False; continue
        low=str(a).lower()
        if any(k in low for k in ['password','api_key','api-key','token','secret']):
            out.append(a.split('=')[0]+'=[REDACTED]' if '=' in a else a); secret='=' not in a
        else: out.append(a)
    return out

def system():
    rec=memory(); x=Performance(); x.cb=ctypes.sizeof(x)
    if not ctypes.windll.psapi.GetPerformanceInfo(ctypes.byref(x),x.cb): raise ctypes.WinError()
    rec.update(commit_peak_gib=x.CommitPeak*x.PageSize/2**30,
               pagefile_available_gib=rec['pagefile_total_gib']-rec['pagefile_used_gib'],
               commit_headroom_gib=rec['commit_limit_gib']-rec['commit_gib'])
    return rec

def audit(destination=None):
    before=system(); records=[]; denied=[]
    for p in psutil.process_iter():
        try:
            m=p.memory_info(); args=p.cmdline(); command=' '.join(args).lower()
            project=('h:\\fluent' in command or 'h:/fluent' in command)
            records.append(dict(pid=p.pid,name=p.name(),command_line=' '.join(redact(args)),
                parent_pid=p.ppid(),created=p.create_time(),
                start_time=datetime.fromtimestamp(p.create_time(),timezone.utc).isoformat(),
                working_set_gib=m.rss/2**30,private_bytes_gib=getattr(m,'private',0)/2**30,
                commit_gib=getattr(m,'pagefile',None)/2**30 if hasattr(m,'pagefile') else None,
                project_path_association=project))
        except psutil.Error: denied.append(p.pid)
    records.sort(key=lambda r:r['private_bytes_gib'],reverse=True)
    script="$ErrorActionPreference='Stop'; [pscustomobject]@{automatic_managed=(Get-CimInstance Win32_ComputerSystem).AutomaticManagedPagefile; usage=@(Get-CimInstance Win32_PageFileUsage | Select-Object Name,AllocatedBaseSize,CurrentUsage,PeakUsage); settings=@(Get-CimInstance Win32_PageFileSetting | Select-Object Name,InitialSize,MaximumSize)} | ConvertTo-Json -Depth 5 -Compress"
    config=json.loads(subprocess.check_output(['powershell','-NoProfile','-Command',script],text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW))
    project=sum(r['private_bytes_gib'] for r in records if r['project_path_association'])
    residual=before['commit_gib']-sum(r['private_bytes_gib'] for r in records)
    # Historical values do not identify which process caused either earlier spike.
    cause='F_UNRESOLVED_HISTORICAL_PEAK_ATTRIBUTION'
    if not config['automatic_managed'] and config['settings'] and all(x['InitialSize']==x['MaximumSize'] for x in config['settings']):
        cause='E_MIXED_FIXED_PAGEFILE_AND_SYSTEM_COMMIT_PRESSURE'
    rec={'timestamp':stamp(),'memory':before,'pagefile_configuration':config,'cause':cause,
        'processes_ranked_by_private_bytes':records,'top_working_set_pids':[r['pid'] for r in sorted(records,key=lambda x:x['working_set_gib'],reverse=True)[:30]],
        'top_commit_pids':[r['pid'] for r in sorted(records,key=lambda x:x['commit_gib'] or 0,reverse=True)[:30]],
        'processes_unreadable':denied,'project_private_bytes_gib':project,
        'commit_minus_readable_process_private_bytes_gib':residual,
        'interpretation':'Private bytes and system commit differ from working set. Residual includes kernel/shared/pagefile-backed allocations and unreadable processes. Current snapshot cannot prove ownership of historical 95% spikes.',
        'guards':{'runtime_available_gib_min':3,'runtime_commit_fraction_max':.95,'runtime_project_working_set_gib_max':15},
        'cleanup':{'terminated':[],'before':before,'after':system()}}
    destination=destination or EVID/'commit_audit_step29.json'
    atomic(destination,rec)
    with destination.with_suffix('.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0])); w.writeheader(); w.writerows(records)
    return rec

if __name__=='__main__':
    r=audit(); print(json.dumps({k:r[k] for k in ['timestamp','memory','pagefile_configuration','cause','project_private_bytes_gib']}))
    print(json.dumps(r['processes_ranked_by_private_bytes'][:5]))
