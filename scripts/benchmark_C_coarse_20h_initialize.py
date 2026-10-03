"""Initialize only the newly authorized coarse window; never extend an existing one."""
import csv, time
from datetime import datetime,timezone
from zoneinfo import ZoneInfo
from benchmark_C_coarse_common import *
from benchmark_C_coarse_commit_audit import system

def main():
    cont=EVID/'continuation_20h';cont.mkdir(exist_ok=True)
    if (cont/'window.json').exists():raise RuntimeError('Existing 20h window is immutable; no reset or extension')
    old=read(EVID/'state.json'); checkpoint=read(EVID/'resume_stop_checkpoint.json')
    if old['status']!='RESOURCE_BLOCKER' or checkpoint['step']!=29 or native():raise RuntimeError('Expected stopped native step29 only')
    preserve()
    audit=read(EVID/'commit_audit_step29.json');m=system()
    if not audit or time.time()-datetime.fromisoformat(audit['timestamp']).timestamp()>3600:raise RuntimeError('Current commit audit required')
    now=time.time();deadline=now+20*3600;wid='coarse_20h_'+str(int(now))
    window={'window_id':wid,'campaign':'BENCHMARK_C_COARSE_SCREENING','started_epoch':now,'deadline_epoch':deadline,'duration_hours':20,
        'started_at':datetime.fromtimestamp(now,timezone.utc).isoformat(),'deadline':datetime.fromtimestamp(deadline,timezone.utc).isoformat(),
        'deadline_Asia_Shanghai':datetime.fromtimestamp(deadline,ZoneInfo('Asia/Shanghai')).isoformat(),
        'authorization':'User explicitly requested continuing autonomous iteration for 20 hours and periodically writing plans for next steps on 2026-10-03.',
        'scope':'Current4.699M mesh / native step29→40 (1ms), audit, checkpoint verification and exact-time comparison. No t0/remesh/physics/MPI change/2ms/medium/fine/D.',
        'old_fine_deadline_reset':False,'old_heartbeat_replaced':True,'allow_early_completion':True}
    atomic(cont/'window.json',window)
    token=old['timestamp']
    atomic(cont/'state.json',{'timestamp':stamp(),'status':'RESOURCE_WAIT','window_id':wid,'failure_timestamp':token,'repairs':[],'consumed_plan_ids':[],'worker_pid':None})
    config={'attempt':'resume29_01','checkpoint_metadata':'evidence/benchmark_C_coarse_A/resume_stop_checkpoint.json','target_step':40,'dt_s':25e-6,'max_iter_per_time_step':2,'processor_count':1,'ui_mode':'no_gui_or_graphics','mesh_cells':4699301,'runtime_guard_available_gib':3,'runtime_guard_commit_fraction':.95,'runtime_guard_project_working_set_gib':15,'frozen_udf_sha256':FROZEN_SHA}
    config_path=cont/'resume29_01_config.json';atomic(config_path,config)
    job='scripts/benchmark_C_coarse_native_resume.py'
    supporting=['scripts/benchmark_C_coarse_common.py','scripts/benchmark_C_coarse_dynamic.py','scripts/benchmark_C_coarse_report.py','scripts/benchmark_C_coarse_commit_audit.py','scripts/benchmark_C_coarse_resource_checkpoint.py','scripts/benchmark_C_analytic_reference.py','scripts/benchmark_C_recovery_v2_restart.py']
    plan={'plan_id':'resume29_01','window_id':wid,'failure_timestamp':token,'decision':'REPAIR','failure_class':'SYSTEM_COMMIT',
        'root_cause':'Previous resource stops occurred at95% system commit with7.24/9.04GiB physical available. Current audit shows system-managed16GiB pagefile and reduced baseline commit; historical peak process attribution remains unresolved.',
        'concrete_change':f"Fresh audited system commit baseline {m['commit_gib']:.3f}GiB / {m['commit_limit_gib']:.3f}GiB, versus45.64–45.84GiB failed load. Require >=19GiB commit headroom and >=12GiB physical available continuously for60s before one cold native step29 restart. Zero-time continuity gate precedes any new dynamics; runtime guards unchanged.",
        'evidence_files':['evidence/benchmark_C_coarse_A/commit_audit_step29.json','evidence/benchmark_C_coarse_A/resume_stop_checkpoint.json','evidence/benchmark_C_coarse_A/final_dataset_verification.json','evidence/benchmark_C_coarse_A/frozen_provenance.json'],
        'job':job,'job_sha256':sha(ROOT/job),'config':str(config_path.relative_to(ROOT)).replace('\\','/'),'configuration_sha256':sha(config_path),'supporting_code_sha256':{p:sha(ROOT/p) for p in supporting}}
    atomic(cont/'repair_plan.json',plan)
    print({'window_id':wid,'deadline_Asia_Shanghai':window['deadline_Asia_Shanghai'],'initial_plan':plan['plan_id']})

if __name__=='__main__':main()
