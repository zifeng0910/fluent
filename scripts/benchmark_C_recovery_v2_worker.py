"""Task Scheduler owned, one-core direct PyFluent worker. Probe never solves."""
import os, runpy, sys, time, traceback
from pathlib import Path
import psutil
from benchmark_C_recovery_v2_common import *

def main():
    config=read(OUT/'launch_config.json');control=OUT/'worker_control.json'
    report={'status':'LAUNCHING','stage':'launch','pid':os.getpid(),'created':psutil.Process().create_time(),'token':config['token'],'timestamp':stamp(),'time_steps_advanced':0,'ui_mode':'no_gui_or_graphics','processors':1,'owned_identities':[]}
    solver=None
    def save(stage,status='RUNNING',**data):
        report.update(stage=stage,status=status,timestamp=stamp(),**data)
        if solver is not None:
            cp=solver.connection_properties
            identities=[]
            for pid in [os.getpid(),cp.fluent_host_pid,cp.cortex_pid]:
                try:
                    p=psutil.Process(pid);identities.extend([{'pid':x.pid,'created':x.create_time()} for x in [p]+p.children(recursive=True)])
                except (psutil.Error,TypeError):pass
            report['owned_identities']=list({x['pid']:x for x in identities}.values())
        atomic(OUT/'worker.json',report)
    save('launch','LAUNCHING')
    try:
        if sha(ROOT/'fluent_udf/l2300_abaqus100hz_6dof.c')!=FROZEN_SHA:raise RuntimeError('Frozen UDF changed')
        import ansys.fluent.core as pyfluent
        solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=1,
            ui_mode='no_gui_or_graphics',start_timeout=180,
            fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',
            cwd=str(ROOT/'live_cases/benchmark_C_analytic_6dof'))
        transcript=OUT/f"worker_{config['token']}.trn";messages=[]
        solver.transcript.start(str(transcript));solver.transcript.register_callback(messages.append,keep_new_lines=True)
        context={'root':ROOT,'messages':messages,'transcript_path':transcript,'recovery_v2':True,'campaign_stop_path':OUT/'stop_request.json'}
        save('CASE_LOAD',transcript=str(transcript))
        checkpoint=config['checkpoint']
        for key in ['case','data']:
            if sha(checkpoint[key])!=checkpoint[key+'_sha256']:raise RuntimeError('Checkpoint hash mismatch')
        solver.settings.file.read_case(file_name=checkpoint['case'])
        save('DATA_LOAD');solver.settings.file.read_data(file_name=checkpoint['data'])
        save('UDF_LOAD');solver.settings.setup.user_defined.load(udf_library_name=str(ROOT/'fluent_udf/libbenchmark_C_v4'))
        t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
        if abs(t-checkpoint['step']*25e-6)>1e-10:raise RuntimeError('Native restored time differs; no manual reset performed')
        # Reading case/data reconstructs native overset; no mesh update/preview/timestep here.
        save('OVERSET_RECONSTRUCTION_LOADED','LOADED_WAIT_MEMORY',native_time_s=t)
        while True:
            cmd=read(control)
            if cmd.get('token')==config['token'] and cmd.get('decision') in ['CONTINUE','EXIT']:break
            if (OUT/'stop_request.json').exists():cmd={'decision':'EXIT'};break
            time.sleep(1)
        if cmd['decision']=='EXIT':save('PROBE_FINISHED','PROBE_EXITED');return
        # Resource admission precedes expensive restart validation and all solving.
        save('RESTART_VALIDATION')
        from benchmark_C_recovery_v2_restart import validate
        context['native_restart_validation']=validate(solver,context,checkpoint['step'])
        save('NATIVE_RESTART_PASS','RESTART_PASS')
        cfg=config['run_config'];cfg['already_loaded']=True;cfg['recovery_v2']=True;cfg['restart_prevalidated']=True
        # Truncate only after native state passes; the previous full history is immutable in the archive.
        import csv,shutil
        hist=ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv'
        shutil.copy2(hist,OUT/f"history_before_{config['token']}.csv")
        with hist.open() as f:rows=list(csv.DictReader(f))
        keep=[r for r in rows if int(r['step'])<=checkpoint['step']]
        with hist.open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(keep)
        context['fineA_run_config']=cfg
        save('FREE_6DOF','RUNNING')
        runpy.run_path(str(ROOT/'scripts/benchmark_C_fineA_free_6dof_run.py'))['run'](solver,context)
        free=read(ROOT/'evidence/benchmark_C_fineA_free_6dof.json')
        if free.get('status')=='PAUSED_SAFE_CHECKPOINT':save('SAFE_STOP','SAFE_STOP');return
        if free.get('status')!='FREE_6DOF_SOLVED' or free.get('completed_time_steps')!=80:raise RuntimeError('Full 2 ms not verified')
        save('POSTPROCESS','POSTPROCESS')
        os.environ['BENCHMARK_C_RECOVERY_V2']='1'
        runpy.run_path(str(ROOT/'scripts/benchmark_C_fineA_finish_in_same_session.py'))['run'](solver,context)
        if read(ROOT/'evidence/benchmark_C_fineA_cutoff_sensitivity.json').get('status')!='PASS':raise RuntimeError('Cutoff sensitivity not PASS')
        if read(ROOT/'evidence/benchmark_C_fineA_finish_workflow.json').get('status')!='FILES_VERIFIED_VISUAL_INSPECTION_PENDING':raise RuntimeError('Postprocessing incomplete')
        save('COMPLETE','COMPLETE',visual_review='PENDING_HUMAN_OR_LATER_LLM_INSPECTION')
    except Exception as exc:
        save('FAILURE','ERROR',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        if solver is not None:
            try:solver.exit()
            except Exception as exc:report['shutdown_error']=repr(exc)
        report.update(closed=True,closed_at=stamp());atomic(OUT/'worker.json',report)
if __name__=='__main__':main()
