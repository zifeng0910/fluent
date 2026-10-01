"""One direct headless PyFluent session for the C campaign."""
from __future__ import annotations
import argparse,json,os,runpy,sys,time,traceback
from pathlib import Path
import ansys.fluent.core as pyfluent
ROOT=Path(__file__).resolve().parents[1];CASE=ROOT/'live_cases/benchmark_C_analytic_6dof';CMD=CASE/'session_command.json';STATUS=ROOT/'evidence/benchmark_C_single_session.json'
def save(x):
 tmp=STATUS.with_suffix('.json.tmp');tmp.write_text(json.dumps(x,indent=2,default=str),encoding='utf-8');tmp.replace(STATUS)
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--processors',type=int,default=1);args=parser.parse_args()
 import psutil
 report={'status':'LAUNCHING','pid':os.getpid(),'process_created':psutil.Process().create_time(),'processor_count':args.processors,'python':sys.executable,'ui_mode':'no_gui_or_graphics','session_count':1,'campaign':'BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF'};save(report);solver=None
 try:
  solver=pyfluent.launch_fluent(mode='solver',dimension=3,precision='double',processor_count=args.processors,ui_mode='no_gui_or_graphics',start_timeout=120,fluent_path=r'H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe',cwd=str(CASE))
  transcript=CASE/f'benchmark_C_session_{os.getpid()}_{int(time.time())}.trn';solver.transcript.start(str(transcript));messages=[];solver.transcript.register_callback(messages.append,keep_new_lines=True)
  context={'root':ROOT,'case_dir':CASE,'messages':messages,'transcript_path':transcript}
  report.update(status='READY',fluent_version=str(solver.get_fluent_version()),transcript=str(transcript));save(report)
  while True:
   if not CMD.exists():time.sleep(.25);continue
   pending=json.loads(CMD.read_text(encoding='utf-8-sig'));CMD.unlink()
   if pending.get('action')=='exit':break
   job=(ROOT/pending['job']).resolve()
   if not job.is_relative_to(ROOT):raise ValueError('Job must remain inside H:/fluent')
   report.update(status='RUNNING',job=str(job));report.pop('error',None);report.pop('traceback',None);save(report)
   context.pop('fineA_run_config',None)
   if pending.get('config'):context['fineA_run_config']=pending['config']
   try:runpy.run_path(str(job))['run'](solver,context);report.update(status='READY',last_job='COMPLETE');report.pop('traceback',None)
   except Exception as e:report.update(status='READY',last_job='ERROR',error=repr(e),traceback=traceback.format_exc());print(traceback.format_exc(),flush=True)
   save(report);print(json.dumps(report,default=str),flush=True)
 finally:
  if solver is not None:solver.exit()
  report.update(status='CLOSED',final_result=report.get('last_job','NO_JOBS'));save(report)
if __name__=='__main__':main()
