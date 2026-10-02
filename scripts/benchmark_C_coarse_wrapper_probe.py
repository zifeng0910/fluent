"""Reproduce native stderr handling without touching Fluent or its physics."""
import os,subprocess,sys
from benchmark_C_coarse_common import *
probe=EVID/'wrapper_probe';probe.mkdir(exist_ok=True)
py=probe/'warning.py';py.write_text("import sys,time\nprint('probe-warning',file=sys.stderr,flush=True)\ntime.sleep(.2)\nprint('SURVIVED_WARNING',flush=True)\n")
shell=probe/'old_wrapper.ps1';shell.write_text("$ErrorActionPreference='Stop'\n& 'H:\\fluent\\.venv\\Scripts\\python.exe' '"+str(py)+"' 1>> '"+str(probe/'old_stdout.log')+"' 2>> '"+str(probe/'old_stderr.log')+"'\nexit $LASTEXITCODE\n")
exe=Path(os.environ['SystemRoot'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
r=subprocess.run([str(exe),'-NoProfile','-File',str(shell)],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
with (probe/'new_stdout.log').open('wb') as out,(probe/'new_stderr.log').open('wb') as err:
    new=subprocess.run([sys.executable,str(py)],stdout=out,stderr=err,creationflags=subprocess.CREATE_NO_WINDOW)
atomic(EVID/'wrapper_stderr_reproduction.json',{'timestamp':stamp(),'old_PowerShell_exit_code':r.returncode,'old_wrapper_error':r.stderr.decode('utf-8',errors='replace'),'new_raw_redirect_exit_code':new.returncode,'new_stdout':(probe/'new_stdout.log').read_text(),'no_Fluent_launched':True,'warning_only_payload':True})
