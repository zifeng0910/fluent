"""Publish one verified local milestone; exact paths, no blanket git add."""
import subprocess,sys
from benchmark_C_recovery_v2_common import ROOT,OUT,atomic,read,stamp
def main():
    key=sys.argv[1];cfg=read(OUT/'publish'/f'{key}.config.json');result=OUT/'publish'/f'{key}.result.json'
    try:
        paths=cfg['paths']
        if any(not (ROOT/p).is_file() for p in paths):raise RuntimeError('Missing milestone evidence')
        staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines()
        if set(staged)-set(paths):raise RuntimeError('Unrelated staged changes; refusing milestone commit')
        subprocess.run(['git','add','--',*paths],cwd=ROOT,check=True)
        if subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).strip():
            subprocess.run(['git','commit','-m',cfg['message']],cwd=ROOT,check=True)
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        p=subprocess.run(['git','-c','core.sshCommand=ssh -o ConnectTimeout=15','push','origin','main'],cwd=ROOT,capture_output=True,text=True,timeout=90)
        atomic(result,{'status':'PASS' if p.returncode==0 else 'PUSH_FAILED','timestamp':stamp(),'commit':head,'error':p.stderr[-1000:]})
    except Exception as e:atomic(result,{'status':'FAIL','timestamp':stamp(),'error':repr(e)})
if __name__=='__main__':main()
