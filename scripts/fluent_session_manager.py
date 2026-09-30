"""One controlled experiment through the official MCP subprocess. No launch retries.

Usage: python scripts/fluent_session_manager.py experiment.json
The experiment contains purpose, input and a list of MCP run_code snippets.
"""
import asyncio,csv,json,os,platform,re,subprocess,sys,time,uuid
from pathlib import Path
import psutil
from fastmcp import Client
from fastmcp.client.transports import StdioTransport
ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'evidence'; E.mkdir(exist_ok=True)
LOCK=ROOT/'.fluent_session.lock'
BUDGET=E/'fluent_launch_budget.json'
LEDGER=E/'fluent_session_ledger.csv'
FIELDS=['session_id','start_time','host_pid','node_pid','mode','purpose','input','result','failure','closed_cleanly','transcript']
def snapshot():
    rows=[]
    for p in psutil.process_iter(['pid','ppid','name','create_time','cmdline','cwd']):
        try:
            if re.search(r'^(fluent|fl_mpi.*|cortex|tgrid.*)(\.exe)?$',p.name(),re.I): rows.append(p.info)
        except (psutil.NoSuchProcess,psutil.AccessDenied): pass
    return rows
def reserve():
    try: fd=os.open(LOCK,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    except FileExistsError: raise RuntimeError('ACTIVE_FLUENT_SESSION_EXISTS: lock present; inspect owner before recovery')
    os.write(fd,str(os.getpid()).encode());os.close(fd)
    active=snapshot()
    if active:
        LOCK.unlink(); raise RuntimeError('ACTIVE_FLUENT_SESSION_EXISTS PID='+','.join(str(p['pid']) for p in active)+' (unknown ownership also blocks launch)')
def append(row):
    exists=LEDGER.exists()
    with LEDGER.open('a',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS)
        if not exists:w.writeheader()
        w.writerow(row)
async def main():
    exp=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8-sig'))
    reserve()
    owned={}; row={k:'' for k in FIELDS}; sid=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    out=ROOT/'logs'/sid;out.mkdir(parents=True)
    row.update(session_id=sid,start_time=time.strftime('%Y-%m-%dT%H:%M:%S%z'),mode='meshing/no_gui',purpose=exp['purpose'],input=exp['input'],transcript=str(out/'solver.trn'))
    initial=set(psutil.pids());records=[]; server_pid=None
    def capture():
        # Descendants of this manager, never global process-name termination.
        for p in psutil.Process().children(recursive=True):
            try:
                if p.pid not in initial: owned[p.pid]=p.create_time()
            except psutil.NoSuchProcess:pass
    async def call(client,name,args,timeout=90):
        r=await asyncio.wait_for(client.call_tool(name,args,raise_on_error=False),timeout)
        record={'tool':name,'args':args,'result':r.data,'content':[c.model_dump(mode='json') for c in r.content]}
        records.append(record);(out/'mcp.json').write_text(json.dumps(records,indent=2,default=str),encoding='utf-8')
        print(json.dumps(record,default=str),flush=True)
        data=r.data
        if r.is_error or isinstance(data,dict) and data.get('status')=='error':raise RuntimeError(str(data))
        return data
    try:
        budget=json.loads(BUDGET.read_text()) if BUDGET.exists() else {'campaign':'single_session_STL_20260930','launches':0,'maximum':3}
        if budget['launches']>=budget['maximum']:raise RuntimeError('LAUNCH_BUDGET_EXHAUSTED')
        os.environ['PROCESSOR_ARCHITECTURE']='AMD64'
        child=json.loads(subprocess.check_output([sys.executable,'-c','import os,json;print(json.dumps({k:os.environ.get(k) for k in ["PROCESSOR_ARCHITECTURE","PROCESSOR_IDENTIFIER","ProgramFiles"]}))'],text=True))
        (E/'fluent_launch_environment.json').write_text(json.dumps({'parent':{k:os.environ.get(k) for k in child},'child':child,'python_architecture':platform.architecture(),'ansys_path':'H:/Program Files/ANSYS Inc/v261','ui_mode':'no_gui'},indent=2))
        transport=StdioTransport(command=str(ROOT/'.venv/Scripts/ansys-fluent-mcp.exe'),args=[],env=dict(os.environ),cwd=str(ROOT))
        async with Client(transport,timeout=120) as client:
            # Discover the actual installed connect schema before launch.
            catalog=await client.list_tools()
            (out/'connect_schema.json').write_text(json.dumps([t.model_dump(mode='json') for t in catalog if t.name=='connect'],indent=2))
            budget['launches']+=1;BUDGET.write_text(json.dumps(budget,indent=2))
            try:
                await call(client,'connect',{'backend_kind':'pyfluent','connect_kwargs':{'mode':'meshing','dimension':3,'precision':'double','ui_mode':'no_gui','processor_count':1,'cwd':str(out),'fluent_path':'H:/Program Files/ANSYS Inc/v261/fluent/ntbin/win64/fluent.exe','env':{'PROCESSOR_ARCHITECTURE':'AMD64'},'start_timeout':90}},115)
                capture()
                current=snapshot();row['host_pid']=';'.join(str(p['pid']) for p in current if p['name'].lower()=='fluent.exe');row['node_pid']=';'.join(str(p['pid']) for p in current if 'fl_mpi' in p['name'])
                codes=["solver.transcript.start('"+str(out/'solver.trn').replace('\\','/')+"'); print(solver.get_fluent_version()); print(solver.id)"]+exp['codes']
                for code in codes:
                    await call(client,'validate_code',{'code':code})
                    await call(client,'run_code',{'code':code},exp.get('step_timeout',90));capture()
                row['result']='PASS'
            except Exception as err:
                row['result']='FAIL_AND_CLOSED';row['failure']=str(err) or type(err).__name__
            finally:
                capture()
                try:
                    await call(client,'validate_code',{'code':'solver.exit()'},20)
                    await call(client,'run_code',{'code':'solver.exit()'},20)
                except Exception:pass
                try:await call(client,'disconnect',{},10)
                except Exception:pass
    except Exception as err:
        row['result']='FAIL_AND_CLOSED';row['failure']=str(err)
    finally:
        capture()
        deadline=time.monotonic()+10
        while snapshot() and time.monotonic()<deadline: await asyncio.sleep(0.5)
        forced=[]
        for pid,born in reversed(list(owned.items())):
            try:
                p=psutil.Process(pid)
                if p.create_time()==born and p.is_running():p.kill();forced.append(pid)
            except (psutil.NoSuchProcess,psutil.AccessDenied):pass
        await asyncio.sleep(1)
        remaining=[p for p in snapshot() if p['pid'] in owned]
        row['closed_cleanly']=not forced and not remaining
        (out/'closure.json').write_text(json.dumps({'forced_owned_pids':forced,'remaining':remaining},indent=2,default=str))
        if remaining: row['failure']+='; SESSION_DEAD but residual ownership requires investigation'
        else:LOCK.unlink(missing_ok=True)
        append(row); print(json.dumps(row,indent=2),flush=True)
        if row['result']!='PASS':sys.exit(1)
if __name__=='__main__':asyncio.run(main())
