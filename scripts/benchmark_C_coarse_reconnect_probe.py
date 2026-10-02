"""Recover this campaign's SDK connection from its registered watchdog only."""
import os
import psutil
from benchmark_C_coarse_common import *

def main():
    history=read(EVID/'benchmark_C_coarse_memory_profile.json');owned={p['pid']:p for p in history['rows'][-1]['processes']}
    host=owned[12684];p=psutil.Process(host['pid'])
    if abs(p.create_time()-host['created'])>.01:raise RuntimeError('Original Fluent identity mismatch')
    matches=[]
    for proc in psutil.process_iter(['name','cmdline']):
        args=proc.info['cmdline'] or []
        if (proc.info['name'] or '').lower()=='pythonw.exe' and len(args)==11 and args[1].replace('\\','/').endswith('/watchdog_exec') and args[2]=='40076':matches.append(args)
    if not matches:raise RuntimeError('No registered session watchdog connection')
    args=matches[0];private=OUT/'private_session_connection.json'
    atomic(private,{'ip':args[3],'port':int(args[4]),'password':args[5],'fluent_host_pid':host['pid'],'host_created':host['created'],'cortex_pid':39540})
    # Keep credentials out of Git and all printed diagnostics.
    exclude=ROOT/'.git/info/exclude';rule='/live_cases/benchmark_C_coarse_A/private_session_connection.json'
    if rule not in exclude.read_text():
        with exclude.open('a') as f:f.write('\n'+rule+'\n')
    import ansys.fluent.core as pyfluent
    solver=pyfluent.connect_to_fluent(ip=args[3],port=int(args[4]),password=args[5],cleanup_on_exit=False,start_transcript=False,start_watchdog=False)
    if int(solver.connection_properties.fluent_host_pid)!=host['pid']:raise RuntimeError('Connected to another Fluent')
    rec={'timestamp':stamp(),'status':'CONNECTED_SAME_NATIVE_SESSION','host_pid':host['pid'],'solver_launched':False,'flow_time':float(solver.scheme.eval("(rpgetvar 'flow-time)")),'cell_zone_names':list(solver.settings.setup.cell_zone_conditions.fluid.keys()),'data_valid':solver.fields.field_data.is_data_valid()}
    atomic(EVID/'reconnect_probe.json',rec);print({k:v for k,v in rec.items() if k!='cell_zone_names'},flush=True)

if __name__=='__main__':main()
