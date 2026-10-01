"""Only after full 2 ms PASS: compare first 0.25 ms at 1e-50 and 1e-51."""
import json,csv,sys
import numpy as np
def run(solver,context):
    root=context['root'];baseline=json.loads((root/'evidence/benchmark_C_fineA_free_6dof.json').read_text())
    if baseline.get('simulation_status',baseline['status'])!='FREE_6DOF_SOLVED' or baseline.get('completed_time_steps')!=80:raise RuntimeError('Cutoff sensitivity requires full 2 ms PASS')
    sys.path.insert(0,str(root/'scripts'));from benchmark_C_fineA_free_6dof_run import run as free_run
    context['fineA_run_config']={'steps':10,'cutoff':1e-51,'tag':'_cutoff51'}
    try:free_run(solver,context)
    finally:
        context.pop('fineA_run_config',None)
        solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
    base=list(csv.DictReader((root/'evidence/benchmark_C_fineA_free_6dof_history.csv').open()))[:10]
    alt=list(csv.DictReader((root/'evidence/benchmark_C_fineA_free_6dof_history_cutoff51.csv').open()))
    groups={'COM':(['com_x_m','com_y_m','com_z_m'],1e-10),'q':(['q0','q1','q2','q3'],1e-9),'omega':([f'omega_{x}_rad_s' for x in 'xyz'],1e-6),'Fmag':([f'Fmag_{x}_N' for x in 'xyz'],1e-13),'Tmag':([f'Tmag_{x}_Nm' for x in 'xyz'],1e-14)}
    errors={}
    for name,(keys,tol) in groups.items():
        error=max(abs(float(a[k])-float(b[k])) for a,b in zip(base,alt) for k in keys)
        errors[name]={'max_absolute_error':error,'tolerance':tol,'status':'PASS' if error<=tol else 'FAIL'}
    report={'status':'PASS' if len(alt)==10 and all(v['status']=='PASS' for v in errors.values()) else 'FAIL','duration_s':.00025,'baseline_cutoff':1e-50,'comparison_cutoff':1e-51,'physical_inertia_modified':False,'errors':errors}
    (root/'evidence/benchmark_C_fineA_cutoff_sensitivity.json').write_text(json.dumps(report,indent=2))
    solver.scheme.eval("(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments 1e-50)")
