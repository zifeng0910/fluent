"""Assemble final campaign evidence without promoting unfinished solver gates."""
import json,csv,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def read(name):
    p=ROOT/'evidence'/name
    return json.loads(p.read_text()) if p.exists() else {}
def main():
    mesh=read('benchmark_C_fineA_prime_bg020.json');zero=read('benchmark_C_fineA_theta0_connectivity.json');sweep=read('benchmark_C_fineA_short_sweep.json');free=read('benchmark_C_fineA_free_6dof.json');cutoff=read('benchmark_C_fineA_cutoff_sensitivity.json');layers=read('benchmark_C_fineA_actual_layers.json')
    rows=sweep.get('rows',[]);p=ROOT/'evidence/benchmark_C_fineA_free_6dof_history.csv';history=list(csv.DictReader(p.open())) if p.exists() else []
    last=history[-1] if history else {};sim=free.get('simulation_status',free.get('status','NOT RUN'))
    def remaining_blocker():
        if free.get('error'):return free['error']
        if sweep.get('status')!='BENCHMARK_C_OVERSET_RESOLUTION_PASS':return 'STATIC_POSE_SWEEP_PENDING'
        if sim!='FREE_6DOF_SOLVED' or free.get('completed_time_steps')!=80:return 'FULL_2MS_DYNAMIC_GATE_PENDING'
        if cutoff.get('status')!='PASS':return 'CUTOFF_SENSITIVITY_PENDING'
        if free.get('rendering_status')!='PASS':return 'SAVED_STATE_FIELDDATA_AND_OFFSCREEN_RENDER_PENDING'
        return None
    final={'Campaign':'BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF','Previous blocker':'OVERSET RESOLUTION MISMATCH','Old background cell edge mm':.18742598,'Selected component shell':'C_FINE_A, 0.050 mm normal offset, 0.050 mm axial extension each end','Selected component nominal size mm':[.010,.0125],'Actual component sampled minimum layers':layers.get('component',{}).get('minimum_crossed_cells'),'Selected background BOI size mm':.020,'Shared sizing field note':'Nearby background also responds to component fine control; actual cell crossings and donor ratios are recorded','Refinement region':read('benchmark_C_background_refinement_region.json'),'Background cells before':127360,'Background cells after':mesh.get('background_water_cells'),'Component cells':mesh.get('robot_component_fluid_cells'),'Theta=0':zero.get('status','NOT RUN'),'Theta=0 orphan count':zero.get('orphans'),'Theta=0 donor length ratio':zero.get('donor_characteristic_length_ratio'),'Static poses tested':len(rows),'Static pose sweep':sweep.get('status','NOT RUN'),'Maximum robust angle rad':sweep.get('theta_max_validated_rad'),'Minimum static physical clearance mm':min((r['physical_clearance_mm'] for r in rows),default=None),'Minimum effective sampled layers':min((r['background_actual_minimum_layers'] for r in rows),default=None),'Static orphan peak':max((r['orphans'] for r in rows),default=None),'2 ms free 6DOF':'PASS' if sim=='FREE_6DOF_SOLVED' and free.get('completed_time_steps')==80 else 'FAIL' if sim=='FAIL' else 'NOT RUN' if not free else 'RUNNING','Dynamic orphan peak':max((int(r['orphan_count']) for r in history),default=None),'Dynamic donor length ratio peak':max((float(r['donor_length_ratio_max']) for r in history),default=None),'Final COM m':[float(last[k]) for k in ['com_x_m','com_y_m','com_z_m']] if last else None,'Final quaternion scalar first':[float(last[f'q{i}']) for i in range(4)] if last else None,'Peak angular velocity rad/s':max((float(np.linalg.norm([float(r[f'omega_{x}_rad_s']) for x in 'xyz'])) for r in history),default=None),'Minimum dynamic physical clearance mm':min((float(r['robot_wall_clearance_m'])*1000 for r in history),default=None),'Maximum quaternion norm error':max((abs(float(r['q_norm'])-1) for r in history),default=None),'Determinant cutoff':1e-50,'Cutoff sensitivity':cutoff.get('status','NOT RUN'),'GIF':free.get('gif'),'Velocity GIF':free.get('velocity_gif'),'Old GIF status':'FAILED_PARTIAL_OLD_OVERSET','New output directory':str(ROOT/'evidence/benchmark_C_fineA_pyvista'),'Git commits':subprocess.check_output(['git','log','-4','--format=%h %s'],cwd=ROOT,text=True).splitlines(),'Remaining blocker':free.get('error') or ('STATIC_POSE_SWEEP_PENDING' if sweep.get('status')!='BENCHMARK_C_OVERSET_RESOLUTION_PASS' else 'FULL_2MS_DYNAMIC_GATE_PENDING' if sim!='FREE_6DOF_SOLVED' else 'CUTOFF_SENSITIVITY_PENDING' if not cutoff else None)}
    final['Remaining blocker']=remaining_blocker()
    final['Completed dynamic steps']=free.get('completed_time_steps',0)
    final['Latest dynamic time ms']=free.get('latest_time_s',0)*1000
    final['Actual component layer audit limitation']='Minimum over 120 sampled CAD-normal rays through tetrahedra; not a proof for every surface point'
    final['Final PNG']=free.get('preview')
    final['Primary orphan metric']='Official Fluent overset-cell-type == -1'
    (ROOT/'evidence/benchmark_C_fineA_final_report.json').write_text(json.dumps(final,indent=2))
    print(json.dumps(final,indent=2))
if __name__=='__main__':main()
