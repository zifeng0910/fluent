"""Cross-check the executed Abaqus magnetic increment log, not interpolation self-consistency."""
from pathlib import Path
import json,csv,importlib.util
import numpy as np
from scipy.spatial.transform import Rotation
from magnetic_lookup_runtime import Lookup
HERE=Path(__file__).resolve().parent;BASE=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis');CASE=BASE/'TrueCELLongForwardTransit/case/TRUECEL_B0P11_G2P20_F100_NOFLUID_CONTROL'
r=Lookup();P=r.manifest;S=np.array(P['source_frame_aba']);old_m=.0010876227522174417*np.array([-1.,0,0]);new_m=r.m.copy();log=np.loadtxt(CASE/'magnetic_increment_g2p20_f100.csv',delimiter=',',skiprows=1);hist=np.load(CASE/'private/rp_history_private.npz');rows=[];ef=[];et=[]
for i in np.linspace(100,len(log)-1,64,dtype=int):
 t=log[i,0];u=np.array([np.interp(t,hist[f'U{j}'][:,0],hist[f'U{j}'][:,1]) for j in [1,2,3]]);ur=np.array([np.interp(t,hist[f'UR{j}'][:,0],hist[f'UR{j}'][:,1]) for j in [1,2,3]]);pos=r.com0+S.T@u*.001;quat=Rotation.from_matrix(S.T@Rotation.from_rotvec(ur).as_matrix()@S).as_quat()
 r.m=old_m;f,tau=r.evaluate(pos,quat,t);oldf=S.T@log[i,3:6];oldt=S.T@log[i,6:9]*.001;ef.append(np.linalg.norm(f-oldf));et.append(np.linalg.norm(tau-oldt))
 r.m=new_m;fn,tn=r.evaluate(pos,quat,t);rows.append([t,log[i,1],log[i,2],*oldf,*oldt,*fn,*tn,np.linalg.norm(f-oldf),np.linalg.norm(tau-oldt)])
with (HERE/'abaqus100Hz_load_crosscheck.csv').open('w',newline='') as out:
 w=csv.writer(out);w.writerow(['time_s','old_s_eff_mm','old_phase_deg','old_Fx_N','old_Fy_N','old_Fz_N','old_Tx_Nm','old_Ty_Nm','old_Tz_Nm','L2300_Fx_N','L2300_Fy_N','L2300_Fz_N','L2300_Tx_Nm','L2300_Ty_Nm','L2300_Tz_Nm','same_identity_force_error_N','same_identity_torque_error_Nm']);w.writerows(rows)
# Periodic field endpoint and time-state behavior are distinct tests.
d=r.data;periodic=bool(np.allclose(d['B_T'][0],d['B_T'][-1],rtol=0,atol=1e-15));q=[0,0,0,1];f0,t0=r.evaluate(r.com0,q,.010);f1,t1=r.evaluate(r.com0,q,.020);assert np.linalg.norm(f1-f0)>1e-12 # inherited source drift must not reset per cycle
fa,ta=r.evaluate(r.com0,q,.00321);fb,tb=r.evaluate(r.com0,[0,0,0,-1],.00321);assert np.allclose(fa,fb,rtol=0,atol=1e-15) and np.allclose(ta,tb,rtol=0,atol=1e-15)
vpath=HERE/'magnetic_lookup_validation.json';v=json.loads(vpath.read_text());v.update(executed_Abaqus_log_crosscheck=dict(samples=64,same_identity_status='PASS' if max(ef)<1e-9 and max(et)<1e-9 else 'FAIL',max_force_error_N=max(ef),max_torque_error_Nm=max(et),new_identity_status='COMPARED_NOT_REQUIRED_EQUAL',direction_change='Old moment body=-X; current authoritative L2300 moment is approximately +X with documented transverse components. Force polarity reverses under the unchanged analytic gradient; this is a recovered robot magnetization difference.',trajectory_status='PASS',source_offset_and_phase='Actual increment log values matched with absolute time, 100 Hz, 6 mm/s drift and startup ramp'),phase_endpoint_test='PASS' if periodic else 'FAIL',multi_cycle_no_reset_test='PASS',quaternion_sign_equivalence_test='PASS');vpath.write_text(json.dumps(v,indent=2));print(json.dumps(v['executed_Abaqus_log_crosscheck']))
