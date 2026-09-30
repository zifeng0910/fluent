"""Characterize the saved failed trajectory without evolving or modifying it."""
import csv,json
from pathlib import Path
import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
def main():
    rows=[r for r in csv.DictReader((ROOT/'evidence/benchmark_C_analytic_6dof_history.csv').open()) if r['mode']=='FREE_6DOF']
    # Keep only the final run; the append-only source contains the default-cutoff run too.
    resets=[i for i in range(1,len(rows)) if float(rows[i]['time_s'])<float(rows[i-1]['time_s'])-1e-12]
    if resets:rows=rows[resets[-1]:]
    # The UDF calls twice per time step for associated rigid-body zones.
    rows=list({float(r['time_s']):r for r in rows}.values());last=rows[-1]
    COM=np.array([.0012060186937156343,0,0]);cg=np.array([float(last[f'cg_{c}_m']) for c in 'xyz'])
    q=np.array([float(last[f'q{i}']) for i in range(4)]);R=Rotation.from_quat(np.r_[q[1:],q[0]])
    metrics={}
    for name in ['robot','component']:
        points=pv.read(ROOT/f'evidence/benchmark_C_fielddata/{name}_0000.vtp').points
        actual=R.apply(points-COM)+cg
        metrics[name+'_clearance_to_CAD_pipe_mm']=float(.9-np.max(np.linalg.norm(actual[:,1:],axis=1))*1000)
    qnorm=[float(np.linalg.norm([float(r[f'q{i}']) for i in range(4)])) for r in rows]
    report=dict(status='FAIL_OLD_OVERSET_ENVELOPE',failure_time_s=float(last['time_s']),orphan_count=1,missing_donors=0,
      reconstruction='Saved initial Fluent FieldData vertices transformed by saved native quaternion and COM; radial clearance to authoritative cylindrical CAD pipe R=0.9mm.',
      quaternion_norm_at_failure=float(np.linalg.norm(q)),peak_quaternion_norm_error=max(abs(x-1) for x in qnorm),
      failure_rotation_rad=float(R.magnitude()),failure_axis=(R.as_rotvec()/R.magnitude()).tolist(),
      initial_COM_m=COM.tolist(),failure_COM_m=cg.tolist(),net_COM_displacement_m=(cg-COM).tolist(),
      failure_omega_rad_s=[float(last[f'omega_{c}_rad_s']) for c in 'xyz'],
      peak_angular_speed_rad_s=max(np.linalg.norm([float(r[f'omega_{c}_rad_s']) for c in 'xyz']) for r in rows),
      peak_magnetic_force_N=max(np.linalg.norm([float(r[f'Fmag_{c}_N']) for c in 'xyz']) for r in rows),
      peak_magnetic_torque_Nm=max(np.linalg.norm([float(r[f'Tmag_{c}_Nm']) for c in 'xyz']) for r in rows),
      determinant_cutoff=dict(modified=1e-50,actual_inertia_determinant=1.1206106483031212e-35,
       original_observed_value=None,original_capture_status='NOT_CAPTURED_BEFORE_CHANGE',default_run_angular_acceleration_warning=True,real_inertia_modified=False),**metrics)
    (ROOT/'evidence/benchmark_C_old_failure_characterization.json').write_text(json.dumps(report,indent=2))
    for r,norm in zip(rows,qnorm):r['q_norm']=norm
    with (ROOT/'evidence/benchmark_C_saved_dynamic_history_with_qnorm.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
