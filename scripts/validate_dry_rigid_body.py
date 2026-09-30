"""Independent SI rigid-body magnetic-only reproduction, not a Fluent CFD run."""
from pathlib import Path
import json,sys
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
from magnetic_table_si import MagneticTable

root=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis/TrueCELLongForwardTransit')
name='TRUECEL_B0P11_G2P20_F100_NOFLUID_CONTROL'
case=root/'case'/name
audit=json.loads(Path('H:/fluent/evidence/F100_G2P20_NOFLUID_REALWALL50/geometry_audit.json').read_text())
identity=json.loads((case/'case_identity.json').read_text())
model=MagneticTable(case/'magnetic_field_gradient_table_B0P11_A14P5.dat')
rp0=np.array(identity['initial_center_aba_mm'])*1e-3
com0=np.array(audit['com_global_m'])
arm0=rp0-com0
inertia_model='lumped' if '--lumped' in sys.argv else 'continuous'
mass=audit['mass_kg']; inertia=np.array(audit['lumped_vertex_inertia_global_kg_m2' if inertia_model=='lumped' else 'inertia_global_kg_m2']); inv=np.linalg.inv(inertia)

def derivative(t,y):
    rotation=Rotation.from_quat(y[6:10]).as_matrix()
    arm=rotation@arm0
    rp_displacement=y[:3]+arm-arm0
    force,torque_rp=model.evaluate(t,rp_displacement,rotation)
    torque_com=torque_rp+np.cross(arm,force)
    omega=y[10:13]
    qv=y[6:9];qw=y[9]
    qdot=.5*np.r_[qw*omega+np.cross(qv,omega),-np.dot(qv,omega)]
    odot=inv@(rotation.T@torque_com-np.cross(omega,inertia@omega))
    return np.r_[y[3:6],force/mass,qdot,odot]

times=np.linspace(0,.05,5001)
start=np.zeros(13);start[9]=1
solution=solve_ivp(derivative,(0,.05),start,t_eval=times,rtol=1e-9,atol=1e-12,max_step=1e-5)
assert solution.success,solution.message
rotation=Rotation.from_quat(solution.y[6:10].T)
rp=solution.y[:3].T+rotation.apply(np.tile(arm0,(len(times),1)))+com0
history=np.genfromtxt(root/(name+'_RP_MAGNETIC_10US.csv'),delimiter=',',names=True)
reference=np.column_stack([history[k] for k in ('rp_x_mm','rp_y_mm','rp_z_mm')])*1e-3
position_error=np.linalg.norm(rp-reference,axis=1)
with np.load(case/'private/rp_history_private.npz') as archive:
    ref_rotvec=np.column_stack([np.interp(times,archive[k][:,0],archive[k][:,1]) for k in ('UR1','UR2','UR3')])
angle_error=np.rad2deg((Rotation.from_rotvec(ref_rotvec).inv()*rotation).magnitude())
displacement=(rp-rp0)@model.axis*1000
result={'status':'PASS' if position_error.max()<5e-6 and angle_error.max()<.1 else 'FAIL',
        'scope':'Independent Python magnetic-only rigid body; no fluid and no contact; NOT Fluent CFD',
        'source_case':name,'duration_s':.05,'cycles':5,'nfev':solution.nfev,
        'max_position_error_m':float(position_error.max()),'max_orientation_error_deg':float(angle_error.max()),
        'position_gate_m':5e-6,'orientation_gate_deg':.1,
        'python_final_displacement_mm':float(displacement[-1]),'abaqus_final_displacement_mm':float(history['s_displacement_mm'][-1]),
        'torque_conversion':'tau_COM = tau_RP + (RP-COM) cross force','mass_kg':mass,
        'inertia_model':inertia_model,
        'cycle_displacement_mm':[float(displacement[(i+1)*1000]-displacement[i*1000]) for i in range(5)]}
out=Path('H:/fluent/evidence/dry_rigid_body')/inertia_model;out.mkdir(parents=True,exist_ok=True)
(out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
np.savetxt(out/'trajectory_SI.csv',np.column_stack([times,rp,solution.y[3:6].T,solution.y[6:10].T,solution.y[10:13].T,position_error,angle_error]),delimiter=',',header='time_s,rp_x_m,rp_y_m,rp_z_m,com_vx_m_s,com_vy_m_s,com_vz_m_s,qx,qy,qz,qw,body_omega_x_rad_s,body_omega_y_rad_s,body_omega_z_rad_s,position_error_m,orientation_error_deg',comments='')
print(json.dumps(result,indent=2))
