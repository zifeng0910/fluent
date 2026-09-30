"""Compare SI port with archived Abaqus loads over all five cycles."""
from pathlib import Path
import csv,json,hashlib
import numpy as np
from magnetic_table_si import MagneticTable,rotation_from_rotvec

src=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis/TrueCELLongForwardTransit/case/F100_G2P20_NOFLUID_REALWALL50')
model=MagneticTable(src/'magnetic_field_gradient_table_B0P11_A14P5.dat')
with np.load(src/'private/rp_history_private.npz') as archive:
    history={key:archive[key] for key in ['U1','U2','U3','UR1','UR2','UR3']}
sample=[]
with (src/'magnetic_increment_g2p20_f100.csv').open() as f:
    rows=csv.reader(f);next(rows)
    for i,row in enumerate(rows):
        if i%1000==0:sample.append(list(map(float,row)))
sample=np.array(sample)
errors=[]
for row in sample:
    t=row[0]
    # VUAMP sensors correspond to the already completed previous increment.
    sensor_time=max(0,t-1e-7)
    u=np.array([np.interp(sensor_time,history['U'+str(i)][:,0],history['U'+str(i)][:,1]) for i in range(1,4)])*.001
    ur=np.array([np.interp(sensor_time,history['UR'+str(i)][:,0],history['UR'+str(i)][:,1]) for i in range(1,4)])
    force,torque=model.evaluate(t,u,rotation_from_rotvec(ur))
    errors.append(np.r_[force-row[3:6],torque-row[6:9]*.001])
errors=np.array(errors)
peaks=np.r_[np.max(np.linalg.norm(sample[:,3:6],axis=1)),np.max(np.linalg.norm(sample[:,6:9]*.001,axis=1))]
relative=np.array([np.max(np.linalg.norm(errors[:,:3],axis=1)),np.max(np.linalg.norm(errors[:,3:],axis=1))])/peaks
result={'status':'PASS' if max(relative)<1e-3 else 'FAIL','scope':'Archived pose/load replay only; not a coupled dynamics validation',
        'samples':len(sample),'start_s':sample[0,0],'end_s':sample[-1,0],
        'max_force_vector_error_N':float(np.max(np.linalg.norm(errors[:,:3],axis=1))),
        'max_torque_vector_error_Nm':float(np.max(np.linalg.norm(errors[:,3:],axis=1))),
        'force_error_over_peak':relative[0],'torque_error_over_peak':relative[1],
        'acceptance_relative_to_peak':1e-3,'sensor_lag_s':1e-7,
        'table_sha256':hashlib.sha256((src/'magnetic_field_gradient_table_B0P11_A14P5.dat').read_bytes()).hexdigest()}
out=Path('H:/fluent/evidence/F100_G2P20_NOFLUID_REALWALL50/magnetic_replay_check.json')
out.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
assert result['status']=='PASS'
