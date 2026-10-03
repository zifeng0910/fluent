"""Offline audit of the already-solved native step30. No solver or dynamics launch."""
import csv, io, math, shutil
import numpy as np, pyvista as pv
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_common import *
from benchmark_C_coarse_dynamic import select_native_callback

def main():
    preserve(); cont=EVID/'continuation_20h'; original=cont/'resume29_01'; dest=cont/'callback_repair_review';dest.mkdir(exist_ok=True)
    old=read(original/'failure.json'); saved=read(original/'stop_checkpoint.json')
    t=float(saved['time_s']); step=int(round(t/25e-6))
    if step!=30 or abs(t-step*25e-6)>1e-12 or old['error']!='StopIteration()':raise RuntimeError('Unexpected failure/state')
    for k in ['case','data']:
        if sha(Path(saved[k]))!=saved[k+'_sha256']:raise RuntimeError('Original native checkpoint hash changed')
    for source in [original/'failure.json',original/'stop_checkpoint.json',original/'solver.trn',EVID/'fielddata/connectivity_0030.json',EVID/'dynamic_history.csv']:
        target=dest/source.name
        if not target.exists():shutil.copy2(source,target)
        elif sha(target)!=sha(source):raise RuntimeError('Review archive differs from original')
    callback_path=ROOT/'evidence/benchmark_C_analytic_6dof_history.csv'
    with callback_path.open('rb') as f:
        header=f.readline();f.seek(old['dynamic']['resume_callback_offset']);tail=f.read()
    archived=dest/'callback_tail_0030.csv'
    if not archived.exists():archived.write_bytes(header+tail)
    elif archived.read_bytes()!=header+tail:raise RuntimeError('New callbacks appeared during offline audit')
    callbacks=list(csv.DictReader(io.StringIO((header+tail).decode('utf-8'))))
    passive=[r for r in callbacks if r['mode']=='FREE_6DOF' and r['zone_name']=='robot_component_fluid' and abs(float(r['time_s'])-t)<1e-10]
    if passive:raise RuntimeError('Expected missing passive callback defect not reproduced')
    chosen,gate=select_native_callback(callbacks,t,saved['native_state'])
    # Negative check: an available callback must never bypass native-state agreement.
    wrong={**saved['native_state'],'com':[saved['native_state']['com'][0]+1e-5,0,0]}
    rejected=False
    try:select_native_callback(callbacks,t,wrong)
    except RuntimeError:rejected=True
    if not rejected:raise RuntimeError('Wrong native body state was accepted')
    rowprev=list(csv.DictReader((dest/'dynamic_history.csv').open()))
    if len(rowprev)!=29 or int(rowprev[-1]['step'])!=29:raise RuntimeError('Expected exact history prefix1..29')
    stats=read(dest/'connectivity_0030.json');actual=saved['native_state']
    robot=pv.read(EVID/'fielddata/robot_0000.vtp');env=pv.read(EVID/'fielddata/component_0000.vtp');pipe=pv.read(EVID/'fielddata/pipe.vtp')
    R=Rotation.from_quat(np.array(actual['q'])[[1,2,3,0]])
    for mesh in [robot,env]:mesh.points=R.apply(mesh.points-np.array(COM))+np.array(actual['com'])
    inside=float(np.sign(pv.PolyData(np.array([COM])).compute_implicit_distance(pipe)['implicit_distance'][0]))
    gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*inside));envgap=float(np.min(env.compute_implicit_distance(pipe)['implicit_distance']*inside))
    maxr=old['dynamic']['maximum_component_radius_m']
    sampled=[Rotation.from_rotvec(np.array(a)*theta) for a in [AXIS,ORTH] for theta in np.linspace(0,.35,15)]
    bound=min(2*math.sin(float((R*rot.inv()).magnitude())/2)*maxr for rot in sampled)+float(np.linalg.norm(np.array(actual['com'])-np.array(COM)))
    columns=[f'{p}_{a}_{u}' for p,u in [('omega','rad_s'),('Fmag','N'),('Tmag','Nm')] for a in 'xyz']+[f'v{a}_m_s' for a in 'xyz']+[f'theta_{a}_rad' for a in 'xyz']
    row={'step':step,'time_s':t,**{f'com_{a}_m':float(v) for a,v in zip('xyz',actual['com'])},**{f'q{i}':float(chosen[f'q{i}']) for i in range(4)},'q_norm':float(np.linalg.norm([float(chosen[f'q{i}']) for i in range(4)])),**{k:float(chosen[k]) for k in columns},
        'orphan_count':stats['orphans'],'receptors_without_donors':stats['invalid_donors'],'official_donor_count':stats['donors'],'official_receptor_count':stats['receptors'],'minimum_cell_volume_m3':stats['minimum_volume_m3'],
        'robot_wall_clearance_m':gap,'overset_wall_clearance_m':envgap,**{f'donor_length_ratio_{k}':stats['donor_characteristic_length_ratio'][k] for k in ['min','median','p95','max']},'BOI_pose_displacement_bound_m':bound}
    if list(row)!=list(rowprev[-1]):raise RuntimeError('Recovered history schema differs')
    if not np.isfinite(list(row.values())).all() or stats['orphans'] or stats['invalid_donors'] or stats['cell_type_counts']['-3'] or stats['minimum_volume_m3']<=0 or gap<.0001 or envgap<=0 or abs(row['q_norm']-1)>1e-6 or bound>.00011:
        raise RuntimeError('Offline recovered row failed a physical/numerical gate')
    checkpoint={**saved,'step':step,'original_mislabeled_history_step':saved['step'],'provisional_row':row,'status':'PROVISIONAL_NATIVE_STEP30_PENDING_COLD_GATE'}
    atomic(dest/'step30_checkpoint.json',checkpoint)
    rec={'timestamp':stamp(),'status':'OFFLINE_NATIVE_CALLBACK_MATCH_PASS_PENDING_COLD_GATE','root_cause':'Cold native restart produced four active robot_wall callbacks at0.750ms and no passive component callback. The Python reader required the passive callback and raised StopIteration after step30 completed. Native checkpoint metadata erroneously used last committed history index29 instead of native time index30.',
         'failure_class':'PYTHON_CALLBACK_SELECTION','timesteps_advanced':0,'original_native_time_s':t,'actual_native_step':step,'recorded_history_step':29,'passive_callbacks_at_time':0,'active_callbacks_at_time':len(callbacks),
         'callback_native_gate':gate,'wrong_native_state_rejected':rejected,'provisional_row':row,
         'clearance_measurement':'Offline rigid transform of original FieldData geometry with persisted native COM/orientation; actual cold native FieldData clearance must still pass before row promotion.',
         'case_data_hashes_verified':True,'frozen_UDF_sha256':FROZEN_SHA,'frozen_files_verified':True,
         'required_next':'One minimal callback-reader/accounting repair. Load actual native step30 checkpoint, verify zero-step state/magnetics/FieldData/connectivity, append recovered30 only after PASS, then continue31..40; no replay of30.',
         'evidence_sha256':{str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in dest.iterdir() if p.is_file() and p.name!='review.json'}}
    atomic(dest/'review.json',rec)
    print({k:rec[k] for k in ['status','actual_native_step','timesteps_advanced','callback_native_gate','wrong_native_state_rejected']})

if __name__=='__main__':main()
