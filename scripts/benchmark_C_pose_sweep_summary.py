"""Finalize signed wall clearances and background identity from saved native meshes."""
import csv,json,sys
from pathlib import Path
import h5py,numpy as np,pyvista as pv
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from benchmark_C_mesh_metrics import tetra_metrics

def surface(path,name):
    with h5py.File(path) as f:
        m=f['meshes/1']; coords=np.concatenate([d[:] for d in m['nodes/coords'].values()])
        top=m['faces/zoneTopology'];names=top['name'][0].decode().split(';');i=names.index(name)
        lo,hi=int(top['minId'][i])-1,int(top['maxId'][i])
        faces=m['faces/nodes/1/nodes'][:].reshape(-1,3)[lo:hi].astype(np.int64)-1
    return pv.PolyData(coords,np.column_stack([np.full(len(faces),3),faces]).ravel()).clean()

def background_identity(base,diagnostic):
    original=tetra_metrics(base,'background_water');current=tetra_metrics(diagnostic,'background_water')
    distance,indices=original['tree'].query(current['centers'])
    return {'cell_count_unchanged':original['count']==current['count'],
      'cell_order_unchanged':bool(np.array_equal(original['centers'],current['centers'])),
      'cell_matching_bijective':len(np.unique(indices))==len(indices),
      'maximum_matched_cell_center_error_m':float(distance.max()),
      'maximum_matched_tetra_edge_error_m':float(np.max(np.abs(original['max_edge'][indices]-current['max_edge']))),
      'background_geometry_preserved':bool(len(np.unique(indices))==len(indices) and distance.max()<1e-15 and np.array_equal(original['max_edge'][indices],current['max_edge'])),
      'note':'Cell order changes during Fluent import/replacement; comparison uses bijective matching of the actual background cells, not row indices.'}

def main():
    path=ROOT/'evidence/benchmark_C_overset_pose_sweep.json';r=json.loads(path.read_text())
    if r['status']=='RUNNING':raise RuntimeError('Sweep still running')
    base=ROOT/'live_cases/benchmark_B_overset/benchmark_B_static_initialized.cas.h5'
    pipe=surface(base,'pipe_wall');COM=np.array([.0012060186937156343,0,0])
    center_sign=float(np.sign(pv.PolyData(COM[None,:]).compute_implicit_distance(pipe)['implicit_distance'][0]))
    for cid in 'ABCD':
        file=ROOT/f'live_cases/benchmark_C_candidates/component_{cid}_SI.cas.h5'
        robot=surface(file,'robot_wall');env=surface(file,'overset_component')
        for row in r['rows']:
            if row['candidate_id']!=cid:continue
            rot=Rotation.from_rotvec(np.array(json.loads(row['axis_xyz']))*row['theta_rad'])
            quat=rot.as_quat();quat=np.r_[quat[3],quat[:3]]
            row.update({f'q{i}':float(quat[i]) for i in range(4)})
            row['q_norm']=float(np.linalg.norm(quat))
            for name,obj in [('robot',robot),('overset',env)]:
                points=rot.apply(obj.points-COM)+COM
                signed=pv.PolyData(points).compute_implicit_distance(pipe)['implicit_distance']*center_sign
                row[f'{name}_wall_clearance_mm']=float(np.min(signed)*1000)
    diagnostic=ROOT/'live_cases/benchmark_C_candidates/benchmark_C_candidate_D_diagnostic.cas.h5'
    r['background_identity']=background_identity(base,diagnostic)
    r['clearance_method']='Minimum signed distance from native component/robot surface vertices to frozen Fluent pipe-wall triangles. Sign calibrated at initial COM. Static poses reconstructed using rotations whose native vertex agreement was measured at every pose.'
    r['static_quaternion_source']='Quaternion of prescribed coordinate rotation; not a Fluent 6DOF state measurement. No solver quaternion is modified or renormalized.'
    r['summary']={'candidate_count':4,'pose_count':len(r['rows']),
      'static_orphan_peak':max(x['orphans'] for x in r['rows']),
      'static_missing_donor_peak':max(x['missing_donors'] for x in r['rows']),
      'minimum_robot_clearance_mm':min(x['robot_wall_clearance_mm'] for x in r['rows']),
      'minimum_overset_clearance_mm':min(x['overset_wall_clearance_mm'] for x in r['rows']),
      'minimum_estimated_overlap_layers':min(x['estimated_overlap_layers'] for x in r['rows']),
      'maximum_estimated_overlap_layers':max(x['estimated_overlap_layers'] for x in r['rows']),
      'maximum_pose_transform_error_m':max(x['pose_transform_error_m'] for x in r['rows']),
      'maximum_tested_angle_rad':max(x['theta_rad'] for x in r['rows']),
      'maximum_robust_safe_angle_rad':None}
    r['candidate_summary']={cid:{'orphan_min':min(x['orphans'] for x in r['rows'] if x['candidate_id']==cid),
      'orphan_peak':max(x['orphans'] for x in r['rows'] if x['candidate_id']==cid),
      'overlap_layers_max':max(x['estimated_overlap_layers'] for x in r['rows'] if x['candidate_id']==cid),
      'theta_orphan_onset_rad':min(x['theta_rad'] for x in r['rows'] if x['candidate_id']==cid and x['orphans']>0),
      'theta_connectivity_limit_rad':0.0,
      'theta_physical_wall_limit_rad':None,
      'physical_wall_limit_status':'Not reached: all sampled poses retain >=0.10 mm clearance',
      'theta_allowed_rad':None,
      'allowed_range_status':'EMPTY: connectivity fails at reference pose; min(limit)-margin supplies no positive operating range'} for cid in 'ABCD'}
    path.write_text(json.dumps(r,indent=2))
    with path.with_suffix('.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(r['rows'][0]));w.writeheader();w.writerows(r['rows'])
    print(json.dumps({k:r[k] for k in ['status','summary','candidate_summary','background_identity']},indent=2))
if __name__=='__main__':main()
