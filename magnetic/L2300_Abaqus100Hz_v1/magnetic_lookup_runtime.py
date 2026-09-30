"""Continuous factorized analytic baseline. Physical Magpylib gate remains closed."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
class Lookup:
 def __init__(self,path=HERE/'magnetic_lookup.npz'):
  self.data=np.load(path);self.manifest=json.loads((HERE/'external_source_provenance.json').read_text());self.m=np.array(self.manifest['magnetic_moment_body_Am2']);self.com0=np.array([.0012060186937156343,0,0])
 def evaluate(self,com_m,quaternion_xyzw,t_s):
  p=np.asarray(com_m,float);q=np.asarray(quaternion_xyzw,float)
  if p.shape!=(3,) or not np.isfinite(p).all() or np.any(p<self.data['position_lower']) or np.any(p>self.data['position_upper']):raise ValueError('HARD STOP: position outside coverage')
  if not np.isfinite(t_s) or not 0<=t_s<=.05:raise ValueError('HARD STOP: time outside coverage')
  if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:raise ValueError('HARD STOP: invalid orientation quaternion')
  phase=(2*np.pi*100*t_s)%(2*np.pi);s=.031699921242759284+p[0]-self.com0[0]-.006*t_s
  if not self.data['s_m'][0]<=s<=self.data['s_m'][-1]:raise ValueError('HARD STOP: source axial coordinate outside table')
  B=np.array([np.interp(phase,self.data['phase_rad'],self.data['B_T'][:,i]) for i in range(3)])
  G=np.array([np.interp(s,self.data['s_m'],self.data['G_T_m'][:,i]) for i in range(9)]).reshape(3,3)
  ramp=1 if t_s>=.001 else (t_s/.001)**2*(3-2*t_s/.001)
  m=Rotation.from_quat(q).apply(self.m)
  return ramp*G.T@m,ramp*np.cross(m,B)
def torque_to_com(torque_rp_Nm,rp_m,com_m,force_N):
 return np.asarray(torque_rp_Nm)+np.cross(np.asarray(rp_m)-np.asarray(com_m),force_N)
