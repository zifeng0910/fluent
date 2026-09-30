"""Independent direct-source reference for C campaign validation."""
from __future__ import annotations
import math
import numpy as np
from scipy.spatial.transform import Rotation
M_BODY=np.array([1.0399958044715794e-3,9.084781997334923e-6,3.1131721660494084e-6])
COM0=np.array([.0012060186937156343,0.,0.])

def compute_magnetic_load(com_m, orientation_xyzw, absolute_time_s):
 p=np.asarray(com_m,dtype=float);q=np.asarray(orientation_xyzw,dtype=float);t=float(absolute_time_s)
 if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or not 0<=t<=.05: raise ValueError('invalid analytic baseline state')
 if np.any(p<[-.004,-.0009,-.0009]) or np.any(p>[.006,.0009,.0009]):raise ValueError('ANALYTIC_BASELINE_VALIDATION_ENVELOPE_EXCEEDED')
 mg=Rotation.from_quat(q).apply(M_BODY)
 phase=2*math.pi*100*t;am=math.radians(14.5)*math.sin(phase);ac=math.radians(2.5)*math.cos(phase)
 direction=np.array([-1.,-math.tan(am),math.tan(ac)]);B=.011*direction/np.linalg.norm(direction)
 seff=31.699921242759284+(p[0]-COM0[0])*1000-6*t
 Gxx=-.0022*(seff/(45*45))*math.exp(-.5*(seff/45)**2)*1000
 ramp=1 if t>=.001 else (t/.001)**2*(3-2*t/.001)
 F=np.array([ramp*mg[0]*Gxx,0.,0.]);T=ramp*np.cross(mg,B)
 return F,T,dict(moment_global=mg,B_global=B,s_eff_mm=seff,phase_rad=phase,ramp=ramp)
