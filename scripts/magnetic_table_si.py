"""SI port of the audited F100 VUAMP table law; no fluid/contact surrogate.

Position input is displacement of the original Abaqus reference point, NOT COM.
Rotation maps the initial body into the current global frame. Force and torque
outputs are N and N*m; source grad_unit already has units 1/m.
"""
from pathlib import Path
import numpy as np

class MagneticTable:
    axis = np.array([.9762799602464296,-.0618320247446977,.2074951564186524])
    initial_moment = np.array([-1.061824299255672e-3,6.72499168508204e-5,-2.25676452654195e-4])

    def __init__(self, path):
        with Path(path).open() as f:
            n_s,n_phase,*_=map(float,f.readline().split())
            self.phases=np.array([list(map(float,f.readline().split())) for _ in range(int(n_phase))])
            self.spatial=np.array([list(map(float,f.readline().split())) for _ in range(int(n_s))])

    def evaluate(self, time_s, reference_displacement_m, rotation_matrix):
        rotation=np.asarray(rotation_matrix,dtype=float)
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-9) or np.linalg.det(rotation)<0:
            raise ValueError('Invalid proper rotation matrix')
        station=31.699921242759284+np.dot(reference_displacement_m,self.axis)*1000-6*time_s
        if not self.spatial[0,0] <= station <= self.spatial[-1,0]:
            raise ValueError(f'Magnetic table extrapolation forbidden: {station} mm')
        phase=(36000*time_s)%360
        b=.011*np.array([np.interp(phase,self.phases[:,0],self.phases[:,i]) for i in range(1,4)])
        gradient=.0022*np.array([np.interp(station,self.spatial[:,0],self.spatial[:,i]) for i in range(1,10)]).reshape(3,3)
        moment=rotation@self.initial_moment
        q=np.clip(time_s/.001,0,1); ramp=q*q*(3-2*q)
        return ramp*(moment@gradient),ramp*np.cross(moment,b)

def rotation_from_rotvec(vector):
    vector=np.asarray(vector);theta=np.linalg.norm(vector)
    if theta<1e-14:return np.eye(3)
    x,y,z=vector/theta
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(theta)*cross+(1-np.cos(theta))*(cross@cross)
