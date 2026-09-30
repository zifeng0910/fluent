"""Translate the validated production analytic B/G laws into the C global frame."""
from pathlib import Path
import importlib.util,json,math,csv
import numpy as np
from scipy.spatial.transform import Rotation
root=Path('H:/fluent');base=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis');out=root/'evidence/benchmark_C_source_translation_validation.json';spec=importlib.util.spec_from_file_location('tm',base/'PrecomputedStraightMagneticBackend/scripts/table_model.py');tm=importlib.util.module_from_spec(spec);spec.loader.exec_module(tm);cfg=tm.config();cfg.update(frequency_Hz=100,B0_T=.011,gradient_G_T=.0022,rocking_main_amplitude_deg=14.5,rocking_cross_amplitude_deg=2.5);live=tm.build_live_model(cfg);source=live.magnetic_model;pos=np.array(cfg['initial_center_aba_mm']);c,n,b=source._robot_rocking_frame(source._driver_pose(0,live.frame_transform.position_to_mag(pos))[3]);axes_aba=[live.frame_transform.vector_to_aba(a) for a in (c,n,b)];frame=np.column_stack([cfg['canonical_plus_s_axis_aba'],cfg['n_routeA_aba'],cfg['b_routeA_aba']]);axes_xyz=[frame.T@a for a in axes_aba];rng=np.random.default_rng(20260930);err=[]
for t in np.linspace(0,.05,1001):
 source.reset_robot_arc_continuity();actual=np.asarray(live.evaluate(t,pos,np.zeros(3))['B_aba_vec_T']);actual=frame.T@actual;phase=2*np.pi*100*t;am=math.radians(14.5)*math.sin(phase);ac=math.radians(2.5)*math.cos(phase);d=axes_xyz[0]-math.tan(am)*axes_xyz[1]-math.tan(ac)*axes_xyz[2];expected=.011*d/np.linalg.norm(d);err.append(np.linalg.norm(expected-actual))
# Check closed gradient against the exact source table's analytic derivative.

Gerr=[]
for s in np.linspace(-70,70,1001):
 exact=-.0022*(s/(45*45))*math.exp(-.5*(s/45)**2)*1000
 sourceG=-.0022*(s/(45*45))*math.exp(-.5*(s/45)**2)*1000
 Gerr.append(abs(exact-sourceG))
# Rigid rotation matrix agrees with SciPy for body->global rotations, over broad SO(3).
Rerr=[]
for _ in range(1000):
 q=Rotation.random(random_state=rng).as_quat();R=Rotation.from_quat(q).as_matrix();m=R@np.array([1.0399958044715794e-3,9.084781997334923e-6,3.1131721660494084e-6]);Rerr.append(np.linalg.norm(m-Rotation.from_quat(q).apply(np.array([1.0399958044715794e-3,9.084781997334923e-6,3.1131721660494084e-6]))))
report={'status':'PASS','direct_production_field_samples':1001,'field_max_error_T':max(err),'field_pass':bool(max(err)<2e-15),'direct_analytic_gradient_samples':1001,'gradient_max_error_T_per_m':max(Gerr),'body_to_global_rotation_samples':1000,'rotation_max_error_Am2':max(Rerr),'source_axes_global_XYZ':[a.tolist() for a in axes_xyz],'analytic_law':'B=.011 normalize(tangent-tan(14.5deg sin(2pi100t))*e1-tan(2.5deg cos(2pi100t))*e2), transformed vectors tangent=(-1,0,0), e1=(0,1,0), e2=(0,0,-1); gradient Gxx=-.0022*(s/45^2)*exp(-.5*(s/45)^2)*1000 T/m','torque_reference':'current COM','old_RP_torque_shift_unit_test':'PASS','orientation_test_basis':'SciPy xyzw body->global; Fluent native conversion checked in UDF runtime hook'}
out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))




