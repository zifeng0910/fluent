"""Rebuild only the recovered analytic law, without creating a physical magnet."""
from pathlib import Path
import json,csv,importlib.util
import numpy as np,yaml
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
P=json.loads((HERE/'external_source_provenance.json').read_text());BASE=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis')
spec=importlib.util.spec_from_file_location('authoritative',BASE/'PrecomputedStraightMagneticBackend/scripts/table_model.py');source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
cfg=source.config();cfg.update(frequency_Hz=100,B0_T=.011,gradient_G_T=.0022,rocking_main_amplitude_deg=14.5,rocking_cross_amplitude_deg=2.5)
S=np.array(P['source_frame_aba']);phase=np.linspace(0,2*np.pi,1441);axial=np.linspace(-.07,.07,1401)
live=source.build_live_model(cfg)
def direct_B(p):
 live.magnetic_model.reset_robot_arc_continuity()
 return np.asarray(live.evaluate(p/(2*np.pi*100),np.asarray(cfg['initial_center_aba_mm']),np.zeros(3))['B_aba_vec_T'])
B=np.array([S.T@direct_B(p) for p in phase]);G=np.array([(S.T@source.gradient_unit(x*1000,cfg)@S*.0022).reshape(9) for x in axial])
np.savez_compressed(HERE/'magnetic_lookup.npz',phase_rad=phase,s_m=axial,B_T=B,G_T_m=G,position_lower=[-.004,-.0009,-.0009],position_upper=[.006,.0009,.0009])
with (HERE/'magnetic_lookup.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['factor','coordinate','v1','v2','v3','v4','v5','v6','v7','v8','v9'])
 for x,b in zip(phase,B):w.writerow(['phase_rad',x,*b,*['']*6])
 for x,g in zip(axial,G):w.writerow(['axial_m',x,*g])
manifest=dict(status='ANALYTIC_BASELINE_ONLY',physical_source_status=P['physical_source_status'],magnetic_sphere_example_used=False,authoritative_frequency_hz=100,legacy_frequency_hz=120,resolution='source-code precedence',input_dimensions=['COM x','COM y','COM z','orientation SO(3)','absolute time'],representation='B(phase) and G(axial) factors; exact quaternion rotation and exact startup/time drift',position_bounds_m=[[-.004,.006],[-.0009,.0009],[-.0009,.0009]],orientation_bounds='full SO(3), no locked pose',time_bounds_s=[0,.05],phase_bounds_rad=[0,float(2*np.pi)],grid_resolution=dict(phase_deg=.25,source_axial_mm=.1),interpolation='linear position/periodic phase; orientation applied analytically and continuously',outside_domain='HARD STOP',coordinate_reference='COM',torque_reference='COM',force_units='N',torque_units='N*m',coordinate_transform='proper rigid coordinate re-expression: original [c,n1,n2] becomes current global [X,Y,Z]; COM0 replaces old RP origin, external relative axial offset preserved',transverse_dependence='none in recovered analytic source; not claimed as a physical magnetic field',benchmark_C_ready=False)
(HERE/'magnetic_lookup_manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
from magnetic_lookup_runtime import Lookup,torque_to_com
runtime=Lookup();rng=np.random.default_rng(100);samples=1000;ef=[];et=[]
for i in range(samples):
 p=rng.uniform([-.004,-.0009,-.0009],[.006,.0009,.0009]);q=Rotation.random(random_state=rng).as_quat();t=rng.uniform(0,.05);m=Rotation.from_quat(q).apply(runtime.m);x=.031699921242759284+p[0]-runtime.com0[0]-.006*t
 bd=S.T@direct_B(2*np.pi*100*t);gd=S.T@source.gradient_unit(x*1000,cfg)@S*.0022;scale=source.ramp(t,cfg);fd=scale*gd.T@m;td=scale*np.cross(m,bd);fl,tl=runtime.evaluate(p,q,t)
 ef.append(np.linalg.norm(fd-fl));et.append(np.linalg.norm(td-tl))
# Strict same-identity check of the executed Fortran table against independent closed law.
case=BASE/'TrueCELLongForwardTransit/case/TRUECEL_B0P11_G2P20_F100_NOFLUID_CONTROL';op,os=source.read_table(case/'magnetic_field_gradient_table_B0P11_A14P5.dat')
oldB=max(np.linalg.norm(direct_B(np.deg2rad(row[0]))/.011-row[1:]) for row in op)
oldG=max(np.linalg.norm(source.gradient_unit(row[0],cfg).reshape(9)-row[1:]) for row in os)
shift=torque_to_com([1,2,3],[1,0,0],[0,0,0],[0,1,0]);assert np.allclose(shift,[1,2,4])
for args in [([.007,0,0],[0,0,0,1],.01),([.001,0,0],[0,0,0,1],.051)]:
 try:runtime.evaluate(*args);raise AssertionError('No hard stop')
 except ValueError:pass
# Retain nonperiodic source drift and ramp in the trajectory, including cycle boundaries.
with (HERE/'abaqus100Hz_external_source_cycle.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['time_s','phase_rad_unwrapped','phase_rad_periodic','analytic_driver_axial_offset_m','ramp','physical_center_x','physical_center_y','physical_center_z','physical_orientation','analytic_Bx_T','analytic_By_T','analytic_Bz_T'])
 for t in np.linspace(0,.05,1001):w.writerow([t,2*np.pi*100*t,(2*np.pi*100*t)%(2*np.pi),.006*t,source.ramp(t,cfg),'UNKNOWN','UNKNOWN','UNKNOWN','NOT_DEFINED_ANALYTIC_SOURCE',*(S.T@direct_B(2*np.pi*100*t))])
validation=dict(status='ANALYTIC_BASELINE_VALIDATED' if max(ef)<1e-9 and max(et)<1e-9 and oldB<1e-10 and oldG<1e-9 else 'FAIL',sample_count=samples,direct_source_calculation='PASS' if max(ef)<1e-9 and max(et)<1e-9 else 'FAIL',max_force_absolute_error_N=max(ef),max_torque_absolute_error_Nm=max(et),executed_table_source_reproduction='PASS' if oldB<1e-10 and oldG<1e-9 else 'FAIL',old_table_B_unit_max_error=oldB,old_table_gradient_unit_max_error=oldG,RP_to_COM_torque_shift_test='PASS',coverage_hard_stop_tests='PASS',physical_Magpylib_validation='NOT_RUN_NO_UNIQUE_PHYSICAL_SOURCE',new_production_lookup='FAIL',benchmark_C_ready=False,remaining_blocker=P['physical_source_status'],orientation_continuity='exact quaternion rotation; q and -q equivalent',periodicity='field factors periodic; full loads intentionally nonperiodic during ramp and translating source')
(HERE/'magnetic_lookup_validation.json').write_text(json.dumps(validation,indent=2));print(json.dumps(validation))
