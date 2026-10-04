"""Frozen-attitude native contact probes at five measured gaps; zero dynamics."""
import json
import numpy as np
import pyvista as pv
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from benchmark_D_common import *
from benchmark_C_coarse_resource_checkpoint import native_state
from benchmark_C_fineA_free_6dof_run import surface_mesh

def event_rows():
    p=EVID/'contact_events.jsonl'
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []

def run(solver,context):
    contact=solver.settings.setup.dynamic_mesh.options.contact_detection
    if read(EVID/'contact_compile_gate.json').get('status')!='PASS':raise RuntimeError('Compiled contact gate required')
    checkpoint=read(ROOT/'evidence/benchmark_C_coarse_2ms/checkpoint_0070.json')
    t0=float(solver.scheme.eval("(rpgetvar 'flow-time)"));initial=native_state(solver,'robot_wall')
    pipe=surface_mesh(solver.fields.field_data,'pipe_wall');robot=surface_mesh(solver.fields.field_data,'robot_wall')
    sign=float(np.sign(pv.PolyData(np.array([initial['com']])).compute_implicit_distance(pipe)['implicit_distance'][0]))
    distances=robot.compute_implicit_distance(pipe)['implicit_distance']*sign
    nearest=robot.points[int(np.argmin(distances))];direction=np.array([0.,nearest[1],nearest[2]]);direction/=np.linalg.norm(direction)
    def gap(distance):
        moved=robot.copy();moved.points=robot.points+distance*direction
        return float(np.min(moved.compute_implicit_distance(pipe)['implicit_distance']*sign))
    report=dict(timestamp=stamp(),status='RUNNING',native_time_s=t0,timesteps_advanced=0,
        frozen_orientation=initial['q'],proximity_threshold_m=.0001,translation_direction=direction.tolist(),
        detection_only=True,probe_records=[],native_detection_entry='Installed Detect_Contact(Domain*,CURRENT_TIME,0.0,cxboolean)',
        last_flag_semantics='Undocumented boolean; both values are explicitly tested without assigning a meaning')
    atomic(EVID/'contact_detection_semantics.json',report)
    previous=0.
    user=solver.settings.setup.user_defined
    library_name=read(EVID/'contact_capability_audit.json').get('contact_library_name','libbenchmark_D_contact')
    def detect(shift,flag,mode):
        oldcount=len(event_rows())
        (OUT/'probe_control.txt').write_text(f'{flag} {mode} '+ ' '.join(format(float(x),'.17g') for x in shift)+'\n')
        user.execute_on_demand(lib_name='benchmark_D_static_native_detect::'+library_name)
        current=surface_mesh(solver.fields.field_data,'robot_wall')
        measured=float(np.min(current.compute_implicit_distance(pipe)['implicit_distance']*sign))
        actual=native_state(solver,'robot_wall')
        if abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t0)>1e-12:raise RuntimeError('Static detection advanced time')
        if np.linalg.norm(np.asarray(actual['q'])-initial['q'])>1e-10:raise RuntimeError('Static test changed orientation')
        rows=event_rows()[oldcount:]
        return dict(flag=flag,mode=mode,measured_gap_m=measured,callbacks=rows,callback_count=len(rows),native_state=actual)
    for target_mm in [.140,.110,.100,.090,.070]:
        context['profile'].check('STATIC_SEMANTICS')
        target=target_mm/1000
        distance=brentq(lambda x:gap(x)-target,-1e-5,.0001,xtol=1e-14)
        record=dict(target_gap_m=target,translation_m=(distance*direction).tolist(),tests=[])
        record['tests'].append(detect((distance-previous)*direction,0,0));previous=distance
        record['tests'].append(detect(np.zeros(3),1,0))
        if any(abs(x['measured_gap_m']-target)>1e-9 for x in record['tests']):raise RuntimeError('Native geometry translation differs from offline target')
        report['probe_records'].append(record);atomic(EVID/'contact_detection_semantics.json',report)
    callbacks=[x for r in report['probe_records'] for test in r['tests'] for x in test['callbacks']]
    checks={}
    for flag in [0,1]:
        pattern=[]
        for record in report['probe_records']:
            test=next(x for x in record['tests'] if x['flag']==flag)
            pattern.append(dict(gap_mm=record['target_gap_m']*1000,triggered=test['callback_count']>0))
        checks[str(flag)]=pattern
    report['observed_trigger_patterns']=checks
    normal_pass=bool(callbacks)
    for e in callbacks:
        point=np.array(e['contact_point_m']);n=np.array(e['normal']);inward=np.array([0.,-point[1],-point[2]])
        cosine=float(n@inward/np.linalg.norm(inward))
        if cosine<.9 or not np.isfinite(np.r_[point,n]).all() or abs(np.linalg.norm(n)-1)>1e-8:normal_pass=False
    # A successful trigger at strictly positive physical gap is required.
    safe=any(test['callback_count'] for rec in report['probe_records'] if rec['target_gap_m']<.0001 for test in rec['tests'])
    report.update(contact_point_normal_sanity='PASS' if normal_pass else 'NOT_VERIFIABLE_NO_CALLBACK' if not callbacks else 'FAIL',positive_gap_detection_pass=bool(safe),
        active_library=library_name,compiled_source_sha256=read(EVID/'contact_compile_gate.json').get('source_sha256'))
    if safe and normal_pass:
        # Native response sanity must happen inside DEFINE_CONTACT only.
        # Use the flag that actually emitted native callbacks, not its undocumented complement.
        response_flag=next(test['flag'] for rec in reversed(report['probe_records']) for test in rec['tests'] if test['callback_count'])
        report['artificial_inward_velocity_sanity']=detect(np.zeros(3),response_flag,2)
        events=report['artificial_inward_velocity_sanity']['callbacks']
        response=bool(events) and any(e['impulse_applied'] for e in events) and all(abs(e['v_n_after_m_s'])<1e-10 for e in events)
        from benchmark_D_validate_impulse import reference
        xy,xz,yz=INERTIA_PRODUCTS
        Ibody=np.array([[INERTIA_DIAG[0],-xy,-xz],[-xy,INERTIA_DIAG[1],-yz],[-xz,-yz,INERTIA_DIAG[2]]])
        independent=[]
        for e in events:
            R=Rotation.from_quat(np.array(e['quaternion_wxyz'])[[1,2,3,0]]).as_matrix()
            Iworld=R@Ibody@R.T;r=np.array(e['r_m']);n=np.array(e['normal']);v=np.array(e['v_before_m_s']);w=np.array(e['omega_before_rad_s'])
            V,W,J=reference(MASS,Iworld,r,n,v,w)
            errors=dict(velocity_error_m_s=float(np.max(abs(V-e['v_after_m_s']))),
                omega_error_rad_s=float(np.max(abs(W-e['omega_after_rad_s']))),impulse_error_N_s=abs(J-e['impulse_N_s']))
            independent.append(errors)
            if errors['velocity_error_m_s']>1e-10 or errors['omega_error_rad_s']>1e-8 or errors['impulse_error_N_s']>1e-16:response=False
        report['independent_native_callback_impulse_comparison']=independent
        report['normal_impulse_sanity_status']='PASS' if response else 'FAIL'
    else:report['normal_impulse_sanity_status']='NOT_RUN'
    # Restore native CASE/DATA; no shifted diagnostic pose is ever a continuation source.
    solver.settings.file.read_case(file_name=checkpoint['case']);solver.settings.file.read_data(file_name=checkpoint['data'])
    actual=native_state(solver,'robot_wall')
    from benchmark_C_recovery_v2_restart import compare
    restore=compare(actual,checkpoint['native_state']);report['native_restore_errors']=restore
    if any(x['status']!='PASS' for x in restore.values()):raise RuntimeError('Failed to restore step70 after static probes')
    full_pass=safe and normal_pass and report['normal_impulse_sanity_status']=='PASS'
    report.update(status='PASS' if full_pass else 'CONTACT_DETECTION_NOT_SAFE_FOR_CURRENT_OVERSET',
        timestamp=stamp(),native_time_after_s=float(solver.scheme.eval("(rpgetvar 'flow-time)")),production_response_started=False)
    atomic(EVID/'contact_detection_semantics.json',report)
    audit=read(EVID/'contact_capability_audit.json')
    audit.update(supported_in_current_configuration='YES' if full_pass else 'NOT_DEMONSTRATED_SAFE',zero_dynamics_gate=report['status'])
    atomic(EVID/'contact_capability_audit.json',audit)
