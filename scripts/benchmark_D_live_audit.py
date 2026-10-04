"""Observe native settings and compile independent contact UDF; advance zero steps."""
import json
from benchmark_D_common import *

def run(solver,context):
    if context['command'].get('action') == 'PRESERVE_FAILURE_AND_RESTORE_C70':
        preserve_failure_and_restore(solver, context)
        return
    if context['command'].get('action') == 'READ_NATIVE_MOTION_STATE':
        read_native_motion(solver,context)
        return
    s=solver.settings;contact=s.setup.dynamic_mesh.options.contact_detection
    report=read(EVID/'contact_capability_audit.json')
    report.update(live_Fluent_version=str(solver.get_fluent_version()),timestamp=stamp(),
        contact_initial_state=contact.get_state(),contact_is_active=contact.is_active())
    atomic(EVID/'contact_capability_audit.json',report)
    contact.enabled=True
    report['selectable_face_zones']=contact.face_zones.allowed_values()
    report['selectable_boundaries']=contact.boundaries.allowed_values() if contact.boundaries.is_active() else []
    for zone in ['robot_wall','pipe_wall']:
        report[zone+'_selectable']=zone in report['selectable_face_zones'] or zone in report['selectable_boundaries']
    report['contact_enabled_state']=contact.get_state()
    atomic(EVID/'contact_capability_audit.json',report)
    library_name=context['command'].get('contact_library_name','libbenchmark_D_contact')
    library=str(OUT/library_name)
    # A versioned directory avoids Fluent silently retaining an already loaded DLL.
    expected=[OUT/library_name/'win64'/kind/'libudf.dll' for kind in ['3ddp_host','3ddp_node']]
    before={str(p):sha(p) for p in expected if p.is_file()}
    s.setup.user_defined.compiled_udf(library_name=library,
        source_files=[str(ROOT/'fluent_udf/l2300_frictionless_contact.c')],
        header_files=[str(ROOT/'fluent_udf/l2300_contact_impulse_math.h')],use_built_in_compiler=True)
    if not all(p.is_file() for p in expected):raise RuntimeError('Native compiler returned without both contact DLLs')
    source_time=(ROOT/'fluent_udf/l2300_frictionless_contact.c').stat().st_mtime
    if any(p.stat().st_mtime<source_time for p in expected):raise RuntimeError('Compiler retained an older contact DLL; invalidate compilation gate')
    s.setup.user_defined.load(udf_library_name=library)
    report['contact_udf_allowed']=contact.contact_udf.allowed_values()
    hook='l2300_frictionless_contact::'+library_name
    if hook not in report['contact_udf_allowed']:raise RuntimeError('Compiled contact hook not offered by native Fluent')
    contact.face_zones=['robot_wall','pipe_wall']
    contact.proximity_threshold=.0001
    contact.contact_udf=hook
    contact.flow_control.enabled=False
    contact.verbosity=1
    report.update(contact_hook_available=True,contact_configured_state=contact.get_state(),
        supported_in_current_configuration='LIVE_HOOK_PASS_PENDING_STATIC_CALLBACK_TEST',
        contact_udf_sha256=sha(ROOT/'fluent_udf/l2300_frictionless_contact.c'),contact_library_name=library_name,
        compiled_dll_sha256={str(p):sha(p) for p in expected},prior_dll_sha256=before,timesteps_advanced=0)
    atomic(EVID/'contact_capability_audit.json',report)
    atomic(EVID/'contact_compile_gate.json',dict(status='PASS',timestamp=stamp(),timesteps_advanced=0,library=library,
        source_sha256=report['contact_udf_sha256'],dll_sha256=report['compiled_dll_sha256']))

def preserve_failure_and_restore(solver, context):
    """Archive the failed translated probe, reload immutable C70, advance zero steps."""
    import shutil
    import numpy as np
    import pyvista as pv
    from benchmark_C_coarse_resource_checkpoint import native_state
    from benchmark_C_fineA_free_6dof_run import surface_mesh
    archive=EVID/'diagnostic_archive/native_response_failure_01'
    archive.mkdir(parents=True,exist_ok=True)
    for source in [EVID/'state.json',EVID/'active_job.json',EVID/'contact_detection_semantics.json',
                   EVID/'contact_events.csv',EVID/'contact_events.jsonl',EVID/'contact_compile_gate.json',
                   EVID/'contact_capability_audit.json',ROOT/'fluent_udf/l2300_frictionless_contact.c',
                   OUT/'fluent-20261004-081129-33312.trn']:
        target=archive/source.name
        if not target.exists():shutil.copy2(source,target)
    t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    wall=native_state(solver,'robot_wall')
    robot=surface_mesh(solver.fields.field_data,'robot_wall')
    pipe=surface_mesh(solver.fields.field_data,'pipe_wall')
    sign=float(np.sign(pv.PolyData(np.array([wall['com']])).compute_implicit_distance(pipe)['implicit_distance'][0]))
    gap=float(np.min(robot.compute_implicit_distance(pipe)['implicit_distance']*sign))
    robot.save(archive/'diagnostic_translated_robot.vtp')
    atomic(archive/'failure_pose.json',dict(timestamp=stamp(),native_time_s=t,
        native_settings_state=wall,settings_state_caveat='Static diagnostic DT_CG translation is visible in callback/FieldData, while settings retain the saved motion state',
        measured_physical_gap_m=gap,probe_control=(OUT/'probe_control.txt').read_text(),
        error='D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED',timesteps_advanced=0,
        command_id=context['command']['command_id']))
    atomic(archive/'manifest.json',dict(timestamp=stamp(),files={p.name:sha(p) for p in archive.iterdir() if p.is_file() and p.name!='manifest.json'}))
    checkpoint=read(ROOT/'evidence/benchmark_C_coarse_2ms/checkpoint_0070.json')
    for key in ['case','data']:
        if sha(checkpoint[key])!=checkpoint[key+'_sha256']:raise RuntimeError('Immutable C70 checkpoint changed')
    solver.settings.file.read_case(file_name=checkpoint['case'])
    solver.settings.file.read_data(file_name=checkpoint['data'])
    import benchmark_C_coarse_native_resume as restart
    restart.EVID=EVID;restart.OUT=OUT
    restored=restart.validation(solver,checkpoint,70)
    if abs(restored['native_time_s']-t)>1e-12:raise RuntimeError('Restore changed absolute native time')
    atomic(EVID/'post_probe_C70_restore.json',{**restored,'status':'PASS',
        'reason':'Discard failed synthetic zero-dynamics diagnostic state; restore exact native C70',
        'failure_preserved_under':str(archive.relative_to(ROOT)),'contact_production_response_started':False})
    event('FAILED_PROBE_ARCHIVED_AND_C70_RESTORED',timesteps_advanced=0,time_s=t)

def read_native_motion(solver,context):
    """Compile a separate read-only audit; neither hook contact nor alter body state."""
    from benchmark_C_coarse_resource_checkpoint import native_state
    from benchmark_C_recovery_v2_restart import compare
    source=ROOT/'fluent_udf/l2300_contact_motion_audit.c'
    if sha(source)!=context['command']['supporting_code_sha256']:raise RuntimeError('Motion audit code changed')
    before=native_state(solver,'robot_wall')
    t=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    library=OUT/context['command']['motion_library_name']
    dlls=[library/'win64'/kind/'libudf.dll' for kind in ['3ddp_host','3ddp_node']]
    solver.settings.setup.user_defined.compiled_udf(library_name=str(library),source_files=[str(source)],
        header_files=[],use_built_in_compiler=True)
    if any(not p.exists() or p.stat().st_mtime<source.stat().st_mtime for p in dlls):raise RuntimeError('Read-only audit DLL not current')
    solver.settings.setup.user_defined.load(udf_library_name=str(library))
    solver.settings.setup.user_defined.execute_on_demand(lib_name='benchmark_D_read_native_motion::'+library.name)
    after=native_state(solver,'robot_wall');checks=compare(after,before)
    if any(x['status']!='PASS' for x in checks.values()) or abs(float(solver.scheme.eval("(rpgetvar 'flow-time)"))-t)>1e-12:raise RuntimeError('Read-only motion audit changed native state')
    atomic(EVID/'native_motion_state_audit.json',dict(status='DIAGNOSTIC_COMPLETE',timestamp=stamp(),timesteps_advanced=0,
        native_state_before=before,native_state_after=after,unchanged_checks=checks,
        source_sha256=sha(source),dll_sha256={str(p):sha(p) for p in dlls},
        records=[json.loads(x) for role in ['host','node'] for x in (EVID/f'native_motion_state_{role}.jsonl').read_text().splitlines() if x.strip()]))
