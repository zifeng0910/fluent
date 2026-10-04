"""Freeze C evidence and record installed 2026 R1 contact interfaces."""
import csv, re, json, shutil
from benchmark_D_common import *
from benchmark_C_coarse_2ms_common import checkpoint_audit

def main():
    initialize()
    previous=read(ROOT/'evidence/benchmark_C_coarse_2ms/configuration.json')
    frozen=dict(previous['frozen_files_sha256'])
    for p in ['evidence/benchmark_C_coarse_2ms/checkpoint_0070.json',
              'evidence/benchmark_C_coarse_2ms/stop_0073.json',
              'evidence/benchmark_C_coarse_2ms/dynamic_history.csv',
              'evidence/benchmark_C_coarse_2ms/final_report.json',
              'evidence/benchmark_C_coarse_2ms/clearance_stop_audit.json',
              'evidence/benchmark_C_coarse_2ms/clearance_trend_review/clearance_trend_report.json',
              'evidence/benchmark_C_analytic_6dof_history.csv']:
        if (ROOT/p).is_file():frozen[p]=sha(ROOT/p)
    for p,digest in frozen.items():
        if sha(ROOT/p)!=digest:raise RuntimeError('Frozen source changed: '+p)
    checkpoint=read(ROOT/'evidence/benchmark_C_coarse_2ms/checkpoint_0070.json')
    integrity=checkpoint_audit(checkpoint)
    stop=read(ROOT/'evidence/benchmark_C_coarse_2ms/stop_0073.json')
    stop_integrity=checkpoint_audit(stop) if stop else None
    atomic(EVID/'benchmark_C_terminal_classification.json',dict(timestamp=stamp(),
        campaign='BENCHMARK_C_FREE_MOTION',terminal_state='FREE_MOTION_NEAR_WALL_LIMIT_REACHED',
        original_evidence_unchanged=True,latest_solved_step=73,time_s=.001825,target_2ms='INCOMPLETE',
        stop_reason='Physical clearance 0.086598436206 mm below previous 0.100 mm gate',
        classification_excludes=['CFD_FAIL','OVERSET_FAIL','RESOURCE_FAIL'],
        step70_integrity=integrity,step73_integrity=stop_integrity,frozen_files_sha256=frozen))
    config=dict(campaign=CAMPAIGN,authorization='User attachment 8ee7f9cc; resumed by user on 2026-10-04',
        start_step=70,start_time_s=.00175,target_time_s=.002,initial_dt_s=25e-6,max_iterations_per_time_step=2,
        mesh_cells=4699301,background_cells=3345695,component_cells=1353606,
        processor_count=1,precision='double',ui_mode='no_gui_or_graphics',
        source_checkpoint_metadata='evidence/benchmark_C_coarse_2ms/checkpoint_0070.json',
        contact_model='Native detection + frictionless normal rigid-body impulse',normal_restitution=0,friction=0,
        source_law='100 Hz, 1ms smoothstep, 6 mm/s absolute-time offset; unchanged',gravity=0,cutoff=1e-50,
        frozen_files_sha256=frozen,magnetic_sphere_example_used=False,
        original_C_deadline_unchanged=True,new_scheduled_time_budget=None,
        admission=dict(available_gib=12,commit_headroom_gib=19,disk_free_gib=35,stable_seconds=60),
        runtime_guards=dict(available_gib=3,commit_fraction=.95,project_working_set_gib=15,disk_free_gib=15),
        dynamic_admission='Impulse validation, native restart and zero-dynamics detection semantics all PASS required')
    atomic(EVID/'configuration.json',config)
    src=Path(r'H:/Program Files/ANSYS Inc/v261/fluent/fluent26.1.0/src')
    headers={str(p):sha(p) for p in [src/'udf/udf.h',src/'mesh/six_dof.h',src/'mesh/dynamesh_contact.h',src/'mesh/dynamesh_tools.h']}
    api=Path(r'H:/Program Files/ANSYS Inc/v261/fluent/fluent26.1.0/cortex/pylib/flapi/generated/solver/settings_261.py')
    gui=Path(r'H:/Program Files/ANSYS Inc/v261/commonfiles/help/en-us/fluent_gui_help/fluent_gui_help.xml')
    audit=dict(timestamp=stamp(),fluent_version='2026 R1 / 26.1.0 (installed v261)',
        DEFINE_CONTACT_available=True,SDOF_Get_Motion_available=True,SDOF_Overwrite_Motion_available=True,
        contact_hook_available='INSTALLED_API_PRESENT_PENDING_LIVE_SELECTION',
        robot_wall_selectable='PENDING_LIVE_TEST',pipe_wall_selectable='PENDING_LIVE_TEST',
        supported_in_current_configuration='PENDING_ZERO_DYNAMICS_TEST',
        proximity_threshold_semantics='Documented face-zone proximity distance; actual current mesh callback threshold pending test',
        flow_control_enabled=False,installed_headers_sha256=headers,
        local_generated_settings=dict(path=str(api),sha256=sha(api),group='setup.dynamic_mesh.options.contact_detection'),
        local_GUI_help=dict(path=str(gui),sha256=sha(gui),section='setup..dynamic-mesh..options..contact-detection'),
        local_full_UDF_manual_found=False,local_contact_examples_found=False,
        official_2026R1_sources=[
            'https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html',
            'https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_ug/flu_ug_sec_dynamic_using.html'],
        motion_overwrite_rule='Only call SDOF_Overwrite_Motion inside DEFINE_CONTACT',
        contact_face_access='Objp linked list, O_F / O_F_THREAD; F_CENTROID and F_AREA',
        geometry_safety='Do not infer practical overset compatibility from macro availability')
    atomic(EVID/'contact_capability_audit.json',audit)
    rows=list(csv.DictReader((ROOT/'evidence/benchmark_C_coarse_2ms/dynamic_history.csv').open()))
    with (EVID/'dynamic_history.csv').open('w',newline='') as fp:
        writer=csv.DictWriter(fp,fieldnames=list(rows[0]));writer.writeheader();writer.writerow(checkpoint['row'])
    (EVID/'fielddata').mkdir(exist_ok=True)
    shutil.copy2(ROOT/'evidence/benchmark_C_coarse_2ms/fielddata/robot_0000.vtp',EVID/'fielddata/robot_0000.vtp')
    state('OFFLINE_IMPULSE_PASS_PENDING_NATIVE_AUDIT',current_step=70,time_s=.00175,solver_alive=False,
        timesteps_advanced=0,impulse_validation_status=read(EVID/'contact_impulse_unit_validation.json')['status'])
    event('CAMPAIGN_PREPARED',C_terminal='FREE_MOTION_NEAR_WALL_LIMIT_REACHED',restart_step=70)
    print(json.dumps(dict(status='PREPARED',step70_integrity=integrity['status'],step73_integrity=stop_integrity['status'] if stop_integrity else 'MISSING')))

if __name__=='__main__':main()
