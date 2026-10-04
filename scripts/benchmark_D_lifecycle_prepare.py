import shutil
from benchmark_D_lifecycle_common import *

def main():
    if (EVID/'configuration.json').exists():raise RuntimeError('Lifecycle already configured; preserve earlier diagnostics')
    EVID.mkdir(parents=True,exist_ok=True);OUT.mkdir(parents=True,exist_ok=True)
    old=ROOT/'evidence/benchmark_D_contact_baseline';config=read(old/'configuration.json')
    frozen=dict(config['frozen_files_sha256'])
    for name in ['final_report.json','state.json','contact_detection_semantics.json','contact_capability_audit.json',
                 'contact_impulse_unit_validation.json','native_motion_state_audit.json','step70_restart_continuity.json']:
        p=old/name;frozen[p.relative_to(ROOT).as_posix()]=sha(p)
    for name in ['l2300_frictionless_contact.c','l2300_contact_impulse_math.h']:
        p=ROOT/'fluent_udf'/name;frozen[p.relative_to(ROOT).as_posix()]=sha(p)
    atomic(EVID/'configuration.json',dict(campaign=CAMPAIGN,timestamp=stamp(),frozen_files_sha256=frozen,
        authorization='User attachment ca807d44: actual transient contact lifecycle and gated micro-run to1.900ms only',
        original_zero_step_result_preserved=True,start_step=70,start_time_s=.00175,dt_s=DT,
        contact_threshold_m=.0001,mesh_cells=4699301,processor_count=1,precision='double',ui_mode='no_gui_or_graphics',
        admission=config['admission'],runtime_guards=config['runtime_guards'],new_time_budget_or_automation=None,
        branch_order=['read_only','noop','controlled'],read_only_max_steps=4,
        true_solver_callback_only=True,controlled_single_event_only=True,maximum_micro_time_s=.0019,
        normal_restitution=0,friction=0,penetration_tolerance_m=1e-9,
        official_source='https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html'))
    verify_frozen();state('PREPARED',solver_alive=False,worker_alive=False,timesteps_advanced=0)
    event('LIFECYCLE_CONFIGURED',old_zero_step_files_unchanged=True)
    print(CAMPAIGN+' PREPARED')

if __name__=='__main__':main()
