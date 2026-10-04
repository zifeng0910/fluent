"""Architecture ranking only; no force implementation, solver launch or state mutation."""
from benchmark_D_overwrite_common import *

def write_audit():
    doc='https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html'
    rec=dict(timestamp=stamp(),scope='AUDIT_ONLY; no fallback implemented',
        preferred_if_native_one_event_validates='Retain native DEFINE_CONTACT + same-returned-theta Overwrite, accounting for measured endpoint-pose reconstruction',
        preferred_if_native_write_fails='B: normal contactF/T through existing verified DEFINE_SDOF_PROPERTIES load path',
        candidates=[
            dict(id='A',fallback_rank=2,name='Alternative Fluent-supported6DOF collision/state-update API',
                support='UNKNOWN: installed public example supports Overwrite. No separately documented public6DOF motion setter located.',
                installed_exports=['SDOF_Fill_RB_State','SDOF_Threads_Compute_Motion'],
                limitation='Internal declarations do not establish a supported plugin state-update contract. Calling the internal integrator could advance/recompute motion twice; no experiment performed.',
                action='Seek version-specific supported API guidance before implementation'),
            dict(id='B',fallback_rank=1,name='Normal contact force/torque through existing SDOF load properties',
                support='DOCUMENTED: external global/body forces and moments in DEFINE_SDOF_PROPERTIES; existing magnetic COM F/T path empirically verified',
                installed_symbols=['SDOF_LOAD_F_X','SDOF_LOAD_F_Y','SDOF_LOAD_F_Z','SDOF_LOAD_M_X','SDOF_LOAD_M_Y','SDOF_LOAD_M_Z','SDOF_LOAD_LOCAL'],
                principle='Let the native6DOF integrator update motion. Evaluate a normal contact force and COM torque with one consistent event/time law.',
                required_validation=['Event timing vs predictor/load calls','Impulse integral over absolute time, independent of number of property calls','Nonpenetration and passivity','Resolved contact-time/stiffness scale at frozen25us','COM torque and reaction accounting'],
                limitation='F=J/dt alone is not a validated collision model. Several property evaluations per timestep must not accumulate the force multiple times. Detection latency and integration can change achieved impulse.',
                implemented=False,official_source=doc+'#flu_udf_sec_define_sdof_properties'),
            dict(id='C',fallback_rank=3,name='Calibrated short-range penalty contact force',
                support='Load path available; stiffness/damping/model accuracy unvalidated',
                limitation='Adds contact constitutive parameters and timestep/stiffness constraints; requires new authorization and calibration. No penalty or repulsion added.',implemented=False),
            dict(id='D',fallback_rank=4,name='External rigid-body integrator coupled to Fluent',
                support='Coupling architecture not audited as a supported replacement in this frozen case',
                limitation='Largest architecture change; requires force/torque exchange, native overset pose synchronization, stability and double-integration prevention. No prescribed CG_MOTION substitution.',implemented=False)],
        analytic_detection_resolves_overwrite_blocker=False,physics_changed=False)
    atomic(EVID/'fallback_audit.json',rec);return rec

if __name__=='__main__':print(write_audit()['preferred_if_native_write_fails'])
