"""Finite readiness gate; the Windows supervisor performs every solver launch."""
import json,py_compile,subprocess,sys
from benchmark_D_native_contact_overnight_common import *

def main():
    verify_frozen()
    if read(EVID/'state.json').get('status')!='STARTUP':raise RuntimeError('Only prepared STARTUP may be armed')
    if native() or read(EVID/'active_job.json').get('status') not in {None,'FINISHED'}:
        raise RuntimeError('Existing active solver/job; readiness cannot alter its inputs')
    files=[ROOT/'scripts'/('benchmark_D_native_contact_overnight_'+name+'.py')
        for name in ['common','prepare','arm','supervisor','worker','review','render']]
    files.extend([ROOT/'scripts/run_benchmark_D_native_contact_overnight.ps1',
        ROOT/'scripts/test_benchmark_D_native_contact_overnight_control.py',
        ROOT/'fluent_udf/l2300_native_contact_overnight.c'])
    for path in files:
        if not path.is_file():raise RuntimeError('Incomplete local capability: '+str(path))
        if path.suffix=='.py':py_compile.compile(str(path),doraise=True)
    gate=read(EVID/'source_change_review.json')
    if not gate.get('cache_fill128_PASS') or not gate.get('mathematical_kernel_code_unchanged') or not gate.get('magnetic_wrapper_code_unchanged'):
        raise RuntimeError('Source/cache review did not pass')
    if gate['new_source_sha256']!=sha(ROOT/'fluent_udf/l2300_native_contact_overnight.c'):
        raise RuntimeError('Reviewed contact source changed')
    test=subprocess.run([sys.executable,str(ROOT/'scripts/test_benchmark_D_native_contact_overnight_control.py')],
        cwd=ROOT,capture_output=True,text=True)
    atomic(EVID/'control_validation.json',dict(timestamp=stamp(),status='PASS' if test.returncode==0 else 'FAIL',
        returncode=test.returncode,stdout=test.stdout,stderr=test.stderr,tests=8,no_solver_launched=True,
        checks=['Second solver refused','Hard deadline owned checkpoint only','Fallback deadline checkpoint',
            'Resource failure not contact-model failure','Latest native boundary retained','Unverified checkpoint refused']))
    if test.returncode:raise RuntimeError('Control regression gate failed')
    checked={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    review=dict(status='PASS',timestamp=stamp(),configuration_sha256=sha(EVID/'configuration.json'),
        window_sha256=sha(EVID/'window.json'),supporting_code_sha256=checked,
        control_validation_sha256=sha(EVID/'control_validation.json'),
        source_review_sha256=sha(EVID/'source_change_review.json'),
        prior_semantics_reused_not_reopened=True,full_UDF_compile_pending_actual_owned_Fluent=True,
        zero_step_native_restart_required_before_each_branch=True,
        cold_restart_and_actual_dynamic_gates_are_not_preclaimed=True)
    atomic(EVID/'launch_review.json',review)
    event('LOCAL_READINESS_PASS',launch_review_sha256=sha(EVID/'launch_review.json'))
    print(json.dumps(dict(status='LOCAL_READINESS_PASS',window=read(EVID/'window.json'))))

if __name__=='__main__':main()
