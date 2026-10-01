"""Preserve the actual stopped dynamic state before offline FieldData export."""
import json
import shutil
from pathlib import Path


def run(solver, context):
    root = context['root']
    path = root / 'evidence/benchmark_C_fineA_free_6dof.json'
    report = json.loads(path.read_text(encoding='utf-8'))
    if report['status'] != 'FAIL':
        return
    failure = json.loads((root / 'evidence/benchmark_C_fineA_dynamic_failure.json').read_text(encoding='utf-8'))
    step, expected_time = failure['step'], failure['time_s']
    actual_time = float(solver.scheme.eval("(rpgetvar 'flow-time)"))
    if abs(actual_time - expected_time) > 1e-10:
        raise RuntimeError('Stopped-state time differs from failure evidence')
    frames = report.get('frames', [])
    if any(f['step'] == step for f in frames):
        return
    source = Path(report['fielddata_directory'])
    for name in ['robot', 'component']:
        shutil.copyfile(source / f'{name}_failure_{step:04d}.vtp', source / f'{name}_{step:04d}.vtp')
    folder = root / 'live_cases/benchmark_C_fineA/bg020'
    case = folder / f'fielddata_checkpoint_failure_{step:04d}.cas.h5'
    data = folder / f'fielddata_checkpoint_failure_{step:04d}.dat.h5'
    solver.settings.file.write_case(file_name=str(case))
    solver.settings.file.write_data(file_name=str(data))
    com = failure['com_m']
    frames.append({'step': step, 'time_s': actual_time, 'com_m': com,
                   'radial_displacement_m': (com[1]**2 + com[2]**2)**.5,
                   'orphan_count': failure['official_connectivity']['orphans'],
                   'velocity_max_m_s': None, 'checkpoint_case': str(case),
                   'checkpoint_data': str(data), 'terminal_failed_state': True})
    report.update(frames=frames, terminal_state_checkpoint=str(case),
                  terminal_state_time_s=actual_time, terminal_state_capture='PASS')
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f'Preserved stopped state at {actual_time:g} s for actual final-frame export', flush=True)
