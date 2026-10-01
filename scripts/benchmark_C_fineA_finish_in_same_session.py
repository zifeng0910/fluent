"""Finish the authorized campaign gates in the existing one-core session."""
import json
import runpy
import subprocess
import sys
from pathlib import Path


def run(solver, context):
    root = context['root']
    report_path = root / 'evidence/benchmark_C_fineA_free_6dof.json'
    baseline = json.loads(report_path.read_text(encoding='utf-8'))
    baseline['status']=baseline.get('simulation_status',baseline['status'])
    stop_path=context.get('campaign_stop_path',root/'evidence/benchmark_C_longrun/stop_request.json')
    if stop_path.exists():
        return
    if baseline['status'] not in ['FREE_6DOF_SOLVED', 'FAIL']:
        raise RuntimeError('Free motion must finish before this workflow')
    if context.get('recovery_v2') and (baseline['status']!='FREE_6DOF_SOLVED' or baseline.get('completed_time_steps')!=80):
        raise RuntimeError('Recovery V2 postprocessing requires verified 80 steps / 2 ms')
    workflow_path = root / 'evidence/benchmark_C_fineA_finish_workflow.json'
    workflow = {'status': 'RUNNING', 'same_existing_solver_session': True,
                'python': sys.executable, 'ui_mode': 'no_gui_or_graphics'}

    def stage(name):
        if stop_path.exists():
            raise InterruptedError('Campaign stop requested between safe workflow operations')
        workflow['stage'] = name
        workflow_path.write_text(json.dumps(workflow, indent=2), encoding='utf-8')
        print(f'C_FINE_A finish: {name}', flush=True)

    def job(name):
        runpy.run_path(str(root / 'scripts' / name))['run'](solver, context)

    try:
        if baseline['status'] == 'FREE_6DOF_SOLVED' and baseline.get('completed_time_steps') == 80:
            stage('CUTOFF_SENSITIVITY')
            try:
                cutoff_path=root/'evidence/benchmark_C_fineA_cutoff_sensitivity.json'
                if not cutoff_path.exists() or json.loads(cutoff_path.read_text()).get('status')!='PASS':
                    job('benchmark_C_fineA_cutoff_sensitivity.py')
            except Exception as exc:
                (root / 'evidence/benchmark_C_fineA_cutoff_sensitivity.json').write_text(
                    json.dumps({'status': 'FAIL', 'error': repr(exc),
                                'physical_inertia_modified': False}, indent=2), encoding='utf-8')
                workflow['sensitivity_error'] = repr(exc)
                if context.get('recovery_v2'):raise
        else:
            stage('PRESERVE_ACTUAL_STOPPED_STATE')
            job('benchmark_C_fineA_capture_terminal_state.py')
        stage('SAVED_STATE_FIELDDATA_EXPORT')
        job('benchmark_C_fineA_export_saved_fielddata.py')
        stage('PYVISTA_OFFSCREEN_RENDER')
        subprocess.run([sys.executable, str(root / 'scripts/benchmark_C_fineA_render_offscreen.py')], cwd=root, check=True)
        stage('CHECK_RENDERED_FILES')
        from PIL import Image
        final = json.loads(report_path.read_text(encoding='utf-8'))
        expected = len(final['frames'])
        counts = {}
        for key in ['gif', 'velocity_gif']:
            with Image.open(final[key]) as img:
                counts[key] = img.n_frames
                if img.n_frames != expected:
                    raise RuntimeError(f'{key} frame count differs from actual snapshots')
                for index in range(img.n_frames):
                    img.seek(index)
                    img.load()
        with Image.open(final['preview']) as img:
            img.load()
            workflow['png_size'] = list(img.size)
        workflow['gif_frame_counts'] = counts
        subprocess.run([sys.executable, str(root / 'scripts/benchmark_C_fineA_final_report.py')], cwd=root, check=True)
        workflow['status'] = 'FILES_VERIFIED_VISUAL_INSPECTION_PENDING'
        stage('AWAIT_VISUAL_INSPECTION')
    except InterruptedError as exc:
        workflow.update(status='PAUSED_SAFE_OPERATION',error=str(exc))
        workflow_path.write_text(json.dumps(workflow,indent=2),encoding='utf-8')
    except Exception as exc:
        workflow.update(status='FAIL', error=repr(exc))
        workflow_path.write_text(json.dumps(workflow, indent=2), encoding='utf-8')
        raise
