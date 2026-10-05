"""Render saved native FieldData only after full numerical PASS; no solver API.

Three fixed-camera GIFs retain every available actual velocity snapshot. Visual
acceptance must be performed separately on the PNG and extracted GIF frames.
"""
import json
import math
from pathlib import Path

import numpy as np
import pyvista as pv
from PIL import Image

from benchmark_D_native_contact_overnight_common import ROOT, EVID, read, atomic, sha, stamp


def absolute(path, branch_dir=None):
    p = Path(path)
    if p.is_absolute():
        return p
    root = ROOT/p
    if root.exists():
        return root
    if branch_dir is not None and (branch_dir/p).exists():
        return branch_dir/p
    return root


def load_frames(report):
    merged = []
    for name in (report['selected_micro_branch'], report['production_branch']):
        config = read(EVID/'branches'/name/'configuration.json')
        physical_parts = [*config.get('history_prefix_branches', []), name]
        for i, physical in enumerate(physical_parts):
          b = EVID/'branches'/physical
          cutoff = float('inf')
          if i+1 < len(physical_parts):
            next_config = read(EVID/'branches'/physical_parts[i+1]/'configuration.json')
            checkpoint = read(absolute(next_config['source_checkpoint']))
            cutoff = float(checkpoint['time_s'])
          snapshots = read(b/'snapshots.json')
          if not snapshots.get('frames'):
            raise RuntimeError('Actual saved velocity snapshots missing: '+physical)
          for s in snapshots['frames']:
            if not s.get('actual_FieldData'):
                raise RuntimeError('A frame is not certified actual Fluent FieldData')
            t = float(s['time_s'])
            if t > cutoff+1e-12:
                continue
            # The production initial1.9ms snapshot duplicates the reviewed micro
            # endpoint. Use production FieldData at this exact same native time.
            merged = [r for r in merged if abs(r['time_s']-t) > 1e-12]
            step = int(s.get('native_step_index', s.get('step', -1)))
            rp = absolute(s.get('robot_path', str(b/'fielddata'/f'state_{step:04d}_robot.vtp')), b)
            vp = absolute(s.get('velocity_path', str(b/'fielddata'/f'state_{step:04d}_velocity.vtp')), b)
            native = read(b/f'state_{step:04d}.json')
            if not native or abs(native['time_s']-t) > 1e-12:
                raise RuntimeError('Frame has no exact-time native state')
            merged.append({'branch': physical, 'native_step_index': step, 'time_s': t,
                'robot_path': str(rp), 'velocity_path': str(vp), 'native_state_path': str(b/f'state_{step:04d}.json'),
                'signed_gap_m': float(native.get('signed_gap_m', native['physical_gap_m'])),
                'actual_FieldData': True})
    merged.sort(key=lambda r: r['time_s'])
    if not merged or abs(merged[0]['time_s']-.00175) > 1e-12 or abs(merged[-1]['time_s']-.002) > 1e-12:
        raise RuntimeError('Actual C70 and2ms endpoint frames required')
    return merged


def choose_review_indices(frames, report):
    contact = report['metrics']['first_contact_completed_time_s']
    times = [f['time_s'] for f in frames]
    if contact is None:
        raise RuntimeError('Contact event needed for contact visualization')
    closest = min(range(len(times)), key=lambda i: abs(times[i]-contact))
    pre = max((i for i, t in enumerate(times) if t < contact-1e-12), default=0)
    post = min((i for i, t in enumerate(times) if t > contact+1e-12), default=len(times)-1)
    return {'first': 0, 'pre_contact': pre, 'first_contact': closest, 'post_contact': post, 'last': len(times)-1}


def main():
    report = read(EVID/'final_report.json')
    if report.get('status') != 'BENCHMARK_D_NATIVE_CONTACT_2MS_PASS' or not all(report.get('numerical_gates', {}).values()):
        raise RuntimeError('Full2ms numerical PASS is required before rendering')
    frames = load_frames(report)
    out = EVID/'visualization'
    out.mkdir(parents=True, exist_ok=True)
    files = {kind: out/f'benchmark_D_contact_{kind}.gif' for kind in ('motion', 'velocity', 'closeup')}
    final = out/'benchmark_D_final_frame.png'
    if any(p.exists() for p in [*files.values(), final]):
        prior = read(EVID/'render_manifest.json')
        if prior.get('status') == 'RENDERED_AWAITING_VISUAL_REVIEW' and all(p.exists() and sha(p) == prior.get('outputs_sha256', {}).get(p.name) for p in [*files.values(), final]):
            print(json.dumps({'status': prior['status'], 'frames': len(prior['frames']), 'existing_verified_render_reused': True}))
            return
        raise RuntimeError('Existing renders changed or incomplete; archive a reviewed defect before replacing')
    # Read all actual scalar ranges first, then use one immutable scale throughout
    # every GIF. This reads native output and does not interpolate fluid values.
    vmax = 0.
    source_hashes = {}
    for frame in frames:
        robot, fluid = pv.read(frame['robot_path']), pv.read(frame['velocity_path'])
        if not robot.n_points or not fluid.n_points or 'velocity_m_s' not in fluid.point_data:
            raise RuntimeError('Actual robot/velocity FieldData empty or missing')
        vel = np.asarray(fluid.point_data['velocity_m_s'])
        if not np.isfinite(vel).all() or len(vel) != fluid.n_points:
            raise RuntimeError('Actual velocity field invalid')
        vmax = max(vmax, float(vel.max()))
        frame.update(velocity_min_m_s=float(vel.min()), velocity_max_m_s=float(vel.max()), velocity_points=int(fluid.n_points))
        source_hashes[frame['robot_path']] = sha(Path(frame['robot_path']))
        source_hashes[frame['velocity_path']] = sha(Path(frame['velocity_path']))
    if vmax <= 0:
        raise RuntimeError('Blank velocity field')
    first_branch = EVID/'branches'/report['selected_micro_branch']
    pipe_path = first_branch/'fielddata/pipe.vtp'
    pipe = pv.read(pipe_path)
    if not pipe.n_points:
        raise RuntimeError('Actual pipe geometry empty')
    source_hashes[str(pipe_path)] = sha(pipe_path)
    camera = [(0.0012, 0., .012), (.0012, 0., 0.), (0., 1., 0.)]
    contact = np.asarray(report['metrics']['first_contact_point_m'], dtype=float)
    close_target = np.array([contact[0]-.00035, contact[1]+.00010, 0.])
    close_camera = [(close_target[0], close_target[1], .012), tuple(close_target), (0., 1., 0.)]
    for kind, dest in files.items():
        plot = pv.Plotter(off_screen=True, window_size=(1280,820))
        plot.set_background('#f4f7fa')
        plot.open_gif(str(dest), fps=8)
        try:
            for frame in frames:
                robot, fluid = pv.read(frame['robot_path']), pv.read(frame['velocity_path'])
                plot.clear()
                plot.add_mesh(pipe, color='#748b9d', opacity=.12 if kind != 'closeup' else .18,
                              show_edges=False, smooth_shading=True)
                plot.add_mesh(fluid, scalars='velocity_m_s', cmap='viridis', clim=(0., vmax),
                    opacity=1. if kind == 'velocity' else .35,
                    show_scalar_bar=True, scalar_bar_args={'title':'Velocity (m/s)', 'fmt':'%.3g', 'n_labels':5,
                        'color':'#172434', 'title_font_size':18, 'label_font_size':16,
                        'position_x': .06, 'position_y': .04, 'width': .88, 'height': .09})
                plot.add_mesh(robot, color='#ef812b', smooth_shading=True)
                pen = max(0., -frame['signed_gap_m'])*1e6
                plot.add_text('Benchmark D | frictionless native reaction | e = 0, mu = 0 | proximity 0.100 mm\n'
                    f'Actual Fluent FieldData | t = {frame["time_s"]*1000:.4f} ms | dt = {report["selected_dt_s"]*1e6:g} us\n'
                    f'Ideal tube signed gap = {frame["signed_gap_m"]*1e6:.2f} um | penetration = {pen:.2f} um\n'
                    f'Fixed velocity scale 0 to {vmax:.4g} m/s', font_size=13, color='#172434', position='upper_left')
                plot.camera_position = close_camera if kind == 'closeup' else camera
                plot.camera.parallel_projection = True
                plot.camera.parallel_scale = .00085 if kind == 'closeup' else .00155
                plot.write_frame()
                if kind == 'motion' and abs(frame['time_s']-.002) <= 1e-12:
                    plot.screenshot(str(final))
        finally:
            plot.close()
    counts = {}
    for kind, p in files.items():
        with Image.open(p) as im:
            counts[kind] = im.n_frames
    if any(n != len(frames) for n in counts.values()):
        raise RuntimeError('GIF frame count does not match actual saved FieldData snapshots')
    review_dir = out/'review_frames'
    review_dir.mkdir(exist_ok=True)
    indices = choose_review_indices(frames, report)
    review_paths = {}
    for kind, path in files.items():
        with Image.open(path) as im:
            review_paths[kind] = {}
            for label, idx in indices.items():
                im.seek(idx)
                dst = review_dir/f'{kind}_{label}_{idx:03d}.png'
                im.convert('RGB').save(dst)
                review_paths[kind][label] = {'path':str(dst), 'frame_index':idx, 'time_s':frames[idx]['time_s']}
    manifest = {'timestamp':stamp(), 'status':'RENDERED_AWAITING_VISUAL_REVIEW',
        'saved_FieldData_snapshot_count':len(frames), 'GIF_frame_counts':counts, 'frames':frames,
        'motion_gif':str(files['motion']), 'velocity_gif':str(files['velocity']), 'contact_closeup_gif':str(files['closeup']),
        'final_png':str(final), 'fixed_velocity_scale_m_s':[0.,vmax], 'fixed_camera':True,
        'motion_camera':camera, 'closeup_camera':close_camera, 'actual_motion_only':True,
        'no_synthetic_velocity_or_interpolated_frames':True, 'PyVista_off_screen':True,
        'representative_review_frames':review_paths, 'visual_inspection_completed':False,
        'numerical_report_sha256':sha(EVID/'final_report.json'), 'source_sha256':source_hashes,
        'outputs_sha256':{p.name:sha(p) for p in [*files.values(),final]},
        'visual_QA_note':'Actual PNG and all relevant first/precontact/firstcontact/postcontact/last GIF frames must be inspected with view_image. Rendering and pixel/file checks do not establish visual PASS.'}
    atomic(EVID/'render_manifest.json', manifest)
    print(json.dumps({'status':manifest['status'], 'frames':len(frames)}))


if __name__ == '__main__':
    main()
