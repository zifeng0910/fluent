"""Offline tilt/impulse diagnostics and diagnostic visualizations.

This module reads only saved native histories, callback traces and VTP files.
It never starts Fluent, a worker, a scheduled task, or a fallback route.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np
import pyvista as pv
from PIL import Image

from benchmark_D_native_contact_overnight_common import ROOT, EVID, read, atomic, sha, stamp


OUT = EVID / 'offline_diagnostics'
VIS = EVID / 'diagnostic_visuals'
BRANCHES = {'micro25': 25e-6, 'micro12p5': 12.5e-6, 'micro6p25': 6.25e-6}
TIME_TOL = 1e-12
PIPE_AXIS = np.array([1., 0., 0.])


def norm(v):
    return float(np.linalg.norm(np.asarray(v, dtype=float)))


def quat_angle_axis(q):
    q = np.asarray(q, dtype=float)
    q = q / norm(q)
    angle = 2.0 * math.atan2(norm(q[1:]), abs(float(q[0])))
    s = norm(q[1:])
    axis = (q[1:] / s).tolist() if s > 1e-15 else [1., 0., 0.]
    return angle, axis


def rotate_x(q):
    # WXYZ quaternion, matching the review implementation.
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y)])


def tilt(q):
    return float(math.acos(np.clip(float(np.dot(rotate_x(q), PIPE_AXIS)), -1., 1.)))


def history(branch):
    d = read(EVID / 'branches' / branch / 'native_history.json')
    records = [d['initial_state'], *d.get('records', [])]
    out = []
    for r in records:
        ns = r['native_state']
        out.append({'time_s': float(r['time_s']), 'step': int(r.get('native_step_index', -1)),
            'com_x_m': ns['com'][0], 'com_y_m': ns['com'][1], 'com_z_m': ns['com'][2],
            'q0': ns['q'][0], 'q1': ns['q'][1], 'q2': ns['q'][2], 'q3': ns['q'][3],
            'tilt_rad': tilt(ns['q']), 'tilt_deg': math.degrees(tilt(ns['q'])),
            'rotation_angle_rad': quat_angle_axis(ns['q'])[0],
            'rotation_axis_x': quat_angle_axis(ns['q'])[1][0],
            'rotation_axis_y': quat_angle_axis(ns['q'])[1][1],
            'rotation_axis_z': quat_angle_axis(ns['q'])[1][2],
            'omega_x_rad_s': ns['omega'][0], 'omega_y_rad_s': ns['omega'][1],
            'omega_z_rad_s': ns['omega'][2], 'omega_magnitude_rad_s': norm(ns['omega']),
            'v_x_m_s': ns['velocity'][0], 'v_y_m_s': ns['velocity'][1], 'v_z_m_s': ns['velocity'][2],
            'signed_gap_m': float(r.get('signed_gap_m', r.get('physical_gap_m', float('nan')))),
            'native_state': ns})
    return out


def event_rows(branch):
    path = EVID / 'branches' / branch / 'native_contact_callback_trace_node.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def nearest(rows, t):
    return min(rows, key=lambda r: abs(r['time_s'] - t))


def exact(rows, t):
    return next((r for r in rows if abs(r['time_s'] - t) <= TIME_TOL), None)


def applied_events(branch):
    return [r for r in event_rows(branch) if r.get('action') == 'APPLIED']


def episodes(branch, dt):
    events = applied_events(branch)
    eps = []
    for e in events:
        if not eps or e['CURRENT_TIME'] - eps[-1][-1]['CURRENT_TIME'] > 1.5 * dt:
            eps.append([])
        eps[-1].append(e)
    result = []
    for ep in eps:
        J = np.array([float(e['impulse_N_s']) for e in ep])
        normal_sum = np.sum([float(e['impulse_N_s']) * np.asarray(e['contact_normal'], dtype=float) for e in ep], axis=0)
        angular = []
        for e in ep:
            r = np.asarray(e['contact_point'], dtype=float) - np.asarray(e['DT_CG'], dtype=float)
            angular.append(np.cross(r, float(e['impulse_N_s']) * np.asarray(e['contact_normal'], dtype=float)))
        angular_sum = np.sum(angular, axis=0)
        start = float(ep[0]['CURRENT_TIME'])
        end = float(ep[-1]['CURRENT_TIME'] + dt)
        result.append({'start_time_s': start, 'end_time_s': end, 'duration_s': end-start,
            'impulse_count': len(ep), 'impulse_density_per_s': len(ep)/(end-start) if end > start else None,
            'J_values_N_s': J.tolist(), 'sum_J_N_s': float(J.sum()),
            'sum_normal_impulse_vector_N_s': normal_sum.tolist(),
            'sum_normal_impulse_magnitude_N_s': norm(normal_sum),
            'sum_torque_impulse_N_m_s': angular_sum.tolist(),
            'sum_torque_impulse_magnitude_N_m_s': norm(angular_sum),
            'event_times_s': [float(e['CURRENT_TIME']) for e in ep]})
    return result


def timeline_csv(branch, rows):
    fields = [k for k in rows[0] if k != 'native_state']
    path = OUT / f'tilt_timeline_{branch.replace("micro", "").replace("p", "p")}us.csv'
    # Keep the requested filenames exactly.
    path = OUT / {'micro25': 'tilt_timeline_25us.csv', 'micro12p5': 'tilt_timeline_12p5us.csv',
                  'micro6p25': 'tilt_timeline_6p25us.csv'}[branch]
    with path.open('w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{k: r.get(k) for k in fields} for r in rows])
    return path


def impulse_cumulative(branch, rows, dt):
    events = applied_events(branch)
    ep = episodes(branch, dt)
    out = []
    total_n = np.zeros(3)
    total_t = np.zeros(3)
    i = 0
    for r in rows:
        while i < len(events) and events[i]['CURRENT_TIME'] <= r['time_s'] + TIME_TOL:
            e = events[i]
            Jn = float(e['impulse_N_s']) * np.asarray(e['contact_normal'], dtype=float)
            rr = np.asarray(e['contact_point'], dtype=float) - np.asarray(e['DT_CG'], dtype=float)
            total_n += Jn
            total_t += np.cross(rr, Jn)
            i += 1
        out.append({'time_s': r['time_s'], 'cumulative_normal_impulse_magnitude_N_s': norm(total_n),
            'cumulative_normal_impulse_x_N_s': total_n[0], 'cumulative_normal_impulse_y_N_s': total_n[1],
            'cumulative_normal_impulse_z_N_s': total_n[2], 'cumulative_angular_impulse_magnitude_N_m_s': norm(total_t),
            'cumulative_angular_impulse_x_N_m_s': total_t[0], 'cumulative_angular_impulse_y_N_m_s': total_t[1],
            'cumulative_angular_impulse_z_N_m_s': total_t[2]})
    return out, ep


def common_time_csv(a_rows, z_rows, a_cum, z_cum, a_branch='micro12p5', z_branch='micro6p25'):
    am = {round(r['time_s'], 15): r for r in a_rows}
    zm = {round(r['time_s'], 15): r for r in z_rows}
    ac = {round(r['time_s'], 15): r for r in a_cum}
    zc = {round(r['time_s'], 15): r for r in z_cum}
    rows = []
    for key in sorted(set(am) & set(zm)):
        a, z = am[key], zm[key]
        aa, zz = ac[key], zc[key]
        qdot = abs(float(np.dot([a['q0'], a['q1'], a['q2'], a['q3']], [z['q0'], z['q1'], z['q2'], z['q3']])))
        rows.append({'time_s': a['time_s'],
            'com_difference_m': norm(np.array([a['com_x_m'], a['com_y_m'], a['com_z_m']]) - np.array([z['com_x_m'], z['com_y_m'], z['com_z_m']])),
            'orientation_difference_rad': 2.0 * math.acos(np.clip(qdot, -1., 1.)),
            'tilt_12p5_deg': a['tilt_deg'], 'tilt_6p25_deg': z['tilt_deg'], 'tilt_difference_deg': abs(a['tilt_deg']-z['tilt_deg']),
            'v_difference_m_s': norm(np.array([a['v_x_m_s'],a['v_y_m_s'],a['v_z_m_s']]) - np.array([z['v_x_m_s'],z['v_y_m_s'],z['v_z_m_s']])),
            'omega_difference_rad_s': norm(np.array([a['omega_x_rad_s'],a['omega_y_rad_s'],a['omega_z_rad_s']]) - np.array([z['omega_x_rad_s'],z['omega_y_rad_s'],z['omega_z_rad_s']])),
            'signed_gap_12p5_m': a['signed_gap_m'], 'signed_gap_6p25_m': z['signed_gap_m'],
            'signed_gap_difference_m': abs(a['signed_gap_m']-z['signed_gap_m']),
            'cumulative_normal_impulse_12p5_N_s': aa['cumulative_normal_impulse_magnitude_N_s'],
            'cumulative_normal_impulse_6p25_N_s': zz['cumulative_normal_impulse_magnitude_N_s'],
            'cumulative_normal_impulse_difference_N_s': abs(aa['cumulative_normal_impulse_magnitude_N_s']-zz['cumulative_normal_impulse_magnitude_N_s']),
            'cumulative_angular_impulse_12p5_N_m_s': aa['cumulative_angular_impulse_magnitude_N_m_s'],
            'cumulative_angular_impulse_6p25_N_m_s': zz['cumulative_angular_impulse_magnitude_N_m_s'],
            'cumulative_angular_impulse_difference_N_m_s': abs(aa['cumulative_angular_impulse_magnitude_N_m_s']-zz['cumulative_angular_impulse_magnitude_N_m_s'])})
    path = OUT / 'common_time_12p5_vs_6p25.csv'
    with path.open('w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    return path, rows


def write_cumulative(branch, data):
    path = OUT / f'cumulative_{"normal" if branch else ""}_impulse_vs_time.csv'
    path = OUT / {'micro12p5': 'cumulative_normal_impulse_12p5us.csv', 'micro6p25': 'cumulative_normal_impulse_6p25us.csv'}[branch]
    with path.open('w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    return path


def write_common_cumulative(common):
    normal_path = OUT / 'cumulative_normal_impulse_vs_time.csv'
    angular_path = OUT / 'cumulative_angular_impulse_vs_time.csv'
    with normal_path.open('w', newline='', encoding='utf-8') as fp:
        fields = ['time_s', 'normal_12p5_N_s', 'normal_6p25_N_s', 'difference_N_s']
        writer = csv.DictWriter(fp, fieldnames=fields); writer.writeheader()
        writer.writerows({'time_s': r['time_s'], 'normal_12p5_N_s': r['cumulative_normal_impulse_12p5_N_s'],
            'normal_6p25_N_s': r['cumulative_normal_impulse_6p25_N_s'], 'difference_N_s': r['cumulative_normal_impulse_difference_N_s']} for r in common)
    with angular_path.open('w', newline='', encoding='utf-8') as fp:
        fields = ['time_s', 'angular_12p5_N_m_s', 'angular_6p25_N_m_s', 'difference_N_m_s']
        writer = csv.DictWriter(fp, fieldnames=fields); writer.writeheader()
        writer.writerows({'time_s': r['time_s'], 'angular_12p5_N_m_s': r['cumulative_angular_impulse_12p5_N_m_s'],
            'angular_6p25_N_m_s': r['cumulative_angular_impulse_6p25_N_m_s'], 'difference_N_m_s': r['cumulative_angular_impulse_difference_N_m_s']} for r in common)
    return normal_path, angular_path


def phase_summary(branch, rows, eps, dt):
    first = eps[0]['start_time_s'] if eps else None
    completed = first + dt if first is not None else None
    def get(t):
        r = exact(rows, t)
        return r if r else nearest(rows, t)
    vals = {'at_1p750ms': get(.00175), 'at_1p800ms': get(.0018),
        'immediately_pre_response': get(first) if first is not None else None,
        'immediately_post_response': get(completed) if completed is not None else None,
        'at_1p850ms': get(.00185), 'at_1p900ms': get(.0019)}
    initial, pre, post = vals['at_1p750ms'], vals['immediately_pre_response'], vals['immediately_post_response']
    d_pre = pre['tilt_rad'] - initial['tilt_rad'] if initial and pre else None
    d_step = post['tilt_rad'] - pre['tilt_rad'] if post and pre else None
    d_post = vals['at_1p900ms']['tilt_rad'] - post['tilt_rad'] if post else None
    return {'first_callback_time_s': first, 'first_completed_response_time_s': completed,
        'tilt_rad': {k: (v['tilt_rad'] if v else None) for k,v in vals.items()},
        'tilt_deg': {k: (v['tilt_deg'] if v else None) for k,v in vals.items()},
        'omega_magnitude_rad_s': {k: (v['omega_magnitude_rad_s'] if v else None) for k,v in vals.items()},
        'delta_tilt_pre_response_rad': d_pre, 'delta_tilt_first_response_step_rad': d_step,
        'delta_tilt_post_response_to_final_rad': d_post,
        'tilt_rate_first_response_rad_s': d_step/dt if d_step is not None else None,
        'max_tilt_rad': max(r['tilt_rad'] for r in rows), 'max_tilt_deg': max(r['tilt_deg'] for r in rows),
        'peak_omega_rad_s': max(r['omega_magnitude_rad_s'] for r in rows),
        'omega_components_pre_response': [pre[k] for k in ('omega_x_rad_s','omega_y_rad_s','omega_z_rad_s')] if pre else None,
        'omega_components_post_response': [post[k] for k in ('omega_x_rad_s','omega_y_rad_s','omega_z_rad_s')] if post else None,
        'phase_labels': {'A': 'pre-proximity', 'B': 'threshold crossing', 'C': 'first native proximity response',
                         'D': 'immediate post-response', 'E': 'late near-wall response', 'F': '1.900 ms'},
        'interpretation_basis': 'Tilt is measured from the global pipe axis using the recorded rigid-body quaternion; all values are exact saved native states when available.'}


def make_plots(all_rows, common):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10,5), constrained_layout=True)
    for b, rows in all_rows.items(): ax.plot([r['time_s']*1e3 for r in rows], [r['tilt_deg'] for r in rows], label=b)
    ax.set(xlabel='Time (ms)', ylabel='Tilt from pipe axis (deg)'); ax.grid(alpha=.25); ax.legend(); fig.savefig(VIS/'benchmark_D_tilt_vs_time.png', dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10,5), constrained_layout=True)
    for b, rows in all_rows.items(): ax.plot([r['time_s']*1e3 for r in rows], [r['signed_gap_m']*1e6 for r in rows], label=b)
    ax.set(xlabel='Time (ms)', ylabel='Signed gap (µm)'); ax.grid(alpha=.25); ax.legend(); fig.savefig(VIS/'benchmark_D_gap_vs_time.png', dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10,5), constrained_layout=True)
    ax.plot([r['time_s']*1e3 for r in common], [r['cumulative_normal_impulse_12p5_N_s'] for r in common], label='12.5 µs')
    ax.plot([r['time_s']*1e3 for r in common], [r['cumulative_normal_impulse_6p25_N_s'] for r in common], label='6.25 µs')
    ax.set(xlabel='Time (ms)', ylabel='Cumulative normal impulse (N·s)'); ax.grid(alpha=.25); ax.legend(); fig.savefig(VIS/'benchmark_D_cumulative_impulse_vs_time.png', dpi=160); plt.close(fig)


def render_gifs(rows, report):
    branch_dir = EVID / 'branches' / 'micro6p25' / 'fielddata'
    frames = []
    vmax = 0.
    for r in rows:
        step = r['step']; robot_path = branch_dir/f'state_{step:04d}_robot.vtp'; velocity_path = branch_dir/f'state_{step:04d}_velocity.vtp'
        robot, velocity = pv.read(robot_path), pv.read(velocity_path)
        if not robot.n_points or not velocity.n_points or 'velocity_m_s' not in velocity.point_data: raise RuntimeError('Missing actual FieldData')
        vv = np.asarray(velocity.point_data['velocity_m_s']); vmax = max(vmax, float(np.max(vv)))
        frames.append((r, robot_path, velocity_path, robot, velocity))
    pipe_path = branch_dir/'pipe.vtp'; pipe = pv.read(pipe_path)
    contact = np.asarray(read(EVID/'branches/micro6p25/review.json')['metrics']['first_contact_point_m'])
    close_camera = [(contact[0]-.00035, contact[1]+.00010, .012), tuple(contact), (0.,1.,0.)]
    camera = [(.0012, 0., .012), (.0012,0.,0.), (0.,1.,0.)]
    contact_marker = pv.Sphere(radius=2.0e-5, center=contact, theta_resolution=16, phi_resolution=8)
    VIS.mkdir(parents=True, exist_ok=True)
    outputs = {'motion': VIS/'benchmark_D_micro6p25_motion_1p75_to_1p90ms.gif',
               'closeup': VIS/'benchmark_D_micro6p25_contact_closeup_1p75_to_1p90ms.gif',
               'velocity': VIS/'benchmark_D_micro6p25_velocity_1p75_to_1p90ms.gif'}
    for kind, path in outputs.items():
        p = pv.Plotter(off_screen=True, window_size=(1100, 760)); p.set_background('#f4f7fa'); p.open_gif(str(path), fps=8)
        try:
            for r, rp, vp, robot, velocity in frames:
                p.clear(); p.add_mesh(pipe, color='#4f6678', opacity=.42 if kind=='closeup' else .32,
                    show_edges=True, edge_color='#3b5264', line_width=0.5)
                if kind == 'velocity': p.add_mesh(velocity, scalars='velocity_m_s', cmap='viridis', clim=(0., vmax), opacity=1., show_scalar_bar=True, scalar_bar_args={'title':'Velocity (m/s)'})
                p.add_mesh(robot, color='#e9822d', smooth_shading=True)
                p.add_mesh(contact_marker, color='#d12f2f', smooth_shading=True)
                gap_um = r['signed_gap_m']*1e6; text = f'Diagnostic visualization | micro6p25 | t={r["time_s"]*1e3:.5f} ms\nSigned gap={gap_um:.2f} µm | tilt={r["tilt_deg"]:.3f}° | |ω|={r["omega_magnitude_rad_s"]:.2f} rad/s\nNative near-wall proximity/contact response; not final 2.000-ms production result'
                p.add_text(text, font_size=13, color='#172434', position='upper_left')
                p.camera_position = close_camera if kind=='closeup' else camera; p.camera.parallel_projection=True; p.camera.parallel_scale=.00075 if kind=='closeup' else .00155
                p.write_frame()
                if kind == 'motion' and abs(r['time_s']-.0019) < TIME_TOL: p.screenshot(str(VIS/'benchmark_D_micro6p25_1p90ms_final_frame.png'))
        finally: p.close()
    counts = {}
    for k,pth in outputs.items():
        with Image.open(pth) as im: counts[k] = im.n_frames
    return outputs, vmax, counts


def main():
    OUT.mkdir(parents=True, exist_ok=True); VIS.mkdir(parents=True, exist_ok=True)
    all_rows = {}; all_eps = {}; all_cum = {}
    for branch, dt in BRANCHES.items():
        rows = history(branch); all_rows[branch] = rows; all_eps[branch] = episodes(branch, dt)
        timeline_csv(branch, rows)
        cumulative, _ = impulse_cumulative(branch, rows, dt); all_cum[branch] = cumulative
        if branch in ('micro12p5','micro6p25'): write_cumulative(branch, cumulative)
    common_path, common = common_time_csv(all_rows['micro12p5'], all_rows['micro6p25'], all_cum['micro12p5'], all_cum['micro6p25'])
    normal_cum_path, angular_cum_path = write_common_cumulative(common)
    make_plots(all_rows, common)
    phase = {b: phase_summary(b, rows, all_eps[b], BRANCHES[b]) for b, rows in all_rows.items()}
    final_n12, final_n625 = common[-1]['cumulative_normal_impulse_12p5_N_s'], common[-1]['cumulative_normal_impulse_6p25_N_s']
    final_a12, final_a625 = common[-1]['cumulative_angular_impulse_12p5_N_m_s'], common[-1]['cumulative_angular_impulse_6p25_N_m_s']
    e12, e625 = all_eps['micro12p5'], all_eps['micro6p25']
    p12, p625 = phase['micro12p5'], phase['micro6p25']
    pre_fraction = abs(p625['delta_tilt_first_response_step_rad']) / max(abs(p625['delta_tilt_pre_response_rad']) + abs(p625['delta_tilt_first_response_step_rad']), 1e-15)
    if pre_fraction < .25: tilt_class = 'TILT_PRIMARILY_PREEXISTING'
    elif pre_fraction > .75: tilt_class = 'TILT_PRIMARILY_NATIVE_RESPONSE_DRIVEN'
    else: tilt_class = 'TILT_AMPLIFIED_BY_NATIVE_RESPONSE'
    count_ratio = len(applied_events('micro6p25')) / max(len(applied_events('micro12p5')), 1)
    indiv12 = [j for e in e12 for j in e['J_values_N_s']]; indiv625 = [j for e in e625 for j in e['J_values_N_s']]
    normal_rel = abs(final_n12-final_n625)/max(abs(final_n625),1e-30); angular_rel = abs(final_a12-final_a625)/max(abs(final_a625),1e-30)
    impulse_class = 'RAW_COUNT_TIMESTEP_ARTIFACT_LIKELY' if count_ratio >= 1.8 and count_ratio <= 2.2 and np.median(indiv625) < np.median(indiv12) and normal_rel <= .1 and angular_rel <= .1 else 'MIXED'
    outputs, vmax, gif_counts = render_gifs(all_rows['micro6p25'], read(EVID/'branches/micro6p25/review.json'))
    diagnostic = {'timestamp': stamp(), 'campaign': 'BENCHMARK_D_NATIVE_CONTACT_OVERNIGHT', 'offline_only': True,
        'scientific_scope': 'Native near-wall contact/proximity response triggered by the saved ~0.100 mm threshold; no ideal zero-gap impact is claimed.',
        'frozen_convergence_unchanged': True, 'frozen_result': 'FAIL', 'frozen_failure': {'impulse_counts': [7,14], 'difference':7, 'allowed':3},
        'tilt_classification': tilt_class, 'tilt_classification_method': 'Compare pre-response tilt accumulation with the first completed native response step using exact recorded states.',
        'phase_summary': phase, 'episodes': {'micro12p5': e12, 'micro6p25': e625},
        'impulse_interpretation': impulse_class, 'impulse_count_ratio_6p25_over_12p5': count_ratio,
        'individual_impulse_median_N_s': {'micro12p5': float(np.median(indiv12)), 'micro6p25': float(np.median(indiv625))},
        'cumulative_impulse': {'normal_12p5_N_s': final_n12, 'normal_6p25_N_s': final_n625, 'normal_difference_N_s': abs(final_n12-final_n625), 'normal_relative_difference': normal_rel,
            'angular_12p5_N_m_s': final_a12, 'angular_6p25_N_m_s': final_a625, 'angular_difference_N_m_s': abs(final_a12-final_a625), 'angular_relative_difference': angular_rel},
        'trajectory_comparison': {'common_time_csv': str(common_path), 'common_time_count': len(common), 'no_interpolation': True, 'final_com_difference_m': common[-1]['com_difference_m'], 'final_tilt_difference_deg': common[-1]['tilt_difference_deg'], 'final_v_difference_m_s': common[-1]['v_difference_m_s'], 'final_omega_difference_rad_s': common[-1]['omega_difference_rad_s']},
        'outputs': {'timelines': [str(OUT/f'tilt_timeline_{x}us.csv') for x in ('25','12p5','6p25')], 'common_time_csv': str(common_path), 'cumulative_normal_impulse_csv': [str(normal_cum_path), str(OUT/f'cumulative_normal_impulse_12p5us.csv'), str(OUT/f'cumulative_normal_impulse_6p25us.csv')], 'cumulative_angular_impulse_csv': [str(angular_cum_path)], 'plots': [str(VIS/x) for x in ('benchmark_D_tilt_vs_time.png','benchmark_D_gap_vs_time.png','benchmark_D_cumulative_impulse_vs_time.png')], 'motion_gif': str(outputs['motion']), 'contact_closeup_gif': str(outputs['closeup']), 'velocity_gif': str(outputs['velocity']), 'final_frame_png': str(VIS/'benchmark_D_micro6p25_1p90ms_final_frame.png'), 'velocity_gif_source': 'actual saved FieldData snapshots', 'gif_frame_counts': gif_counts, 'fixed_velocity_scale_m_s': [0.,vmax]},
        'diagnostic_visualization_metadata': {'not_final_2ms_result': True, 'time_range_s': [0.00175,0.0019], 'branch': 'micro6p25', 'actual_geometry_and_fielddata_only': True}}
    atomic(OUT/'tilt_and_impulse_diagnostic.json', diagnostic)
    md = OUT/'tilt_and_impulse_diagnostic.md'
    md.write_text(f'''# Benchmark D offline tilt and impulse diagnostic

- Tilt classification: **{tilt_class}**. The 6.25 µs branch is already at {p625['tilt_deg']['immediately_pre_response']:.3f}° immediately before the first response; the first completed response adds only {math.degrees(p625['delta_tilt_first_response_step_rad']):.3f}°.
- The robot begins the saved interval at {p625['tilt_deg']['at_1p750ms']:.3f}° and reaches {p625['tilt_deg']['at_1p900ms']:.3f}° at 1.900 ms, so most visible tilt is accumulated before the native proximity response.
- Peak angular rate is {p625['peak_omega_rad_s']:.3f} rad/s, while the first response changes the recorded angular-rate magnitude from {p625['omega_magnitude_rad_s']['immediately_pre_response']:.3f} to {p625['omega_magnitude_rad_s']['immediately_post_response']:.3f} rad/s.
- Frozen 12.5 vs 6.25 µs convergence: **FAIL remains unchanged** (7 vs 14 impulses; allowed difference 3).
- One response episode spans {e12[0]['start_time_s']*1e3:.5f}–{e12[0]['end_time_s']*1e3:.5f} ms in both branches. Halving dt doubles the raw callback impulse count and reduces the median individual impulse, while cumulative normal impulse differs by {normal_rel:.4%} and cumulative angular impulse by {angular_rel:.4%}.
- Impulse interpretation: **{impulse_class}**. This is a physical interpretation of the saved episode, not a retroactive change to the frozen gate.
- Final exact-common-time trajectory differences are COM {common[-1]['com_difference_m']:.6g} m, tilt {common[-1]['tilt_difference_deg']:.6g}°, velocity {common[-1]['v_difference_m_s']:.6g} m/s, and angular rate {common[-1]['omega_difference_rad_s']:.6g} rad/s.
- Visuals cover 1.750–1.900 ms and are diagnostic only; they are not a final 2.000-ms production result. The saved first response is a native near-wall proximity/contact response at the ~100 µm threshold, not an ideal zero-gap impact.
''', encoding='utf-8')
    # A compact manifest is written before visual QA; QA is completed by the caller with view_image.
    atomic(VIS/'visual_manifest.json', {'timestamp':stamp(), 'status':'GENERATED_AWAITING_VISUAL_REVIEW', 'diagnostic_only':True, 'branch':'micro6p25', 'time_range_s':[.00175,.0019], 'outputs':diagnostic['outputs'], 'gif_frame_counts':gif_counts, 'actual_geometry_and_fielddata_only':True, 'sha256':{p.name:sha(Path(p)) for p in [*outputs.values(), VIS/'benchmark_D_micro6p25_1p90ms_final_frame.png']}})
    print(json.dumps({'status':'GENERATED_AWAITING_VISUAL_REVIEW','tilt_classification':tilt_class,'impulse_interpretation':impulse_class,'normal_relative_difference':normal_rel,'gif_frame_counts':gif_counts}))


if __name__ == '__main__':
    main()
