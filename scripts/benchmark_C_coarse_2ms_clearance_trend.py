"""Postprocess the stopped trajectory. No Fluent session or runtime input changes."""
import csv, json, os
from pathlib import Path
import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from benchmark_C_coarse_2ms_common import *

DEST = EVID/'clearance_trend_review'


def main():
    verify_frozen()
    state_before = sha(EVID/'state.json')
    history_before = sha(EVID/'dynamic_history.csv')
    rows = [{k:float(v) for k,v in r.items()} for r in history()]
    if [int(r['step']) for r in rows] != list(range(1,74)):
        raise RuntimeError('This reviewed analysis requires the stopped73-row trajectory')
    if read(EVID/'state.json')['status'] != 'HARD_BLOCKER_REQUIRES_REVIEW':
        raise RuntimeError('Terminal clearance review required; do not touch active campaign')
    DEST.mkdir(exist_ok=True)
    p = np.asarray([[r[f'com_{a}_m'] for a in 'xyz'] for r in rows])
    q = np.asarray([[r[f'q{i}'] for i in [1,2,3,0]] for r in rows])
    rotations = Rotation.from_quat(q)
    angles = rotations.magnitude()
    axes = rotations.apply(np.repeat([[1.,0.,0.]], len(rows), axis=0))
    tilts = np.arccos(np.clip(axes[:,0], -1, 1))
    omega = np.linalg.norm([[r[f'omega_{a}_rad_s'] for a in 'xyz'] for r in rows], axis=1)
    lateral = np.linalg.norm(p[:,1:], axis=1)
    gap = np.asarray([r['robot_wall_clearance_m'] for r in rows])
    times = np.asarray([r['time_s'] for r in rows])
    series = []
    for i,r in enumerate(rows):
        g = gap[i]*1000
        flags = [name for value,name in [(.250,'CLEARANCE_WARNING'),(.150,'CLEARANCE_CRITICAL_WARNING'),(.100,'PHYSICAL_CLEARANCE_HARD_STOP')] if g<value]
        series.append({'step':int(r['step']), 'time_ms':times[i]*1000, 'clearance_mm':g,
            'COM_x_m':p[i,0], 'COM_y_m':p[i,1], 'COM_z_m':p[i,2],
            'lateral_COM_displacement_um':lateral[i]*1e6,
            'orientation_angle_rad':angles[i], 'body_X_tilt_rad':tilts[i],
            'body_axis_x':axes[i,0], 'body_axis_y':axes[i,1], 'body_axis_z':axes[i,2],
            'omega_magnitude_rad_s':omega[i], 'omega_x_rad_s':r['omega_x_rad_s'],
            'omega_y_rad_s':r['omega_y_rad_s'], 'omega_z_rad_s':r['omega_z_rad_s'],
            'clearance_status':flags[-1] if flags else 'NORMAL', 'warning_flags':';'.join(flags),
            'static_pose_status':'STATIC_POSE_ENVELOPE_EXCEEDED' if angles[i]>.35 else 'WITHIN_ANGLE_BOUND',
            'orphan_count':int(r['orphan_count']), 'invalid_donor_count':int(r['receptors_without_donors']),
            'minimum_cell_volume_m3':r['minimum_cell_volume_m3'], 'q_norm_error':abs(r['q_norm']-1),
            'donor_length_ratio_max':r['donor_length_ratio_max'],
            'tracked_history_gates_valid':bool(g>=.100 and r['orphan_count']==0 and r['receptors_without_donors']==0 and r['minimum_cell_volume_m3']>0 and r['overset_wall_clearance_m']>0 and abs(r['q_norm']-1)<=1e-6)})
    with (DEST/'clearance_time_series.csv').open('w', newline='', encoding='utf-8') as fp:
        writer=csv.DictWriter(fp,fieldnames=list(series[0]));writer.writeheader();writer.writerows(series)
    first = {}
    for name,value in [('CLEARANCE_WARNING',.250),('CLEARANCE_CRITICAL_WARNING',.150),('PHYSICAL_CLEARANCE_HARD_STOP',.100)]:
        index=int(np.flatnonzero(gap*1000<value)[0])
        first[name]={'threshold_mm':value,'first_saved_sample':series[index],
                     'previous_saved_sample':series[index-1] if index else None,
                     'time_interpretation':'First saved25us sample below threshold; continuous crossing time not determined'}
    first_static=int(np.flatnonzero(angles>.35)[0])
    milestones={f'{step*DT*1000:.3f}ms':{'step':step,'status':'AVAILABLE','clearance_mm':series[step-1]['clearance_mm']} for step in [40,50,60,70]}
    milestones['2.000ms']={'step':80,'status':'NOT_REACHED','clearance_mm':None}
    extrema={}
    for name,values,unit in [('minimum_clearance',gap*1000,'mm'),('maximum_total_rotation',angles,'rad'),
                             ('maximum_body_X_tilt',tilts,'rad'),('peak_omega',omega,'rad/s'),
                             ('maximum_lateral_COM_displacement',lateral*1e6,'um')]:
        i=int(values.argmin() if name=='minimum_clearance' else values.argmax())
        extrema[name]={'value':float(values[i]),'units':unit,'sample':series[i]}
    def trend(values):
        diff=np.diff(values)
        return {'sample_intervals':len(diff),'decreases':int((diff<0).sum()),'increases':int((diff>0).sum()),
                'equal':int((diff==0).sum()),'maximum_increase_mm':float(max(0.,diff.max())),
                'classification':'STRICTLY_MONOTONIC_DECREASING' if (diff<0).all() else 'MIXED'}
    baseline = read(EVID/'final_report.json')
    intervals={}
    for label,start,end,key in [('0_to_1ms',0,40,'pre_ramp_0_to_1ms'),('1_to_1p825ms',40,73,'post_ramp_1_to_current')]:
        group=rows[max(0,start-1):end]
        rec=dict(baseline[key]);start_p=np.asarray(COM0) if start==0 else p[start-1]
        net=p[end-1]-start_p
        rec.update(axial_COM_displacement_m=float(net[0]),lateral_net_COM_displacement_m=float(np.linalg.norm(net[1:])),
                   peak_linear_speed_m_s=float(np.linalg.norm([[r[f'v{a}_m_s'] for a in 'xyz'] for r in group],axis=1).max()),
                   minimum_clearance_mm=min(r['robot_wall_clearance_m'] for r in group)*1000,
                   includes_clearance_failed_terminal_sample=end==73,
                   full_1_to_2ms_interval=False if start==40 else None)
        intervals[label]=rec
    # Evaluate geometry with axial translation held at its saved value. The two
    # factors are rotation and lateralCOM offset; this is not a dynamics rerun.
    fd=EVID/'fielddata';pipe=pv.read(fd/'pipe.vtp').triangulate();robot0=pv.read(fd/'robot_0000.vtp')
    local=robot0.points-np.asarray(COM0)
    sign=float(np.sign(robot0.compute_implicit_distance(pipe)['implicit_distance'][0]))
    if not sign:raise RuntimeError('Reference wall distance has ambiguous sign')
    def geometry_gap(R, com):
        cloud=pv.PolyData(R.apply(local)+np.asarray(com))
        return float(np.min(cloud.compute_implicit_distance(pipe)['implicit_distance']*sign))
    identity_R=Rotation.identity()
    reference_gap=geometry_gap(identity_R,COM0)
    selected=sorted({40,50,60,70,72,73,first_static+1,*[v['first_saved_sample']['step'] for v in first.values()]})
    geometry=[]
    for step in selected:
        i=step-1;R=rotations[i];com=p[i]
        axial_com=np.asarray([com[0],0.,0.])
        g00=geometry_gap(identity_R,axial_com);g10=geometry_gap(R,axial_com)
        g01=geometry_gap(identity_R,com);g11=geometry_gap(R,com)
        if abs(g11-gap[i])>1e-8:raise RuntimeError('Native pose geometry differs from actual saved clearance')
        rotation_share=.5*((g00-g10)+(g01-g11))
        lateral_share=.5*((g00-g01)+(g10-g11))
        if abs(rotation_share+lateral_share-(g00-g11))>1e-12:raise RuntimeError('Geometric accounting does not close')
        geometry.append({'step':step,'time_ms':times[i]*1000,'clearance_reference_mm':reference_gap*1000,
            'clearance_axial_only_mm':g00*1000,'clearance_rotation_only_at_native_x_mm':g10*1000,
            'clearance_lateral_only_at_native_x_mm':g01*1000,'clearance_full_native_pose_mm':g11*1000,
            'axial_geometry_clearance_reduction_mm':(reference_gap-g00)*1000,
            'rotation_geometry_clearance_reduction_mm':rotation_share*1000,
            'lateral_COM_geometry_clearance_reduction_mm':lateral_share*1000,
            'signed_gap_interaction_mm':(g11-g10-g01+g00)*1000,
            'lateral_at_native_rotation_clearance_change_mm':(g11-g10)*1000,
            'rotation_at_native_lateral_clearance_change_mm':(g11-g01)*1000,
            'native_pose_to_recorded_clearance_error_m':g11-gap[i],
            'geometric_effect_dominant':'ROTATION_TILT' if abs(rotation_share)>abs(lateral_share) else 'LATERAL_COM'})
        print(json.dumps({'geometry_review_step':step,'rotation_gap_reduction_mm':rotation_share*1000,
                          'lateral_gap_reduction_mm':lateral_share*1000}),flush=True)
    with (DEST/'clearance_geometric_contributions.csv').open('w',newline='',encoding='utf-8') as fp:
        writer=csv.DictWriter(fp,fieldnames=list(geometry[0]));writer.writeheader();writer.writerows(geometry)
    old_robots=sorted((BASE/'fielddata').glob('robot_*.vtp'))
    report={'timestamp':stamp(),'campaign':CAMPAIGN,'campaign_status':'HARD_BLOCKER_REQUIRES_REVIEW',
        'numerical_2ms_result':'INCOMPLETE','failure_class':'PHYSICAL_CLEARANCE','completed_steps':73,
        'time_s':times[-1],'last_all_gate_valid_step':72,'analysis_timesteps_advanced':0,
        'latest_saved_native_checkpoint_step':73,'latest_all_gate_valid_native_checkpoint_step':70,
        'checkpoint_distinction':'Step72 has valid history/FieldData; no native CASE+DATA72 was saved. Step73 STOP CASE+DATA is intact but fails physical clearance.',
        'solver_launched_or_connected':False,'active_worker_inputs_changed':False,
        'thresholds_mm':{'warning':.250,'critical_warning':.150,'hard_stop':.100},
        'warning_origin':'RETROSPECTIVE_POSTPROCESSING_ONLY; warnings were not installed in the completed worker',
        'warning_stop_policy':'Only clearance<.100mm is a physical gap hard stop; .250/.150mm are observation-only',
        'first_threshold_samples':first,'clearance_milestones':milestones,
        'full_saved_history_trend':trend(gap*1000),'post1ms_trend':trend(gap[39:]*1000),
        'early_increase_steps':[int(rows[i+1]['step']) for i in np.flatnonzero(np.diff(gap)>0)],
        'extrema':extrema,'first_static_rotation_angle_exceedance':series[first_static],
        'static_angle_exceedance_is_only_warning':True,'orientation_angle_definition':'Quaternion total rotation from reference',
        'body_X_tilt_definition':'Angle between rotated reference body+X and global+X; distinct from total rotation',
        'intervals':intervals,'geometry_samples':geometry,
        'geometry_accounting_method':'Two-factor symmetric marginal accounting: average rotation effect over lateralCOM absent/present, and vice versa; actual axialCOM held fixed for each native sample',
        'signed_gap_interaction_definition':'g(full)-g(rotation-only)-g(lateral-only)+g(axial-only); positive means increased gap, not gap reduction',
        'tracked_history_gates_scope':'Clearance, orphan/invalid donor, positive cell and envelope clearance, quaternion norm only. Full runtime verification remains in final_report.json and archived connectivity/native records.',
        'geometry_scope':'Saved rigid geometry on actual pipe triangulation; attribution of gap change, not of magnetic/fluid dynamical causes. Reference0 gap used for decomposition; interval dynamics reported separately.',
        'pre0_to1ms_FieldData_robot_files_preserved':len(old_robots),
        'continuation_FieldData_snapshot_count':len(read(EVID/'snapshots.json')['frames']),
        'resources_continuation_attempt':read(EVID/'memory_summary.json'),
        'resources_original1ms_verified_summary':{k:v for k,v in read(BASE/'final_report.json')['verified_memory_summary'].items() if k in ['total_actual_sample_count','window_actual_sample_count','peak_project_working_set_gib','window_peak_measured_system_commit_gib','window_peak_measured_system_commit_percent','window_resource_guards_stable']},
        'original1ms_resource_scope':'Original accepted20h window,765 resource samples; historical Oct2 resource-stop attempts excluded from the window commit peak.',
        'original1ms_resource_source':'evidence/benchmark_C_coarse_A/continuation_20h/verified_memory_summary.json',
        'fine_comparison':'FINE_REFERENCE_NOT_AVAILABLE_AT_2MS','mesh_convergence_claim':False,
        'native2ms_checkpoint':'NOT_REACHED','motion_GIF':'NOT_READY','velocity_GIF':'NOT_READY','visual_review':'NOT_READY',
        'source_sha256':{'evidence/benchmark_C_coarse_2ms/dynamic_history.csv':history_before,
                         'evidence/benchmark_C_coarse_2ms/state.json':state_before,
                         'evidence/benchmark_C_coarse_2ms/final_report.json':sha(EVID/'final_report.json'),
                         'scripts/benchmark_C_coarse_2ms_clearance_trend.py':sha(Path(__file__))}}
    atomic(DEST/'clearance_trend_report.json',report)
    # Diagnostic scientific plot; no interpolation or replacement2ms data.
    os.environ.setdefault('MPLCONFIGDIR',str(DEST/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
    fig,axs=plt.subplots(2,2,figsize=(11.5,7.5),layout='constrained')
    x=times*1000; blue='#225a82'; orange='#ba661c'; red='#a12c33';gray='#777777'
    ax=axs[0,0];ax.plot(x,gap*1000,color=blue,lw=2,label='Recorded physical gap')
    for level,label,color in [(.250,'Warning0.250mm',orange),(.150,'Critical warning0.150mm',gray),(.100,'Hard stop0.100mm',red)]:
        ax.axhline(level,color=color,lw=1,ls='--',label=label)
    ax.scatter([x[-1]],[gap[-1]*1000],c=red,s=25,zorder=5)
    ax.set(title='a  Robot-wall clearance',ylabel='Physical clearance (mm)',ylim=(0,.54))
    ax.legend(fontsize=8,loc='upper right')
    ax=axs[0,1];ax.plot(x,angles,color=blue,lw=2,label='Quaternion total rotation')
    ax.plot(x,tilts,color=orange,lw=1.1,ls='--',label='Body+X tilt relative to global+X')
    ax.axhline(.35,color=gray,lw=1,ls=':',label='Static angle bound (warning only)')
    ax.set(title='b  Native rigid-body orientation',ylabel='Angle (rad)');ax.legend(fontsize=8,loc='upper left')
    ax=axs[1,0];ax.plot(x,p[:,1]*1e6,color=blue,lw=2,label='COMy');ax.plot(x,p[:,2]*1e6,color=orange,lw=1.7,label='COMz')
    ax.plot(x,lateral*1e6,color=gray,ls='--',lw=1.2,label='Lateral displacement magnitude')
    ax.set(title='c  Lateral COM migration',ylabel='Displacement (um)');ax.legend(fontsize=8,loc='upper left')
    ax=axs[1,1];ax.plot(x,omega,color=blue,lw=2,label='Native angular speed')
    ax.set(title='d  Angular velocity magnitude',ylabel='Angular speed (rad/s)');ax.legend(fontsize=8,loc='upper left')
    for ax in axs.flat:
        ax.set(xlim=(0,2),xlabel='Absolute native time (ms)')
        ax.axvline(1,color=gray,ls=':',lw=1)
        ax.axvspan(x[-1],2,color='#eeeeee',zorder=-5)
        ax.grid(alpha=.16)
    fig.suptitle('Benchmark C coarse2ms continuation — INCOMPLETE at73/80 (1.825ms)',fontsize=14,fontweight='bold')
    fig.supxlabel('Actual saved steps1–73; warning flags computed afterward. Shaded interval after1.825ms is UNSOLVED.',fontsize=9)
    fig.savefig(DEST/'clearance_trend.png',dpi=160);fig.savefig(DEST/'clearance_trend.svg');plt.close(fig)
    svg=DEST/'clearance_trend.svg'
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf-8').splitlines())+'\n',encoding='utf-8')
    atomic(DEST/'artifact_manifest.json',{'timestamp':stamp(),'status':'GENERATED_AWAITING_PLOT_REVIEW',
        'files_sha256':{p.name:sha(p) for p in DEST.iterdir() if p.is_file() and p.name!='artifact_manifest.json'},
        'scientific_movie_visual_review_status':'NOT_READY','diagnostic_plot_not_a_2ms_run_visualization':True})
    verify_frozen()
    if sha(EVID/'state.json')!=state_before or sha(EVID/'dynamic_history.csv')!=history_before:
        raise RuntimeError('Authoritative state/history changed during analysis')
    print(json.dumps({'status':'GENERATED_AWAITING_PLOT_REVIEW','output_directory':str(DEST),'samples':len(series)}))


if __name__=='__main__':main()
