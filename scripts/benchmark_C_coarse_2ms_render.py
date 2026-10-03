"""Render only actual saved FieldData, fixed camera and velocity scale."""
import json
import numpy as np, pyvista as pv
from PIL import Image
from benchmark_C_coarse_2ms_common import *

def main():
    report=read(EVID/'final_report.json')
    if report.get('status')!='BENCHMARK_C_COARSE_FREE_6DOF_2MS_PASS':raise RuntimeError('Full2ms numerical PASS required')
    frames=read(EVID/'snapshots.json')['frames'];fd=EVID/'fielddata';out=EVID/'visualization';out.mkdir(exist_ok=True)
    if not frames or frames[0]['step']!=40 or frames[-1]['step']!=80:raise RuntimeError('Actual1ms and2ms endpoint snapshots required')
    vmax=max(float(f['velocity_max_m_s']) for f in frames)
    if vmax<=0:raise RuntimeError('Blank velocity field')
    pipe=pv.read(fd/'pipe.vtp')
    files={kind:out/f'benchmark_C_coarse_2ms_{suffix}.gif' for kind,suffix in [('motion','actual_motion'),('velocity','velocity')]}
    final=out/'benchmark_C_coarse_2ms_final_frame.png'
    for kind,dest in files.items():
        if dest.exists():raise RuntimeError('Existing render must be reviewed before replacing')
        plot=pv.Plotter(off_screen=True,window_size=(1200,760));plot.set_background('#f4f7fa');plot.open_gif(str(dest),fps=10)
        try:
            for frame in frames:
                step=int(frame['step']);robot=pv.read(fd/f'robot_{step:04d}.vtp');fluid=pv.read(fd/f'midplane_{step:04d}.vtp')
                if not robot.n_points or not pipe.n_points or not fluid.n_points:raise RuntimeError('Empty actual FieldData')
                plot.clear();plot.add_mesh(pipe,color='#6f879e',opacity=.13)
                plot.add_mesh(fluid,scalars='velocity_m_s',cmap='viridis',clim=[0,vmax],opacity=.55 if kind=='motion' else 1.,
                    show_scalar_bar=True,scalar_bar_args={'title':'Velocity (m/s)','fmt':'%.3g','color':'#172434','n_labels':5})
                plot.add_mesh(robot,color='#ef812b',smooth_shading=True)
                plot.add_text(f'Benchmark C | coarse free magnetic 6DOF | {step}/80\nActual Fluent FieldData | t = {frame["time_s"]*1000:.3f} ms | 100 Hz\nFixed velocity scale 0 to {vmax:.4g} m/s',font_size=14,color='#172434')
                plot.camera_position=[(.00115,0,.012),(.00115,0,0),(0,1,0)]
                plot.camera.parallel_projection=True;plot.camera.parallel_scale=.0016;plot.write_frame()
                if kind=='motion' and step==80:plot.screenshot(str(final))
        finally:plot.close()
    counts={}
    for kind,path in files.items():
        with Image.open(path) as im:counts[kind]=im.n_frames
    if any(n!=len(frames) for n in counts.values()):raise RuntimeError('GIF frame count differs from actual saved FieldData')
    atomic(EVID/'render_manifest.json',{'timestamp':stamp(),'status':'RENDERED_AWAITING_VISUAL_REVIEW',
        'frames':frames,'saved_FieldData_snapshot_count':len(frames),'GIF_frame_counts':counts,
        'motion_gif':str(files['motion']),'velocity_gif':str(files['velocity']),'final_png':str(final),
        'fixed_velocity_scale_m_s':[0,vmax],'camera_fixed':True,'actual_motion_only':True,
        'time_range_s':[frames[0]['time_s'],frames[-1]['time_s']],'PyVista_off_screen':True,
        'outputs_sha256':{p.name:sha(p) for p in [*files.values(),final]}})
    print(json.dumps({'status':'RENDERED_AWAITING_VISUAL_REVIEW','frames':len(frames)}))

if __name__=='__main__':main()
