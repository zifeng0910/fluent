"""Offline rendering of actual Fluent FieldData, fixed camera and common scales."""
from pathlib import Path
import json,numpy as np,pyvista as pv
ROOT=Path(__file__).resolve().parents[1];report_path=ROOT/'evidence/benchmark_B_prescribed_motion.json';report=json.loads(report_path.read_text());assert report['status']=='MOTION_PASS_RENDER_PENDING'
src=Path(report['fielddata_directory']);out=ROOT/'evidence/benchmark_B_pyvista';out.mkdir(exist_ok=True)
plot=pv.Plotter(off_screen=True,window_size=(1100,650));plot.set_background('#f4f7fa');plot.open_gif(str(out/'benchmark_B_actual_robot_motion.gif'),fps=12)
for row in report['frames']:
 i=row['index'];plot.clear();robot=pv.read(src/f'robot_{i:04d}.vtp');fluid=pv.read(src/f'fluid_{i:04d}.vtp');pipe=pv.read(src/'pipe_wall.vtp')
 plot.add_mesh(pipe,style='wireframe',color='#8b99a8',opacity=.18)
 plot.add_mesh(fluid,scalars='velocity_m_s',cmap='viridis',clim=[0,2.5],show_scalar_bar=True,scalar_bar_args={'title':'Velocity (m/s)'},opacity=.45)
 plot.add_mesh(robot,color='#ef812b',smooth_shading=True)
 plot.add_text(f"Benchmark B | actual Fluent surface\nt = {row['time_s']*1000:.2f} ms   dx = {row['measured_robot_dx_m']*1e6:.1f} um\nOrphans: {row['orphan_count']} | prescribed +X, no rotation",font_size=14,color='#172434')
 plot.camera_position=[(.0013,0,.012),(.0013,0,0),(0,1,0)];plot.camera.parallel_projection=True;plot.camera.parallel_scale=.0018
 plot.write_frame()
 if i==40:plot.screenshot(str(out/'benchmark_B_final.png'))
plot.close()
report.update(status='PASS',stage='OFFSCREEN_RENDER_VERIFIED',gif=str(out/'benchmark_B_actual_robot_motion.gif'),preview=str(out/'benchmark_B_final.png'),rendering='PyVista offscreen; 41 actual FieldData frames; fixed camera and velocity scale',maximum_orphan_count=max(r['orphan_count'] for r in report['frames']),final_robot_displacement_m=report['frames'][-1]['measured_robot_dx_m'])
report_path.write_text(json.dumps(report,indent=2));print(json.dumps({k:report[k] for k in ['status','gif','final_robot_displacement_m','maximum_orphan_count']}))
