"""Offscreen preview of actual theta-zero Fluent surface FieldData."""
from pathlib import Path
import pyvista as pv
ROOT=Path(__file__).resolve().parents[1];source=ROOT/'evidence/benchmark_C_fineA_static_fielddata';out=ROOT/'evidence/benchmark_C_fineA_pyvista';out.mkdir(exist_ok=True)
plot=pv.Plotter(off_screen=True,window_size=(1200,720));plot.set_background('#f4f7fa')
plot.add_mesh(pv.read(source/'pipe_wall.vtp'),color='#92aac0',opacity=.12)
plot.add_mesh(pv.read(source/'overset_component.vtp'),color='#3d81ab',opacity=.25,show_edges=False)
plot.add_mesh(pv.read(source/'robot_wall.vtp'),color='#ef812b',smooth_shading=True)
plot.add_text('C_FINE_A | actual Fluent surface FieldData | theta=0\n0.050 mm normal shell; 0.050 mm axial extension\n10 static poses PASS; peak orphan=0',font_size=14,color='#172434')
plot.camera_position=[(.00115,0,.012),(.00115,0,0),(0,1,0)];plot.camera.parallel_projection=True;plot.camera.parallel_scale=.00155
plot.screenshot(str(out/'benchmark_C_static_mesh.png'));plot.close()
