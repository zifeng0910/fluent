"""Render exported Benchmark C Fluent surface FieldData with PyVista off-screen."""
from __future__ import annotations

import json
from pathlib import Path

import pyvista as pv

ROOT = Path(__file__).resolve().parents[1]
report_path = ROOT / "evidence/benchmark_C_fineA_free_6dof.json"
report = json.loads(report_path.read_text(encoding="utf-8"))
simulation_status = report.get("status")
if simulation_status not in {"FREE_6DOF_SOLVED", "FAIL"}:
    raise RuntimeError(f"Benchmark C free motion is not complete: {simulation_status}")
source = Path(report["fielddata_directory"])
out = ROOT / "evidence/benchmark_C_fineA_pyvista"
out.mkdir(exist_ok=True)
gif = out / "benchmark_C_actual_magnetic_motion.gif"
preview = out / "benchmark_C_final_frame.png"
plot = pv.Plotter(off_screen=True, window_size=(1200, 760))
plot.set_background("#f4f7fa")
plot.open_gif(str(gif), fps=10)
for frame in report["frames"]:
    step = int(frame["step"])
    robot = pv.read(source / f"robot_{step:04d}.vtp")
    fluid = pv.read(source / f"midplane_{step:04d}.vtp")
    component = pv.read(source / f"component_{step:04d}.vtp")
    plot.clear()
    plot.add_mesh(fluid, scalars="velocity_m_s", cmap="viridis", clim=[0.0, 2.5],
                  show_scalar_bar=True, scalar_bar_args={"title": "Velocity (m/s)"}, opacity=0.5)
    plot.add_mesh(component, style="wireframe", color="#586b80", opacity=0.32, line_width=1)
    plot.add_mesh(robot, color="#ef812b", smooth_shading=True, opacity=1.0)
    plot.add_text(
        f"Benchmark C | direct Abaqus 100 Hz analytic field\n"
        f"t = {frame['time_s']*1000:.3f} ms   COM radial = {frame['radial_displacement_m']*1e6:.3f} µm\n"
        f"Orphans = {frame['orphan_count']}   max midplane speed = {frame['velocity_max_m_s']:.4g} m/s"
        + ("\nPARTIAL: C_FINE_A RUN STOPPED AT A GATE" if simulation_status == "FAIL" else ""),
        font_size=13, color="#172434")
    plot.camera_position = [(0.00115, 0, 0.012), (0.00115, 0, 0), (0, 1, 0)]
    plot.camera.parallel_projection = True
    plot.camera.parallel_scale = 0.00155
    plot.write_frame()
    if step == int(report["frames"][-1]["step"]):
        plot.screenshot(str(preview))
plot.close()
velocity_gif=out/'benchmark_C_velocity.gif'
plot=pv.Plotter(off_screen=True,window_size=(1200,760));plot.set_background('#f4f7fa');plot.open_gif(str(velocity_gif),fps=10)
vmax=max(float(f['velocity_max_m_s']) for f in report['frames'])
for frame in report['frames']:
    step=int(frame['step']);fluid=pv.read(source/f'midplane_{step:04d}.vtp');robot=pv.read(source/f'robot_{step:04d}.vtp')
    plot.clear();plot.add_mesh(fluid,scalars='velocity_m_s',cmap='viridis',clim=[0,max(vmax,1e-12)],scalar_bar_args={'title':'Velocity (m/s)'})
    plot.add_mesh(robot,color='#ef812b',smooth_shading=True)
    plot.add_text(f"C_FINE_A | actual Fluent velocity FieldData | t={frame['time_s']*1000:.3f} ms\nOrphans={frame['orphan_count']}"+('\nPARTIAL: RUN STOPPED AT A GATE' if simulation_status=='FAIL' else ''),font_size=13,color='#172434')
    plot.camera_position=[(0,0,.02),(0,0,0),(0,1,0)];plot.camera.parallel_projection=True;plot.camera.parallel_scale=.0054;plot.write_frame()
plot.close()
report.update(
    simulation_status=simulation_status, rendering_status="PASS",
    status="PASS" if simulation_status == "FREE_6DOF_SOLVED" else "FAIL",
    stage="OFFSCREEN_RENDER_VERIFIED" if simulation_status == "FREE_6DOF_SOLVED" else "PARTIAL_OFFSCREEN_RENDER_VERIFIED",
    rendering="PyVista off-screen; actual Fluent robot_wall, overset_component, and midplane velocity FieldData",
    gif=str(gif),velocity_gif=str(velocity_gif), preview=str(preview), rendered_frame_count=len(report["frames"]),
)
report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps({k: report[k] for k in ["status", "simulation_status", "rendering_status", "gif", "preview", "rendered_frame_count"]}, indent=2))
