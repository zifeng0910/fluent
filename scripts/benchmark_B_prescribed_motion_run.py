"""B2: solve 2 ms of rigid motion and export every actual robot surface."""
from __future__ import annotations

import csv
import json
import re
import time

import numpy as np
import pyvista as pv
from ansys.fluent.core.fields.field_data_interfaces import (
    ScalarFieldDataRequest, SurfaceDataType, SurfaceFieldDataRequest,
)


def surface_mesh(field_data, name):
    data = field_data.get_field_data(SurfaceFieldDataRequest(
        surfaces=[name], data_types=[SurfaceDataType.Vertices, SurfaceDataType.FacesConnectivity]
    ))[name]
    faces = np.concatenate([np.r_[len(face), face] for face in data.connectivity]).astype(np.int64)
    return pv.PolyData(np.asarray(data.vertices, dtype=float), faces)


def run(solver, context):
    assert context["b1"]["status"] == "PASS"
    root, case = context["root"], context["case_dir"]
    evidence = root / "evidence/benchmark_B_prescribed_motion.json"
    output = case / "prescribed_motion_fielddata"
    output.mkdir(exist_ok=True)
    report = {"status": "RUNNING", "stage": "configure_transient", "dt_s": 5e-5,
              "duration_s": 0.002, "time_steps": 40, "maximum_iterations_per_step": 5,
              "prescribed_velocity_m_s": [0.05, 0.0, 0.0], "six_dof": False,
              "magnetic_loads": False, "contact": False, "fielddata_directory": str(output)}

    def save():
        evidence.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    save()
    try:
        settings = solver.settings
        calculation = settings.solution.run_calculation
        report["transient_controls_before"] = calculation.transient_controls.get_state()
        report["iterate_arguments"] = calculation.dual_time_iterate.argument_names
        calculation.parameters.time_step_size = report["dt_s"]
        node = settings.setup.dynamic_mesh.dynamic_zones[context["component_dynamic_zone_name"]]
        report["component_state_initial"] = node.get_state()
        field = solver.fields.field_data
        plane_name = "benchmark_b_midplane"
        planes = settings.results.surfaces.plane_surface
        if plane_name not in planes.keys():
            planes.create(name=plane_name)
        planes[plane_name].method = "xy-plane"
        planes[plane_name].z = 0.0
        initial_robot = surface_mesh(field, "robot_wall")
        initial_overset = surface_mesh(field, "overset_component")
        initial_robot.save(output / "robot_0000.vtp")
        initial_overset.save(output / "component_0000.vtp")
        initial_pipe = surface_mesh(field, "pipe_wall")
        initial_pipe.save(output / "pipe_wall.vtp")
        initial_x = float(initial_robot.bounds[0])
        initial_yz = np.asarray(initial_robot.bounds)[2:]
        frames = []
        history_path = root / "evidence/benchmark_B_prescribed_motion_history.csv"
        report["motion_history_csv"] = str(history_path)
        report["stage"] = "time_step_loop"
        save()
        transcript_start = len(context["messages"])
        with history_path.open("w", newline="", encoding="utf-8") as stream:
            columns = ["time_s", "component_com_x_m", "component_com_y_m", "component_com_z_m",
                       "prescribed_displacement_x_m", "measured_robot_displacement_x_m",
                       "measured_component_displacement_x_m", "mesh_motion_status", "overset_connectivity_status",
                       "orphan_count", "receptor_count", "donor_count", "minimum_cell_volume_m3"]
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for step in range(41):
                if step:
                    calculation.dual_time_iterate(time_step_count=1, max_iter_per_step=5)
                solver.settings.setup.user_defined.execute_on_demand(
                    lib_name="benchmark_B_overset_statistics::libbenchmark_B")
                stats = json.loads(context["statistics_path"].read_text())
                actual_time = float(stats["time_s"])
                (output / f"connectivity_{step:04d}.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
                robot = surface_mesh(field, "robot_wall")
                component = surface_mesh(field, "overset_component")
                dx = float(robot.bounds[0] - initial_x)
                component_dx = float(component.bounds[0] - initial_overset.bounds[0])
                prescribed = 0.05 * actual_time
                orphan = sum(zone["orphan"] for zone in stats["zones"])
                missing = sum(zone["receptors_without_donors"] for zone in stats["zones"])
                minimum_volume = min(zone["min_volume_m3"] for zone in stats["zones"])
                good_motion = abs(dx - prescribed) < 1e-10 and abs(component_dx - prescribed) < 1e-10
                good_motion = good_motion and np.allclose(np.asarray(robot.bounds)[2:], initial_yz, atol=1e-10, rtol=0)
                good_connectivity = orphan == 0 and missing == 0 and minimum_volume > 0
                cg = node.motion.rigid_body_properties.cg_position.get_state()
                writer.writerow(dict(zip(columns, [actual_time, *cg, prescribed, dx, component_dx,
                    "PASS" if good_motion else "FAIL", "PASS" if good_connectivity else "FAIL", orphan,
                    sum(zone["receptor"] for zone in stats["zones"]), sum(zone["donor"] for zone in stats["zones"]), minimum_volume])))
                stream.flush()
                robot.save(output / f"robot_{step:04d}.vtp")
                fluid = surface_mesh(field, plane_name)
                values = np.asarray(field.get_field_data(ScalarFieldDataRequest(
                    field_name="velocity-magnitude", surfaces=[plane_name], node_value=True,
                    boundary_value=False))[plane_name], dtype=float)
                if len(values) != fluid.n_points or not np.isfinite(values).all():
                    raise RuntimeError("Nonfinite or inconsistent Fluent velocity FieldData")
                fluid.point_data["velocity_m_s"] = values
                fluid.save(output / f"fluid_{step:04d}.vtp")
                frames.append({"index": step, "time_s": actual_time, "robot_bounds_m": list(robot.bounds),
                               "measured_robot_dx_m": dx, "measured_component_dx_m": component_dx,
                               "component_com_m": cg, "orphan_count": orphan,
                               "velocity_max_m_s": float(values.max())})
                report["completed_time_steps"] = step
                report["latest_frame"] = frames[-1]
                save()
                if not good_motion or not good_connectivity:
                    raise RuntimeError(f"Motion/connectivity gate failed at step {step}")
                print(f"B2 step {step}/40 time={actual_time:.6g} robot_dx={dx:.6g} orphans={orphan}", flush=True)
        report["stage"] = "final_mesh_check"
        save()
        check_start = len(context["messages"])
        settings.mesh.check()
        time.sleep(0.3)
        check_text = "".join(context["messages"][check_start:])
        report["final_mesh_check_transcript"] = check_text
        if re.search(r"mesh check failed|does not belong|negative volume|invalid cell", check_text, re.I):
            raise RuntimeError("Final mesh check failed")
        settings.file.write_case(file_name=str(case / "benchmark_B_prescribed_motion_final.cas.h5"))
        settings.file.write_data(file_name=str(case / "benchmark_B_prescribed_motion_final.dat.h5"))
        report["frames"] = frames
        (root / "evidence/benchmark_B_motion_solver_transcript.txt").write_text(
            "".join(context["messages"][transcript_start:]), encoding="utf-8")
        report["status"] = "MOTION_PASS_RENDER_PENDING"
        report["stage"] = "CFD_FieldData_complete"
        save()
    except Exception as exc:
        report.update(status="BLOCKED", error=repr(exc))
        report["frames"] = locals().get("frames", [])
        save()
        raise


