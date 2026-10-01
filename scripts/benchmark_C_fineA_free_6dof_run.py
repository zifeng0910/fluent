"""Run C's 2 ms analytic magnetic 6DOF case in the existing PyFluent session."""
from __future__ import annotations

import csv
import json
import re

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
    root = context['root']
    case=root/'live_cases/benchmark_C_fineA/bg020'
    import sys
    sys.path.insert(0,str(root/'scripts'))
    from benchmark_C_fineA_theta0 import summarize_csv
    run_config=context.get('fineA_run_config',{})
    step_count=int(run_config.get('steps',80));cutoff=float(run_config.get('cutoff',1e-50))
    run_tag=run_config.get('tag','')
    defer_velocity=True
    gate_path = root / 'evidence/benchmark_C_fineA_final_static_gate.json'
    if not gate_path.exists():
        raise RuntimeError('Free 6DOF requires BENCHMARK_C_OVERSET_RESOLUTION_PASS final static audit')
    gate = json.loads(gate_path.read_text())
    if gate.get('status') != 'BENCHMARK_C_OVERSET_RESOLUTION_PASS':
        raise RuntimeError('Final C component static gate has not passed')
    out_json = root / f"evidence/benchmark_C_fineA_free_6dof{run_tag}.json"
    field_dir = root / f"evidence/benchmark_C_fineA_deferred_fielddata{run_tag}"
    field_dir.mkdir(exist_ok=True)
    history_path = root / f"evidence/benchmark_C_fineA_free_6dof_history{run_tag}.csv"
    report = {
        "status": "RUNNING", "stage": "reset_from_B_static_checkpoint",
        "campaign": "BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF",
        "dt_s": 25e-6, "duration_s": step_count*25e-6, "requested_steps": step_count,
        'run_purpose':'CUTOFF_SENSITIVITY' if run_tag else 'FULL_2MS_FREE_6DOF',
        'velocity_fielddata_export':'DEFERRED_UNTIL_SOLVING_FINISHES',
        "maximum_iterations_per_step": 2, "gravity_enabled": False,
        "six_dof_gravity_m_s2": [0, 0, 0], "contact": False,
        "physical_wall_clearance_limit_m": 0.0001,
        "fielddata_directory": str(field_dir),
    }

    def save():
        out_json.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    save()
    try:
        settings = solver.settings
        settings.file.read_case(file_name=gate['start_case'])
        settings.file.read_data(file_name=gate['start_data'])
        solver.tui.define.overset_interfaces.adapt.set.automatic('no')
        solver.scheme.eval(f"(rpsetvar 'dynamesh/sdof/minimum-cutoff-moments {cutoff:.17g})")
        inertia_cutoff = solver.scheme.eval("(rpgetvar 'dynamesh/sdof/minimum-cutoff-moments)")
        if float(inertia_cutoff) > 1e-50:
            raise RuntimeError(f"Fluent inertia determinant cutoff did not persist: {inertia_cutoff}")
        user = settings.setup.user_defined
        user.load(udf_library_name=str(root/'fluent_udf/libbenchmark_C_v4'))
        dyn = settings.setup.dynamic_mesh
        dyn.enabled = True
        dyn.methods.smoothing.enabled = False
        dyn.options.six_dof.enabled = True
        dyn.options.six_dof.gravity.set_state({"x": 0.0, "y": 0.0, "z": 0.0})
        settings.setup.general.operating_conditions.gravity.enable = False
        settings.setup.general.solver.time = "transient"
        zones = {}
        for zone in ["robot_component_fluid", "robot_wall"]:
            dyn.dynamic_zones.create(zone=zone)
            name = next(n for n in dyn.dynamic_zones.keys()
                        if dyn.dynamic_zones[n].zone.get_state() == zone)
            zones[zone] = name
            node = dyn.dynamic_zones[name]
            node.type = "rigid-body"
            node.motion.six_dof.enabled = True
            node.motion.six_dof.passive = (zone == "robot_component_fluid")
            node.motion.rigid_body_properties.cg_position = [0.0012060186937156343, 0.0, 0.0]
            node.motion.rigid_body_properties.orientation.set_state({"angle": 0.0, "axis": [1.0, 0.0, 0.0]})
            node.motion.motion_def = "l2300_magnetic_6dof::libbenchmark_C_v4"
        context["c_component_dynamic_zone"] = zones["robot_component_fluid"]
        context["c_wall_dynamic_zone"] = zones["robot_wall"]
        calc = settings.solution.run_calculation
        calc.parameters.time_step_size = 25e-6
        calc.parameters.max_iter_per_time_step = 2
        report["zone_states_initial"] = {z: dyn.dynamic_zones[n].get_state() for z, n in zones.items()}
        report["gravity_flow"] = settings.setup.general.operating_conditions.gravity.get_state()
        report["gravity_sixdof"] = dyn.options.six_dof.gravity.get_state()
        report["udf_library"] = "libbenchmark_C_v4"
        report['automatic_overset_adaption']=False
        report["inertia_determinant_cutoff"] = str(inertia_cutoff)
        report["body_inertia_tensor_kg_m2"] = {
            "Ixx": 7.257810693523445e-13, "Iyy": 3.929384741151496e-12,
            "Izz": 3.92938474115149e-12, "Ixy": -2.782693607479792e-28,
            "Ixz": 1.9327591150313555e-29, "Iyz": 2.4813784672159598e-29,
        }
        report["load_validation"] = json.loads((root / "evidence/benchmark_C_load_validation.json").read_text())

        field = solver.fields.field_data
        plane_name = "benchmark_c_midplane"
        planes = settings.results.surfaces.plane_surface
        if plane_name not in planes.keys():
            planes.create(name=plane_name)
        planes[plane_name].method = "xy-plane"
        planes[plane_name].z = 0.0
        robot0 = surface_mesh(field, "robot_wall")
        component0 = surface_mesh(field, "overset_component")
        pipe_wall = surface_mesh(field, 'pipe_wall')
        pipe_inside_sign = float(np.sign(pv.PolyData(np.array([[.0012060186937156343,0,0]])).compute_implicit_distance(pipe_wall)['implicit_distance'][0]))
        robot0.save(field_dir / "robot_0000.vtp")
        component0.save(field_dir / "component_0000.vtp")
        robot_x0 = float(robot0.bounds[0])
        component_bounds0 = np.asarray(component0.bounds, dtype=float)
        output_steps = set(range(4, step_count+1, 4)) | {step_count}
        frames = []
        transcript_start = len(context["messages"])
        stats_path = root / "evidence/benchmark_C_overset_live.json"
        report["stage"] = "free_6dof_time_step_loop"
        report["fielddata_export_interval_steps"] = 4
        report["motion_history_csv"] = str(history_path)
        save()
        with history_path.open("w", newline="", encoding="utf-8") as stream:
            cols = ["step", "time_s", "com_x_m", "com_y_m", "com_z_m", "radial_displacement_m",
                    "robot_wall_dx_m", "component_bounds_delta_m", "orphan_count",
                    "receptors_without_donors", "minimum_cell_volume_m3", "mesh_motion_status",
                    'q0','q1','q2','q3','q_norm','robot_wall_clearance_m','overset_wall_clearance_m']
            native_columns = [f'{prefix}_{axis}_{unit}' for prefix,unit in
                              [('omega','rad_s'),('Fmag','N'),('Tmag','Nm')]
                              for axis in 'xyz']
            native_columns += [f'v{axis}_m_s' for axis in 'xyz']
            native_columns += [f'theta_{axis}_rad' for axis in 'xyz']
            cols += native_columns
            cols += ['donor_length_ratio_min','donor_length_ratio_median','donor_length_ratio_p95','donor_length_ratio_max','official_donor_count','official_receptor_count']
            writer = csv.DictWriter(stream, fieldnames=cols)
            writer.writeheader()
            for step in range(1, step_count+1):
                calc.dual_time_iterate(time_step_count=1, max_iter_per_step=2)
                current_fields=case/'free_current_official_cells.csv'
                settings.file.export.ascii(file_name=str(current_fields),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
                stats = summarize_csv(current_fields)
                actual_time=float(solver.scheme.eval("(rpgetvar 'flow-time)"))
                stats['time_s']=actual_time
                donor_data=solver.fields.solution_variable_data.get_data(variable_name='SV_OVERSET_NDONOR',zone_names=['background_water','robot_component_fluid'])
                positive_donors=sum(int(np.count_nonzero(donor_data[z]>0)) for z in ['background_water','robot_component_fluid'])
                stats['receptors_with_valid_donors']=positive_donors
                stats['native_donor_count_statistics']={}
                for z in ['background_water','robot_component_fluid']:
                    values=donor_data[z];positive=values[values>0]
                    stats['native_donor_count_statistics'][z]={'positive_cells':int(len(positive)),'min_positive':int(positive.min()) if len(positive) else None,'median_positive':float(np.median(positive)) if len(positive) else None,'max':int(values.max())}
                (field_dir / f"connectivity_{step:04d}.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
                orphan = stats['orphans']
                missing = max(0,stats['receptors']-positive_donors)
                min_volume = stats['minimum_volume_m3']
                node = dyn.dynamic_zones[zones["robot_wall"]]
                state = node.motion.rigid_body_properties.get_state()
                com = state["cg_position"]
                radial = float(np.hypot(com[1], com[2]))
                actual_time = float(stats["time_s"])
                native_rows = list(csv.DictReader((root/'evidence/benchmark_C_analytic_6dof_history.csv').open()))
                native = next(r for r in reversed(native_rows) if r['mode']=='FREE_6DOF' and abs(float(r['time_s'])-actual_time)<1e-10)
                quaternion = np.array([float(native[f'q{i}']) for i in range(4)])
                q_norm = float(np.linalg.norm(quaternion))
                mesh_status = "PASS" if orphan == 0 and missing == 0 and min_volume > 0 else "FAIL"
                robot = surface_mesh(field, 'robot_wall')
                component = surface_mesh(field, 'overset_component')
                robot_clearance = float(np.min(robot.compute_implicit_distance(pipe_wall)['implicit_distance']*pipe_inside_sign))
                envelope_clearance = float(np.min(component.compute_implicit_distance(pipe_wall)['implicit_distance']*pipe_inside_sign))
                robot_dx = component_delta = None
                if step in output_steps:
                    robot = surface_mesh(field, "robot_wall")
                    component = surface_mesh(field, "overset_component")
                    robot_dx = float(robot.bounds[0] - robot_x0)
                    component_delta = float(np.max(np.abs(np.asarray(component.bounds) - component_bounds0)))
                    robot.save(field_dir / f"robot_{step:04d}.vtp")
                    component.save(field_dir / f"component_{step:04d}.vtp")
                    checkpoint_case=case/f'fielddata_checkpoint{run_tag}_{step:04d}.cas.h5'
                    checkpoint_data=case/f'fielddata_checkpoint{run_tag}_{step:04d}.dat.h5'
                    if not run_tag:
                        settings.file.write_case(file_name=str(checkpoint_case))
                        settings.file.write_data(file_name=str(checkpoint_data))
                    frames.append({"step": step, "time_s": actual_time, "robot_bounds_m": list(robot.bounds),
                                   "robot_wall_dx_m": robot_dx, "component_bounds_delta_m": component_delta,
                                   "com_m": list(com), "radial_displacement_m": radial,
                                   "orphan_count": orphan, "velocity_max_m_s": None,
                                   'checkpoint_case':str(checkpoint_case),'checkpoint_data':str(checkpoint_data)})
                writer.writerow({"step": step, "time_s": actual_time, "com_x_m": com[0], "com_y_m": com[1],
                                 "com_z_m": com[2], "radial_displacement_m": radial,
                                 "robot_wall_dx_m": robot_dx if robot_dx is not None else "",
                                 "component_bounds_delta_m": component_delta if component_delta is not None else "",
                                 "orphan_count": orphan, "receptors_without_donors": missing,
                                 "minimum_cell_volume_m3": min_volume, "mesh_motion_status": mesh_status,
                                 **{f'q{i}':quaternion[i] for i in range(4)},'q_norm':q_norm,
                                 **{key:float(native[key]) for key in native_columns},
                                 **{f'donor_length_ratio_{key}':stats['donor_characteristic_length_ratio'][key] for key in ['min','median','p95','max']},
                                 'official_donor_count':stats['donors'],'official_receptor_count':stats['receptors'],
                                 'robot_wall_clearance_m':robot_clearance,'overset_wall_clearance_m':envelope_clearance})
                stream.flush()
                report.update(completed_time_steps=step, latest_time_s=actual_time,
                              latest_com_m=list(com), latest_radial_displacement_m=radial,
                              latest_orphan_count=orphan, fielddata_frames=len(frames))
                save()
                if orphan or missing or stats['cell_type_counts']['-3'] or min_volume <= 0:
                    raise RuntimeError(f"Overset connectivity/volume gate failed at step {step}: {stats}")
                if not np.isfinite(np.r_[quaternion,com,[float(native[k]) for k in native_columns],robot.points.ravel(),component.points.ravel()]).all():
                    raise RuntimeError(f'Nonfinite state at step {step}')
                if abs(q_norm-1)>1e-6:
                    raise RuntimeError(f'Native quaternion norm drift at step {step}: {q_norm}')
                if robot_clearance < .0001 or envelope_clearance <= 0:
                    raise RuntimeError(f'Actual surface clearance gate at step {step}: robot={robot_clearance}, envelope={envelope_clearance}')
                print(f"C free 6DOF step {step}/{step_count} t={actual_time:.6g}s COM={com} radial={radial:.4g} orphan={orphan}", flush=True)

        check_start = len(context["messages"])
        settings.mesh.check()
        check_text = "".join(context["messages"][check_start:])
        if re.search(r"mesh check failed|does not belong|negative volume|invalid cell", check_text, re.I):
            raise RuntimeError("Final mesh check failed")
        report["final_mesh_check_transcript"] = check_text
        final_case = case / f"benchmark_C_analytic_6dof_final{run_tag}.cas.h5"
        settings.file.write_case(file_name=str(final_case))
        settings.file.write_data(file_name=str(case / f"benchmark_C_analytic_6dof_final{run_tag}.dat.h5"))
        (root / f"evidence/benchmark_C_fineA_free_6dof_solver_transcript{run_tag}.txt").write_text(
            "".join(context["messages"][transcript_start:]), encoding="utf-8")
        report.update(status="FREE_6DOF_SOLVED", stage="VELOCITY_FIELDDATA_EXPORT_PENDING", frames=frames,
                      solver_case=str(final_case), total_orphan_count=sum(
                          int(r["orphan_count"]) for r in csv.DictReader(history_path.open())),
                      nonzero_motion=(any(abs(f["robot_wall_dx_m"]) > 1e-12 or
                                          f["component_bounds_delta_m"] > 1e-12 for f in frames)))
        save()
    except Exception as exc:
        if 'step' in locals() and 'robot' in locals():
            robot.save(field_dir/f'robot_failure_{step:04d}.vtp')
            component.save(field_dir/f'component_failure_{step:04d}.vtp')
            failure={'step':step,'time_s':locals().get('actual_time'),'com_m':locals().get('com'),'q':locals().get('quaternion',np.array([])).tolist(),'q_norm':locals().get('q_norm'),'robot_clearance_m':locals().get('robot_clearance'),'envelope_clearance_m':locals().get('envelope_clearance'),'official_connectivity':locals().get('stats'),'current_official_cell_export':str(locals().get('current_fields','')),'robot_fielddata':str(field_dir/f'robot_failure_{step:04d}.vtp'),'component_fielddata':str(field_dir/f'component_failure_{step:04d}.vtp'),'error':repr(exc),'automatic_adaption':False,'remeshing_performed_after_failure':False}
            (root/f'evidence/benchmark_C_fineA_dynamic_failure{run_tag}.json').write_text(json.dumps(failure,indent=2,default=str))
        report.update(status="FAIL", error=repr(exc), frames=locals().get("frames", []))
        save()
        raise
