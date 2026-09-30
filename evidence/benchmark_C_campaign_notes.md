# Benchmark C: frozen physics and overset characterization

Campaign: `BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF`.

Benchmark A and B remain PASS / FROZEN. The current C source is the recovered
authoritative Abaqus 100 Hz analytic external field. No physical Magpylib magnet
geometry has been invented, and the historical sphere example is not used.

## Frozen validation

The existing source and load validation are retained without rerunning them:

- `benchmark_C_source_translation_validation.json`: PASS.
- `benchmark_C_load_validation.json`: PASS, force error
  6.776263578034403e-21 N and torque error 4.235164736271502e-22 N m.
- The 44 actual callback times start at 25 us; t=0 is covered by the separate
  direct Fluent quaternion/load probe. `required_times_present["0"]=false`
  describes the callback history alone and does not describe combined coverage.
- Body mass, COM, inertia, magnetization vector, 100 Hz frequency, 1 ms smoothstep,
  6 mm/s source offset, zero gravity, and 25 us timestep remain unchanged.
- `l2300_abaqus100hz_6dof.c` is frozen. New pose statistics are a separate UDF
  with no magnetic load callback.

## Determinant cutoff

The real inertia determinant is 1.1206106483031212e-35 (kg m^2)^3.
The chosen cutoff is 1e-50. The original failed session's prechange numeric value
was not captured; it must not be presented as an experimentally recorded value.
`benchmark_C_cutoff_documentation.json` records the default observed in the
fresh recovered session and restoration of the same frozen 1e-50 value.
The default-cutoff trial completed translation with zero angular velocity and
angular-acceleration warnings; the chosen cutoff restored angular motion without
changing the inertia tensor. Sensitivity is gated on successful final 2 ms C.

## Old moving envelope

The old B component radius was 0.65 mm with bounds [-0.5, 2.8] mm.
The active-angular-motion run stopped at 1.65 ms with one orphan. Its saved
quaternion corresponds to 0.2797885736990376 rad, not merely an approximate
0.25 rad angle. Reconstructed actual robot clearance to the cylindrical CAD
pipe is 0.22715124853682445 mm; old component clearance is -0.2154691727070016
mm. These two distances are distinct from the COM radial displacement.
The saved trajectory's maximum quaternion norm error is 1.1102230246251565e-16.

## Static candidate audit

Four component-only Prime meshes use radial shells 0.050, 0.0625, 0.075,
and 0.100 mm, with 0.100 mm axial extension at each end. Frozen B background
mesh is retained by replacing only the component cell zone in a C working copy.
The boundary SI checkpoint has the same original B mesh but no preexisting DCI;
default overset interface is created during initialization after replacement.
This avoids changing or rerunning Benchmark B.

The sweep rotates about initial COM in the measured failure-axis direction and
an orthogonal pitch/yaw direction, at 0:0.05:0.35 rad. There is no magnetic hook,
free 6DOF, or transient fluid advancement. Hybrid initialization does run its
auxiliary initialization scalars; these are not Navier–Stokes time steps.
Every pose gets newly established connectivity and native cell-flag statistics.
The native rotate-zone API takes radians, verified against transformed actual
surface vertices. A separate discovery probe used the wrong unit conversion;
that probe is not part of the formal sweep.

Clearance is the minimum signed distance from current robot/envelope mesh
vertices to the frozen Fluent pipe-wall triangle surface. Positive means inside
the pipe. The final postprocessor reconstructs the verified native static poses
and calibrates the sign at initial COM, including any envelope penetration.

Overlap layers use the minimum radial/end thickness divided by the largest
actual nearest tetrahedron edge from either mesh. This is a conservative
geometric estimate, not a measured donor-stencil layer count. The frozen
background's local actual tetrahedron edges are about 0.11–0.19 mm, substantially
larger than the candidate thicknesses. Direct per-donor size ratio is not exposed
by this audit and is marked NOT_EXPOSED, not fabricated.

Orphans are counted separately from receptor cells lacking donors; in Fluent's
flags, the latter can be zero while orphan count is nonzero. Donor count is the
number of flagged donor cells, not the number of receptor-donor pairs.

The final CSV/JSON determine selection. A failing static candidate cannot be
used to resume or restart free 6DOF. The free-run script requires a dedicated
`BENCHMARK_C_OVERSET_ENVELOPE_PASS` final gate and checks actual surface clearance,
allowed angle, finite state, and native quaternion norm without renormalization.

Final outcome: all 64 poses completed, `FAIL_NO_ROBUST_CANDIDATE`. All four
candidates have orphans at theta=0. Static orphan peak is 24808, missing donor
peak is 0. Minimum sampled robot clearance is 0.1396568015 mm; minimum signed
envelope clearance is -0.0276967647 mm. Conservative effective overlap estimates
range from 0.2667719787 to 0.5335439574, below the required four layers.
No positive robust angular operating range or selected envelope exists.
The final static recheck, new free run, and cutoff sensitivity are not run.

The background identity check matches all 127360 cells bijectively after native
cell renumbering. Maximum matched centroid difference is 1.7389534834e-18 m;
matched tetrahedron maximum edges are exactly unchanged. Direct row-order
comparison is inappropriate because Fluent renumbers cells on import/replacement.
The recovered session's measured default cutoff is 1e-20, restored to 1e-50.

Remaining blocker: the tested shells cannot provide the required overlap with
the frozen B background cell sizes. Increasing component radius alone also
violates pipe-wall clearance at large angles. A further mesh plan would need to
revisit the C background overlap resolution; that is outside this round's
instruction to replace only the component geometry/mesh.

## Session and rendering evidence

Exactly one solver is active at a time, using `.venv` direct PyFluent
`no_gui_or_graphics`. A blocking overset replacement confirmation required
closing the owned session and recovering a single session from frozen files.
Subsequent replacement uses the uninitialized B mesh and legacy component case
format because Fluent replace-zone rejects HDF5 input.
Prime meshing uses the already installed `.venv-prime`; it is not a second
Fluent solver session.

The existing actual FieldData/PyVista motion GIF is partial failure evidence
through 1.6 ms, explicitly watermarked. It is not a successful 2 ms C run and
must not be presented as one. No final velocity GIF or successful final frame
is produced until the static and free-run gates pass. Benchmark D remains frozen.
