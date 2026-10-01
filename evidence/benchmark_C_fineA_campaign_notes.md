# Benchmark C — C_FINE_A local overlap refinement

Campaign: `BENCHMARK_C_ANALYTIC_MAGNETIC_6DOF`.

## Frozen physics

The existing analytic external field, 100 Hz trajectory, 1 ms startup ramp,
6 mm/s source offset, current L2300 magnetic moment vector, mass, COM,
inertia, main magnetic UDF, gravity zero, 25 microsecond time step, and
6DOF determinant cutoff `1e-50` remain the dynamic configuration.
No source/load validation is rerun, and no polarity or main UDF edit is made.
Static connectivity checks do not advance physical time or execute magnetic callbacks.

## Geometry and local sizing

- C_FINE_A normal offset: 0.050 mm; axial extension: 0.050 mm each end.
- Outer surface follows the frozen degree-five head profile and cylinder,
  with a rounded tail offset. The unchanged authoritative STEP is subtracted.
- The initial two-dimensional profile face left by the revolve operation was
  removed before exporting the final closed shell.
- Component nominal surface/volume size controls: 0.010–0.0125 mm.
- Background BOI nominal limit: 0.020 mm. The shared volumetric sizing field
  also propagates fine component controls into nearby background cells.
  Actual donor ratios and ray crossings, rather than nominal size alone,
  determine acceptance.
- Background coarse curvature controls: 0.060–0.120 mm; growth rate 1.2.
- Conservative swept box clipped to the pipe spans x=-0.3557 to 2.6625 mm,
  with 0.125 mm buffer and 0.100 mm translation allowance.
- Background: 127,360 old cells to 10,011,216 new tetrahedra.
- Component: 1,353,606 tetrahedra. This is a C-only mesh; A/B files stay frozen.

## Authoritative metrics

Standard Fluent ASCII export, no selected surfaces and cell-center location,
exports all cells. The primary orphan count is the number of rows with
`overset-cell-type == -1`. The old missing-donor-only gate is deprecated.
`overset-donor-size-ratio` is a volume ratio; its cube root is reported as
the three-dimensional characteristic length ratio.
Donor and receptor auxiliary counts come from `SV_OVERSET_NDONOR` and
`SV_OVERSET_NRECEPTOR` through PyFluent solution variable data.

An exploratory native diagnostic helper caused a solver connection loss.
It is disabled and is not used as a gate. The owned dead solver was replaced
with one direct no_gui_or_graphics session; the crash record is retained.

## Theta zero

Official orphan count: 0. Receptors: 202,918. Donors: 633,244.
All 202,918 receptors match cells with positive native donor count.
Length ratio median/p95/max: 1.0323 / 1.1889 / 1.7093; no ratio exceeds 3.
Minimum cell volume: 6.100596812e-17 m³; mesh check completes.
120 wall-normal audit rays cross at least 12 actual tetrahedra in each mesh.
Component continuous mesh spans are at least 0.0490018 mm; background spans
at least 0.0499 mm. Small endpoint CAD/tessellation differences are recorded;
no interior sample gaps are present. Counts are sampled actual intersections,
not a nominal thickness/spacing division or proof for every surface location.

## Remaining gates

Short static pose sweep is in progress. No dynamic PASS or new GIF is claimed.
The new free-run and rendering scripts use independent C_FINE_A output paths.
The old GIF is labeled `FAILED_PARTIAL_OLD_OVERSET` and its bytes are preserved.

## Static sweep completed

All 10 prescribed poses PASS. Peak official orphan count is 0; minimum sampled
background crossings are 7; minimum physical clearance is 0.1403270937 mm.
Maximum donor characteristic length ratio is 2.7681409544.
The accepted tested angular bound is 0.35 rad on each specified axis.

## Dynamic runtime

The one-core preliminary run completed step 1 with zero official orphans,
valid donors, positive volumes, and quaternion norm error 1.1e-16.
Its solver estimate was several minutes per step. That attempt and history
are archived as `SERIAL_PRELIMINARY_INTERRUPTED_FOR_4CORE_RESTART`.
The owned solver was closed and replaced with one four-core solver in the
same venv and no_gui_or_graphics mode. The final run restarts at t=0 from
the accepted static checkpoint; mesh and physics are unchanged.

The first four-core attempt completed four steps and one velocity FieldData
frame, then produced no new solver log for more than 18 minutes after that
export. CPU counters still advanced. The cause is not confirmed; it is
archived as `NO_PROGRESS_AFTER_FIRST_VELOCITY_FIELDDATA_EXPORT`.
The retry defers velocity-plane FieldData until solving ends. Surface
clearance checks and official cell statistics remain per-step. Native
case/data checkpoints are saved every four steps for one serial headless
postprocessing session, followed by PyVista offscreen rendering.
