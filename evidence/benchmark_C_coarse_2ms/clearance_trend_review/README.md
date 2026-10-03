# Benchmark C coarse2ms: clearance trend review

**Campaign result: INCOMPLETE — HARD_BLOCKER_REQUIRES_REVIEW.**
Actual continuation ended at step73/80,1.825 ms. This is an offline extension of the verified stop audit, with zero dynamics advanced and no Fluent launch/connection. Original1ms evidence, current configuration, native history and worker inputs remain unchanged.

## Clearance milestones

|Native time|Step|Physical clearance|
|---|---:|---:|
|1.000ms|40|0.449634530 mm|
|1.250ms|50|0.390594199 mm|
|1.500ms|60|0.289985853 mm|
|1.750ms|70|0.140512064 mm|
|2.000ms|80|NOT REACHED|
|1.825ms|73|0.086598436 mm (hard stop)|

First saved sample below0.250mm: step64/1.600ms. Below0.150mm: step70/1.750ms. Below0.100mm: step73/1.825ms. These are retrospective classifications, not new runtime events. Threshold crossing occurred between the adjacent saved25us samples; no exact continuous crossing time is claimed.

After1ms, all33 saved step-to-step clearance changes decrease. Across the entire saved history there are7 tiny early increases at steps3–9 (maximum4.96nm), followed by decrease; the whole history is not strictly monotonic.

## Geometry of gap reduction

At step73: body+X tilt=0.413919951rad (23.715866deg); quaternion total rotation=0.414252279rad. They are distinct metrics. Both first exceed0.35rad at step70 while actual connectivity and clearance remain valid. Static angle exceedance alone does not stop the run.

At the native COM, applying the native orientation instead of the reference orientation reduces clearance by 0.401783597mm. At that native orientation, applying lateralCOM migration raises clearance by 1.186903um. Rotation/tilt dominates this geometric comparison. The signed interaction and symmetric marginal accounting are recorded separately in the geometric-contribution CSV; they do not establish magnetic or hydrodynamic dynamical causality.

## Dynamics by interval

|Metric|0–1ms|1–1.825ms (includes stopped sample)|
|---|---:|---:|
|Axial COM displacement|-0.445512um|-2.080298um|
|Lateral net COM displacement|0.067713um|1.120706um|
|Mean axial velocity|-0.000445512m/s|-0.002521573m/s|
|Peak linear speed|0.001512238m/s|0.004695341m/s|
|Orientation change|0.037757509rad|0.377113164rad|
|Peak angular speed|157.311008rad/s|895.154138rad/s|
|Minimum clearance|0.449634530mm|0.086598436mm|
|Mean / peak magnetic force magnitude|13.971343 / 27.930356uN|27.385273 / 27.930356uN|
|Mean / peak magnetic torque magnitude|0.688156 / 2.068356uN*m|4.009868 / 6.760939uN*m|

Peak angular speed895.154138rad/s, maximum body-axis tilt0.413919951rad, maximum lateralCOM displacement1.187236916um, minimum clearance0.086598436mm all occur at step73/1.825ms. There is no complete1–2ms interval and no verified causal attribution to the ramp alone.

## Numerical, checkpoint and resource outcome

- Orphan/invalid donor peak:0/0. Minimum cell volume:6.100596812e-17m3. Max donor length ratio:3.099239817 (warning-only).
- Native checkpoints50/60/70: verified. Native2ms checkpoint80: NOT REACHED. Latest saved native checkpoint73 is intact but physically below the gap gate; last all-gate-valid native checkpoint is70, and the last all-gate-valid history/FieldData sample is72.
- This continuation attempt: peak WS8.727478GiB, peak Private Bytes15.909618GiB, peak system commit85.052719%. No resource abort.
- Original accepted1ms20h window: peak WS8.984581GiB, window peak commit88.319458%. Resource windows are kept separate.
- Fine comparison: FINE_REFERENCE_NOT_AVAILABLE_AT_2MS; only the retained exact0.825ms comparison exists. No mesh-convergence claim.
- Motion GIF /velocity GIF /full2ms visual review: NOT READY. The diagnostic trend PNG is not a full2ms CFD visualization. Seventeen actual continuation snapshots40..72 and the original0..1ms FieldData remain preserved.
- Worker/owned solver are closed; scheduler tasks remain disabled and heartbeat paused. No unchanged retry from70 or73 can cure the verified physical gap failure under the frozen scope. A changed physical configuration or clearance policy requires a new explicit decision.

## Files

- `clearance_time_series.csv`:73 actual rows, retrospectively evaluated warnings, COM/orientation/omega and tracked numerical gates.
- `clearance_geometric_contributions.csv`:fixed-pose/COM gap comparisons, signed interaction and accounting.
- `clearance_trend_report.json`:complete derived metrics, extrema times, milestone and source hashes.
- `clearance_trend.png` /`clearance_trend.svg`:diagnostic four-panel figure, ending at the actual stop.
