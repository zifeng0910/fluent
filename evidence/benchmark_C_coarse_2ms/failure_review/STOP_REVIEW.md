# Coarse 2 ms continuation: clearance stop

Campaign result: **INCOMPLETE**, reviewed terminal status
**HARD_BLOCKER_REQUIRES_REVIEW**. The accepted 1 ms milestone remains frozen.

The single restart at step40 passed native time, COM, quaternion, velocity,
absolute-time magnetic source and overset continuity checks. Steps41..72
continued with valid numerical gates. Step73 was solved and saved at
**1.825 ms**, but its physical clearance failed the required0.10 mm gate.
The last state satisfying all gates is step72 at1.800 ms, clearance0.104959 mm.

## Exact failure review

- Actual step73 robot-wall clearance: **0.086598436 mm**.
- Independent closest-point calculation on actual saved robot/pipe FieldData
  reproduces the recorded gap. Native COM/quaternion reproduce the wall points
  within2.276e-10 m.
- Rotation magnitude0.414252 rad exceeds the static0.35 rad envelope; envelope
  exceedance remains warning-only. Actual clearance is the stopping condition.
- Orphan0, invalid donor0, unidentified0; minimum volume6.100596812e-17 m³.
- Failure-step maximum donor length ratio3.005003 is warning-only.
- Resource abort: none. Measured continuation peak WS8.72748 GiB,
  Private Bytes15.90962 GiB, system commit40.68905 GiB /85.05272%.
- Native step73 STOP CASE+DATA hashes, sizes and HDF5 integrity pass.
  Native HDF5 contains time1.825 ms and step73. This review did not launch,
  connect or cold-reload a solver and advanced zero dynamics.
- Checkpoints50/60/70 pass offline integrity review; step80 does not exist.
- Owned engines and worker are closed. All Fluent campaign tasks are disabled;
  the existing Codex heartbeat is paused. The original deadline is unchanged.

## Dynamics up to the stop

| Interval | Mean axial velocity | Orientation change | Peak angular speed | Mean torque magnitude |
|---|---:|---:|---:|---:|
|0–1 ms|−0.000445512 m/s|0.0377575 rad|157.311 rad/s|6.88156e-7 N·m|
|1–1.825 ms|−0.00252157 m/s|0.377113 rad|895.154 rad/s|4.00987e-6 N·m|

These interval observations do not establish causality from the ramp alone.
No1–2 ms result or full2 ms PASS is claimed. Fine data end at the exact matched
0.825 ms reference; no interpolated2 ms or mesh-convergence claim is made.
Seventeen actual FieldData snapshots40..72 are preserved. The requested final
2 ms visualizations are **NOT_READY**, because the numerical2 ms gate failed.

## Decision

Preserve native step73 and the last valid history at72. Do not retry unchanged,
revert to40, lower the clearance gate, remesh or introduce contact. A subsequent
continuation requires an explicit new decision authorizing any changed physical
configuration or clearance policy. No such change is authorized by this review.

Full metrics: `../final_report.json`.
Diagnosis and evidence hashes: `clearance_audit.json`.
Reviewed decision: `../repair_plan.json`.
Original failure/state evidence: `archive_0073/`.
