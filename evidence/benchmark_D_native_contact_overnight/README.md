# Benchmark D native contact overnight

Campaign: `BENCHMARK_D_NATIVE_CONTACT_OVERNIGHT`.

The local Windows task `Fluent-Benchmark-D-Native-Contact-Overnight` owns the
supervisor and all future launches. Codex/SSE and the review heartbeat do not own
solver survival. Start this task through
`scripts/run_benchmark_D_native_contact_overnight.ps1` only after confirming no
existing supervisor owns the campaign. It preserves the original deadline.

Immutable window, China time on 2026-10-05:

- Start: 09:41:18.
- Target: 19:41:18.
- Hard deadline: 21:41:18; only safe checkpoint completion may continue after it.

`activation_audit.json` records actual local task, exact supervisor/worker/native
process identities, a zero-step C70 restart, a real UDF compile, and a later
verified native checkpoint. This is a launch milestone. It is not micro-run,
timestep adequacy, 2 ms, or visual PASS.

Read `supervisor_state.json` and the active branch's `worker_state.json` for live
ownership/progress. Root `state.json` records the FSM stage; fields retained from
an earlier admission record are not an authoritative live process inventory.
Routine progress and resource samples stay local.

Sequence: C70 (1.750 ms) -> 25 us micro-run to 1.900 ms -> 12.5 us comparison ->
6.25 us only when measured differences require it -> largest adequate dt ->
selected D checkpoint to 2.000 ms -> numerical review -> render -> actual visual
review. Frozen physics, mesh, original magnetic law, and prior C/overwrite
evidence are preserved. Every valid completed boundary has native CASE/DATA and
hash/HDF5/state metadata. Large native and FieldData files remain local.

The signed gap uses the actual triangulated robot surface and ideal cylinder of
radius 0.9 mm; negative gap is penetration. Tolerance is 5 um. The old positive
0.100 mm C clearance hard stop is not used for D.

`control_repairs/0001_journal_collision` preserves the observed controller
failure. Its reviewed fix adopted the same surviving worker, preserving all
active inputs and the original window. It did not launch another Fluent.

Conditional LOAD fallback preparation is not a measured contact PASS. A switch
requires archived native contact-route failure and reviewed capability hashes;
actual Fluent compile, controlled load response, dt adequacy, full numerical
gates and visual review remain required. Unrelated resource/lifecycle failures
do not demonstrate contact-model instability.

Review heartbeat: `benchmark-d-native-contact-overnight-review`; quiet during
unchanged or normal progress, actual PNG/GIF review required before COMPLETE.
No work beyond 2.000 ms is authorized.
