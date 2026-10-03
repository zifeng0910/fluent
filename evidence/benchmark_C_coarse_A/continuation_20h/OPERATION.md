# Authorized coarse continuation

The immutable `window.json` records the new user-authorized 20-hour budget.
It does not reset or extend the historical fine campaign. Completion at the
accepted 1 ms screening milestone ends this continuation early.

## Ownership and recovery

Windows task `Fluent-Benchmark-C-Coarse-20h` runs the local supervisor via the
existing venv's `pythonw.exe`, without a PowerShell console parent. Only the
supervisor dispatches workers. The supervisor and workers take separate
exclusive Windows file locks. Old coarse/fine tasks remain disabled.

Read `state.json`, `events.jsonl`, the coarse campaign state and archived failure
before an action. While a worker runs, monitor files only. Verify PID plus
creation time before trusting ownership. An unexpected supervisor exit requires
reading `supervisor_stderr.log` and `supervisor_failure.json`; do not blindly
restart Fluent. Starting the existing task again never creates a new deadline.
When a prior worker is still alive, the restarted supervisor refuses to launch.

The app heartbeat wakes this chat every 15 minutes, checks the files, diagnoses
new failures, and writes a specific reviewed plan when necessary. It stays quiet
during normal progress. It pauses on completion, exhausted budget, or a proven
blocker requiring review.

## Plan interface

Write `repair_plan.json` with atomic replacement. Every plan includes:

- Unique `plan_id`, matching `window_id` and `failure_timestamp` from state.
- `decision`: `DIAGNOSE`, `REPAIR`, or `HARD_BLOCKER`.
- `root_cause`, existing repository-relative `evidence_files`.
- For jobs: existing `scripts/` `job` and `job_sha256`.
- For repairs: `failure_class`, one `concrete_change`, a `config` below this
  directory and SHA-256 `configuration_sha256` of that actual configuration.
- `supporting_code_sha256` verifies dependencies reviewed with the native job.

Only the read-only commit audit and reviewed native continuation worker are
allowlisted. Adding a diagnostic capability requires reviewing the concrete
supervisor change. No repeated identical action; at most two technically
distinct repairs per failure class. Preserve evidence and latest native
checkpoint; never replay earlier dynamics or restart t=0.

## Resource and science gates

Admission: physical available >=12 GiB, commit headroom >=19 GiB, disk >=35 GiB
continuously for 60 s. The historical system commit increase was approximately
16.57 GiB; the admission reserve includes a margin. Runtime guards remain
available >=3 GiB, system commit <=95%, project working set <=15 GiB.
Private bytes, commit and working set are measured separately. Pagefile is
currently system managed, allocated 16 GiB. No OS/pagefile configuration changes
or unrelated application termination are authorized by this continuation.

The accepted 4,699,301-cell mesh, UDF/source, dt, cutoff, gravity and one-rank
launch configuration remain frozen. Native CASE+DATA restart validates time,
COM, quaternion, velocities, magnetic F/T, surface orientation, connectivity,
positive volume and clearance without advancing time before dynamics resumes.
Donor length ratio is recorded as a warning metric. Output is capped at step40,
1 ms. No automatic 2 ms/medium/fine/D campaign.

The final checkpoint gets native save-state/HDF5/hash verification. This is not
a second cold reload. Missing exact 1 ms fine reference is reported explicitly;
comparison uses the latest exactly matched valid time without interpolation.
No partial output or development screening is labelled mesh convergence.
