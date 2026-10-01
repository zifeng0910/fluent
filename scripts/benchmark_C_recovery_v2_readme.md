# Benchmark C Recovery V2

Start with `powershell -NoProfile -ExecutionPolicy Bypass -File H:\fluent\scripts\run_benchmark_C_recovery_v2.ps1`.
Task Scheduler owns the hidden PowerShell wrapper, Python supervisor, worker,
and one-core `no_gui_or_graphics` Fluent session. No recurring model invocation
is involved. The old and V2 supervisors use the same exclusive Windows lock.

The first start persists an 8-hour target and 10-hour hard deadline in
`evidence/benchmark_C_recovery_v2/state.json`. Relaunching does not extend it.
This operational window never changes native CFD or magnetic time.

The FSM performs one zero-timestep native checkpoint-load memory probe,
including CASE, DATA, native overset reconstruction, and unchanged UDF loading.
It samples owned working sets, physical RAM, system commit, and English PDH
paging counters. It aborts only captured PID/creation-time identities if RAM
falls below 3 GiB, commit reaches 95%, or low-memory heavy paging persists.
Reads during CASE load alone are not treated as pagefile thrashing.

Production admission uses max(load footprint, recovered historical solve
resident sample) plus max(20%, 3 GiB). Historical samples are not a complete
peak curve; the running watchdog remains required. A failed probe or failed
production admission produces a concrete RESOURCE_BLOCKER instead of waiting
for a model. No pagefile resizing or unknown-process cleanup occurs.

Before solving, native persisted zone origin/orientation/velocity/omega and
CURRENT_TIME must agree with archived checkpoint values. Native serialized
orientation is converted using the Q_From_Theta equivalent rotation-vector
mapping, verified against archived DT_Q and actual surface coordinates. The
validator does not normalize or change Fluent state. Direct DT_Q access is
not claimed. Source F/T uses the already validated independent Python law.
Official overset fields, native donor counts, positive volumes, and actual
pipe clearance are checked without a timestep.

Step33 is recomputed and compared before step34. Main checkpoints are distinct,
hashed, and include full pose and velocities, including 1/1.25/1.5/1.75/2 ms.
An unexplained worker lifecycle loss with valid numerical state permits at most
two local recoveries; numerical/restart failures become HARD_BLOCKER.

Only the complete 80-step main run enters cutoff comparison and actual saved
FieldData export, then PyVista off-screen rendering. Numerical/file completion
does not claim visual inspection PASS: actual PNG/GIF review remains a later
human/LLM task, independent of solver lifetime. No Benchmark D or contact.

## Evidence

`old_heartbeat_task_audit.json`: app heartbeat identification and paused status.
`memory_audit_v2.json/.csv`: local process audit (do not publish full command lines).
`memory_samples.jsonl`: probe and runtime samples.
`benchmark_C_memory_probe.json`: measured probe outcome and guard calculation.
`production_admission.json`: loaded-session admission decision.
`restart_v2_step*_state_validation.json`: zero-timestep native state gates.
`checkpoints/*.json`: complete native pairs and full state.
`detached_process_test.json`: Scheduler ancestry and launcher shell survival.
`state.json`, `events.jsonl`, `worker.json`, `final.json`: durable FSM evidence.
