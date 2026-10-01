Benchmark C durable continuation (Windows)

Start/resume within the existing persisted deadline:
  powershell -NoProfile -ExecutionPolicy Bypass -File H:\fluent\scripts\run_benchmark_C_longrun.ps1

One Windows Task Scheduler task: Fluent-Benchmark-C-Longrun.
One Python supervisor holds a Windows byte-range lock. Repeating the command
checks both the scheduled task and the supervisor process and refuses duplicates.
Default target is 8 h; persisted hard deadline is 12 h from the FIRST start.
No automatic new budget. COMPLETE/hard-blocker/deadline/resource-stop states
require reading the saved result before any separately authorized resumption.

Supervisor checks lightweight files/processes every 180 s. The existing worker
performs expensive official connectivity and native surface audits once after
each completed step, never via a second Fluent polling session. Production is
one core because two archived four-core attempts stalled at step 5; neither
four-core retry is repeated. No GUI, and all processing uses H:/fluent/.venv.

Recovery guards: no existing Fluent or worker, >=22 GiB available RAM,
>=35 GiB disk; closed complete HDF5 case/data and SHA256; frozen UDF hash.
After loading a checkpoint, native flow-time, actual FieldData surface pose
and COM are checked before stepping. Native case/data preserve orientation
and velocities; the resume path never sets the reference COM/orientation.
The archive retains the original history before truncating to the selected
checkpoint. Checkpoints occur every four steps under separate names.

Stage sequence:
  restore/adopt -> free 80 x 25 us -> full 2 ms gate -> 0.25 ms cutoff check
  -> saved native FieldData -> PyVista offscreen -> actual visual inspection
  -> scoped milestone Git commits + origin/main push -> close solver.
The actual finish scripts contain solver/time/finite/clearance guards.

Repair decision layer:
  Codex heartbeat benchmark-c-12-hour-continuation runs every 15 minutes
  in the SAME chat, reading scripts/benchmark_C_longrun_continuation_prompt.txt.
  It is quiet while work is progressing or resources remain unchanged.
  On failure it diagnoses archived evidence and prepares one reviewed job.
  Supervisor accepts evidence-backed repair_plan.json with matching failure
  timestamp and script hash, rejects repeated configuration fingerprints,
  and permits at most two technically distinct repairs per failure class.
  Mesh repair algorithms are generated only after a concrete failure diagnosis;
  they are not speculative automatic remeshing or parameter tuning.
  This decision layer depends on Codex desktop heartbeat availability; solver
  monitoring/checkpointing/deadline run independently in the scheduled task.

State: evidence/benchmark_C_longrun_state.json
Logs/events/checkpoint inventory/archives: evidence/benchmark_C_longrun/
End report: evidence/benchmark_C_longrun_final.json
Review plan fields and examples: continuation_prompt.txt and supervisor source.

Deadline: request stop before the 12 h boundary, finish the current safe atomic
step/checkpoint or FieldData frame, then process the queued clean exit.
If an operation cannot return within the final 15 min grace, only captured
worker-tree process identities are terminated; the last previously verified
checkpoint remains intact and forced shutdown is explicitly recorded.
Task Scheduler ceiling is 12 h 30 min including shutdown grace.

Safety tests (temporary fixtures; never launch Fluent):
  H:\fluent\.venv\Scripts\python.exe H:\fluent\scripts\test_benchmark_C_longrun_supervisor.py

Do not use --last CLI to spawn unrelated work. Installed codex exec resume
help is recorded as a capability, not as proof of successful agent execution.
Do not reset frozen A/B/source/UDF physics, reuse the old partial failure GIF,
or start contact/Benchmark D.
