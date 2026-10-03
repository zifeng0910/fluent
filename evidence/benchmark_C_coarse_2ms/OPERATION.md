# Coarse native 2 ms continuation

Windows task Fluent-Benchmark-C-Coarse-2ms owns exactly one detached worker.
Start scripts/run_benchmark_C_coarse_2ms.ps1 only from PREPARED/RESOURCE_WAIT.
The worker shares the accepted campaign pipeline lock, validates native step40
at zero advanced time, then solves41..80 locally without LLM availability.
Resource admission12GiB physical /19GiB commit headroom /35GiB disk for60s.
Runtime3GiB physical /95%commit /15GiB projectWS /15GiB disk.
The inherited original20h absolute deadline is never reset or extended.

While the worker runs, monitor files only. Never attach a second solver or edit
its inputs. All old tasks remain disabled. No unchanged retry after failure.
Failure captures native saved time/COM/quaternion/velocity/omega; review exact
failure evidence and preserve latest checkpoint before a distinct repair.
Full static envelope exceeded is a warning when dynamic connectivity is valid.
Actual orphan/invalid/nonpositiveV/gap below0.10mm/nonfinite/q/fatal gates stop.

Checkpoints50/60/70/80 are separate native CASE+DATA with hashes and state.
FieldData snapshots40..80 every2 steps are from the existing dynamics session.
Rendering uses PyVista offscreen after owned solver closure, fixed velocity
scale and fixed camera. Actual first/middle/last GIF frames and final PNG need
human-agent visual inspection before visual_review.json can say PASS.
The supervisor ends at AWAITING_VISUAL_REVIEW; this never labels unseen images
PASS. Commit only verified restart/numerical/visual milestones and push main.

Read state.json, supervisor_state.json and newest events before any action.
Numerical PASS requires80/80,2ms, all hard gates,50/60/70/80 checkpoint integrity
and zero-step80 save-state verification. No exact fine2ms reference exists;
only retained0.825ms exact matched data is valid. No convergence/cutoff/D work.
