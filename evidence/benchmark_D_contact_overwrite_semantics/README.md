# BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS

Result: **NATIVE_CONTACT_RESPONSE_PATH_VALIDATED**. Native read path: **PASS**. Native response pathway: **VALIDATED**, limited to one event.

## Verified experiment

Frozen C70 at1.750ms; 4,699,301 cells;25us;2iterations;1rank/double; unchanged magnetic physics/mass/COM/inertia. A1/A2 are read-only; B1/B2 write the exact Get-returned arrays. All independently restart C70 and end1.875ms. Controlled write: **PASS**. Previous C/D evidence and source hashes are unchanged. All owned engines closed.

## Repeatability and invocation artifact

| Measurement | COM norm | Orientation |
|---|---:|---:|
| A1 vs A2 peak | 0m | 0rad |
| B1 vs B2 peak | 0m | 0rad |
| First-contact B1 vs A1 | 4.94405200035e-09m | 0.0264776266046deg |
| Five-boundary accumulated B1 vs A1 peak | 1.64931009513e-08m | 0.000922633177576rad |

COM maximum-component difference: 4.01799695047e-09m. Normal actual step COM motion: 1.19286506805e-07m; orientation increment: 0.0228407756997rad. Endpoint |v|dt: 1.21361189885e-07m; |omega|dt: 0.0233027058699rad.

Relative artifact: **4.14469% COM**, **2.02323% orientation**. It is material relative to ordinary motion and deterministic, with an empirically verified integration explanation. It is not solver repeat noise. The controlled intended signal/floor ratios are **453.36x linear** and **15.433x angular**; the conservative floor includes pose artifact/dt. This admission is limited to the one-event experiment.

## Theta and lifecycle

Installed prototypes lack argument names and implementation bodies. The rigid-body theta comment says global orientation/Euler angles for6DOF; that does not specify the Get third-array contract. The installed official contact example was **not found**. The [same-release official example](https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html) demonstrates returning Get theta to Overwrite; it does not guarantee no-op equivalence to omission.

In these actual callbacks Get theta remains caller-zero, while DT_THETA is nonzero and matches native Theta_From_Q and the independently observed absolute quaternion rotation vector. It differs from native Euler_From_Q. Get theta is therefore neither the evolving absolute DT_THETA nor omega*dt here. Whether the third output is intentionally unassigned/unused is **UNKNOWN**. Outside contact the Get motion arrays remain zero; those calls are observational only.

Empirical reconstruction: overwrite CG_next=CG_start+dt*v_endpoint; Q_next=worldRotation(omega_endpoint*dt)*Q_start. Read-only translation uses the average start/end velocity. Immediately after the same-value setter, visible CG/Q remain unchanged; the difference appears at the completed boundary. Callback occurs with endpoint native state, old CURRENT_TIME/N_TIME and old surface coordinates, before final mesh relocation. Exact predictor/corrector internals and buffer/flag identity remain **UNKNOWN**. No manual quaternion-to-Euler values were sent to the setter.

## Controlled write and gates

See native_controlled_write_validation.json for independent predicted/actual v,omega and impulse, completed-step response, the next native start and actual PROPERTIES_ENTRY inheritance. Immediate callback reread is not a PASS gate. Exactly one e_n=0,mu=0 event is permitted; no full collision rollout was run.

In the physical event, Get immediately after Overwrite reflects the requested new v/omega, while direct DT_VEL/DT_OMEGA/DT_CG/DT_Q fields still show their prior values. At the completed boundary and next-step entry, native fields retain the requested change. This provides an empirical pending-contact-motion explanation; the exact internal buffer/flag is unknown. The no-op second callback already shows its completed pose before the call, so persistence and a repeated call's separate contribution were not isolated.

Orphan peak=0; invalid donor peak=0; nonpositive volume peak=0; minimum V=6.10059681204e-17m3; minimum gap=4.81774574845e-05m.

## Audit-only fallbacks

A: no separately supported alternative state setter found; internal declarations are insufficient. B: documented existing SDOF_PROPERTIES COM force/torque path is the first fallback; event timing, force integral independent of callback count, passivity and frozen-dt adequacy require validation. C: calibrated penalty adds stiffness/damping and timestep requirements. D: external integration is the largest coupling change. No fallback was implemented; analytic detection alone does not solve writeback semantics.

Preferred next architecture: Native DEFINE_CONTACT + same-returned-theta Overwrite with explicitly accounted endpoint-pose integration semantics.

Recommended next: Retain this verified one-event baseline; separately authorize longer contact validation with nonpenetration and energy checks.

## Evidence

local_api_audit.json distinguishes DOCUMENTED / INFERRED_FROM_HEADER / EMPIRICALLY_MEASURED / UNKNOWN. read_only_reproducibility.json, noop_reproducibility.json, noop_vs_read_only.json and no_op_artifact_quantification.json report each exact time. theta_quaternion_semantics.csv and per-branch host/node JSONL retain callback before/after, completed boundaries and next-step entry. Each branch retains native checkpoint SHA/HDF5 metadata, restart continuity, magnetic oracle, resource summary and clean engine exit. Large native CASE/DATA and surfaces remain local.
