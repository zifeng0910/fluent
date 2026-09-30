# Abaqus → Fluent migration workspace

## Current status

- PyFluent MCP → Fluent 2026 R1 connection: PASS
- Magpylib/verified magnetic load replay: PASS
- Magnetic lookup interface and provenance: PASS
- Gmsh topology audit: PASS
- Benchmark A: NOT RUN; STL import option error; launch budget exhausted (3/3).

Authoritative geometry: STEP/BREP/FCStd. Derived Fluent meshing representation:
`geometry/Robot_L2300_D0815_WallWobble_fluent.stl` (metres).
See `evidence/robot_stl_geometry_audit.json` and `evidence/stl_campaign_20260930.md`.
- Benchmark B: NOT ATTEMPTED
- Benchmark C: NOT ATTEMPTED
- Benchmark D: NOT ATTEMPTED

This workspace is a migration in progress, not a completed coupled simulation.
Original evidence under `J:/abaqusfangzhen` is read-only for this work.

## Installed integration

- Upstream checkout: `H:/fluent/pyfluent-mcp`, commit `c966560cdbcff631bb5c5684ae040898199781d1`.
- Skill: `C:/Users/zfzha/.codex/skills/ansys-fluent-mcp/SKILL.md`.
- Dedicated Python 3.13 environment: `H:/fluent/.venv`.
- Codex STDIO server name: `ansys-fluent-mcp`.
- Fluent binary: `H:/Program Files/ANSYS Inc/v261/fluent/ntbin/win64/fluent.exe`.
- `scripts/mcp_call.py` uses real MCP HTTP transport on loopback port 8765 and records calls under `logs/`. It is not a fake solver or a direct-function replacement for MCP.

## Source identification

Two distinct archived animations exist in
`J:/abaqusfangzhen/abaqus_robot/calibration_analysis/TrueCELLongForwardTransit/`:

1. `TRUECEL_B0P11_G2P20_F100_NOFLUID_CONTROL.gif`: magnetic-only control, fluid and all contact removed, passive wall geometry. Five cycles completed with forward motion. This cannot qualify a wall-contact law.
2. `F100_G2P20_NOFLUID_REALWALL.gif`: actual robot-wall contact; associated `F100_G2P20_NOFLUID_REALWALL50_REPORT.md` explicitly classifies it as `NOFLUID_REALWALL_CHATTER_FAIL`, despite forward net travel.

The user's intended successful GIF has not yet been uniquely identified. Neither animation is silently promoted to a validated coupled baseline.

## Completed source checks

`scripts/audit_abaqus_candidate.py` reads the real-wall candidate INP and exports its exact exterior surface in metres. Its 7,302 tetrahedra and 1,406 exterior triangles form a watertight robot surface. Input, Fortran and magnetic-table SHA256 values match the archived setup audit. Independently integrated mass, COM and inertia also match. See `evidence/F100_G2P20_NOFLUID_REALWALL50/geometry_audit.json`.

`scripts/magnetic_table_si.py` implements the archived magnetic field/gradient interpolation law in SI. It consumes reference-point displacement and a proper rotation matrix, returns N and N·m, and rejects table extrapolation. It uses the authoritative Fortran settings: 100 Hz, 11 mT, gradient amplitude 2.2 mT, 1 ms smooth ramp. The gradient basis is already per metre; the spatial lookup coordinate remains mm to match the table. A 500-sample archived five-cycle replay passed the predeclared 0.1% peak-normalized error gate. This checks code/units/pose interpolation only, not coupled dynamics.

## Fluent magnetic 6DOF contract

`scripts/generate_magnetic_lookup.py` generates `magnetic/magnetic_lookup.npz`,
`magnetic/magnetic_lookup.csv`, and `magnetic/magnetic_lookup_manifest.yaml` from
the same audited load implementation. The offline table is the required magnetic
input for the final Fluent coarse runs; it is not optional post-processing. The
manifest records the verified 100 Hz source and the stale 120 Hz legacy label.
`scripts/magnetic_lookup_runtime.py` rejects every pose outside the declared
coverage. `fluent_udf/magnetic_lookup_6dof.c` supplies magnetic force and COM
torque to the Fluent SDOF hook, which is combined with Fluent pressure/viscous
surface loads. Benchmark C and D configurations are under `benchmarks/` and
require force/torque histories with separate magnetic, fluid, total, and (for D)
contact terms. Run `scripts/check_magnetic_fluent_report.py` on that history to
produce the final load provenance report.

## Dry dynamics gate: unresolved

`scripts/validate_dry_rigid_body.py` independently integrates Newton–Euler rigid-body dynamics for 50 ms using the archived magnetic law and the RP-to-COM torque conversion. The position gate is 5 micrometres and the orientation gate is 0.1 degrees, specified before running.

- Continuous tetrahedron inertia (matches the old setup audit): maximum position error 0.558 micrometres; maximum orientation error 2.436 degrees. **FAIL** on orientation.
- Vertex-lumped tetrahedron inertia, with all other physics unchanged: maximum position error 0.0962 micrometres; maximum orientation error 1.061 degrees. **FAIL** on orientation.

The second comparison is supported by the Abaqus documentation's statement that low-order rigid-body elements use lumped mass; continuum-integrated inertia need not equal solver inertia. Source: https://docs.software.vt.edu/abaqusv2025/English/SIMACAEMODRefMap/simamod-c-rigidoverview.htm (mass/inertia by discretization). It reduces the discrepancy but does not establish its complete cause. Do not tune inertia/damping to force a pass, call the old inertia audit a solver-level inertia verification, or treat this dry Python integration as Fluent CFD. Both failed results are preserved under `evidence/dry_rigid_body/`.

Important distinctions:

- Rigid-body source; this is not a validated elastic-structure FSI model.
- Actual mass 10.018808990 mg supersedes stale 9.207793514 mg metadata.
- Fortran phase `36000*t` means 100 Hz; table sidecar's 120 Hz is stale.
- The archived load is applied at the Abaqus reference point. Fluent 6DOF COM application requires an audited reference-point/COM lever-arm torque conversion.
- Wall-contact Coulomb friction is not liquid no-slip. Fluid pressure and viscous traction must be integrated separately.
- A geometrically watertight STL is not a fluid volume mesh and does not pass a CFD mesh gate.

## Remaining simulation gates

Follow the existing `J:/abaqusfangzhen/docs/FLUENT_PRECHECK.md` and `FLUENT_DIAGNOSTIC_PROTOCOL.md`: Level 0 static fluid/geometry, Level 1 prescribed translation, Level 2 prescribed rotation, Level 3 magnetic 6DOF without contact, Level 4 contact, Level 5 full coupling with at least two complete cycles. Preserve case/data, source hashes, mesh/zone checks, pressure and viscous force/torque, contact, mass balance, and motion histories. Output-only and restart equivalence are separate comparisons. Do not claim success from a GIF, first-cycle displacement, solver startup or magnetic replay alone.
