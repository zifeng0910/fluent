# Controlled STL campaign — 2026-09-30

Benchmark A is NOT RUN. Robot STL import has NOT passed. No native fluid mesh,
tube, timestep, hydrodynamic result or laminar confirmation exists from this campaign.
B/C/D and magnetic/contact work remain frozen.

Authoritative STEP/BREP/FCStd are unchanged. Derived STL has 10,420 triangles,
is watertight/manifold with outward normals, and uses metres. Sampled maximum
surface deviation is 0.000312881 mm (not a certified Hausdorff bound).

Three sequential headless meshing launches were used; all ended FAIL_AND_CLOSED.
No fourth launch is permitted under this campaign. All three connected to 2026 R1.
Parent and child PROCESSOR_ARCHITECTURE were AMD64.

1. 20260930_112306_506f04: agent used solver-only `file` on a Meshing session.
   AttributeError before any STL import.
2. 20260930_112716_04097a: legacy Watertight initialized. Agent attempted to reuse
   `wf` across stateless MCP calls; validation rejected it before import.
3. 20260930_112914_712a42: agent omitted `file_format='Mesh'`; Fluent rejected
   import while the default was CAD. This is an import configuration error,
   not evidence of an invalid STL or a runtime segmentation fault.

The experiment JSON now explicitly sets Mesh and metres, but this correction is
UNTESTED. A later authorized campaign must verify object/zone/triangle counts and
bounding box before declaring ROBOT_STL_IMPORT_PASS. Do not reset the budget automatically.

The manager closed each session and cleaned owned descendants. Its
`closed_cleanly=False` includes residual helper processes, not only Fluent;
closure JSON preserves forced PID lists. Initial inventory had no Fluent solver
processes. Final inventory and cleanup evidence are recorded separately.

Small MCP logs and native transcripts are retained with each session. No physics
parameters or magnetic source implementations were modified.
