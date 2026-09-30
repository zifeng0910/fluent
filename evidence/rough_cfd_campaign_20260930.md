# Rough CFD campaign status

Robot STL and parameterized tube surface imports both succeeded in Fluent 2026 R1. Four sequential headless sessions were used and closed cleanly by the single-session manager. No Gmsh, CGNS, STEP reader, magnetic load, 6DOF, contact, overset, or solver session was started.

1. Separate wall/inlet/outlet STL files: surface remesh failed with 2 free nodes.
2. Merged `tube_all.stl`: same 2 free-node failure.
3. Direct `create_regions`: Fluent internally invoked the same surface remesh and failed identically.
4. `remesh_imported_mesh=No`: Fluent accepted the setting, then required an undocumented `Labels` argument.

Result: tube surface import PASS; surface mesh, fluid region, volume mesh, Benchmark A, transient rough flow, GIFs and case/data files are NOT generated. The blocker is the Fluent 2026 R1 Watertight workflow task contract.
