# Fluent 2026 R1 mesh import diagnosis

The source files are Gmsh meshes, not Fluent native meshes. Their headers begin
with `$MeshFormat` and version `2.2`; they contain tetrahedral volume cells and
triangle boundary cells. `evidence/gmsh_mesh_audit.json` records the topology.

MCP API help for `file.read_mesh` says only that it reads a file of type case,
data, mesh, or case-data; it does not identify the Gmsh format or provide a
Gmsh conversion guarantee. Calling `solver.settings.file.read_mesh` on both the
Gmsh pipe and a simple Gmsh box produced `Null Domain Pointer` after Fluent's
mesh scan. The same error occurs with the Gmsh 2.2 file, so this is not caused
by the pipe topology alone.

The 2026 R1 Meshing API exposes `solver.meshing.ImportGeometry`. Its live
docstring explicitly supports a `FileFormat` of `Mesh` for surface or volume
mesh and a separate `MeshUnit`. The first MCP call using `MeshFileName` reached
the live Meshing API but returned `CDR: invalid argument [1]: wrong type [not a pair]`
from the generated command wrapper. This is a separate API argument-shape issue,
not evidence that the Gmsh mesh topology is invalid.

Chosen next path: use the live Fluent Meshing `ImportGeometry(FileFormat='Mesh',
...)` route with the exact generated argument shape, or generate a native Fluent
mesh through the Meshing workflow. Do not repeatedly feed the Gmsh file to the
solver-mode `file.read_mesh` reader.
