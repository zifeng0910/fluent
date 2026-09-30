import gmsh, json, os
step=r"H:/fluent/geometry/benchmark_A_fluid_domain/BenchmarkA_FluidDomain.step"; out=r"H:/fluent/live_cases/benchmark_A_stationary/benchmark_A_water_gmsh.msh"; os.makedirs(os.path.dirname(out),exist_ok=True)
gmsh.initialize(); gmsh.option.setNumber("General.Terminal",1); gmsh.model.add("benchmark_A_water"); gmsh.model.occ.importShapes(step); gmsh.model.occ.synchronize(); vs=[t for d,t in gmsh.model.getEntities(3)]
if len(vs)!=1: raise RuntimeError(vs)
gmsh.model.addPhysicalGroup(3,vs,1); gmsh.model.setPhysicalName(3,1,"water")
groups={"inlet":[] ,"outlet":[],"pipe_wall":[],"robot_wall":[]}
for ft in [t for d,t in gmsh.model.getBoundary([(3,vs[0])],oriented=False,recursive=False) if d==2]:
 bb=gmsh.model.getBoundingBox(2,ft)
 if abs(bb[0]+5)<1e-4 and abs(bb[3]+5)<1e-4: n="inlet"
 elif abs(bb[0]-5)<1e-4 and abs(bb[3]-5)<1e-4: n="outlet"
 else: n="pipe_wall" if max(abs(bb[1]),abs(bb[2]),abs(bb[4]),abs(bb[5]))>0.88 else "robot_wall"
 groups[n].append(ft)
for tag,n in enumerate(groups,10):
 if groups[n]: gmsh.model.addPhysicalGroup(2,groups[n],tag); gmsh.model.setPhysicalName(2,tag,n)
gmsh.option.setNumber("Mesh.MeshSizeMin",0.04); gmsh.option.setNumber("Mesh.MeshSizeMax",0.08)
# Fluent's 2026 R1 mesh reader is more reliable with the legacy MSH 2.2
# element record format than the default MSH 4.x block format.
gmsh.option.setNumber("Mesh.MshFileVersion",2.2); gmsh.option.setNumber("Mesh.Binary",0)
gmsh.model.mesh.generate(3); gmsh.write(out)
m={"step":step,"mesh":out,"exact_occ_volume_mm3":gmsh.model.occ.getMass(3,vs[0]),"mesh_size_mm":[0.04,0.08],"physical_surface_counts":{k:len(v) for k,v in groups.items()},"units":"mm"}; json.dump(m,open(r"H:/fluent/evidence/benchmark_A_gmsh_manifest.json","w"),indent=2); print(json.dumps(m)); gmsh.finalize()
