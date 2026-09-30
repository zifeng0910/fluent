"""Read-only Abaqus provenance/geometry extraction. Does not select a baseline."""
from pathlib import Path
import json, hashlib, collections
import numpy as np

ROOT = Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis/TrueCELLongForwardTransit')
CASE = 'F100_G2P20_NOFLUID_REALWALL50'
SRC = ROOT / 'case' / CASE
OUT = Path('H:/fluent/evidence') / CASE
OUT.mkdir(parents=True, exist_ok=True)
deck = SRC / (CASE + '.inp')
text = deck.read_text()
identity = json.loads((SRC/'case_identity.json').read_text())
prior = json.loads((SRC/'setup_audit.json').read_text())
nodes, elements = {}, []
inpart, mode = False, None
for line in text.splitlines():
    if line.startswith('**') or not line.strip(): continue
    if line.startswith('*'):
        key = line.lower()
        if key.startswith('*part,'): inpart = 'name=robot_solid' in key
        if key.startswith('*end part'): inpart = False
        mode = 'node' if key == '*node' else ('tet' if key.startswith('*element,') and 'type=c3d4' in key else None)
        continue
    if inpart and mode:
        cols = [x.strip() for x in line.split(',') if x.strip()]
        if mode == 'node': nodes[int(cols[0])] = list(map(float, cols[1:4]))
        elif mode == 'tet': elements.append(list(map(int, cols[1:5])))
assert nodes and elements
# The source instance has no translation/rotation records.
instance = text.split('*Instance, name=Robot_SOLID-1, part=Robot_SOLID')[1].split('*End Instance')[0]
assert not instance.strip(), 'Implement instance transform before exporting'
p = np.array([[nodes[n] for n in e] for e in elements])
vol = np.abs(np.linalg.det(p[:,1:] - p[:,0,None]))/6
com = np.einsum('n,ni->i',vol,p.mean(axis=1))/vol.sum()
q = p - com
s = q.sum(axis=1)
second = (np.einsum('ni,nj->nij',s,s) + np.einsum('nki,nkj->nij',q,q))/20
density_tonne_mm3 = float(text.split('*Material, name=MAT_ROBOT_RIGID')[1].split('*Density')[1].splitlines()[1].strip().rstrip(','))
cov = np.einsum('n,nij->ij',vol*density_tonne_mm3,second)
inertia = np.trace(cov)*np.eye(3)-cov
lumped_cov = np.einsum('n,nij->ij',vol*density_tonne_mm3,np.einsum('nki,nkj->nij',q,q)/4)
lumped_inertia = np.trace(lumped_cov)*np.eye(3)-lumped_cov
faces = collections.defaultdict(list)
for e in elements:
    for opposite in range(4):
        f = [e[j] for j in range(4) if j != opposite]
        xyz = np.array([nodes[i] for i in f])
        if np.dot(np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0]), np.array(nodes[e[opposite]])-xyz[0]) > 0:
            f[1],f[2] = f[2],f[1]
        faces[tuple(sorted(f))].append(f)
assert max(map(len,faces.values())) == 2
exterior = [v[0] for v in faces.values() if len(v)==1]
edges = collections.Counter(tuple(sorted((f[i],f[(i+1)%3]))) for f in exterior for i in range(3))
assert set(edges.values()) == {2}, 'Non-watertight robot surface'
with (OUT/'robot_exact_abaqus_global_SI.stl').open('w') as f:
    f.write('solid robot_exact_abaqus_global_SI\n')
    for face in exterior:
        xyz = np.array([nodes[n] for n in face])*1e-3
        normal = np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0]); normal /= np.linalg.norm(normal)
        f.write('facet normal '+' '.join(map(str,normal))+'\nouter loop\n')
        for point in xyz: f.write('vertex '+' '.join(map(str,point))+'\n')
        f.write('endloop\nendfacet\n')
    f.write('endsolid robot_exact_abaqus_global_SI\n')
names=[CASE+'.inp','case_identity.json','setup_audit.json','vuamp_precomputed_truecel.f90','magnetic_field_gradient_table_B0P11_A14P5.dat','magnetic_field_gradient_table_B0P11_A14P5.dat.json']
hashes={n:hashlib.sha256((SRC/n).read_bytes()).hexdigest() for n in names}
mass_mg=vol.sum()*density_tonne_mm3*1e9
checks = {
    'input_hash_matches_saved_audit': hashes[CASE+'.inp'].upper()==prior['input_sha256'],
    'fortran_hash_matches_saved_audit': hashes['vuamp_precomputed_truecel.f90'].upper()==prior['fortran_sha256'],
    'table_hash_matches_saved_audit': hashes['magnetic_field_gradient_table_B0P11_A14P5.dat'].upper()==prior['magnetic_table_sha256'],
    'mass_matches_saved_audit': bool(np.isclose(mass_mg,prior['robot_mass_mg'],rtol=1e-10)),
    'inertia_matches_saved_audit': bool(np.allclose(inertia,prior['robot_inertia_about_com_Nmm_s2'],rtol=1e-7,atol=1e-17)),
}
result={'case_id':CASE,'source':str(SRC),'selected_baseline':False,'classification':identity['classification'],
        'status':'CANDIDATE_AUDITED_NOT_QUALIFIED','source_sha256':hashes,'checks':checks,
        'robot_nodes':len(nodes),'robot_tetrahedra':len(elements),'exterior_triangles':len(exterior),
        'watertight_surface':True,'volume_mm3':float(vol.sum()),'mass_kg':float(mass_mg*1e-6),
        'com_global_m':(com*1e-3).tolist(),'inertia_global_kg_m2':(inertia*1e-3).tolist(),
        'principal_inertia_kg_m2':(np.linalg.eigvalsh(inertia)*1e-3).tolist(),
        'lumped_vertex_inertia_global_kg_m2':(lumped_inertia*1e-3).tolist(),
        'frequency_Hz_from_fortran':100,'B0_T_from_fortran':0.011,'gradient_scale_T_from_fortran':0.0022,
        'notes':['Do not use inherited INP comments as settings','Table sidecar 120 Hz is stale; Fortran phase=36000*t is authoritative','Candidate has contact chatter; not a validated successful baseline','Rigid-body source; no claim of elastic FSI validation']}
(OUT/'geometry_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
assert all(checks.values()), 'Source or mass identity mismatch'
