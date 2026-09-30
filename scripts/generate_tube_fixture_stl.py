"""Generate reproducible mm-coordinate tube wall and cap STL fixtures from YAML."""
from pathlib import Path
import math, yaml

ROOT=Path(__file__).resolve().parents[1]
cfg=yaml.safe_load((ROOT/'geometry/tube_fixture.yaml').read_text())['tube']
R=cfg['radius_mm']; x0=cfg['inlet_x_mm']; x1=cfg['outlet_x_mm']; n=96
out=ROOT/'geometry/tube_fixture_stl'; out.mkdir(exist_ok=True)
def norm(a,b,c):
    u=[b[i]-a[i] for i in range(3)]; v=[c[i]-a[i] for i in range(3)]
    q=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
    m=math.sqrt(sum(z*z for z in q)) or 1.; return [z/m for z in q]
def write(name, tris):
    p=out/f'{name}.stl'
    with p.open('w',encoding='ascii') as f:
        f.write(f'solid {name}\n')
        for a,b,c in tris:
            q=norm(a,b,c); f.write(f' facet normal {q[0]:.9g} {q[1]:.9g} {q[2]:.9g}\n  outer loop\n')
            for v in (a,b,c): f.write(f'   vertex {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n')
            f.write('  endloop\n endfacet\n')
        f.write(f'endsolid {name}\n')
    return p
ring=lambda x,i:(x,R*math.cos(2*math.pi*i/n),R*math.sin(2*math.pi*i/n))
wall=[]
for i in range(n):
    j=(i+1)%n; a,b,c,d=ring(x0,i),ring(x0,j),ring(x1,j),ring(x1,i)
    wall += [(a,b,c),(a,c,d)]
inlet=[]; outlet=[]; ci=(x0,0.,0.); co=(x1,0.,0.)
for i in range(n):
    j=(i+1)%n; inlet.append((ci,ring(x0,j),ring(x0,i))); outlet.append((co,ring(x1,i),ring(x1,j)))
for name,tris in [('pipe_wall',wall),('inlet',inlet),('outlet',outlet)]: write(name,tris)
write('tube_all', wall + inlet + outlet)
print({'radius_mm':R,'length_mm':x1-x0,'facets':{'pipe_wall':len(wall),'inlet':len(inlet),'outlet':len(outlet) }})
