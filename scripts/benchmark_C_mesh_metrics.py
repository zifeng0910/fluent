"""Read actual CFF tetrahedra for conservative local overlap layer estimates."""
import h5py
import numpy as np
from scipy.spatial import cKDTree

def tetra_metrics(path, zone_name, scale=1.):
    with h5py.File(path) as f:
        mesh=f['meshes/1']; top=mesh['cells/zoneTopology']
        names=top['name'][0].decode().split(';'); k=names.index(zone_name)
        lo,hi=int(top['minId'][k]),int(top['maxId'][k]); count=hi-lo+1
        coords=np.concatenate([d[:] for d in mesh['nodes/coords'].values()])*scale
        fg=mesh['faces/nodes/1']; nn=fg['nnodes'][:]
        if not np.all(nn==3): raise ValueError('Expected all triangular faces')
        fn=fg['nodes'][:].reshape(-1,3).astype(np.int64)-1
        pairs=[]
        for side in ['c0','c1']:
            for d in mesh['faces/'+side].values():
                start=int(d.attrs['minId'][0])-1; cells=d[:].astype(np.int64)
                mask=(cells>=lo)&(cells<=hi)
                pairs.append(np.column_stack([np.repeat(cells[mask]-lo,3),fn[start:start+len(cells)][mask].reshape(-1)]))
        pairs=np.unique(np.concatenate(pairs),axis=0)
        if len(pairs)!=count*4 or not np.array_equal(pairs[:,0],np.repeat(np.arange(count),4)):
            raise ValueError('Non-tetra or incomplete topology')
        verts=coords[pairs[:,1]].reshape(count,4,3)
        size=np.max(np.stack([np.linalg.norm(verts[:,i]-verts[:,j],axis=1) for i in range(4) for j in range(i)]),axis=0)
        centers=verts.mean(axis=1)
        return {'centers':centers,'max_edge':size,'tree':cKDTree(centers),'count':count}

def local_size(metrics, points):
    # Largest actual nearest-cell edge: conservative stencil length estimate.
    _,indices=metrics['tree'].query(points,k=1)
    return float(np.max(metrics['max_edge'][indices]))
