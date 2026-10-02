"""Verify complete triangular face topology independent of CFF numbering."""
import hashlib,json
import h5py,numpy as np
from benchmark_C_coarse_common import ROOT,OUT,EVID,atomic,stamp

def signature(path):
    with h5py.File(path,'r') as f:
        m=f['meshes/1'];top=m['cells/zoneTopology'];names=top['name'][0].decode().split(';');i=names.index('robot_component_fluid');lo,hi=int(top['minId'][i]),int(top['maxId'][i])
        xyz=np.concatenate([x[:] for x in m['nodes/coords'].values()]);faces=[]
        for key,g in m['faces/nodes'].items():
            c0=m['faces/c0/'+key][:];c1=m['faces/c1/'+key][:] if key in m['faces/c1'] else np.zeros_like(c0)
            mask=((c0>=lo)&(c0<=hi))|((c1>=lo)&(c1<=hi))
            if not np.all(g['nnodes'][:]==3):raise RuntimeError('Non-triangular component topology')
            faces.append(g['nodes'][:].reshape(-1,3)[mask].astype(np.int64)-1)
        faces=np.concatenate(faces);ids=np.unique(faces);points=xyz[ids];order=np.lexsort(points.T[::-1]);points=points[order]
        # Coordinates exported by Prime are bit-identical, zero signs canonicalized.
        points[points==0]=0
        canonical=np.zeros(len(xyz),dtype=np.int64);canonical[ids[order]]=np.arange(len(order));conn=np.sort(canonical[faces],axis=1);conn=conn[np.lexsort(conn.T[::-1])]
        return {'cells':hi-lo+1,'nodes':len(points),'all_faces':len(conn),'node_sha256':hashlib.sha256(points.tobytes()).hexdigest(),'face_coordinate_connectivity_sha256':hashlib.sha256(conn.tobytes()).hexdigest()}

def main():
    source=ROOT/'live_cases/benchmark_C_fineA/bg020/benchmark_C_fineA.cas.h5'
    rec={'timestamp':stamp(),'source':str(source),'extracted':str(OUT/'validated_component.cas.h5'),'source_signature':signature(source),'extracted_signature':signature(OUT/'validated_component.cas.h5')}
    rec['status']='PASS' if rec['source_signature']==rec['extracted_signature'] else 'FAIL'
    if (OUT/'coarse_raw.cas.h5').exists():
        rec['coarse_signature']=signature(OUT/'coarse_raw.cas.h5')
        if rec['coarse_signature']!=rec['source_signature']:rec['status']='FAIL'
    atomic(EVID/'component_identity.json',rec)
    if rec['status']!='PASS':raise RuntimeError('Component topology/geometry changed')
    from benchmark_C_coarse_common import read
    extraction=read(EVID/'component_extraction.json')
    extraction.update(status='PASS',topology_identity_evidence='evidence/benchmark_C_coarse_A/component_identity.json')
    atomic(EVID/'component_extraction.json',extraction)

if __name__=='__main__':main()
