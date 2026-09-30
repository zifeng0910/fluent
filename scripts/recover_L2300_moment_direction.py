import importlib.util,ezdxf,json,numpy as np
p='J:/magpy/magpylib_socket_server.py'
s=importlib.util.spec_from_file_location('old',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
a=m.MagneticCouplingModel.__new__(m.MagneticCouplingModel)
c=a._read_dxf_curve(ezdxf,'J:/magpy/curvenew_CEL_xyrot56_exact.dxf'); v=c[1]-c[0];v/=np.linalg.norm(v)
r=np.array(json.load(open('J:/abaqusfangzhen/abaqus_magpylib_frame_transform.json'))['R_aba_to_mag'])
v=r.T@v;axis=np.array([.9647382600216,-.1188742372140,.2348382536499]);axis/=np.linalg.norm(axis)
print(json.dumps({'tangent_aba':v.tolist(),'CAD_axis_aba':axis.tolist(),'dot':float(v@axis),'difference':float(np.linalg.norm(v-axis))}))
