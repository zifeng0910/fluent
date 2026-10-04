"""Compare the actual compiled C kernel to independent NumPy/SciPy mechanics."""
import ctypes, json, subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark_D_common import *

def reference(mass, Iworld, r, n, v, w):
    vn = float(n @ (v+np.cross(w,r)))
    b = np.cross(r,n)
    J = max(0., -vn/(1/mass+b @ np.linalg.solve(Iworld,b)))
    return v+J*n/mass, w+np.linalg.solve(Iworld, J*b), J

def main():
    initialize()
    clang = Path(r'H:/Program Files/ANSYS Inc/v261/fluent/ntbin/clang/bin/clang-cl.exe')
    source = OUT/'impulse_math_export.c'
    source.write_text('int _fltused=0;\n#include "'+(ROOT/'fluent_udf/l2300_contact_impulse_math.h').as_posix()+'"\n'
        '__declspec(dllexport) void world_inverse(const double*R,const double*B,double*W){contact_world_inverse(R,B,W);}\n'
        '__declspec(dllexport) int impulse(double m,const double*A,const double*r,const double*n,const double*v,const double*w,double*V,double*O,double*J,double*a,double*b){return contact_impulse(m,A,r,n,v,w,V,O,J,a,b);}\n')
    dll=OUT/'impulse_math.dll'
    command=[str(clang),'/nologo','/O2','/LD',str(source),'/Fo'+str(OUT/'impulse_math.obj'),'/link','/noentry','/nodefaultlib','/out:'+str(dll),'/implib:'+str(OUT/'impulse_math.lib')]
    result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',errors='replace')
    (EVID/'impulse_math_build.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode: raise RuntimeError('Pure C kernel compilation failed')
    lib=ctypes.CDLL(str(dll)); pointer=ctypes.POINTER(ctypes.c_double)
    lib.world_inverse.argtypes=[pointer]*3
    lib.impulse.argtypes=[ctypes.c_double]+[pointer]*10
    lib.impulse.restype=ctypes.c_int
    def ptr(a):return a.ctypes.data_as(pointer)
    xy,xz,yz=INERTIA_PRODUCTS
    Ibody=np.array([[INERTIA_DIAG[0],-xy,-xz],[-xy,INERTIA_DIAG[1],-yz],[-xz,-yz,INERTIA_DIAG[2]]])
    inverse=np.ascontiguousarray(np.linalg.inv(Ibody))
    rng=np.random.default_rng(261004); peaks={}; counts={'approaching':0,'separating':0}; failures=[]
    samples=2048
    for k in range(samples):
        R=np.ascontiguousarray(Rotation.random(random_state=rng).as_matrix())
        Iworld=R@Ibody@R.T; A=np.zeros((3,3));lib.world_inverse(ptr(R),ptr(inverse),ptr(A))
        r=rng.normal(size=3);r*=rng.uniform(1e-6,1.8e-3)/np.linalg.norm(r)
        n=rng.normal(size=3);n/=np.linalg.norm(n)
        v=rng.normal(size=3)*rng.uniform(.001,1)
        w=rng.normal(size=3)*rng.uniform(1,1500)
        desired=(-1 if k%2==0 else 1)*rng.uniform(1e-5,2)
        v+=(desired-n@(v+np.cross(w,r)))*n
        V=np.zeros(3);W=np.zeros(3);J=ctypes.c_double();before=ctypes.c_double();after=ctypes.c_double()
        mode=lib.impulse(MASS,ptr(A),ptr(r),ptr(n),ptr(v),ptr(w),ptr(V),ptr(W),ctypes.byref(J),ctypes.byref(before),ctypes.byref(after))
        Vr,Wr,Jr=reference(MASS,Iworld,r,n,v,w)
        counts['approaching' if desired<0 else 'separating']+=1
        dP=MASS*(V-v); dL=Iworld@(W-w); expected=J.value*n
        errors={
            'normal_after_abs_m_s':abs(after.value) if desired<0 else 0.,
            'normal_after_negative_m_s':max(0.,-after.value),
            'linear_impulse_relative':float(np.linalg.norm(dP-expected)/max(np.linalg.norm(expected),1e-18)),
            'angular_impulse_relative':float(np.linalg.norm(dL-np.cross(r,expected))/max(np.linalg.norm(np.cross(r,expected)),1e-20)),
            'tangential_impulse_N_s':float(np.linalg.norm(dP-(dP@n)*n)),
            'reference_velocity_m_s':float(np.max(abs(V-Vr))),
            'reference_omega_rad_s':float(np.max(abs(W-Wr))),
            'reference_impulse_N_s':abs(J.value-Jr),
            'inverse_tensor_relative':float(np.max(abs(A-np.linalg.inv(Iworld)))/np.max(abs(A))),
        }
        thresholds={'normal_after_abs_m_s':1e-10,'normal_after_negative_m_s':1e-10,
            'linear_impulse_relative':1e-9,'angular_impulse_relative':1e-9,'tangential_impulse_N_s':1e-16,
            'reference_velocity_m_s':1e-10,'reference_omega_rad_s':1e-8,'reference_impulse_N_s':1e-16,'inverse_tensor_relative':1e-12}
        for name,value in errors.items():peaks[name]=max(peaks.get(name,0.),value)
        finite=np.isfinite(np.r_[V,W,J.value,before.value,after.value]).all()
        separate_unchanged=desired<0 or (J.value==0 and np.array_equal(V,v) and np.array_equal(W,w) and mode==0)
        if not finite or not separate_unchanged or mode<0 or any(errors[n]>thresholds[n] for n in errors): failures.append(k)
    rec=dict(timestamp=stamp(),status='PASS' if not failures else 'FAIL',random_states=samples,seed=261004,
        counts=counts,normal_restitution=0,friction=0,thresholds=thresholds,maximum_errors=peaks,
        failing_states=failures,actual_compiled_C_kernel=True,independent_reference='NumPy solve of rotated inertia, SciPy quaternion rotations',
        c_header_sha256=sha(ROOT/'fluent_udf/l2300_contact_impulse_math.h'),validation_script_sha256=sha(Path(__file__)),
        dll_sha256=sha(dll),solver_launched=False)
    atomic(EVID/'contact_impulse_unit_validation.json',rec)
    print(json.dumps(rec))
    if failures:raise RuntimeError('Impulse validation failed')

if __name__=='__main__':main()
