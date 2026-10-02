"""Independent component-ray audit; no Fluent launch or dynamics."""
from benchmark_C_coarse_common import *
from benchmark_C_actual_ray_layers import audit,rays
import faulthandler
faulthandler.enable()
atomic(EVID/'layer_probe_state.json',{'timestamp':stamp(),'status':'RUNNING'})
result=audit(OUT/'coarse_raw.cas.h5','robot_component_fluid',rays())
atomic(EVID/'layer_probe_result.json',result)
atomic(EVID/'layer_probe_state.json',{'timestamp':stamp(),'status':'PASS' if result['minimum_crossed_cells']>=4 else 'FAIL'})
