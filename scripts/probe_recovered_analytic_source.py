import importlib.util,json,numpy as np
from pathlib import Path
p=Path('J:/abaqusfangzhen/abaqus_robot/calibration_analysis/PrecomputedStraightMagneticBackend/scripts/table_model.py');s=importlib.util.spec_from_file_location('src',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
c=m.config();c.update(frequency_Hz=100,B0_T=.011,gradient_G_T=.0022,rocking_main_amplitude_deg=14.5,rocking_cross_amplitude_deg=2.5)
live=m.build_live_model(c)
for d in [0,90,180]:
 live.magnetic_model.reset_robot_arc_continuity();r=live.evaluate(d/360/100,np.array(c['initial_center_aba_mm']),np.zeros(3));print(d,r['B_aba_vec_T'])
