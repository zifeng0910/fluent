"""First connectivity gate using standard Fluent cell-center field export."""
import json,time,sys,hashlib
import numpy as np
from pathlib import Path

def summarize_csv(path):
    # Stream chunks to keep 11M-cell exports within memory limits.
    counts={str(k):0 for k in [-3,-2,-1,0,1,2]};ratios=[];minv=float('inf');total=0;orphans=[]
    with path.open() as fp:
        header=fp.readline().strip().split(',');header=[x.strip() for x in header]
        it=header.index('overset-cell-type');ir=header.index('overset-donor-size-ratio');iv=header.index('cell-volume')
        while True:
            lines=[fp.readline() for _ in range(100000)];lines=[s for s in lines if s]
            if not lines:break
            a=np.loadtxt(lines,delimiter=',',ndmin=2);types=a[:,it].astype(int)
            for k in counts:counts[k]+=int(np.count_nonzero(types==int(k)))
            if len(orphans)<20:orphans.extend(a[types==-1,:4][:20-len(orphans)].tolist())
            r=a[:,ir];ratios.append(r[(types==2)&(r>0)])
            minv=min(minv,float(a[:,iv].min()));total+=len(a)
    r=np.concatenate(ratios);length=np.cbrt(r)
    stats=lambda x:dict(zip(['min','median','p95','max'],map(float,np.quantile(x,[0,.5,.95,1])))) if len(x) else None
    return {'cell_type_counts':counts,'orphans':counts['-1'],'orphan_cellnumber_xyz_first20':orphans,'donors':counts['2'],'receptors':counts['0'],'total_cells':total,'minimum_volume_m3':minv,'donor_volume_ratio':stats(r),'donor_characteristic_length_ratio':stats(length),'donor_length_ratio_gt3_count':int(np.count_nonzero(length>3)),'donor_ratio_samples':len(r)}

def run(solver,context):
    root=context['root'];sys.path.insert(0,str(root/'scripts'));s=solver.settings
    case=root/'live_cases/benchmark_C_fineA/bg020';out=root/'evidence/benchmark_C_fineA_theta0_connectivity.json'
    rec={'status':'RUNNING','stage':'read_mesh','theta_rad':0,'magnetic_physics_frozen':True,'main_udf_sha256':hashlib.sha256((root/'fluent_udf/l2300_abaqus100hz_6dof.c').read_bytes()).hexdigest(),'primary_orphan_metric':'overset-cell-type == -1','component_nominal_size_mm':[.010,.0125],'background_local_nominal_size_mm':.020,'overlap_layers':'PENDING_ACTUAL_RAY_COUNT'}
    def save():out.write_text(json.dumps(rec,indent=2,default=str))
    save();start=time.time();msgstart=len(context['messages'])
    try:
        s.file.read_case(file_name=str(case/'benchmark_C_fineA.cas.h5'));s.mesh.scale(x_scale=.001,y_scale=.001,z_scale=.001)
        s.setup.boundary_conditions.set_zone_type(zone_list=['overset_component'],new_type='overset')
        s.setup.boundary_conditions.set_zone_type(zone_list=['inlet'],new_type='pressure-inlet')
        s.setup.boundary_conditions.set_zone_type(zone_list=['outlet'],new_type='pressure-outlet')
        s.setup.models.viscous.model='laminar';mat=s.setup.materials.fluid
        if 'water' not in mat.keys():mat.create(name='water')
        mat['water'].density.set_state({'option':'constant','value':998.2});mat['water'].viscosity.set_state({'option':'constant','value':.001003})
        for z in ['background_water','robot_component_fluid']:s.setup.cell_zone_conditions.fluid[z].general.material='water'
        rec['stage']='standard_initialize';save();s.solution.initialization.standard_initialize()
        rec['stage']='mesh_check';save();i=len(context['messages']);s.mesh.check();rec['mesh_check_transcript']=''.join(context['messages'][i:])
        rec['stage']='official_cell_field_export';save()
        fields=case/'theta0_official_cells.csv'
        s.file.export.ascii(file_name=str(fields),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
        rec.update(summarize_csv(fields));rec['field_export']=str(fields)
        info=solver.fields.solution_variable_info.get_variables_info(zone_names=['background_water','robot_component_fluid']).solution_variables
        rec['auxiliary_count_available']=list(info)
        for variable in ['SV_OVERSET_NDONOR','SV_OVERSET_NRECEPTOR']:
            if variable in info:
                data=solver.fields.solution_variable_data.get_data(variable_name=variable,zone_names=['background_water','robot_component_fluid'])
                rec[variable]={z:{'positive_cells':int(np.count_nonzero(data[z]>0)),'max':int(np.max(data[z]))} for z in ['background_water','robot_component_fluid']}
        rec['stage']='save_SI_checkpoint';save();s.file.write_case(file_name=str(case/'theta0_SI.cas.h5'))
        rec['status']='FAIL_THETA0_ORPHANS' if rec['orphans'] else 'CONNECTIVITY_CHECKED_LAYER_GATE_PENDING'
        rec['elapsed_seconds']=time.time()-start;save()
    except Exception as e:rec.update(status='ERROR',error=repr(e));save();raise
    finally:(root/'evidence/benchmark_C_fineA_theta0_transcript.txt').write_text(''.join(context['messages'][msgstart:]))
