"""Prepare isolated same-C70 overwrite-semantics experiments; never edit earlier evidence."""
import json,re
from pathlib import Path
from benchmark_C_recovery_v2_common import atomic,read,sha,stamp
ROOT=Path(__file__).resolve().parents[1]
EVID=ROOT/'evidence/benchmark_D_contact_overwrite_semantics'
OUT=ROOT/'live_cases/benchmark_D_contact_overwrite_semantics'

HELPER=r'''
/* Read-only stage instrumentation. Native conversions below are measured, never
   fed back to SDOF_Overwrite_Motion and never assigned to the Dynamic_Thread. */
static void semantics_state(const char *stage,Dynamic_Thread *dt,real property_time,real dtime)
{
  real gv[3]={0,0,0},gw[3]={0,0,0},gt[3]={0,0,0},native_theta[3],native_euler[3];
  double cg[3],qv[4],v[3],w[3],th[3],tcg[3],tq[4],tv[3],tw[3],tth[3],acc[3],alpha[3],tmpcg[3]={0,0,0};
  double converted_theta[3],converted_euler[3],getter_v[3],getter_w[3],getter_t[3],roundtrip[4];
  real round_theta[3];quaternion qq,qr;int i;FILE *f;if(dt==NULL)return;
  SDOF_Get_Motion(dt,gv,gw,gt);Q_Assign(DT_Q(dt),&qq);
  Theta_From_Q(qq,native_theta);Q_Assign(DT_Q(dt),&qq);Euler_From_Q(qq,native_euler);
  for(i=0;i<3;i++)round_theta[i]=DT_THETA(dt)[i];Q_From_Theta(round_theta,&qr);
  for(i=0;i<3;i++){
    cg[i]=DT_CG(dt)[i];v[i]=DT_VEL_CG(dt)[i];w[i]=DT_OMEGA_CG(dt)[i];th[i]=DT_THETA(dt)[i];
    tcg[i]=DT_RB_TMP_STATE(dt).cg[i];tv[i]=DT_RB_TMP_STATE(dt).v_cg[i];tw[i]=DT_RB_TMP_STATE(dt).omega_cg[i];tth[i]=DT_RB_TMP_STATE(dt).theta[i];
    acc[i]=DT_ACC_CG(dt)[i];alpha[i]=DT_ALPHA_CG(dt)[i];
    converted_theta[i]=native_theta[i];converted_euler[i]=native_euler[i];getter_v[i]=gv[i];getter_w[i]=gw[i];getter_t[i]=gt[i];
    if(dt->tmp_cg!=NULL)tmpcg[i]=(*dt->tmp_cg)[i];
  }
  for(i=0;i<4;i++){qv[i]=DT_Q(dt)[i];tq[i]=DT_RB_TMP_STATE(dt).q[i];roundtrip[i]=qr[i];}
  f=fopen(RP_HOST?SEMANTICS_HOST_JSON:SEMANTICS_NODE_JSON,"a");if(!f)Error("D_SEMANTICS_LOG_OPEN_FAILED\n");
  fprintf(f,"{\"stage\":\"%s\",\"CURRENT_TIME\":%.17g,\"property_time\":%.17g,\"dtime\":%.17g,\"callback_index\":%lu,\"timestep_index\":%d,\"iteration_index\":%d,\"process_role\":\"%s\",",stage,(double)CURRENT_TIME,(double)property_time,(double)dtime,callback_index,N_TIME,N_ITER,RP_HOST?"host":"node");
  jvec(f,"CG",cg,3);jvec(f,"Q",qv,4);jvec(f,"velocity",v,3);jvec(f,"omega",w,3);jvec(f,"DT_THETA",th,3);
  jvec(f,"get_velocity",getter_v,3);jvec(f,"get_omega",getter_w,3);jvec(f,"get_theta",getter_t,3);
  jvec(f,"Theta_From_Q_native",converted_theta,3);jvec(f,"Euler_From_Q_native",converted_euler,3);jvec(f,"Q_From_DT_THETA_native",roundtrip,4);
  jvec(f,"tmp_state_CG",tcg,3);jvec(f,"tmp_state_Q",tq,4);jvec(f,"tmp_state_velocity",tv,3);jvec(f,"tmp_state_omega",tw,3);jvec(f,"tmp_state_theta",tth,3);
  jvec(f,"acceleration",acc,3);jvec(f,"angular_acceleration",alpha,3);jvec(f,"motion_tmp_cg_if_present",tmpcg,3);
  fprintf(f,"\"contact_active\":%d,\"dt_contact_p\":%d,\"sdof_p\":%d,\"tmp_cg_NULL\":%s,\"tmp_theta_NULL\":%s,\"conversions_read_only\":true}\n",(int)contact_active_p,(int)dt->contact_p,(int)dt->sdof_p,dt->tmp_cg==NULL?"true":"false",dt->tmp_theta==NULL?"true":"false");fclose(f);
}
DEFINE_ON_DEMAND(overwrite_semantics_snapshot)
{
  Domain *d=Get_Domain(1);Thread *t;
  thread_loop_f(t,d){if(!strcmp(THREAD_NAME(t),"robot_wall")){semantics_state("BOUNDARY_SNAPSHOT",THREAD_DT(t),CURRENT_TIME,RP_Get_Real("physical-time-step"));break;}}
}
'''

def replace_once(source,old,new):
    if source.count(old)!=1:raise RuntimeError('Expected exactly one source anchor: '+old[:80])
    return source.replace(old,new)

def main():
    if (EVID/'configuration.json').exists():raise RuntimeError('Already prepared; preserve earlier experiments')
    EVID.mkdir(parents=True,exist_ok=True);OUT.mkdir(parents=True,exist_ok=True)
    old=ROOT/'evidence/benchmark_D_contact_lifecycle';frozen=dict(read(old/'configuration.json')['frozen_files_sha256'])
    for p in [old/'final_report.json',old/'state.json',old/'native_motion_read_validation.json',old/'native_noop_write_validation.json',
              old/'noop_trajectory_change_diagnosis.json',ROOT/'fluent_udf/l2300_native_contact_lifecycle.c',ROOT/'scripts/benchmark_D_lifecycle_worker.py',
              ROOT/'scripts/benchmark_D_lifecycle_common.py']:
        frozen[p.relative_to(ROOT).as_posix()]=sha(p)
    common=(ROOT/'scripts/benchmark_D_lifecycle_common.py').read_text()
    common=common.replace('benchmark_D_contact_lifecycle','benchmark_D_contact_overwrite_semantics').replace('BENCHMARK_D_NATIVE_CONTACT_LIFECYCLE_DIAGNOSTIC','BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS')
    common=replace_once(common,"BRANCHES={'read_only':0,'noop':1,'controlled':2,'micro25':3,'micro12_5':3}","BRANCHES={'A1':0,'A2':0,'B1':1,'B2':1,'controlled':2}")
    common=common.replace('l2300_native_contact_lifecycle.c','l2300_contact_overwrite_semantics.c')
    common=common.replace("'CONNECTIVITY_JSON':'native_connectivity_current.json'","'CONNECTIVITY_JSON':'native_connectivity_current.json','SEMANTICS_HOST_JSON':'theta_lifecycle_host.jsonl','SEMANTICS_NODE_JSON':'theta_lifecycle_node.jsonl'")
    common=common.replace("dt_s=12.5e-6 if branch=='micro12_5' else DT","dt_s=DT")
    (ROOT/'scripts/benchmark_D_overwrite_common.py').write_text(common)
    worker=(ROOT/'scripts/benchmark_D_lifecycle_worker.py').read_text().replace('benchmark_D_lifecycle_common','benchmark_D_overwrite_common')
    worker=worker.replace('libD_lifecycle_','libD_overwrite_').replace('l2300_native_contact_lifecycle.c','l2300_contact_overwrite_semantics.c')
    start=worker.index("    if branch!='read_only':");end=worker.index('    b,o,config=prepare_branch(branch)',start)
    worker=worker[:start]+"    if branch=='controlled' and read(EVID/'controlled_write_admission.json').get('status')!='PASS':raise RuntimeError('Semantics and large-signal admission required')\n"+worker[end:]
    worker=replace_once(worker,"lock=(OUT/'lifecycle_worker.lock')","lock=(OUT/'semantics_worker.lock')")
    worker=replace_once(worker,"    t=float(solver.scheme.eval(\"(rpgetvar 'flow-time)\"));actual=native_state(solver,'robot_wall')","    solver.settings.setup.user_defined.execute_on_demand(lib_name='overwrite_semantics_snapshot::'+library)\n    t=float(solver.scheme.eval(\"(rpgetvar 'flow-time)\"));actual=native_state(solver,'robot_wall')")
    worker=replace_once(worker,"        if branch=='micro12_5':calc.parameters.time_step_size=config['dt_s']\n",'')
    worker=replace_once(worker,"        limit=12 if branch=='micro12_5' else 6 if branch=='micro25' else 5 if branch=='controlled' else 4","        limit=5  # Exact1.750->1.875ms diagnostic interval, includes next-step start")
    worker=replace_once(worker,"            if branch in ['read_only','noop'] and moving:break\n",'')
    worker=worker.replace("branch!='read_only' or", "branch not in ['A1','A2'] or")
    worker=replace_once(worker,"worker_alive=True,solver_alive=False)\n        stable=None","worker_alive=True,solver_alive=False,completed_new_steps=0,current_time_s=.00175)\n        stable=None")
    (ROOT/'scripts/benchmark_D_overwrite_worker.py').write_text(worker)
    source=(ROOT/'fluent_udf/l2300_native_contact_lifecycle.c').read_text()
    source=replace_once(source,'DEFINE_CONTACT(l2300_native_contact_lifecycle,dt,contacts)',HELPER+'\nDEFINE_CONTACT(l2300_native_contact_lifecycle,dt,contacts)')
    source=replace_once(source,'    if(LIFECYCLE_MODE==1){','    semantics_state("CALLBACK_BEFORE",dt,CURRENT_TIME,RP_Get_Real("physical-time-step"));\n    if(LIFECYCLE_MODE==1){')
    source=replace_once(source,'  f=fopen(RP_HOST?TRACE_HOST_JSON:TRACE_NODE_JSON,"a");','  if(moving && count>0)semantics_state("CALLBACK_AFTER",dt,CURRENT_TIME,RP_Get_Real("physical-time-step"));\n  f=fopen(RP_HOST?TRACE_HOST_JSON:TRACE_NODE_JSON,"a");')
    source=replace_once(source,'  l2300_magnetic_6dof(prop,dt,time,dtime);','  if(!strcmp(THREAD_NAME(DT_THREAD(dt)),"robot_wall"))semantics_state("PROPERTIES_ENTRY",dt,time,dtime);\n  l2300_magnetic_6dof(prop,dt,time,dtime);')
    (ROOT/'fluent_udf/l2300_contact_overwrite_semantics.c').write_text(source)
    # Constants define a diagnostic decision before repetition results are known.
    atomic(EVID/'configuration.json',dict(campaign='BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS',timestamp=stamp(),
        authorization='User attachment4f0cadaf',frozen_files_sha256=frozen,start_time_s=.00175,end_time_s=.001875,
        dt_s=25e-6,steps=5,iterations_per_step=2,branch_order=['A1','A2','B1','B2'],
        threshold_m=.0001,mesh_cells=4699301,processor_count=1,precision='double',ui_mode='no_gui_or_graphics',
        no_micro_or_2ms=True,converters_read_only=True,no_theta_conversion_fed_to_overwrite=True,
        same_returned_theta_only=True,physical_controlled_write_requires_separate_admission=True,
        admission=dict(available_gib=12,commit_headroom_gib=19,disk_free_gib=35,stable_seconds=60),
        runtime_guards=dict(available_gib=3,commit_fraction=.95,project_working_set_gib=15,disk_free_gib=15),
        reproducibility_tolerance=dict(com_m=1e-10,orientation_rad=1e-8,velocity_m_s=1e-8,omega_rad_s=1e-6),
        controlled_minimum_signal_to_floor=10,controlled_change_relative_tolerance=.05,
        material_normal_step_fraction=.01,material_artifact_policy='>1% of a normal step needs an explanation; an empirically verified endpoint-pose reconstruction may be acceptable only with>10x intended signal and unchanged motion arrays',
        no_automation_or_new_wall_time_budget=True))
    for p,h in frozen.items():
        if sha(ROOT/p)!=h:raise RuntimeError('Frozen previous source/evidence changed: '+p)
    atomic(EVID/'state.json',dict(campaign='BENCHMARK_D_CONTACT_OVERWRITE_SEMANTICS',status='PREPARED',timestamp=stamp(),worker_alive=False,solver_alive=False))
    print('Isolated overwrite-semantics diagnostics PREPARED')

if __name__=='__main__':main()
