/* Read-only native motion-state diagnostic. Never overwrites or integrates motion. */
#include "udf.h"
#include "dynamesh_tools.h"
#include "six_dof.h"
#include <stdio.h>
#include <string.h>

#if RP_HOST
#define AUDIT_LOG "H:\\fluent\\evidence\\benchmark_D_contact_baseline\\native_motion_state_host.jsonl"
#else
#define AUDIT_LOG "H:\\fluent\\evidence\\benchmark_D_contact_baseline\\native_motion_state_node.jsonl"
#endif
static void vector(FILE *fp,const char *name,const real *x,int size)
{int i;fprintf(fp,"\"%s\":[",name);for(i=0;i<size;i++)fprintf(fp,i?",%.17g":"%.17g",(double)x[i]);fprintf(fp,"],");}

DEFINE_ON_DEMAND(benchmark_D_read_native_motion)
{
  Domain *d=Get_Domain(1);Dynamic_Thread *dt;FILE *fp;
  for(dt=d->dynamic_threads;dt;dt=dt->next){
    real vel[3],omega[3],theta[3];Dynamic_Thread_Rigid_Body_State *tmp=&DT_RB_TMP_STATE(dt);
    const char *name=THREAD_NAME(DT_THREAD(dt));
    if(strcmp(name,"robot_wall")&&strcmp(name,"robot_component_fluid"))continue;
    SDOF_Get_Motion(dt,vel,omega,theta);
    fp=fopen(AUDIT_LOG,"a");if(!fp)Error("D_MOTION_AUDIT_LOG_OPEN_FAILED\n");
    fprintf(fp,"{\"time_s\":%.17g,\"process_role\":\"%s\",\"zone\":\"%s\",",(double)CURRENT_TIME,RP_HOST?"host":"node",name);
    vector(fp,"get_motion_velocity",vel,3);vector(fp,"get_motion_omega",omega,3);
    vector(fp,"get_motion_theta",theta,3);vector(fp,"current_COM",DT_CG(dt),3);
    vector(fp,"current_quaternion",DT_Q(dt),4);vector(fp,"current_velocity",DT_VEL_CG(dt),3);
    vector(fp,"current_omega",DT_OMEGA_CG(dt),3);vector(fp,"temporary_velocity",tmp->v_cg,3);
    vector(fp,"temporary_omega",tmp->omega_cg,3);
    fprintf(fp,"\"contact_active\":%d,\"contact_mark\":%d,\"six_dof_enabled\":%d,\"motion_tmp_cg_null\":%d,\"motion_tmp_theta_null\":%d,\"timesteps_advanced\":0}\n",(int)contact_active_p,(int)DT_CONTACT_P(dt),(int)DT_SDOF_P(dt),dt->tmp_cg==NULL,dt->tmp_theta==NULL);fclose(fp);
  }
}
