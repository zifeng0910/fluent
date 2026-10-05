/* Actual transient callbacks only. No manual detector or zero-step motion calls. */
#include "udf.h"
#include "dynamesh_tools.h"
#include "six_dof.h"
#include "overset.h"
#include "l2300_contact_impulse_math.h"
#include "lifecycle_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned long callback_index=0;
/* One-rank host/node copies maintain independent identical event caches.
   A cluster retains the validated 20 um centroid radius. Keep ALL applied
   clusters at the current native timestep: A->B->A cannot reapply A. */
#define OVERNIGHT_EVENT_CACHE_CAPACITY 128
#define OVERNIGHT_EVENT_CLUSTER_DISTANCE_SQUARE 4e-10
static struct {int body,wall;double point[3];} applied_events[OVERNIGHT_EVENT_CACHE_CAPACITY];
static int event_cache_step=-1,event_cache_count=0;
static double event_cache_time=-1;
static unsigned long applied_event_total=0;

static void jvec(FILE *f,const char *key,const double *v,int n)
{int i;fprintf(f,"\"%s\":[",key);for(i=0;i<n;i++)fprintf(f,i?",%.17g":"%.17g",v[i]);fprintf(f,"],");}


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

DEFINE_CONTACT(l2300_native_contact_lifecycle,dt,contacts)
{
  Objp *o;Thread *thread=NULL,*opposite=NULL;Dynamic_Thread *ndt=NULL;
  int i,j,k,cache_i,cluster_index=-1,tid=-1,nid=-1,count=0,dt_null=dt==NULL,moving=0,duplicate=0,applied=0,zero=1;
  real A[3],xc[3],sum_n[3]={0,0,0},sum_x[3]={0,0,0},vel[3]={0,0,0},omega[3]={0,0,0},theta[3]={0,0,0},RR[3][3];
  double cg[3]={0,0,0},qlog[4]={0,0,0,0},current_v[3]={0,0,0},current_w[3]={0,0,0},temporary_v[3]={0,0,0},temporary_w[3]={0,0,0};
  double point[3]={0,0,0},normal[3]={0,0,0},r[3]={0,0,0},v[3],w[3],th[3],V[3],W[3],vc[3],cross[3];
  double R[9],inverse_body[9],inverse_world[9],norm=0,J=0,vnb=0,vna=0,distance=0;
  real maximum_radial_square=0;double time=(double)CURRENT_TIME;const char *action="READ_ONLY";FILE *f;int step=N_TIME,iteration=N_ITER;
  ++callback_index;
  if(dt!=NULL){thread=DT_THREAD(dt);tid=THREAD_ID(thread);moving=!strcmp(THREAD_NAME(thread),"robot_wall");}
#if !RP_HOST
  if(moving){face_t body_face;int n;begin_f_loop(body_face,thread){f_node_loop(body_face,thread,n){Node *node=F_NODE(body_face,thread,n);real r2=NODE_Y(node)*NODE_Y(node)+NODE_Z(node)*NODE_Z(node);if(r2>maximum_radial_square)maximum_radial_square=r2;}}end_f_loop(body_face,thread)}
  if(thread!=NULL)loop(o,contacts){
    face_t face=O_F(o);Thread *other=O_F_THREAD(o);
    if(THREAD_ID(other)==tid)continue;
    if(nid==-1)nid=THREAD_ID(other);
    if(THREAD_ID(other)!=nid)continue;
    F_AREA(A,face,other);F_CENTROID(xc,face,other);
    for(i=0;i<3;i++){sum_n[i]-=A[i];sum_x[i]+=xc[i];}++count;
  }
#endif
#if RP_NODE
  maximum_radial_square=PRF_GRHIGH1(maximum_radial_square);
  nid=PRF_GIHIGH1(nid);count=PRF_GISUM1(count);
  PRF_GRSUM3(sum_n[0],sum_n[1],sum_n[2]);PRF_GRSUM3(sum_x[0],sum_x[1],sum_x[2]);
#endif
  node_to_host_int_2(nid,count);node_to_host_real_3(sum_n[0],sum_n[1],sum_n[2]);node_to_host_real_3(sum_x[0],sum_x[1],sum_x[2]);
  node_to_host_real_1(maximum_radial_square);
  if(thread!=NULL && nid>=0){opposite=Lookup_Thread(THREAD_DOMAIN(thread),nid);if(opposite!=NULL)ndt=THREAD_DT(opposite);}
  if(count>0){norm=sqrt(contact_dot(sum_n,sum_n));if(!(norm>0))Error("D_LIFECYCLE_INVALID_NORMAL\n");
    for(i=0;i<3;i++){point[i]=sum_x[i]/count;normal[i]=sum_n[i]/norm;}}
  if(dt!=NULL){
    for(i=0;i<3;i++){cg[i]=DT_CG(dt)[i];current_v[i]=DT_VEL_CG(dt)[i];current_w[i]=DT_OMEGA_CG(dt)[i];temporary_v[i]=DT_RB_TMP_STATE(dt).v_cg[i];temporary_w[i]=DT_RB_TMP_STATE(dt).omega_cg[i];}
    for(i=0;i<4;i++)qlog[i]=DT_Q(dt)[i];
    /* Stationary opposite wall is allowed to have no Dynamic_Thread. */
    if(moving)SDOF_Get_Motion(dt,vel,omega,theta);
  }
  for(i=0;i<3;i++){v[i]=vel[i];w[i]=omega[i];th[i]=theta[i];V[i]=v[i];W[i]=w[i];r[i]=point[i]-cg[i];if(v[i]!=0 || w[i]!=0)zero=0;}
  contact_cross(w,r,cross);for(i=0;i<3;i++)vc[i]=v[i]+cross[i];vnb=contact_dot(vc,normal);vna=vnb;
  if(moving && count>0 && opposite!=NULL && !strcmp(THREAD_NAME(opposite),"pipe_wall")){
    if(!Data_Valid_P())Error("D_LIFECYCLE_DATA_INVALID\n");
    for(i=0;i<3;i++)if(!isfinite(v[i])||!isfinite(w[i])||!isfinite(th[i]))Error("D_LIFECYCLE_NONFINITE_MOTION\n");
    if(normal[1]*(-point[1])+normal[2]*(-point[2])<=0)Error("D_LIFECYCLE_NORMAL_WRONG_DIRECTION\n");
    if(step!=event_cache_step || fabs(time-event_cache_time)>=1e-13){
      event_cache_step=step;event_cache_time=time;event_cache_count=0;
    }
    for(cache_i=0;cache_i<event_cache_count;cache_i++){
      if(tid!=applied_events[cache_i].body || nid!=applied_events[cache_i].wall)continue;
      distance=0;
      for(i=0;i<3;i++)distance+=(point[i]-applied_events[cache_i].point[i])*(point[i]-applied_events[cache_i].point[i]);
      if(distance<=OVERNIGHT_EVENT_CLUSTER_DISTANCE_SQUARE){duplicate=1;cluster_index=cache_i;break;}
    }
    semantics_state("CALLBACK_BEFORE",dt,CURRENT_TIME,RP_Get_Real("physical-time-step"));
    if(LIFECYCLE_MODE==1){
      /* Diagnostic no-op: exactly the original arrays, never a reflection. */
      SDOF_Overwrite_Motion(dt,vel,omega,theta);applied=1;action="NOOP_OVERWRITE";
    }else if(LIFECYCLE_MODE>=2){
      if(duplicate){action="SKIPPED_DUPLICATE";}
      else if(LIFECYCLE_MODE==2 && applied_event_total>0){action="CONTROLLED_EVENT_ALREADY_TESTED";}
      else {
        quaternion q;Q_Assign(DT_Q(dt),&q);Rot_Matrix_From_Q(q,RR);
        for(i=0;i<3;i++)for(j=0;j<3;j++)R[3*i+j]=RR[i][j];
        {double a=7.257810693523445e-13,b=3.929384741151496e-12,c=3.9293847411514897e-12;
         double d=2.782693607479792e-28,e=-1.9327591150313555e-29,ff=-2.4813784672159598e-29;
         double determinant=a*(b*c-ff*ff)-d*(d*c-e*ff)+e*(d*ff-b*e);
         inverse_body[0]=(b*c-ff*ff)/determinant;inverse_body[1]=(e*ff-d*c)/determinant;inverse_body[2]=(d*ff-e*b)/determinant;
         inverse_body[3]=inverse_body[1];inverse_body[4]=(a*c-e*e)/determinant;inverse_body[5]=(d*e-a*ff)/determinant;
         inverse_body[6]=inverse_body[2];inverse_body[7]=inverse_body[5];inverse_body[8]=(a*b-d*d)/determinant;}
        contact_world_inverse(R,inverse_body,inverse_world);
        if(contact_impulse(8.904428864007322e-6,inverse_world,r,normal,v,w,V,W,&J,&vnb,&vna)<0)Error("D_LIFECYCLE_IMPULSE_INVALID\n");
        if(J>0){
          if(!isfinite(J)||fabs(vna)>1e-9)Error("D_LIFECYCLE_NORMAL_IMPULSE_GATE\n");
          if(event_cache_count>=OVERNIGHT_EVENT_CACHE_CAPACITY)Error("D_OVERNIGHT_EVENT_CACHE_OVERFLOW\n");
          for(i=0;i<3;i++){vel[i]=V[i];omega[i]=W[i];}
          SDOF_Overwrite_Motion(dt,vel,omega,theta);
          /* No immediate Get_Motion PASS: worker audits completed timestep. */
          applied=1;action="APPLIED";cluster_index=event_cache_count++;
          applied_events[cluster_index].body=tid;applied_events[cluster_index].wall=nid;
          for(i=0;i<3;i++)applied_events[cluster_index].point[i]=point[i];
          ++applied_event_total;
        }else action="SEPARATING_NO_IMPULSE";
      }
    }
  }
  if(moving && count>0)semantics_state("CALLBACK_AFTER",dt,CURRENT_TIME,RP_Get_Real("physical-time-step"));
  f=fopen(RP_HOST?TRACE_HOST_JSON:TRACE_NODE_JSON,"a");if(!f)Error("D_LIFECYCLE_TRACE_OPEN_FAILED\n");
  fprintf(f,"{\"callback_index\":%lu,\"CURRENT_TIME\":%.17g,\"timestep_index\":%d,\"iteration_index\":%d,\"process_role\":\"%s\",\"dt_pointer\":\"%p\",\"dt_NULL\":%s,\"moving_body\":%s,\"dynamic_zone_id\":%d,\"opposite_zone_id\":%d,\"opposite_ndt_NULL\":%s,\"contact_face_count\":%d,",callback_index,time,step,iteration,RP_HOST?"host":"node",(void*)dt,dt_null?"true":"false",moving?"true":"false",tid,nid,ndt==NULL?"true":"false",count);
  fprintf(f,"\"event_cache_cluster_index\":%d,\"event_cache_count\":%d,\"event_cache_capacity\":%d,\"event_cluster_radius_m\":%.17g,",cluster_index,event_cache_count,OVERNIGHT_EVENT_CACHE_CAPACITY,2e-5);
  jvec(f,"contact_point",point,3);jvec(f,"contact_normal",normal,3);jvec(f,"DT_CG",cg,3);jvec(f,"DT_Q",qlog,4);
  jvec(f,"get_velocity",v,3);jvec(f,"get_omega",w,3);jvec(f,"get_theta",th,3);
  jvec(f,"native_current_velocity",current_v,3);jvec(f,"native_current_omega",current_w,3);
  jvec(f,"native_temporary_velocity",temporary_v,3);jvec(f,"native_temporary_omega",temporary_w,3);
  jvec(f,"contact_velocity_before",vc,3);jvec(f,"requested_velocity",V,3);jvec(f,"requested_omega",W,3);
  fprintf(f,"\"motion_all_zero\":%s,\"mode\":%d,\"action\":\"%s\",\"overwrite_called\":%s,\"normal_velocity_before\":%.17g,\"impulse_N_s\":%.17g,\"normal_velocity_after\":%.17g,\"contact_active\":%d,\"motion_tmp_cg_null\":%s,\"motion_tmp_theta_null\":%s,\"ideal_tube_gap_from_actual_vertices_m\":%.17g}\n",zero?"true":"false",LIFECYCLE_MODE,action,applied?"true":"false",vnb,J,vna,(int)contact_active_p,dt==NULL||dt->tmp_cg==NULL?"true":"false",dt==NULL||dt->tmp_theta==NULL?"true":"false",.0009-sqrt(maximum_radial_square));fclose(f);
  f=fopen(RP_HOST?TRACE_HOST_CSV:TRACE_NODE_CSV,"a+");if(!f)Error("D_LIFECYCLE_CSV_OPEN_FAILED\n");
  fseek(f,0,SEEK_END);if(ftell(f)==0)fprintf(f,"callback_index,CURRENT_TIME,timestep,iteration,role,dt_pointer,dt_NULL,body,opposite,opposite_ndt_NULL,n_faces,point_x,point_y,point_z,n_x,n_y,n_z,cg_x,cg_y,cg_z,q0,q1,q2,q3,vx,vy,vz,wx,wy,wz,theta_x,theta_y,theta_z,vn_before,J,vn_after,all_zero,action\n");
  fprintf(f,"%lu,%.17g,%d,%d,%s,%p,%d,%d,%d,%d,%d",callback_index,time,step,iteration,RP_HOST?"host":"node",(void*)dt,dt_null,tid,nid,ndt==NULL,count);
  {const double *arrays[7]={point,normal,cg,qlog,v,w,th};for(k=0;k<7;k++)for(i=0;i<(k==3?4:3);i++)fprintf(f,",%.17g",arrays[k][i]);}
  fprintf(f,",%.17g,%.17g,%.17g,%d,%s\n",vnb,J,vna,zero,action);fclose(f);
}

/* Wrapper delegates original magnetic properties exactly once. Additional code only logs. */
extern void l2300_magnetic_6dof(real *,Dynamic_Thread *,real,real);
DEFINE_SDOF_PROPERTIES(lifecycle_magnetic_state_probe,prop,dt,time,dtime)
{
  double v[3],w[3],q[4],cg[3];int i;FILE *f;
  if(!strcmp(THREAD_NAME(DT_THREAD(dt)),"robot_wall"))semantics_state("PROPERTIES_ENTRY",dt,time,dtime);
  l2300_magnetic_6dof(prop,dt,time,dtime);
  if(strcmp(THREAD_NAME(DT_THREAD(dt)),"robot_wall"))return;
  for(i=0;i<3;i++){v[i]=DT_VEL_CG(dt)[i];w[i]=DT_OMEGA_CG(dt)[i];cg[i]=DT_CG(dt)[i];}for(i=0;i<4;i++)q[i]=DT_Q(dt)[i];
  f=fopen(RP_HOST?STATE_HOST_JSON:STATE_NODE_JSON,"a");if(!f)Error("D_LIFECYCLE_STATE_LOG_OPEN_FAILED\n");
  fprintf(f,"{\"CURRENT_TIME\":%.17g,\"property_time\":%.17g,\"dtime\":%.17g,\"timestep_index\":%d,\"iteration_index\":%d,\"process_role\":\"%s\",",(double)CURRENT_TIME,(double)time,(double)dtime,N_TIME,N_ITER,RP_HOST?"host":"node");
  jvec(f,"velocity",v,3);jvec(f,"omega",w,3);jvec(f,"quaternion",q,4);jvec(f,"COM",cg,3);
  fprintf(f,"\"magnetic_formula_delegated_once\":true}\n");fclose(f);
}

DEFINE_ON_DEMAND(lifecycle_connectivity)
{
#if !RP_HOST
  Domain *d=Get_Domain(1);Thread *t;cell_t c;FILE *f=fopen(CONNECTIVITY_JSON,"w");int first=1;
  if(!f)Error("D_LIFECYCLE_CONNECTIVITY_OPEN_FAILED\n");
  fprintf(f,"{\"time_s\":%.17g,\"zones\":[",(double)CURRENT_TIME);
  thread_loop_overset_c(t,d){long total=0,orphan=0,receptor=0,missing=0,unidentified=0,nonpositive=0,nonfinite=0;real minv=1e300;
    begin_c_loop_int(c,t){++total;if(OVERSET_ORPHAN_CELL_P(c,t))++orphan;
      if(OVERSET_UNIDENTIFIED_CELL_P(c,t))++unidentified;if(C_VOLUME(c,t)<=0)++nonpositive;if(!isfinite(C_VOLUME(c,t)))++nonfinite;
      if(OVERSET_RECEPTOR_CELL_P(c,t)){++receptor;if(C_OVERSET_NDONOR(c,t)<=0)++missing;}
      if(C_VOLUME(c,t)<minv)minv=C_VOLUME(c,t);
    }end_c_loop_int(c,t)
    fprintf(f,"%s{\"name\":\"%s\",\"total\":%ld,\"orphan\":%ld,\"receptors\":%ld,\"invalid_donors\":%ld,\"unidentified\":%ld,\"nonpositive_volume\":%ld,\"nonfinite_volume\":%ld,\"minimum_volume_m3\":%.17g}",first?"":",",THREAD_NAME(t),total,orphan,receptor,missing,unidentified,nonpositive,nonfinite,(double)minv);first=0;
  }fprintf(f,"]}");fclose(f);
#endif
}
