/* Conditional analytic normal contact through native SDOF loads only.
 * Magnetic properties delegate exactly once. No motion setter or state reset.
 * Geometry is the actual canonical triangulated robot, stored about its COM.
 */
#include "udf.h"
#include "dynamesh_tools.h"
#include "six_dof.h"
#include "overset.h"
#include "l2300_contact_impulse_math.h"
#include "lifecycle_config.h"
#include "load_contact_geometry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned long load_call_index=0,load_event_total=LOAD_INITIAL_EVENT_TOTAL;
static int interval_step=-1,interval_valid=0;
static double interval_time=-1,interval_dt=0,interval_J=0;
static double load_F[3]={0,0,0},load_T[3]={0,0,0};
static double saved_v[3],saved_w[3],saved_V[3],saved_W[3],saved_cg[3],saved_q[4];
static double saved_point[3],saved_normal[3],saved_contact_velocity[3];
static double gap_now=0,gap_next=0,vn_before=0,vn_after=0,effective_mass=0;

static void jvec(FILE *f,const char *key,const double *v,int n)
{int i;fprintf(f,"\"%s\":[",key);for(i=0;i<n;i++)fprintf(f,i?",%.17g":"%.17g",v[i]);fprintf(f,"],");}

static void world_inverse(Dynamic_Thread *dt,double R[9],double inverse[9])
{
  int i,j;real RR[3][3];quaternion q;double B[9];
  double a=7.257810693523445e-13,b=3.929384741151496e-12,c=3.9293847411514897e-12;
  double d=2.782693607479792e-28,e=-1.9327591150313555e-29,ff=-2.4813784672159598e-29;
  double determinant=a*(b*c-ff*ff)-d*(d*c-e*ff)+e*(d*ff-b*e);
  Q_Assign(DT_Q(dt),&q);Rot_Matrix_From_Q(q,RR);
  for(i=0;i<3;i++)for(j=0;j<3;j++)R[3*i+j]=RR[i][j];
  B[0]=(b*c-ff*ff)/determinant;B[1]=(e*ff-d*c)/determinant;B[2]=(d*ff-e*b)/determinant;
  B[3]=B[1];B[4]=(a*c-e*e)/determinant;B[5]=(d*e-a*ff)/determinant;
  B[6]=B[2];B[7]=B[5];B[8]=(a*b-d*d)/determinant;
  contact_world_inverse(R,B,inverse);
}

/* Finite world-axis rotation predicts the next pose without assigning it. */
static void rotate_increment(const double *x,const double *w,double dt,double *out)
{
  int i;double length=sqrt(contact_dot(w,w)),a=length*dt,k[3],cross[3],dot;
  if(length<1e-20){for(i=0;i<3;i++)out[i]=x[i];return;}
  for(i=0;i<3;i++)k[i]=w[i]/length;contact_cross(k,x,cross);dot=contact_dot(k,x);
  for(i=0;i<3;i++)out[i]=x[i]*cos(a)+cross[i]*sin(a)+k[i]*dot*(1-cos(a));
}

static void compute_interval(Dynamic_Thread *dt,double dtime)
{
  int i,p,index=0;double R[9],inverse[9],r[3],rnext[3],point[3],radial2,max2=-1,maxnext=-1;
  double lever[3],angular[3],denominator,cross[3],J=0;
  if(!(dtime>0) || !isfinite(dtime))Error("D_LOAD_INVALID_DT\n");
  world_inverse(dt,R,inverse);
  for(i=0;i<3;i++){saved_cg[i]=DT_CG(dt)[i];saved_v[i]=DT_VEL_CG(dt)[i];saved_w[i]=DT_OMEGA_CG(dt)[i];load_F[i]=load_T[i]=0;}
  for(i=0;i<4;i++)saved_q[i]=DT_Q(dt)[i];
  for(p=0;p<LOAD_BODY_POINT_COUNT;p++){
    contact_matvec(R,load_body_points[p],r);
    for(i=0;i<3;i++)point[i]=saved_cg[i]+r[i];
    radial2=point[1]*point[1]+point[2]*point[2];if(radial2>max2)max2=radial2;
    rotate_increment(r,saved_w,dtime,rnext);
    for(i=0;i<3;i++)point[i]=saved_cg[i]+saved_v[i]*dtime+rnext[i];
    radial2=point[1]*point[1]+point[2]*point[2];if(radial2>maxnext){maxnext=radial2;index=p;}
  }
  gap_now=LOAD_TUBE_RADIUS_M-sqrt(max2);gap_next=LOAD_TUBE_RADIUS_M-sqrt(maxnext);
  contact_matvec(R,load_body_points[index],r);
  for(i=0;i<3;i++)saved_point[i]=saved_cg[i]+r[i];
  radial2=saved_point[1]*saved_point[1]+saved_point[2]*saved_point[2];
  if(!(radial2>0))Error("D_LOAD_INVALID_CONTACT_NORMAL\n");
  saved_normal[0]=0;saved_normal[1]=-saved_point[1]/sqrt(radial2);saved_normal[2]=-saved_point[2]/sqrt(radial2);
  contact_cross(saved_w,r,cross);for(i=0;i<3;i++)saved_contact_velocity[i]=saved_v[i]+cross[i];
  vn_before=contact_dot(saved_contact_velocity,saved_normal);vn_after=vn_before;interval_J=0;
  for(i=0;i<3;i++){saved_V[i]=saved_v[i];saved_W[i]=saved_w[i];}
  contact_cross(r,saved_normal,lever);contact_matvec(inverse,lever,angular);
  denominator=1/8.904428864007322e-6+contact_dot(lever,angular);
  if(!(denominator>0))Error("D_LOAD_INVALID_EFFECTIVE_MASS\n");effective_mass=1/denominator;
  /* A positive precontact gap alone is never a load trigger. */
  if(LIFECYCLE_MODE>0 && gap_next<=0 && vn_before<0 && !(LIFECYCLE_MODE==2 && load_event_total>0)){
    if(LOAD_CONTACT_MODEL==0){
      if(contact_impulse(8.904428864007322e-6,inverse,r,saved_normal,saved_v,saved_w,saved_V,saved_W,&J,&vn_before,&vn_after)<0)
        Error("D_LOAD_IMPULSE_INVALID\n");
      if(!isfinite(J)||fabs(vn_after)>1e-9)Error("D_LOAD_NORMAL_IMPULSE_GATE\n");
    }else{
      /* This conditional numerical compliance is armed only after measured
         impulse-load instability. k/c come from the archived m_eff/vn/target
         penetration and critical damping, never a guessed wall barrier. */
      double delta=gap_next<0?-gap_next:0;
      double Fn=LOAD_STIFFNESS_N_M*delta+LOAD_DAMPING_N_S_M*(-vn_before);
      if(Fn<0)Fn=0;J=Fn*dtime;
      for(i=0;i<3;i++){saved_V[i]=saved_v[i]+J*saved_normal[i]/8.904428864007322e-6;saved_W[i]=saved_w[i]+J*angular[i];}
      contact_cross(saved_W,r,cross);for(i=0;i<3;i++)point[i]=saved_V[i]+cross[i];vn_after=contact_dot(point,saved_normal);
      if(!isfinite(J)||!(LOAD_STIFFNESS_N_M>0)||!(LOAD_DAMPING_N_S_M>=0))Error("D_LOAD_COMPLIANT_PARAMETER_GATE\n");
    }
    interval_J=J;
    for(i=0;i<3;i++)load_F[i]=J*saved_normal[i]/dtime;
    contact_cross(r,load_F,load_T);
    if(J>0)++load_event_total;
  }
  for(i=0;i<3;i++)if(!isfinite(load_F[i])||!isfinite(load_T[i])||!isfinite(saved_v[i])||!isfinite(saved_w[i]))Error("D_LOAD_NONFINITE_STATE\n");
}

static void log_properties(Dynamic_Thread *dt,real time,real dtime)
{
  double v[3],w[3],cg[3],q[4];int i;FILE *f;
  for(i=0;i<3;i++){v[i]=DT_VEL_CG(dt)[i];w[i]=DT_OMEGA_CG(dt)[i];cg[i]=DT_CG(dt)[i];}for(i=0;i<4;i++)q[i]=DT_Q(dt)[i];
  f=fopen(RP_HOST?SEMANTICS_HOST_JSON:SEMANTICS_NODE_JSON,"a");if(!f)Error("D_LOAD_STATE_LOG_OPEN_FAILED\n");
  fprintf(f,"{\"stage\":\"PROPERTIES_ENTRY\",\"CURRENT_TIME\":%.17g,\"property_time\":%.17g,\"dtime\":%.17g,\"timestep_index\":%d,",(double)CURRENT_TIME,(double)time,(double)dtime,N_TIME);
  jvec(f,"velocity",v,3);jvec(f,"omega",w,3);jvec(f,"CG",cg,3);jvec(f,"Q",q,4);fprintf(f,"\"no_motion_overwrite\":true}\n");fclose(f);
}

extern void l2300_magnetic_6dof(real *,Dynamic_Thread *,real,real);
DEFINE_SDOF_PROPERTIES(lifecycle_magnetic_state_probe,prop,dt,time,dtime)
{
  int i,new_interval,physical_applied;FILE *f;const char *action;
  l2300_magnetic_6dof(prop,dt,time,dtime);
  if(strcmp(THREAD_NAME(DT_THREAD(dt)),"robot_wall"))return;
  log_properties(dt,time,dtime);++load_call_index;
  new_interval=!interval_valid || N_TIME!=interval_step || fabs((double)CURRENT_TIME-interval_time)>1e-13;
  if(new_interval){
    interval_step=N_TIME;interval_time=(double)CURRENT_TIME;interval_dt=(double)dtime;interval_valid=1;
    compute_interval(dt,(double)dtime);
  }else if(fabs((double)dtime-interval_dt)>1e-13)Error("D_LOAD_DT_CHANGED_INSIDE_INTERVAL\n");
  if(prop[SDOF_LOAD_LOCAL]!=FALSE)Error("D_LOAD_GLOBAL_COORDINATE_REQUIRED\n");
  /* Re-evaluations return the SAME total load, never accumulate it. The
     original magnetic delegate has reset its load slots on every call. */
  for(i=0;i<3;i++){prop[SDOF_LOAD_F_X+i]+=load_F[i];prop[SDOF_LOAD_M_X+i]+=load_T[i];}
  physical_applied=new_interval && interval_J>0;
  action=physical_applied?"APPLIED":(interval_J>0?"CACHED_LOAD_REUSED":(LIFECYCLE_MODE==0?"LOAD_DISABLED_REFERENCE":"NO_PREDICTED_APPROACH_CONTACT"));
  f=fopen(RP_HOST?TRACE_HOST_JSON:TRACE_NODE_JSON,"a");if(!f)Error("D_LOAD_EVENT_LOG_OPEN_FAILED\n");
  fprintf(f,"{\"callback_index\":%lu,\"CURRENT_TIME\":%.17g,\"property_time\":%.17g,\"dtime\":%.17g,\"timestep_index\":%d,\"iteration_index\":%d,\"process_role\":\"%s\",\"moving_body\":true,\"dt_NULL\":false,\"dynamic_zone_id\":%d,\"opposite_zone_id\":-1,\"contact_face_count\":0,\"analytic_contact_point_count\":1,\"contact_geometry\":\"EXACT_CANONICAL_SURFACE_IDEAL_CYLINDER\",\"route\":\"%s\",\"motion_value_source\":\"NATIVE_DT_FIELDS_NOT_GET_MOTION\",\"mode\":%d,\"action\":\"%s\",\"overwrite_called\":false,\"physical_impulse_applied\":%s,",load_call_index,(double)CURRENT_TIME,(double)time,(double)dtime,N_TIME,N_ITER,RP_HOST?"host":"node",THREAD_ID(DT_THREAD(dt)),LOAD_CONTACT_MODEL==0?"SDOF_LOAD_PATH":"COMPLIANT_LOAD_PATH",LIFECYCLE_MODE,action,physical_applied?"true":"false");
  jvec(f,"DT_CG",saved_cg,3);jvec(f,"DT_Q",saved_q,4);jvec(f,"get_velocity",saved_v,3);jvec(f,"get_omega",saved_w,3);
  jvec(f,"contact_point",saved_point,3);jvec(f,"contact_normal",saved_normal,3);jvec(f,"contact_velocity_before",saved_contact_velocity,3);
  jvec(f,"requested_velocity",saved_V,3);jvec(f,"requested_omega",saved_W,3);jvec(f,"Fcontact_N",load_F,3);jvec(f,"Tcontact_COM_Nm",load_T,3);
  fprintf(f,"\"gap_current_m\":%.17g,\"gap_predicted_m\":%.17g,\"effective_contact_mass_kg\":%.17g,\"normal_velocity_before\":%.17g,\"normal_velocity_after\":%.17g,\"impulse_N_s\":%.17g,\"load_interval_J_N_s\":%.17g,\"ideal_tube_gap_from_actual_vertices_m\":%.17g,\"magnetic_formula_delegated_once\":true,\"event_load_total\":%lu,\"load_contact_model\":%d,\"stiffness_N_m\":%.17g,\"damping_N_s_m\":%.17g}\n",gap_now,gap_next,effective_mass,vn_before,vn_after,physical_applied?interval_J:0,interval_J,gap_now,load_event_total,LOAD_CONTACT_MODEL,(double)LOAD_STIFFNESS_N_M,(double)LOAD_DAMPING_N_S_M);fclose(f);
}

DEFINE_ON_DEMAND(overwrite_semantics_snapshot)
{
  Domain *d=Get_Domain(1);Thread *t;double cg[3],q[4],v[3],w[3];FILE *f;int i;
  thread_loop_f(t,d){if(!strcmp(THREAD_NAME(t),"robot_wall")){
    Dynamic_Thread *dt=THREAD_DT(t);if(dt==NULL)Error("D_LOAD_NO_NATIVE_BODY\n");
    for(i=0;i<3;i++){cg[i]=DT_CG(dt)[i];v[i]=DT_VEL_CG(dt)[i];w[i]=DT_OMEGA_CG(dt)[i];}for(i=0;i<4;i++)q[i]=DT_Q(dt)[i];
    f=fopen(RP_HOST?SEMANTICS_HOST_JSON:SEMANTICS_NODE_JSON,"a");if(!f)Error("D_LOAD_BOUNDARY_LOG_OPEN_FAILED\n");
    fprintf(f,"{\"stage\":\"BOUNDARY_SNAPSHOT\",\"CURRENT_TIME\":%.17g,\"property_time\":%.17g,\"timestep_index\":%d,",(double)CURRENT_TIME,(double)CURRENT_TIME,N_TIME);
    jvec(f,"CG",cg,3);jvec(f,"Q",q,4);jvec(f,"velocity",v,3);jvec(f,"omega",w,3);fprintf(f,"\"no_motion_overwrite\":true}\n");fclose(f);break;
  }}
}

DEFINE_ON_DEMAND(lifecycle_connectivity)
{
#if !RP_HOST
  Domain *d=Get_Domain(1);Thread *t;cell_t c;FILE *f=fopen(CONNECTIVITY_JSON,"w");int first=1;
  if(!f)Error("D_LOAD_CONNECTIVITY_OPEN_FAILED\n");fprintf(f,"{\"time_s\":%.17g,\"zones\":[",(double)CURRENT_TIME);
  thread_loop_overset_c(t,d){long total=0,orphan=0,receptor=0,missing=0,unidentified=0,nonpositive=0,nonfinite=0;real minv=1e300;
    begin_c_loop_int(c,t){++total;if(OVERSET_ORPHAN_CELL_P(c,t))++orphan;
      if(OVERSET_UNIDENTIFIED_CELL_P(c,t))++unidentified;if(C_VOLUME(c,t)<=0)++nonpositive;if(!isfinite(C_VOLUME(c,t)))++nonfinite;
      if(OVERSET_RECEPTOR_CELL_P(c,t)){++receptor;if(C_OVERSET_NDONOR(c,t)<=0)++missing;}if(C_VOLUME(c,t)<minv)minv=C_VOLUME(c,t);
    }end_c_loop_int(c,t)
    fprintf(f,"%s{\"name\":\"%s\",\"total\":%ld,\"orphan\":%ld,\"receptors\":%ld,\"invalid_donors\":%ld,\"unidentified\":%ld,\"nonpositive_volume\":%ld,\"nonfinite_volume\":%ld,\"minimum_volume_m3\":%.17g}",first?"":",",THREAD_NAME(t),total,orphan,receptor,missing,unidentified,nonpositive,nonfinite,(double)minv);first=0;
  }fprintf(f,"]}");fclose(f);
#endif
}
