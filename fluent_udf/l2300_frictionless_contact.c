/* Benchmark D: native contact geometry and e_n=0, mu=0 impulse.
 * Default is detection-only. No magnetic or fluid force is added here.
 * The production response remains disabled until static semantic gates PASS.
 */
#include "udf.h"
#include "dynamesh_tools.h"
/* Installed dynamesh_contact.h includes a missing Prime internal header.
 * Use its exact exported prototypes; no Prime data types are used here. */
FLUENT_EXPORT void Init_Contact(Domain *);
FLUENT_EXPORT void Detect_Contact(Domain *, real, real, cxboolean);
FLUENT_EXPORT void Download_Contact_Parameters(void);
#include "six_dof.h"
#include "metric.h"
#include "l2300_contact_impulse_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CONTACT_LOG "H:\\fluent\\evidence\\benchmark_D_contact_baseline\\contact_events.jsonl"
#define CONTACT_CSV "H:\\fluent\\evidence\\benchmark_D_contact_baseline\\contact_events.csv"
#define PROBE_CONTROL "H:\\fluent\\live_cases\\benchmark_D_contact_baseline\\probe_control.txt"
static int response_enabled=0, sanity_enabled=0;
static double last_impulse_time=-1;
static int last_wall=-1;

static void json_vector(FILE *fp,const char *key,const double *v,int size)
{ int i;fprintf(fp,"\"%s\":[",key);for(i=0;i<size;i++)fprintf(fp,i?",%.17g":"%.17g",v[i]);fprintf(fp,"],"); }

DEFINE_CONTACT(l2300_frictionless_contact, dt, contacts)
{
  Objp *item; int count=0, tid, nid=-1, i,j,k, result=0, duplicate=0;
  real sum_normal[3]={0,0,0}, sum_point[3]={0,0,0}, area[3],centroid[3];
  double point[3],normal[3],r[3],V[3],W[3],v[3],w[3],cg[3],qlog[4],Rflat[9],inverse_world[9];
  /* SDOF products are essentially zero but are retained in inverse tensor. */
  double body_inverse[9];
  double magnitude,J=0,before=0,after=0,time=(double)CURRENT_TIME;
  real vel[3],omega[3],theta[3],R[3][3],verify_v[3],verify_w[3],verify_theta[3];quaternion q;
  Thread *wall=DT_THREAD(dt);FILE *fp;
  if(strcmp(THREAD_NAME(wall),"robot_wall")!=0) return;
  tid=THREAD_ID(wall);
#if !RP_HOST
  loop(item,contacts) {
    face_t f=O_F(item);Thread *t=O_F_THREAD(item);
    if(THREAD_ID(t)==tid || strcmp(THREAD_NAME(t),"pipe_wall")!=0)continue;
    nid=THREAD_ID(t);F_AREA(area,f,t);F_CENTROID(centroid,f,t);
    for(i=0;i<3;i++){sum_normal[i]-=area[i];sum_point[i]+=centroid[i];}count++;
  }
#endif
#if RP_NODE
  count=PRF_GISUM1(count);nid=PRF_GIHIGH1(nid);
  PRF_GRSUM3(sum_normal[0],sum_normal[1],sum_normal[2]);
  PRF_GRSUM3(sum_point[0],sum_point[1],sum_point[2]);
#endif
  node_to_host_int_2(count,nid);
  node_to_host_real_3(sum_normal[0],sum_normal[1],sum_normal[2]);
  node_to_host_real_3(sum_point[0],sum_point[1],sum_point[2]);
  if(count<=0)return;
  magnitude=sqrt(contact_dot(sum_normal,sum_normal));
  if(!(magnitude>0))Error("D_CONTACT_NORMAL_UNDEFINED\n");
  SDOF_Get_Motion(dt,vel,omega,theta);Q_Assign(DT_Q(dt),&q);Rot_Matrix_From_Q(q,R);
  for(i=0;i<3;i++) {
    normal[i]=sum_normal[i]/magnitude;point[i]=sum_point[i]/count;
    cg[i]=DT_CG(dt)[i];r[i]=point[i]-cg[i];v[i]=vel[i];w[i]=omega[i];
    for(j=0;j<3;j++)Rflat[3*i+j]=R[i][j];
  }
  for(i=0;i<4;i++)qlog[i]=q[i];
  /* Replace diagonal inverse with the exact full CAD tensor inverse. */
  {
    double a=7.257810693523445e-13,b=3.929384741151496e-12,c=3.9293847411514897e-12;
    double d=2.782693607479792e-28,e=-1.9327591150313555e-29,f=-2.4813784672159598e-29;
    double determinant=a*(b*c-f*f)-d*(d*c-e*f)+e*(d*f-b*e);
    body_inverse[0]=(b*c-f*f)/determinant;body_inverse[1]=(e*f-d*c)/determinant;body_inverse[2]=(d*f-e*b)/determinant;
    body_inverse[3]=body_inverse[1];body_inverse[4]=(a*c-e*e)/determinant;body_inverse[5]=(d*e-a*f)/determinant;
    body_inverse[6]=body_inverse[2];body_inverse[7]=body_inverse[5];body_inverse[8]=(a*b-d*d)/determinant;
  }
  contact_world_inverse(Rflat,body_inverse,inverse_world);
  if(sanity_enabled) { for(i=0;i<3;i++){v[i]=-.01*normal[i];w[i]=0;} }
  result=contact_impulse(8.904428864007322e-6,inverse_world,r,normal,v,w,V,W,&J,&before,&after);
  duplicate=fabs(time-last_impulse_time)<1e-13 && nid==last_wall;
  if(result<0 || !isfinite(J) || !isfinite(after))Error("D_CONTACT_NONFINITE_IMPULSE\n");
  if((response_enabled || sanity_enabled) && !duplicate && J>0) {
    if(normal[1]*(-point[1])+normal[2]*(-point[2])<=0)Error("D_CONTACT_NORMAL_WRONG_DIRECTION\n");
    if(after < -1e-10)Error("D_CONTACT_NORMAL_VELOCITY_INWARD\n");
    for(i=0;i<3;i++){vel[i]=V[i];omega[i]=W[i];}
    SDOF_Overwrite_Motion(dt,vel,omega,theta);
    SDOF_Get_Motion(dt,verify_v,verify_w,verify_theta);
    for(i=0;i<3;i++)if(fabs(verify_v[i]-V[i])>1e-10 || fabs(verify_w[i]-W[i])>1e-8)
      Error("D_CONTACT_NATIVE_MOTION_OVERWRITE_NOT_VERIFIED\n");
    last_impulse_time=time;last_wall=nid;
  }
#if !RP_NODE
  fp=fopen(CONTACT_LOG,"a");if(!fp)Error("D_CONTACT_LOG_OPEN_FAILED\n");
  fprintf(fp,"{\"time_s\":%.17g,\"number_of_contact_faces\":%d,\"robot_thread_id\":%d,\"pipe_thread_id\":%d,",time,count,tid,nid);
  json_vector(fp,"COM_m",cg,3);json_vector(fp,"quaternion_wxyz",qlog,4);json_vector(fp,"contact_point_m",point,3);
  json_vector(fp,"normal",normal,3);json_vector(fp,"r_m",r,3);json_vector(fp,"v_before_m_s",v,3);json_vector(fp,"omega_before_rad_s",w,3);
  json_vector(fp,"v_after_m_s",V,3);json_vector(fp,"omega_after_rad_s",W,3);
  if((response_enabled||sanity_enabled)&&!duplicate&&J>0) {
    double vv[3],ww[3];for(i=0;i<3;i++){vv[i]=verify_v[i];ww[i]=verify_w[i];}
    json_vector(fp,"native_motion_v_after_m_s",vv,3);json_vector(fp,"native_motion_omega_after_rad_s",ww,3);
  }
  {double wxr[3],vc[3];contact_cross(w,r,wxr);for(i=0;i<3;i++)vc[i]=v[i]+wxr[i];json_vector(fp,"contact_velocity_before_m_s",vc,3);}
  fprintf(fp,"\"v_n_before_m_s\":%.17g,\"impulse_N_s\":%.17g,\"v_n_after_m_s\":%.17g,\"response_enabled\":%d,\"sanity_test\":%d,\"duplicate_suppressed\":%d,\"impulse_applied\":%s}\n",before,J,after,response_enabled,sanity_enabled,duplicate,((response_enabled||sanity_enabled)&&!duplicate&&J>0)?"true":"false");
  fclose(fp);
  fp=fopen(CONTACT_CSV,"a+");if(!fp)Error("D_CONTACT_CSV_OPEN_FAILED\n");
  fseek(fp,0,SEEK_END);if(ftell(fp)==0)fprintf(fp,"time_s,n_faces,COM_x,COM_y,COM_z,q0,q1,q2,q3,point_x,point_y,point_z,normal_x,normal_y,normal_z,r_x,r_y,r_z,vx_before,vy_before,vz_before,wx_before,wy_before,wz_before,vx_after,vy_after,vz_after,wx_after,wy_after,wz_after,vn_before,J,vn_after,response_enabled,sanity_test,duplicate_suppressed\n");
  fprintf(fp,"%.17g,%d",time,count);
  {const double *arrays[10]={cg,qlog,point,normal,r,v,w,V,W,0};
   for(k=0;k<9;k++)for(i=0;i<(k==1?4:3);i++)fprintf(fp,",%.17g",arrays[k][i]);}
  fprintf(fp,",%.17g,%.17g,%.17g,%d,%d,%d\n",before,J,after,response_enabled,sanity_enabled,duplicate);fclose(fp);
#endif
}

/* Diagnostic native detection entry: zero dtime, no motion integration.
 * The last boolean is tested and recorded, not assumed to define tolerance.
 */
DEFINE_ON_DEMAND(benchmark_D_static_native_detect)
{
  FILE *fp;int detection_flag=0,mode=0;double dx=0,dy=0,dz=0;real shift[3];Domain *d=Get_Domain(1);
  fp=fopen(PROBE_CONTROL,"r");if(!fp)Error("D_PROBE_CONTROL_MISSING\n");
  if(fscanf(fp,"%d %d %lf %lf %lf",&detection_flag,&mode,&dx,&dy,&dz)!=5)Error("D_PROBE_CONTROL_INVALID\n");fclose(fp);
  response_enabled=0;sanity_enabled=mode==2;last_impulse_time=-1;last_wall=-1;
  shift[0]=dx;shift[1]=dy;shift[2]=dz;
  if(dx!=0 || dy!=0 || dz!=0) {
    Thread *t;thread_loop_c(t,d)if(strcmp(THREAD_NAME(t),"robot_component_fluid")==0)Translate_Cell_Thread(d,t,shift);
    /* The probe uses frozen attitude and a rigid translation, never production dynamics. */
    Dynamic_Thread *dt;for(dt=d->dynamic_threads;dt;dt=dt->next) {
      const char *name=THREAD_NAME(DT_THREAD(dt));
      if(strcmp(name,"robot_wall")==0 || strcmp(name,"robot_component_fluid")==0) {
        DT_CG(dt)[0]+=dx;DT_CG(dt)[1]+=dy;DT_CG(dt)[2]+=dz;
      }
    }
  }
  Message0("D_STATIC_BEFORE_DOWNLOAD active=%d\n",(int)contact_active_p);
  Download_Contact_Parameters();
  Message0("D_STATIC_AFTER_DOWNLOAD active=%d\n",(int)contact_active_p);
  Init_Contact(d);Detect_Contact(d,CURRENT_TIME,0.0,detection_flag?TRUE:FALSE);
  Message0("D_STATIC_NATIVE_DETECT_RETURN time=%.17g dtime=0 flag=%d mode=%d\n",(double)CURRENT_TIME,detection_flag,mode);
}

DEFINE_ON_DEMAND(benchmark_D_enable_contact_response)
{ response_enabled=1;sanity_enabled=0;last_impulse_time=-1;last_wall=-1;Message0("D_CONTACT_RESPONSE_ENABLED\n"); }
