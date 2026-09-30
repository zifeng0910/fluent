/* L2300 / successful Abaqus 100 Hz analytic source.
 * All values SI. Source equation is the validated production analytic law,
 * rigidly expressed in Fluent global XYZ. Fluent owns 6DOF integration.
 */
#include "udf.h"
#include "dynamesh_tools.h"
#include "six_dof.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.141592653589793238462643383279502884
#define OMEGA (2.0 * PI * 100.0)
#define B0_T 0.011
#define GRADIENT_T 0.0022
#define GRADIENT_LENGTH_MM 45.0
#define RAMP_END_S 0.001
#define SOURCE_OFFSET_MM 31.699921242759284
#define SOURCE_SPEED_MM_S 6.0
#define COM0_X_M 0.0012060186937156343
#define VALID_TIME_MAX_S 0.050
#define NEAR_WALL_LIMIT_M 0.0003925
#define LOAD_CSV "H:\\fluent\\evidence\\benchmark_C_udf_load_validation.csv"
#define HISTORY_CSV "H:\\fluent\\evidence\\benchmark_C_analytic_6dof_history.csv"
#define PROBE_CSV "H:\\fluent\\evidence\\benchmark_C_udf_orientation_probe.csv"

/* Authoritative CAD body-frame dipole; retain small transverse components. */
static const real M_BODY[3] = {
  1.0399958044715794e-3, 9.084781997334923e-6, 3.1131721660494084e-6
};
static const real MASS_KG = 8.904428864007322e-6;
static const real IXX_KGM2 = 7.257810693523445e-13;
static const real IYY_KGM2 = 3.929384741151496e-12;
static const real IZZ_KGM2 = 3.9293847411514897e-12;
static const real IXY_KGM2 = -2.782693607479792e-28;
static const real IXZ_KGM2 = 1.9327591150313555e-29;
static const real IYZ_KGM2 = 2.4813784672159598e-29;

/*
 * Literal analytic source in Fluent XYZ after the recorded proper frame map:
 * source tangent=(-1,0,0), rocking n=(0,1,0), rocking b=(0,0,-1).
 * Independent axial gradient remains along +X as in the Abaqus VUAMP table.
 */
static int compute_magnetic_load(const real cg[3], quaternion q,
                                 real time, real force[3], real torque[3],
                                 real moment_global[3], real B_global[3],
                                 real *s_eff_mm, real *phase_rad, real *ramp)
{
  real R[3][3], alpha_main, alpha_cross, direction[3], norm;
  real grad_xx, q_ramp;
  int i, j;
  if (!isfinite((double)time) || time < 0.0 || time > VALID_TIME_MAX_S)
    return 0;
  if (!isfinite((double)cg[0]) || !isfinite((double)cg[1]) || !isfinite((double)cg[2]))
    return 0;
  if (cg[0] < -0.004 || cg[0] > 0.006 || fabs(cg[1]) > 0.0009 || fabs(cg[2]) > 0.0009)
    return 0;

  Rot_Matrix_From_Q(q, R); /* Fluent 2026 R1 current quaternion, body -> global. */
  for (i = 0; i < 3; ++i) {
    moment_global[i] = 0.0;
    for (j = 0; j < 3; ++j) moment_global[i] += R[i][j] * M_BODY[j];
  }

  *phase_rad = OMEGA * time;
  alpha_main = (14.5 * PI / 180.0) * sin(*phase_rad);
  alpha_cross = (2.5 * PI / 180.0) * cos(*phase_rad);
  direction[0] = -1.0;
  direction[1] = -tan(alpha_main);
  direction[2] = tan(alpha_cross);
  norm = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
  if (!isfinite((double)norm) || norm <= 0.0) return 0;
  for (i = 0; i < 3; ++i) B_global[i] = B0_T * direction[i] / norm;

  *s_eff_mm = SOURCE_OFFSET_MM + (cg[0] - COM0_X_M) * 1000.0 - SOURCE_SPEED_MM_S * time;
  grad_xx = -GRADIENT_T * (*s_eff_mm / (GRADIENT_LENGTH_MM * GRADIENT_LENGTH_MM))
            * exp(-0.5 * (*s_eff_mm / GRADIENT_LENGTH_MM) * (*s_eff_mm / GRADIENT_LENGTH_MM))
            * 1000.0;
  *ramp = 1.0;
  if (time < RAMP_END_S) {
    q_ramp = time / RAMP_END_S;
    *ramp = q_ramp * q_ramp * (3.0 - 2.0 * q_ramp);
  }
  for (i = 0; i < 3; ++i) { force[i] = 0.0; torque[i] = 0.0; }
  force[0] = (*ramp) * moment_global[0] * grad_xx;
  torque[0] = (*ramp) * (moment_global[1]*B_global[2] - moment_global[2]*B_global[1]);
  torque[1] = (*ramp) * (moment_global[2]*B_global[0] - moment_global[0]*B_global[2]);
  torque[2] = (*ramp) * (moment_global[0]*B_global[1] - moment_global[1]*B_global[0]);
  for (i = 0; i < 3; ++i)
    if (!isfinite((double)force[i]) || !isfinite((double)torque[i]) ||
        !isfinite((double)moment_global[i]) || !isfinite((double)B_global[i])) return 0;
  return 1;
}

static void write_history(const char *mode, Dynamic_Thread *dt, real time,
                          const real force[3], const real torque[3],
                          const real moment[3], const real B[3], real s_eff, real phase, real ramp)
{
#if !RP_NODE
  FILE *fp = fopen(HISTORY_CSV, "a+");
  real *cg = DT_CG(dt), *v = DT_VEL_CG(dt), *w = DT_OMEGA_CG(dt);
  quaternion q;
  real theta[3];
  real radial = sqrt(cg[1]*cg[1] + cg[2]*cg[2]);
  int empty;
  if (!fp) { Message("BENCHMARK_C_HISTORY_OPEN_FAIL %s\n", HISTORY_CSV); return; }
  fseek(fp, 0, SEEK_END); empty = (ftell(fp) == 0);
  if (empty) fprintf(fp, "mode,zone_name,time_s,cg_x_m,cg_y_m,cg_z_m,q0,q1,q2,q3,theta_x_rad,theta_y_rad,theta_z_rad,vx_m_s,vy_m_s,vz_m_s,omega_x_rad_s,omega_y_rad_s,omega_z_rad_s,Fmag_x_N,Fmag_y_N,Fmag_z_N,Tmag_x_Nm,Tmag_y_Nm,Tmag_z_Nm,radial_displacement_m,estimated_wall_clearance_m,source_s_eff_mm,phase_rad,ramp,m_global_x,m_global_y,m_global_z,B_x_T,B_y_T,B_z_T\n");
  Q_Assign(DT_Q(dt), &q); N3V_V(theta, =, DT_THETA(dt));
  fprintf(fp, "%s,%s", mode, THREAD_NAME(DT_THREAD(dt)));
#define CSV_REAL(value) fprintf(fp, ",%.17g", (double)(value))
  CSV_REAL(time); CSV_REAL(cg[0]); CSV_REAL(cg[1]); CSV_REAL(cg[2]);
  CSV_REAL(q[0]); CSV_REAL(q[1]); CSV_REAL(q[2]); CSV_REAL(q[3]);
  CSV_REAL(theta[0]); CSV_REAL(theta[1]); CSV_REAL(theta[2]);
  CSV_REAL(v[0]); CSV_REAL(v[1]); CSV_REAL(v[2]);
  CSV_REAL(w[0]); CSV_REAL(w[1]); CSV_REAL(w[2]);
  CSV_REAL(force[0]); CSV_REAL(force[1]); CSV_REAL(force[2]);
  CSV_REAL(torque[0]); CSV_REAL(torque[1]); CSV_REAL(torque[2]);
  CSV_REAL(radial); CSV_REAL(0.0009-radial); CSV_REAL(s_eff); CSV_REAL(phase); CSV_REAL(ramp);
  CSV_REAL(moment[0]); CSV_REAL(moment[1]); CSV_REAL(moment[2]);
  CSV_REAL(B[0]); CSV_REAL(B[1]); CSV_REAL(B[2]);
  fprintf(fp, "\n");
#undef CSV_REAL
  fflush(fp); fclose(fp);
#else
  (void)mode; (void)dt; (void)time; (void)force; (void)torque; (void)moment; (void)B; (void)s_eff; (void)phase; (void)ramp;
#endif
}

static void set_l2300_properties(real prop[], Dynamic_Thread *dt, real time, int load_only)
{
  real F[3], T[3], moment[3], B[3], s_eff, phase, ramp, radial;
  int i;
  prop[SDOF_MASS] = MASS_KG;
  prop[SDOF_IXX] = IXX_KGM2; prop[SDOF_IYY] = IYY_KGM2; prop[SDOF_IZZ] = IZZ_KGM2;
  prop[SDOF_IXY] = IXY_KGM2; prop[SDOF_IXZ] = IXZ_KGM2; prop[SDOF_IYZ] = IYZ_KGM2;
  prop[SDOF_LOAD_LOCAL] = FALSE;
  for (i = 0; i < 3; ++i) { prop[SDOF_LOAD_F_X+i] = 0.0; prop[SDOF_LOAD_M_X+i] = 0.0; }
  if (!compute_magnetic_load(DT_CG(dt), DT_Q(dt), time, F, T, moment, B, &s_eff, &phase, &ramp))
    Error("BENCHMARK_C_ANALYTIC_BASELINE_VALIDATION_ENVELOPE_EXCEEDED or non-finite magnetic state; stopping.\n");
  for (i = 0; i < 3; ++i) { prop[SDOF_LOAD_F_X+i] = F[i]; prop[SDOF_LOAD_M_X+i] = T[i]; }
  radial = sqrt(DT_CG(dt)[1]*DT_CG(dt)[1] + DT_CG(dt)[2]*DT_CG(dt)[2]);
  write_history(load_only ? "LOAD_ONLY_FROZEN" : "FREE_6DOF", dt, time, F, T, moment, B, s_eff, phase, ramp);
  if (!load_only && radial > NEAR_WALL_LIMIT_M)
    Error("NEAR_WALL_SAFETY_STOP: radial center displacement exceeds 0.3925 mm.\n");
  prop[SDOF_ZERO_TRANS_X] = load_only ? TRUE : FALSE;
  prop[SDOF_ZERO_TRANS_Y] = load_only ? TRUE : FALSE;
  prop[SDOF_ZERO_TRANS_Z] = load_only ? TRUE : FALSE;
  prop[SDOF_ZERO_ROT_X] = load_only ? TRUE : FALSE;
  prop[SDOF_ZERO_ROT_Y] = load_only ? TRUE : FALSE;
  prop[SDOF_ZERO_ROT_Z] = load_only ? TRUE : FALSE;
}

DEFINE_SDOF_PROPERTIES(l2300_magnetic_6dof, prop, dt, time, dtime)
{ (void)dtime; set_l2300_properties(prop, dt, time, 0); }

/* Same pure load function; only this temporary validation hook freezes DOFs. */
DEFINE_SDOF_PROPERTIES(l2300_magnetic_load_validation, prop, dt, time, dtime)
{ (void)dtime; set_l2300_properties(prop, dt, time, 1); }



/* Deterministic known-attitude oracle using Fluent's own q construction. */
DEFINE_ON_DEMAND(benchmark_C_analytic_load_probe)
{
  const real sample_time[6]={0.0,0.00025,0.0005,0.00075,0.0010,0.00125};
  real cg[3]={COM0_X_M,0.0,0.0};
  real theta[3]={17.0*PI/180.0*0.36,17.0*PI/180.0*0.48,17.0*PI/180.0*0.8};
  quaternion q; int k;
  Q_From_Theta(theta,&q);
#if !RP_NODE
  FILE *fp=fopen(PROBE_CSV,"w");
  if(!fp) Error("Cannot write C orientation probe\n");
  fprintf(fp,"time_s,theta_x_rad,theta_y_rad,theta_z_rad,q0,q1,q2,q3,Fx_N,Fy_N,Fz_N,Tx_Nm,Ty_Nm,Tz_Nm,mx,my,mz,Bx_T,By_T,Bz_T,s_eff_mm,phase_rad,ramp\n");
  for(k=0;k<6;++k){real F[3],T[3],m[3],B[3],se,ph,ra;int j;
    if(!compute_magnetic_load(cg,q,sample_time[k],F,T,m,B,&se,&ph,&ra))Error("C direct analytic load probe failed\\n");
    fprintf(fp,"%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",(double)sample_time[k],(double)theta[0],(double)theta[1],(double)theta[2],(double)q[0],(double)q[1],(double)q[2],(double)q[3]);
    for(j=0;j<3;++j)fprintf(fp,",%.17g",(double)F[j]);for(j=0;j<3;++j)fprintf(fp,",%.17g",(double)T[j]);
    for(j=0;j<3;++j)fprintf(fp,",%.17g",(double)m[j]);for(j=0;j<3;++j)fprintf(fp,",%.17g",(double)B[j]);
    fprintf(fp,",%.17g,%.17g,%.17g\n",(double)se,(double)ph,(double)ra);
  }
  fclose(fp);Message0("Benchmark C direct analytic load probe complete.\n");
#endif
}
