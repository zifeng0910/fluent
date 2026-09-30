#include "udf.h"
#include "overset.h"
#include <stdio.h>

DEFINE_ON_DEMAND(benchmark_C_pose_statistics)
{
#if !RP_HOST
  Domain *d=Get_Domain(1); Thread *t; cell_t c; int first=1;
  FILE *fp=fopen("H:\\fluent\\evidence\\benchmark_C_pose_live.json","w");
  if(!fp) Error("Cannot write pose statistics\n");
  fprintf(fp,"{\"time_s\":%.17g,\"zones\":[",CURRENT_TIME);
  thread_loop_overset_c(t,d) {
    long total=0,solve=0,donor=0,receptor=0,orphan=0,missing=0,dead=0;
    real minv=1e300;
    begin_c_loop_int(c,t) {
      ++total;
      if(OVERSET_SOLVE_CELL_P(c,t))++solve;
      if(OVERSET_DONOR_CELL_P(c,t))++donor;
      if(OVERSET_DEAD_CELL_P(c,t))++dead;
      if(OVERSET_ORPHAN_CELL_P(c,t))++orphan;
      if(OVERSET_RECEPTOR_CELL_P(c,t)) {++receptor;if(C_OVERSET_NDONOR(c,t)<=0)++missing;}
      if(C_VOLUME(c,t)<minv)minv=C_VOLUME(c,t);
    } end_c_loop_int(c,t)
    fprintf(fp,"%s{\"name\":\"%s\",\"total\":%ld,\"solve\":%ld,\"donor\":%ld,\"receptor\":%ld,\"orphan\":%ld,\"missing\":%ld,\"dead\":%ld,\"min_volume_m3\":%.17g}",first?"":",",THREAD_NAME(t),total,solve,donor,receptor,orphan,missing,dead,minv);
    first=0;
  }
  fprintf(fp,"]}");fclose(fp);
#endif
}
