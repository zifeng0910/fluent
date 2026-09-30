#include "udf.h"
#include "overset.h"
#include <stdio.h>

DEFINE_ON_DEMAND(benchmark_C_overset_statistics)
{
#if !RP_HOST
  Domain *domain=Get_Domain(1); Thread *thread; cell_t cell;
  FILE *fp=fopen("H:\\fluent\\evidence\\benchmark_C_overset_live.json","w"); int first=1;
  if(!fp) Error("Cannot write C overset audit\\n");
  fprintf(fp,"{\"time_s\":%.17g,\"zones\":[",CURRENT_TIME);
  thread_loop_overset_c(thread,domain){long total=0,solve=0,donor=0,receptor=0,orphan=0,missing=0,first_orphan=-1;real minv=1e300,orphan_xyz[ND_ND]={0.0};
    begin_c_loop_int(cell,thread){int nd=C_OVERSET_NDONOR(cell,thread);++total;if(OVERSET_SOLVE_CELL_P(cell,thread))++solve;if(OVERSET_DONOR_CELL_P(cell,thread))++donor;if(OVERSET_RECEPTOR_CELL_P(cell,thread)){++receptor;if(nd<=0)++missing;}if(OVERSET_ORPHAN_CELL_P(cell,thread)){++orphan;if(first_orphan<0){first_orphan=(long)cell;C_CENTROID(orphan_xyz,cell,thread);}}if(C_VOLUME(cell,thread)<minv)minv=C_VOLUME(cell,thread);}end_c_loop_int(cell,thread)
    fprintf(fp,"%s{\"id\":%d,\"name\":\"%s\",\"total\":%ld,\"solve\":%ld,\"donor\":%ld,\"receptor\":%ld,\"orphan\":%ld,\"receptors_without_donors\":%ld,\"min_volume_m3\":%.17g,\"first_orphan_cell\":%ld,\"first_orphan_centroid_m\":[%.17g,%.17g,%.17g]}",first?"":",",THREAD_ID(thread),THREAD_NAME(thread),total,solve,donor,receptor,orphan,missing,minv,first_orphan,(double)orphan_xyz[0],(double)orphan_xyz[1],(double)orphan_xyz[2]);first=0;}
  fprintf(fp,"]}");fclose(fp);Message0("Benchmark C overset audit exported.\n");
#endif
}
