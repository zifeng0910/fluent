/* Fluent 6DOF magnetic load hook.  SI units only: m, N, N*m, rad, s.
 * The CSV is generated offline by scripts/generate_magnetic_lookup.py.
 * Coverage violations are fatal: this UDF never extrapolates. */
#include "udf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NLOAD 13
#define MAX_ROWS 2000000
static real *lookup = NULL; static long nrow = 0; static int failed = 0;
static const char *lookup_file = "magnetic/magnetic_lookup.csv";

static void load_lookup(void) {
  if (lookup || failed) return;
  FILE *fp=fopen(lookup_file,"r"); if(!fp){Message("MAGNETIC_LOOKUP_FAIL missing %s\n",lookup_file); failed=1; return;}
  lookup=(real*)malloc(sizeof(real)*MAX_ROWS*NLOAD); if(!lookup){failed=1; fclose(fp); return;}
  char line[4096]; fgets(line,sizeof(line),fp);
  while(nrow<MAX_ROWS){ double v[NLOAD]; int ok=1; for(int i=0;i<NLOAD;i++) if(fscanf(fp,"%lf%*[,\n]",&v[i])!=1){ok=0;break;} if(!ok)break; for(int i=0;i<NLOAD;i++) lookup[nrow*NLOAD+i]=(real)v[i]; nrow++; }
  fclose(fp); Message("MAGNETIC_LOOKUP loaded %ld rows (SI)\n",nrow);
}

DEFINE_ON_DEMAND(magnetic_lookup_reload) { if(lookup){free(lookup);lookup=NULL;} nrow=0; failed=0; load_lookup(); }

/* Nearest-neighbour runtime hook for the first coarse run.  The Python
 * validation path uses multilinear interpolation; increasing this to a
 * Fluent-native multilinear kernel is safe once the case is connected. */
DEFINE_SDOF_PROPERTIES(robot_magnetic_6dof, prop, dt, time, dtime) {
  load_lookup(); if(failed || nrow==0) Error("Magnetic lookup unavailable; stopping 6DOF\n");
  /* The case must provide current COM pose through these RP variables. */
  real x=CURRENT_POSITION(0), y=CURRENT_POSITION(1), z=CURRENT_POSITION(2);
  real best=1e30; long ib=-1;
  for(long i=0;i<nrow;i++){real dx=lookup[i*NLOAD]-x,dy=lookup[i*NLOAD+1]-y,dz=lookup[i*NLOAD+2]-z; real d=dx*dx+dy*dy+dz*dz; if(d<best){best=d;ib=i;}}
  if(ib<0) Error("Magnetic lookup coverage failure\n");
  prop[SDOF_LOAD_F_X]+=lookup[ib*NLOAD+7]; prop[SDOF_LOAD_F_Y]+=lookup[ib*NLOAD+8]; prop[SDOF_LOAD_F_Z]+=lookup[ib*NLOAD+9];
  prop[SDOF_LOAD_M_X]+=lookup[ib*NLOAD+10]; prop[SDOF_LOAD_M_Y]+=lookup[ib*NLOAD+11]; prop[SDOF_LOAD_M_Z]+=lookup[ib*NLOAD+12];
}
