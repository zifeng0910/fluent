/* Benchmark B only: rigid translation and read-only overset diagnostics.
 * No magnetic loads, 6DOF force balance, contact, or mesh deformation. */
#include "udf.h"
#include "overset.h"
#include <stdio.h>
#include <float.h>

DEFINE_CG_MOTION(benchmark_B_translate_x, dt, vel, omega, time, dtime)
{
    NV_S(vel, =, 0.0);
    NV_S(omega, =, 0.0);
    if (time <= 0.002 + 1.0e-12)
        vel[0] = 0.05; /* m/s: 0.10 mm in 2 ms */
}

DEFINE_ON_DEMAND(benchmark_B_overset_statistics)
{
#if !RP_HOST
    Domain *domain = Get_Domain(1);
    Thread *thread;
    cell_t cell;
    FILE *file = fopen("benchmark_B_connectivity_live.json", "w");
    int first = 1;
    if (!file) Error("Cannot write Benchmark B overset statistics\n");
    fprintf(file, "{\"time_s\":%.17g,\"zones\":[", CURRENT_TIME);
    thread_loop_overset_c(thread, domain)
    {
        long total=0, solve=0, donor=0, receptor=0, orphan=0, dead=0;
        long donor_links=0, missing_donors=0;
        real min_volume=DBL_MAX;
        real orphan_min[3]={DBL_MAX,DBL_MAX,DBL_MAX};
        real orphan_max[3]={-DBL_MAX,-DBL_MAX,-DBL_MAX};
        begin_c_loop_int(cell, thread)
        {
            int k;
            real xyz[ND_ND];
            int ndonor = C_OVERSET_NDONOR(cell, thread);
            ++total;
            if (OVERSET_SOLVE_CELL_P(cell,thread)) ++solve;
            if (OVERSET_DONOR_CELL_P(cell,thread)) ++donor;
            if (OVERSET_RECEPTOR_CELL_P(cell,thread))
            {
                ++receptor;
                donor_links += ndonor;
                if (ndonor <= 0) ++missing_donors;
            }
            if (OVERSET_DEAD_CELL_P(cell,thread)) ++dead;
            if (C_VOLUME(cell,thread) < min_volume) min_volume=C_VOLUME(cell,thread);
            if (OVERSET_ORPHAN_CELL_P(cell,thread))
            {
                ++orphan;
                C_CENTROID(xyz,cell,thread);
                for (k=0;k<3;++k)
                {
                    if(xyz[k]<orphan_min[k]) orphan_min[k]=xyz[k];
                    if(xyz[k]>orphan_max[k]) orphan_max[k]=xyz[k];
                }
            }
        }
        end_c_loop_int(cell,thread)
        fprintf(file,"%s{\"id\":%d,\"name\":\"%s\",\"total\":%ld,"
                     "\"solve\":%ld,\"donor\":%ld,\"receptor\":%ld,\"orphan\":%ld,"
                     "\"dead\":%ld,\"donor_links\":%ld,\"receptors_without_donors\":%ld,"
                     "\"min_volume_m3\":%.17g,\"orphan_bounds_m\":",
                first?"":",",THREAD_ID(thread),THREAD_NAME(thread),total,
                solve,donor,receptor,orphan,dead,donor_links,missing_donors,min_volume);
        if(orphan)
            fprintf(file,"[[%.17g,%.17g,%.17g],[%.17g,%.17g,%.17g]]}",
                    orphan_min[0],orphan_min[1],orphan_min[2],orphan_max[0],orphan_max[1],orphan_max[2]);
        else fprintf(file,"null}");
        first=0;
    }
    fprintf(file,"]}");
    fclose(file);
    Message0("Benchmark B overset cell statistics exported.\n");
#endif
}
