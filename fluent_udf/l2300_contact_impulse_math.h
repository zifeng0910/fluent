/* Pure e_n=0, mu=0 impulse kernel, tested independently before Fluent use.
 * No Fluent, file, solver, or magnetic load dependencies.
 * R is the row-major body-to-world rotation, all other vectors are world SI.
 */
#ifndef L2300_CONTACT_IMPULSE_MATH_H
#define L2300_CONTACT_IMPULSE_MATH_H
static double contact_dot(const double *a, const double *b)
{ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static void contact_cross(const double *a, const double *b, double *c)
{ c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0]; }
static void contact_matvec(const double *A, const double *v, double *out)
{ int i; for(i=0;i<3;i++) out[i]=contact_dot(A+3*i,v); }
static void contact_world_inverse(const double *R, const double *body_inverse, double *world_inverse)
{
  int i,j,k,l;
  for(i=0;i<3;i++) for(j=0;j<3;j++) {
    world_inverse[3*i+j]=0;
    for(k=0;k<3;k++) for(l=0;l<3;l++)
      world_inverse[3*i+j]+=R[3*i+k]*body_inverse[3*k+l]*R[3*j+l];
  }
}
static int contact_impulse(double mass, const double *inverse_world,
                           const double *r, const double *normal,
                           const double *velocity, const double *omega,
                           double *vnew, double *wnew, double *J,
                           double *vn_before, double *vn_after)
{
  double wxr[3], vc[3], rxn[3], angular[3], angularxr[3], denominator;
  int i;
  contact_cross(omega,r,wxr);
  for(i=0;i<3;i++) vc[i]=velocity[i]+wxr[i];
  *vn_before=contact_dot(vc,normal); *J=0;
  for(i=0;i<3;i++){vnew[i]=velocity[i];wnew[i]=omega[i];}
  if(*vn_before<0) {
    contact_cross(r,normal,rxn);contact_matvec(inverse_world,rxn,angular);
    contact_cross(angular,r,angularxr);
    denominator=1/mass+contact_dot(normal,angularxr);
    if(!(denominator>0)) return -1;
    *J=-*vn_before/denominator;
    for(i=0;i<3;i++){vnew[i]+=*J*normal[i]/mass;wnew[i]+=*J*angular[i];}
  }
  contact_cross(wnew,r,wxr);
  for(i=0;i<3;i++) vc[i]=vnew[i]+wxr[i];
  *vn_after=contact_dot(vc,normal);
  return *J>0?1:0;
}
#endif
