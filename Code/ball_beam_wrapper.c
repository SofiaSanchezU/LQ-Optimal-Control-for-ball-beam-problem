
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void ball_beam_Outputs_wrapper(const real_T *u0,
			real_T *y0,
			const real_T *xC,
			const real_T *p0, const int_T p_width0,
			const real_T *p1, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
y0[0]=xC[0];
y0[1]=xC[1];
y0[2]=xC[2];
y0[3]=xC[3];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Derivatives function
 *
 */
void ball_beam_Derivatives_wrapper(const real_T *u0,
			real_T *y0,
			real_T *dx,
			real_T *xC,
			const real_T *p0, const int_T p_width0,
			const real_T *p1, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_BEGIN --- EDIT HERE TO _END */
real_T I, m, J, R, g=9.81, L=0.40;
real_T r,rdot,theta,thetadot;

I=p1[0];m=p1[1];J=p1[2];R=p1[3];g=p1[4];L=p1[5];
r=xC[0];rdot=xC[1];theta=xC[2];thetadot=xC[3];
    
    dx[0]=rdot;
    dx[1]=((m*r*thetadot*thetadot)/(m+J/R*R))-((m*g*sin(theta))/(m+J/R*R));
    dx[2]=thetadot;
    dx[3]=(u0[0]/(I+m*r*r))-(m*g*r*cos(theta)/(I+m*r*r))-(2*m*r*rdot*theta/(I+m*r*r));
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_END --- EDIT HERE TO _BEGIN */
}

