/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[4] = acadoVariables.mu[lRun1 * 4];
acadoWorkspace.state[5] = acadoVariables.mu[lRun1 * 4 + 1];
acadoWorkspace.state[6] = acadoVariables.mu[lRun1 * 4 + 2];
acadoWorkspace.state[7] = acadoVariables.mu[lRun1 * 4 + 3];
acadoWorkspace.state[53] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[54] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[55] = acadoVariables.od[lRun1 * 13];
acadoWorkspace.state[56] = acadoVariables.od[lRun1 * 13 + 1];
acadoWorkspace.state[57] = acadoVariables.od[lRun1 * 13 + 2];
acadoWorkspace.state[58] = acadoVariables.od[lRun1 * 13 + 3];
acadoWorkspace.state[59] = acadoVariables.od[lRun1 * 13 + 4];
acadoWorkspace.state[60] = acadoVariables.od[lRun1 * 13 + 5];
acadoWorkspace.state[61] = acadoVariables.od[lRun1 * 13 + 6];
acadoWorkspace.state[62] = acadoVariables.od[lRun1 * 13 + 7];
acadoWorkspace.state[63] = acadoVariables.od[lRun1 * 13 + 8];
acadoWorkspace.state[64] = acadoVariables.od[lRun1 * 13 + 9];
acadoWorkspace.state[65] = acadoVariables.od[lRun1 * 13 + 10];
acadoWorkspace.state[66] = acadoVariables.od[lRun1 * 13 + 11];
acadoWorkspace.state[67] = acadoVariables.od[lRun1 * 13 + 12];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[23];

acadoWorkspace.evGu[lRun1 * 8] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 8 + 1] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 8 + 2] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 8 + 3] = acadoWorkspace.state[27];
acadoWorkspace.evGu[lRun1 * 8 + 4] = acadoWorkspace.state[28];
acadoWorkspace.evGu[lRun1 * 8 + 5] = acadoWorkspace.state[29];
acadoWorkspace.evGu[lRun1 * 8 + 6] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 8 + 7] = acadoWorkspace.state[31];
acadoWorkspace.EH[lRun1 * 36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[32];
acadoWorkspace.EH[lRun1 * 36 + 6] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[33];
acadoWorkspace.EH[lRun1 * 36 + 1] = acadoWorkspace.EH[lRun1 * 36 + 6];
acadoWorkspace.EH[lRun1 * 36 + 7] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[34];
acadoWorkspace.EH[lRun1 * 36 + 12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[35];
acadoWorkspace.EH[lRun1 * 36 + 2] = acadoWorkspace.EH[lRun1 * 36 + 12];
acadoWorkspace.EH[lRun1 * 36 + 13] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[36];
acadoWorkspace.EH[lRun1 * 36 + 8] = acadoWorkspace.EH[lRun1 * 36 + 13];
acadoWorkspace.EH[lRun1 * 36 + 14] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[37];
acadoWorkspace.EH[lRun1 * 36 + 18] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[38];
acadoWorkspace.EH[lRun1 * 36 + 3] = acadoWorkspace.EH[lRun1 * 36 + 18];
acadoWorkspace.EH[lRun1 * 36 + 19] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[39];
acadoWorkspace.EH[lRun1 * 36 + 9] = acadoWorkspace.EH[lRun1 * 36 + 19];
acadoWorkspace.EH[lRun1 * 36 + 20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[40];
acadoWorkspace.EH[lRun1 * 36 + 15] = acadoWorkspace.EH[lRun1 * 36 + 20];
acadoWorkspace.EH[lRun1 * 36 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[41];
acadoWorkspace.EH[lRun1 * 36 + 24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[42];
acadoWorkspace.EH[lRun1 * 36 + 4] = acadoWorkspace.EH[lRun1 * 36 + 24];
acadoWorkspace.EH[lRun1 * 36 + 25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[43];
acadoWorkspace.EH[lRun1 * 36 + 10] = acadoWorkspace.EH[lRun1 * 36 + 25];
acadoWorkspace.EH[lRun1 * 36 + 26] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[44];
acadoWorkspace.EH[lRun1 * 36 + 16] = acadoWorkspace.EH[lRun1 * 36 + 26];
acadoWorkspace.EH[lRun1 * 36 + 27] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[45];
acadoWorkspace.EH[lRun1 * 36 + 22] = acadoWorkspace.EH[lRun1 * 36 + 27];
acadoWorkspace.EH[lRun1 * 36 + 28] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[46];
acadoWorkspace.EH[lRun1 * 36 + 30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[47];
acadoWorkspace.EH[lRun1 * 36 + 5] = acadoWorkspace.EH[lRun1 * 36 + 30];
acadoWorkspace.EH[lRun1 * 36 + 31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[48];
acadoWorkspace.EH[lRun1 * 36 + 11] = acadoWorkspace.EH[lRun1 * 36 + 31];
acadoWorkspace.EH[lRun1 * 36 + 32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[49];
acadoWorkspace.EH[lRun1 * 36 + 17] = acadoWorkspace.EH[lRun1 * 36 + 32];
acadoWorkspace.EH[lRun1 * 36 + 33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[50];
acadoWorkspace.EH[lRun1 * 36 + 23] = acadoWorkspace.EH[lRun1 * 36 + 33];
acadoWorkspace.EH[lRun1 * 36 + 34] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[51];
acadoWorkspace.EH[lRun1 * 36 + 29] = acadoWorkspace.EH[lRun1 * 36 + 34];
acadoWorkspace.EH[lRun1 * 36 + 35] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[52];
}
return ret;
}

void acado_evaluateLagrange(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 455. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt(((pow(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]),2))+(pow(((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]),2)))));
a[1] = (((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0]);
a[2] = (a[1]*od[8]);
a[3] = (((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]))));
a[4] = (a[2]*a[3]);
a[5] = (((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0]);
a[6] = (od[8]*(((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7])))));
a[7] = (a[5]*a[6]);
a[8] = (a[4]+a[7]);
a[9] = ((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]));
a[10] = (a[9]*od[9]);
a[11] = ((((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]))*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]))));
a[12] = (a[10]*a[11]);
a[13] = ((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]));
a[14] = (od[9]*((((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]))*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7])))));
a[15] = (a[13]*a[14]);
a[16] = (a[12]+a[15]);
a[17] = (a[8]+a[16]);
a[18] = (((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]);
a[19] = (real_t)(-1.0000000000000000e+00);
a[20] = (a[18]*a[19]);
a[21] = (a[20]*od[8]);
a[22] = (a[21]*a[3]);
a[23] = (((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]);
a[24] = (real_t)(-1.0000000000000000e+00);
a[25] = (a[23]*a[24]);
a[26] = (a[25]*a[6]);
a[27] = (a[22]+a[26]);
a[28] = (((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0]);
a[29] = (real_t)(-1.0000000000000000e+00);
a[30] = (a[28]*a[29]);
a[31] = (a[30]*od[9]);
a[32] = (a[31]*a[11]);
a[33] = (((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0]);
a[34] = (real_t)(-1.0000000000000000e+00);
a[35] = (a[33]*a[34]);
a[36] = (a[35]*a[14]);
a[37] = (a[32]+a[36]);
a[38] = (a[27]+a[37]);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[41] = (xd[3]-od[12]);
a[42] = (a[40]*a[41]);
a[43] = (((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]));
a[44] = (a[42]+a[43]);
a[45] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[46] = (a[44]+a[45]);
a[47] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[48] = (a[46]*a[47]);
a[49] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[50] = (xd[3]-od[12]);
a[51] = (a[49]*a[50]);
a[52] = (((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]));
a[53] = (a[51]+a[52]);
a[54] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[55] = (a[53]+a[54]);
a[56] = ((real_t)(2.0000000000000000e+00)*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[57] = (a[55]*a[56]);
a[58] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[59] = (xd[3]-od[12]);
a[60] = (a[58]*a[59]);
a[61] = (((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]));
a[62] = (a[60]+a[61]);
a[63] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[64] = (a[62]+a[63]);
a[65] = ((real_t)(2.0000000000000000e+00)*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[66] = (a[64]*a[65]);
a[67] = (1.0/sqrt(((pow(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]),2))+(pow(((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]),2)))));
a[68] = (a[67]*(real_t)(5.0000000000000000e-01));
a[69] = ((a[57]+a[66])*a[68]);
a[70] = (a[47]*a[47]);
a[71] = ((real_t)(-1.0000000000000000e+00)*a[70]);
a[72] = (a[71]*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[73] = (a[69]*a[72]);
a[74] = (xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3]));
a[75] = ((a[48]+a[73])*a[74]);
a[76] = (xd[3]-od[12]);
a[77] = (od[0]*a[76]);
a[78] = (od[0]*(xd[3]-od[12]));
a[79] = (xd[3]-od[12]);
a[80] = ((a[77]+a[78])*a[79]);
a[81] = ((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[82] = (a[80]+a[81]);
a[83] = (xd[3]-od[12]);
a[84] = (od[1]*a[83]);
a[85] = (od[1]*(xd[3]-od[12]));
a[86] = (a[84]+a[85]);
a[87] = (a[82]+a[86]);
a[88] = (a[87]+od[2]);
a[89] = (real_t)(-1.0000000000000000e+00);
a[90] = (a[88]*a[89]);
a[91] = (a[90]*a[1]);
a[92] = (a[75]+a[91]);
a[93] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[94] = (xd[3]-od[12]);
a[95] = (a[93]*a[94]);
a[96] = (((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]));
a[97] = (a[95]+a[96]);
a[98] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[99] = (a[97]+a[98]);
a[100] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[101] = (a[99]*a[100]);
a[102] = (a[100]*a[100]);
a[103] = ((real_t)(-1.0000000000000000e+00)*a[102]);
a[104] = (a[103]*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[105] = (a[69]*a[104]);
a[106] = (xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]));
a[107] = ((a[101]+a[105])*a[106]);
a[108] = (xd[3]-od[12]);
a[109] = (od[4]*a[108]);
a[110] = (od[4]*(xd[3]-od[12]));
a[111] = (xd[3]-od[12]);
a[112] = ((a[109]+a[110])*a[111]);
a[113] = ((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[114] = (a[112]+a[113]);
a[115] = (xd[3]-od[12]);
a[116] = (od[5]*a[115]);
a[117] = (od[5]*(xd[3]-od[12]));
a[118] = (a[116]+a[117]);
a[119] = (a[114]+a[118]);
a[120] = (a[119]+od[6]);
a[121] = (real_t)(-1.0000000000000000e+00);
a[122] = (a[120]*a[121]);
a[123] = (a[122]*a[18]);
a[124] = ((a[107]+a[123])*a[19]);
a[125] = ((a[92]+a[124])*od[8]);
a[126] = (a[125]*a[3]);
a[127] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[128] = (xd[3]-od[12]);
a[129] = (a[127]*a[128]);
a[130] = (((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]));
a[131] = (a[129]+a[130]);
a[132] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[133] = (a[131]+a[132]);
a[134] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[135] = (a[133]*a[134]);
a[136] = (a[134]*a[134]);
a[137] = ((real_t)(-1.0000000000000000e+00)*a[136]);
a[138] = (a[137]*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[139] = (a[69]*a[138]);
a[140] = (xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3]));
a[141] = ((a[135]+a[139])*a[140]);
a[142] = (xd[3]-od[12]);
a[143] = (od[0]*a[142]);
a[144] = (od[0]*(xd[3]-od[12]));
a[145] = (xd[3]-od[12]);
a[146] = ((a[143]+a[144])*a[145]);
a[147] = ((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[148] = (a[146]+a[147]);
a[149] = (xd[3]-od[12]);
a[150] = (od[1]*a[149]);
a[151] = (od[1]*(xd[3]-od[12]));
a[152] = (a[150]+a[151]);
a[153] = (a[148]+a[152]);
a[154] = (a[153]+od[2]);
a[155] = (real_t)(-1.0000000000000000e+00);
a[156] = (a[154]*a[155]);
a[157] = (a[156]*a[5]);
a[158] = (a[141]+a[157]);
a[159] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[160] = (xd[3]-od[12]);
a[161] = (a[159]*a[160]);
a[162] = (((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]));
a[163] = (a[161]+a[162]);
a[164] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[165] = (a[163]+a[164]);
a[166] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[167] = (a[165]*a[166]);
a[168] = (a[166]*a[166]);
a[169] = ((real_t)(-1.0000000000000000e+00)*a[168]);
a[170] = (a[169]*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[171] = (a[69]*a[170]);
a[172] = (xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]));
a[173] = ((a[167]+a[171])*a[172]);
a[174] = (xd[3]-od[12]);
a[175] = (od[4]*a[174]);
a[176] = (od[4]*(xd[3]-od[12]));
a[177] = (xd[3]-od[12]);
a[178] = ((a[175]+a[176])*a[177]);
a[179] = ((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[180] = (a[178]+a[179]);
a[181] = (xd[3]-od[12]);
a[182] = (od[5]*a[181]);
a[183] = (od[5]*(xd[3]-od[12]));
a[184] = (a[182]+a[183]);
a[185] = (a[180]+a[184]);
a[186] = (a[185]+od[6]);
a[187] = (real_t)(-1.0000000000000000e+00);
a[188] = (a[186]*a[187]);
a[189] = (a[188]*a[23]);
a[190] = ((a[173]+a[189])*a[24]);
a[191] = ((a[158]+a[190])*a[6]);
a[192] = (a[126]+a[191]);
a[193] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[194] = (xd[3]-od[12]);
a[195] = (a[193]*a[194]);
a[196] = (((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]));
a[197] = (a[195]+a[196]);
a[198] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[199] = (a[197]+a[198]);
a[200] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[201] = (a[199]*a[200]);
a[202] = (a[200]*a[200]);
a[203] = ((real_t)(-1.0000000000000000e+00)*a[202]);
a[204] = (a[203]*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[205] = (a[69]*a[204]);
a[206] = (real_t)(-1.0000000000000000e+00);
a[207] = ((a[201]+a[205])*a[206]);
a[208] = (xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3]));
a[209] = (a[207]*a[208]);
a[210] = (xd[3]-od[12]);
a[211] = (od[0]*a[210]);
a[212] = (od[0]*(xd[3]-od[12]));
a[213] = (xd[3]-od[12]);
a[214] = ((a[211]+a[212])*a[213]);
a[215] = ((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[216] = (a[214]+a[215]);
a[217] = (xd[3]-od[12]);
a[218] = (od[1]*a[217]);
a[219] = (od[1]*(xd[3]-od[12]));
a[220] = (a[218]+a[219]);
a[221] = (a[216]+a[220]);
a[222] = (a[221]+od[2]);
a[223] = (real_t)(-1.0000000000000000e+00);
a[224] = (a[222]*a[223]);
a[225] = (a[224]*a[9]);
a[226] = (a[209]+a[225]);
a[227] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[228] = (xd[3]-od[12]);
a[229] = (a[227]*a[228]);
a[230] = (((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]));
a[231] = (a[229]+a[230]);
a[232] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[233] = (a[231]+a[232]);
a[234] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[235] = (a[233]*a[234]);
a[236] = (a[234]*a[234]);
a[237] = ((real_t)(-1.0000000000000000e+00)*a[236]);
a[238] = (a[237]*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[239] = (a[69]*a[238]);
a[240] = (xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]));
a[241] = ((a[235]+a[239])*a[240]);
a[242] = (xd[3]-od[12]);
a[243] = (od[4]*a[242]);
a[244] = (od[4]*(xd[3]-od[12]));
a[245] = (xd[3]-od[12]);
a[246] = ((a[243]+a[244])*a[245]);
a[247] = ((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[248] = (a[246]+a[247]);
a[249] = (xd[3]-od[12]);
a[250] = (od[5]*a[249]);
a[251] = (od[5]*(xd[3]-od[12]));
a[252] = (a[250]+a[251]);
a[253] = (a[248]+a[252]);
a[254] = (a[253]+od[6]);
a[255] = (real_t)(-1.0000000000000000e+00);
a[256] = (a[254]*a[255]);
a[257] = (a[256]*a[28]);
a[258] = ((a[241]+a[257])*a[29]);
a[259] = ((a[226]+a[258])*od[9]);
a[260] = (a[259]*a[11]);
a[261] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[262] = (xd[3]-od[12]);
a[263] = (a[261]*a[262]);
a[264] = (((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]));
a[265] = (a[263]+a[264]);
a[266] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[267] = (a[265]+a[266]);
a[268] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[269] = (a[267]*a[268]);
a[270] = (a[268]*a[268]);
a[271] = ((real_t)(-1.0000000000000000e+00)*a[270]);
a[272] = (a[271]*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[273] = (a[69]*a[272]);
a[274] = (real_t)(-1.0000000000000000e+00);
a[275] = ((a[269]+a[273])*a[274]);
a[276] = (xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3]));
a[277] = (a[275]*a[276]);
a[278] = (xd[3]-od[12]);
a[279] = (od[0]*a[278]);
a[280] = (od[0]*(xd[3]-od[12]));
a[281] = (xd[3]-od[12]);
a[282] = ((a[279]+a[280])*a[281]);
a[283] = ((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[284] = (a[282]+a[283]);
a[285] = (xd[3]-od[12]);
a[286] = (od[1]*a[285]);
a[287] = (od[1]*(xd[3]-od[12]));
a[288] = (a[286]+a[287]);
a[289] = (a[284]+a[288]);
a[290] = (a[289]+od[2]);
a[291] = (real_t)(-1.0000000000000000e+00);
a[292] = (a[290]*a[291]);
a[293] = (a[292]*a[13]);
a[294] = (a[277]+a[293]);
a[295] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[296] = (xd[3]-od[12]);
a[297] = (a[295]*a[296]);
a[298] = (((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]));
a[299] = (a[297]+a[298]);
a[300] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[301] = (a[299]+a[300]);
a[302] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[303] = (a[301]*a[302]);
a[304] = (a[302]*a[302]);
a[305] = ((real_t)(-1.0000000000000000e+00)*a[304]);
a[306] = (a[305]*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[307] = (a[69]*a[306]);
a[308] = (xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]));
a[309] = ((a[303]+a[307])*a[308]);
a[310] = (xd[3]-od[12]);
a[311] = (od[4]*a[310]);
a[312] = (od[4]*(xd[3]-od[12]));
a[313] = (xd[3]-od[12]);
a[314] = ((a[311]+a[312])*a[313]);
a[315] = ((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]));
a[316] = (a[314]+a[315]);
a[317] = (xd[3]-od[12]);
a[318] = (od[5]*a[317]);
a[319] = (od[5]*(xd[3]-od[12]));
a[320] = (a[318]+a[319]);
a[321] = (a[316]+a[320]);
a[322] = (a[321]+od[6]);
a[323] = (real_t)(-1.0000000000000000e+00);
a[324] = (a[322]*a[323]);
a[325] = (a[324]*a[33]);
a[326] = ((a[309]+a[325])*a[34]);
a[327] = ((a[294]+a[326])*a[14]);
a[328] = (a[260]+a[327]);
a[329] = (a[192]+a[328]);
a[330] = (u[0]-(real_t)(2.0000000000000001e-01));
a[331] = (od[10]*a[330]);
a[332] = (od[10]*(u[0]-(real_t)(2.0000000000000001e-01)));
a[333] = (a[331]+a[332]);
a[334] = (od[11]*u[1]);
a[335] = (od[11]*u[1]);
a[336] = (a[334]+a[335]);
a[337] = (((a[2]*a[5])*(real_t)(2.0000000000000000e+00))+((a[10]*a[13])*(real_t)(2.0000000000000000e+00)));
a[338] = (((a[21]*a[5])+(a[25]*a[2]))+((a[31]*a[13])+(a[35]*a[10])));
a[339] = (real_t)(0.0000000000000000e+00);
a[340] = (a[3]*od[8]);
a[341] = (a[11]*od[9]);
a[342] = (((((a[125]*a[5])+((a[158]+a[190])*a[2]))+((a[48]+a[73])*a[340]))+((a[135]+a[139])*a[6]))+((((a[259]*a[13])+((a[294]+a[326])*a[10]))+(a[207]*a[341]))+(a[275]*a[14])));
a[343] = (((a[21]*a[25])*(real_t)(2.0000000000000000e+00))+((a[31]*a[35])*(real_t)(2.0000000000000000e+00)));
a[344] = (real_t)(0.0000000000000000e+00);
a[345] = (a[340]*a[19]);
a[346] = (a[6]*a[24]);
a[347] = (a[341]*a[29]);
a[348] = (a[14]*a[34]);
a[349] = (((((a[125]*a[25])+((a[158]+a[190])*a[21]))+((a[101]+a[105])*a[345]))+((a[167]+a[171])*a[346]))+((((a[259]*a[35])+((a[294]+a[326])*a[31]))+((a[235]+a[239])*a[347]))+((a[303]+a[307])*a[348])));
a[350] = (real_t)(0.0000000000000000e+00);
a[351] = (real_t)(0.0000000000000000e+00);
a[352] = (a[340]*a[74]);
a[353] = (a[47]*a[70]);
a[354] = (((real_t)(2.0000000000000000e+00)*a[353])*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[355] = (a[352]*a[354]);
a[356] = (a[352]*a[71]);
a[357] = (a[352]*a[47]);
a[358] = (pow(((pow(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]),2))+(pow(((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]),2))),((real_t)(5.0000000000000000e-01)-(real_t)(2.0000000000000000e+00))));
a[359] = (((real_t)(-2.5000000000000000e-01))*a[358]);
a[360] = (a[68]*a[56]);
a[361] = (real_t)(2.0000000000000000e+00);
a[362] = (a[361]*a[68]);
a[363] = (a[68]*a[65]);
a[364] = (real_t)(2.0000000000000000e+00);
a[365] = (a[364]*a[68]);
a[366] = (((pow((a[57]+a[66]),2))*a[359])+((((a[49]*(real_t)(2.0000000000000000e+00))*a[360])+(((a[55])*(a[55]))*a[362]))+(((a[58]*(real_t)(2.0000000000000000e+00))*a[363])+(((a[64])*(a[64]))*a[365]))));
a[367] = (a[352]*a[72]);
a[368] = (a[340]*a[1]);
a[369] = (a[368]*a[89]);
a[370] = (a[369]*a[79]);
a[371] = (a[345]*a[106]);
a[372] = (a[100]*a[102]);
a[373] = (((real_t)(2.0000000000000000e+00)*a[372])*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[374] = (a[371]*a[373]);
a[375] = (a[371]*a[103]);
a[376] = (a[371]*a[100]);
a[377] = (a[371]*a[104]);
a[378] = (a[345]*a[18]);
a[379] = (a[378]*a[121]);
a[380] = (a[379]*a[111]);
a[381] = (a[6]*a[140]);
a[382] = (a[134]*a[136]);
a[383] = (((real_t)(2.0000000000000000e+00)*a[382])*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[384] = (a[381]*a[383]);
a[385] = (a[381]*a[137]);
a[386] = (a[381]*a[134]);
a[387] = (a[381]*a[138]);
a[388] = (a[6]*a[5]);
a[389] = (a[388]*a[155]);
a[390] = (a[389]*a[145]);
a[391] = (a[346]*a[172]);
a[392] = (a[166]*a[168]);
a[393] = (((real_t)(2.0000000000000000e+00)*a[392])*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[394] = (a[391]*a[393]);
a[395] = (a[391]*a[169]);
a[396] = (a[391]*a[166]);
a[397] = (a[391]*a[170]);
a[398] = (a[346]*a[23]);
a[399] = (a[398]*a[187]);
a[400] = (a[399]*a[177]);
a[401] = (a[341]*a[208]);
a[402] = (a[401]*a[206]);
a[403] = (a[200]*a[202]);
a[404] = (((real_t)(2.0000000000000000e+00)*a[403])*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[405] = (a[402]*a[404]);
a[406] = (a[402]*a[203]);
a[407] = (a[402]*a[200]);
a[408] = (a[402]*a[204]);
a[409] = (a[341]*a[9]);
a[410] = (a[409]*a[223]);
a[411] = (a[410]*a[213]);
a[412] = (a[347]*a[240]);
a[413] = (a[234]*a[236]);
a[414] = (((real_t)(2.0000000000000000e+00)*a[413])*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[415] = (a[412]*a[414]);
a[416] = (a[412]*a[237]);
a[417] = (a[412]*a[234]);
a[418] = (a[412]*a[238]);
a[419] = (a[347]*a[28]);
a[420] = (a[419]*a[255]);
a[421] = (a[420]*a[245]);
a[422] = (a[14]*a[276]);
a[423] = (a[422]*a[274]);
a[424] = (a[268]*a[270]);
a[425] = (((real_t)(2.0000000000000000e+00)*a[424])*((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2]));
a[426] = (a[423]*a[425]);
a[427] = (a[423]*a[271]);
a[428] = (a[423]*a[268]);
a[429] = (a[423]*a[272]);
a[430] = (a[14]*a[13]);
a[431] = (a[430]*a[291]);
a[432] = (a[431]*a[281]);
a[433] = (a[348]*a[308]);
a[434] = (a[302]*a[304]);
a[435] = (((real_t)(2.0000000000000000e+00)*a[434])*((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6]));
a[436] = (a[433]*a[435]);
a[437] = (a[433]*a[305]);
a[438] = (a[433]*a[302]);
a[439] = (a[433]*a[306]);
a[440] = (a[348]*a[33]);
a[441] = (a[440]*a[323]);
a[442] = (a[441]*a[313]);
a[443] = (((((a[125]*(a[158]+a[190]))*(real_t)(2.0000000000000000e+00))+(((((((a[48]+a[73])*a[90])*(real_t)(2.0000000000000000e+00))*a[340])+((((((a[69])*(a[69]))*a[355])+(((a[46]*a[69])*(real_t)(2.0000000000000000e+00))*a[356]))+((a[40]*(real_t)(2.0000000000000000e+00))*a[357]))+(a[366]*a[367])))+(((((a[77]+a[78])*(real_t)(2.0000000000000000e+00))*a[369])+((od[0]*(real_t)(2.0000000000000000e+00))*a[370]))+((od[1]*(real_t)(2.0000000000000000e+00))*a[369])))+((((((a[101]+a[105])*a[122])*(real_t)(2.0000000000000000e+00))*a[345])+((((((a[69])*(a[69]))*a[374])+(((a[99]*a[69])*(real_t)(2.0000000000000000e+00))*a[375]))+((a[93]*(real_t)(2.0000000000000000e+00))*a[376]))+(a[366]*a[377])))+(((((a[109]+a[110])*(real_t)(2.0000000000000000e+00))*a[379])+((od[4]*(real_t)(2.0000000000000000e+00))*a[380]))+((od[5]*(real_t)(2.0000000000000000e+00))*a[379])))))+(((((((a[135]+a[139])*a[156])*(real_t)(2.0000000000000000e+00))*a[6])+((((((a[69])*(a[69]))*a[384])+(((a[133]*a[69])*(real_t)(2.0000000000000000e+00))*a[385]))+((a[127]*(real_t)(2.0000000000000000e+00))*a[386]))+(a[366]*a[387])))+(((((a[143]+a[144])*(real_t)(2.0000000000000000e+00))*a[389])+((od[0]*(real_t)(2.0000000000000000e+00))*a[390]))+((od[1]*(real_t)(2.0000000000000000e+00))*a[389])))+((((((a[167]+a[171])*a[188])*(real_t)(2.0000000000000000e+00))*a[346])+((((((a[69])*(a[69]))*a[394])+(((a[165]*a[69])*(real_t)(2.0000000000000000e+00))*a[395]))+((a[159]*(real_t)(2.0000000000000000e+00))*a[396]))+(a[366]*a[397])))+(((((a[175]+a[176])*(real_t)(2.0000000000000000e+00))*a[399])+((od[4]*(real_t)(2.0000000000000000e+00))*a[400]))+((od[5]*(real_t)(2.0000000000000000e+00))*a[399])))))+((((a[259]*(a[294]+a[326]))*(real_t)(2.0000000000000000e+00))+((((((a[207]*a[224])*(real_t)(2.0000000000000000e+00))*a[341])+((((((a[69])*(a[69]))*a[405])+(((a[199]*a[69])*(real_t)(2.0000000000000000e+00))*a[406]))+((a[193]*(real_t)(2.0000000000000000e+00))*a[407]))+(a[366]*a[408])))+(((((a[211]+a[212])*(real_t)(2.0000000000000000e+00))*a[410])+((od[0]*(real_t)(2.0000000000000000e+00))*a[411]))+((od[1]*(real_t)(2.0000000000000000e+00))*a[410])))+((((((a[235]+a[239])*a[256])*(real_t)(2.0000000000000000e+00))*a[347])+((((((a[69])*(a[69]))*a[415])+(((a[233]*a[69])*(real_t)(2.0000000000000000e+00))*a[416]))+((a[227]*(real_t)(2.0000000000000000e+00))*a[417]))+(a[366]*a[418])))+(((((a[243]+a[244])*(real_t)(2.0000000000000000e+00))*a[420])+((od[4]*(real_t)(2.0000000000000000e+00))*a[421]))+((od[5]*(real_t)(2.0000000000000000e+00))*a[420])))))+((((((a[275]*a[292])*(real_t)(2.0000000000000000e+00))*a[14])+((((((a[69])*(a[69]))*a[426])+(((a[267]*a[69])*(real_t)(2.0000000000000000e+00))*a[427]))+((a[261]*(real_t)(2.0000000000000000e+00))*a[428]))+(a[366]*a[429])))+(((((a[279]+a[280])*(real_t)(2.0000000000000000e+00))*a[431])+((od[0]*(real_t)(2.0000000000000000e+00))*a[432]))+((od[1]*(real_t)(2.0000000000000000e+00))*a[431])))+((((((a[303]+a[307])*a[324])*(real_t)(2.0000000000000000e+00))*a[348])+((((((a[69])*(a[69]))*a[436])+(((a[301]*a[69])*(real_t)(2.0000000000000000e+00))*a[437]))+((a[295]*(real_t)(2.0000000000000000e+00))*a[438]))+(a[366]*a[439])))+(((((a[311]+a[312])*(real_t)(2.0000000000000000e+00))*a[441])+((od[4]*(real_t)(2.0000000000000000e+00))*a[442]))+((od[5]*(real_t)(2.0000000000000000e+00))*a[441]))))));
a[444] = (real_t)(0.0000000000000000e+00);
a[445] = (real_t)(0.0000000000000000e+00);
a[446] = (real_t)(0.0000000000000000e+00);
a[447] = (real_t)(0.0000000000000000e+00);
a[448] = (real_t)(0.0000000000000000e+00);
a[449] = (real_t)(0.0000000000000000e+00);
a[450] = (real_t)(0.0000000000000000e+00);
a[451] = (real_t)(0.0000000000000000e+00);
a[452] = (od[10]*(real_t)(2.0000000000000000e+00));
a[453] = (real_t)(0.0000000000000000e+00);
a[454] = (od[11]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = (((((od[8]*(((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7])))))*(((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7])))))+((od[9]*((((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]))*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7])))))*((((real_t)(0.0000000000000000e+00)-(((((((real_t)(3.0000000000000000e+00)*od[0])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[1])*(xd[3]-od[12])))+od[2])/a[0]))*(xd[0]-((((((od[0]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[1]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[2]*(xd[3]-od[12])))+od[3])))-((((((((real_t)(3.0000000000000000e+00)*od[4])*(xd[3]-od[12]))*(xd[3]-od[12]))+(((real_t)(2.0000000000000000e+00)*od[5])*(xd[3]-od[12])))+od[6])/a[0])*(xd[1]-((((((od[4]*(xd[3]-od[12]))*(xd[3]-od[12]))*(xd[3]-od[12]))+((od[5]*(xd[3]-od[12]))*(xd[3]-od[12])))+(od[6]*(xd[3]-od[12])))+od[7]))))))+((od[11]*u[1])*u[1]))+((od[10]*(u[0]-(real_t)(2.0000000000000001e-01)))*(u[0]-(real_t)(2.0000000000000001e-01))));
out[1] = a[17];
out[2] = a[38];
out[3] = a[39];
out[4] = a[329];
out[5] = a[333];
out[6] = a[336];
out[7] = a[337];
out[8] = a[338];
out[9] = a[339];
out[10] = a[342];
out[11] = a[338];
out[12] = a[343];
out[13] = a[344];
out[14] = a[349];
out[15] = a[339];
out[16] = a[344];
out[17] = a[350];
out[18] = a[351];
out[19] = a[342];
out[20] = a[349];
out[21] = a[351];
out[22] = a[443];
out[23] = a[444];
out[24] = a[445];
out[25] = a[446];
out[26] = a[447];
out[27] = a[448];
out[28] = a[449];
out[29] = a[450];
out[30] = a[451];
out[31] = a[452];
out[32] = a[453];
out[33] = a[453];
out[34] = a[454];
}

void acado_addObjTerm( real_t* const tmpFxx, real_t* const tmpFxu, real_t* const tmpFuu, real_t* const tmpEH )
{
tmpEH[0] += tmpFxx[0];
tmpEH[1] += tmpFxx[1];
tmpEH[2] += tmpFxx[2];
tmpEH[3] += tmpFxx[3];
tmpEH[6] += tmpFxx[4];
tmpEH[7] += tmpFxx[5];
tmpEH[8] += tmpFxx[6];
tmpEH[9] += tmpFxx[7];
tmpEH[12] += tmpFxx[8];
tmpEH[13] += tmpFxx[9];
tmpEH[14] += tmpFxx[10];
tmpEH[15] += tmpFxx[11];
tmpEH[18] += tmpFxx[12];
tmpEH[19] += tmpFxx[13];
tmpEH[20] += tmpFxx[14];
tmpEH[21] += tmpFxx[15];
tmpEH[4] += tmpFxu[0];
tmpEH[5] += tmpFxu[1];
tmpEH[10] += tmpFxu[2];
tmpEH[11] += tmpFxu[3];
tmpEH[16] += tmpFxu[4];
tmpEH[17] += tmpFxu[5];
tmpEH[22] += tmpFxu[6];
tmpEH[23] += tmpFxu[7];
tmpEH[24] += tmpFxu[0];
tmpEH[25] += tmpFxu[2];
tmpEH[26] += tmpFxu[4];
tmpEH[27] += tmpFxu[6];
tmpEH[30] += tmpFxu[1];
tmpEH[31] += tmpFxu[3];
tmpEH[32] += tmpFxu[5];
tmpEH[33] += tmpFxu[7];
tmpEH[28] += tmpFuu[0];
tmpEH[29] += tmpFuu[1];
tmpEH[34] += tmpFuu[2];
tmpEH[35] += tmpFuu[3];
}

void acado_addObjLinearTerm( real_t* const tmpDx, real_t* const tmpDu, real_t* const tmpDF )
{
tmpDx[0] = tmpDF[0];
tmpDx[1] = tmpDF[1];
tmpDx[2] = tmpDF[2];
tmpDx[3] = tmpDF[3];
tmpDu[0] = tmpDF[4];
tmpDu[1] = tmpDF[5];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 13];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 13 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 13 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 13 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 13 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 13 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 13 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 13 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 13 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 13 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 13 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 13 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 13 + 12];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 7 ]), &(acadoWorkspace.objValueOut[ 23 ]), &(acadoWorkspace.objValueOut[ 31 ]), &(acadoWorkspace.EH[ runObj * 36 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 4 ]), &(acadoWorkspace.g[ runObj * 2 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.QDy[100] = 0.0000000000000000e+00;
acadoWorkspace.QDy[101] = 0.0000000000000000e+00;
acadoWorkspace.QDy[102] = 0.0000000000000000e+00;
acadoWorkspace.QDy[103] = 0.0000000000000000e+00;
}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 36 ]) );
acadoWorkspace.Q1[lRun1 * 16] = acadoWorkspace.EH[lRun1 * 36];
acadoWorkspace.Q1[lRun1 * 16 + 1] = acadoWorkspace.EH[lRun1 * 36 + 1];
acadoWorkspace.Q1[lRun1 * 16 + 2] = acadoWorkspace.EH[lRun1 * 36 + 2];
acadoWorkspace.Q1[lRun1 * 16 + 3] = acadoWorkspace.EH[lRun1 * 36 + 3];
acadoWorkspace.Q1[lRun1 * 16 + 4] = acadoWorkspace.EH[lRun1 * 36 + 6];
acadoWorkspace.Q1[lRun1 * 16 + 5] = acadoWorkspace.EH[lRun1 * 36 + 7];
acadoWorkspace.Q1[lRun1 * 16 + 6] = acadoWorkspace.EH[lRun1 * 36 + 8];
acadoWorkspace.Q1[lRun1 * 16 + 7] = acadoWorkspace.EH[lRun1 * 36 + 9];
acadoWorkspace.Q1[lRun1 * 16 + 8] = acadoWorkspace.EH[lRun1 * 36 + 12];
acadoWorkspace.Q1[lRun1 * 16 + 9] = acadoWorkspace.EH[lRun1 * 36 + 13];
acadoWorkspace.Q1[lRun1 * 16 + 10] = acadoWorkspace.EH[lRun1 * 36 + 14];
acadoWorkspace.Q1[lRun1 * 16 + 11] = acadoWorkspace.EH[lRun1 * 36 + 15];
acadoWorkspace.Q1[lRun1 * 16 + 12] = acadoWorkspace.EH[lRun1 * 36 + 18];
acadoWorkspace.Q1[lRun1 * 16 + 13] = acadoWorkspace.EH[lRun1 * 36 + 19];
acadoWorkspace.Q1[lRun1 * 16 + 14] = acadoWorkspace.EH[lRun1 * 36 + 20];
acadoWorkspace.Q1[lRun1 * 16 + 15] = acadoWorkspace.EH[lRun1 * 36 + 21];
acadoWorkspace.S1[lRun1 * 8] = acadoWorkspace.EH[lRun1 * 36 + 4];
acadoWorkspace.S1[lRun1 * 8 + 1] = acadoWorkspace.EH[lRun1 * 36 + 5];
acadoWorkspace.S1[lRun1 * 8 + 2] = acadoWorkspace.EH[lRun1 * 36 + 10];
acadoWorkspace.S1[lRun1 * 8 + 3] = acadoWorkspace.EH[lRun1 * 36 + 11];
acadoWorkspace.S1[lRun1 * 8 + 4] = acadoWorkspace.EH[lRun1 * 36 + 16];
acadoWorkspace.S1[lRun1 * 8 + 5] = acadoWorkspace.EH[lRun1 * 36 + 17];
acadoWorkspace.S1[lRun1 * 8 + 6] = acadoWorkspace.EH[lRun1 * 36 + 22];
acadoWorkspace.S1[lRun1 * 8 + 7] = acadoWorkspace.EH[lRun1 * 36 + 23];
acadoWorkspace.R1[lRun1 * 4] = acadoWorkspace.EH[lRun1 * 36 + 28];
acadoWorkspace.R1[lRun1 * 4 + 1] = acadoWorkspace.EH[lRun1 * 36 + 29];
acadoWorkspace.R1[lRun1 * 4 + 2] = acadoWorkspace.EH[lRun1 * 36 + 34];
acadoWorkspace.R1[lRun1 * 4 + 3] = acadoWorkspace.EH[lRun1 * 36 + 35];
}
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 102] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + R11[0];
acadoWorkspace.H[iRow * 102 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + R11[1];
acadoWorkspace.H[iRow * 102 + 50] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + R11[2];
acadoWorkspace.H[iRow * 102 + 51] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + R11[3];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[8]*Gu1[4] + Gx1[12]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[8]*Gu1[5] + Gx1[12]*Gu1[7];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[9]*Gu1[4] + Gx1[13]*Gu1[6];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[9]*Gu1[5] + Gx1[13]*Gu1[7];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[14]*Gu1[6];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[14]*Gu1[7];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[11]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[11]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Gu2[1];
Gu3[2] = + Q11[4]*Gu1[0] + Q11[5]*Gu1[2] + Q11[6]*Gu1[4] + Q11[7]*Gu1[6] + Gu2[2];
Gu3[3] = + Q11[4]*Gu1[1] + Q11[5]*Gu1[3] + Q11[6]*Gu1[5] + Q11[7]*Gu1[7] + Gu2[3];
Gu3[4] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[2] + Q11[10]*Gu1[4] + Q11[11]*Gu1[6] + Gu2[4];
Gu3[5] = + Q11[8]*Gu1[1] + Q11[9]*Gu1[3] + Q11[10]*Gu1[5] + Q11[11]*Gu1[7] + Gu2[5];
Gu3[6] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[2] + Q11[14]*Gu1[4] + Q11[15]*Gu1[6] + Gu2[6];
Gu3[7] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[3] + Q11[14]*Gu1[5] + Q11[15]*Gu1[7] + Gu2[7];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[4]*w11[1] + Gx1[8]*w11[2] + Gx1[12]*w11[3] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[5]*w11[1] + Gx1[9]*w11[2] + Gx1[13]*w11[3] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[6]*w11[1] + Gx1[10]*w11[2] + Gx1[14]*w11[3] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[7]*w11[1] + Gx1[11]*w11[2] + Gx1[15]*w11[3] + w12[3];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + w12[0];
w13[1] = + Q11[4]*w11[0] + Q11[5]*w11[1] + Q11[6]*w11[2] + Q11[7]*w11[3] + w12[1];
w13[2] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + w12[2];
w13[3] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + w12[3];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
}

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[3] += QDy1[3];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2] + w11[3]*Q11[3];
mu1[1] += + w11[0]*Q11[4] + w11[1]*Q11[5] + w11[2]*Q11[6] + w11[3]*Q11[7];
mu1[2] += + w11[0]*Q11[8] + w11[1]*Q11[9] + w11[2]*Q11[10] + w11[3]*Q11[11];
mu1[3] += + w11[0]*Q11[12] + w11[1]*Q11[13] + w11[2]*Q11[14] + w11[3]*Q11[15];
mu1[0] += + U1[0]*Gu1[0] + U1[1]*Gu1[1];
mu1[1] += + U1[0]*Gu1[2] + U1[1]*Gu1[3];
mu1[2] += + U1[0]*Gu1[4] + U1[1]*Gu1[5];
mu1[3] += + U1[0]*Gu1[6] + U1[1]*Gu1[7];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[4] + mu2[2]*Gx1[8] + mu2[3]*Gx1[12];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[5] + mu2[2]*Gx1[9] + mu2[3]*Gx1[13];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[6] + mu2[2]*Gx1[10] + mu2[3]*Gx1[14];
mu1[3] += + mu2[0]*Gx1[3] + mu2[1]*Gx1[7] + mu2[2]*Gx1[11] + mu2[3]*Gx1[15];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 51)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 25; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (4)) * (4)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (4)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (4)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (25)) - (1)) * (4)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 24; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 8 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 8 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (2)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 16 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 8 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];
acadoWorkspace.sbar[44] = acadoWorkspace.d[40];
acadoWorkspace.sbar[45] = acadoWorkspace.d[41];
acadoWorkspace.sbar[46] = acadoWorkspace.d[42];
acadoWorkspace.sbar[47] = acadoWorkspace.d[43];
acadoWorkspace.sbar[48] = acadoWorkspace.d[44];
acadoWorkspace.sbar[49] = acadoWorkspace.d[45];
acadoWorkspace.sbar[50] = acadoWorkspace.d[46];
acadoWorkspace.sbar[51] = acadoWorkspace.d[47];
acadoWorkspace.sbar[52] = acadoWorkspace.d[48];
acadoWorkspace.sbar[53] = acadoWorkspace.d[49];
acadoWorkspace.sbar[54] = acadoWorkspace.d[50];
acadoWorkspace.sbar[55] = acadoWorkspace.d[51];
acadoWorkspace.sbar[56] = acadoWorkspace.d[52];
acadoWorkspace.sbar[57] = acadoWorkspace.d[53];
acadoWorkspace.sbar[58] = acadoWorkspace.d[54];
acadoWorkspace.sbar[59] = acadoWorkspace.d[55];
acadoWorkspace.sbar[60] = acadoWorkspace.d[56];
acadoWorkspace.sbar[61] = acadoWorkspace.d[57];
acadoWorkspace.sbar[62] = acadoWorkspace.d[58];
acadoWorkspace.sbar[63] = acadoWorkspace.d[59];
acadoWorkspace.sbar[64] = acadoWorkspace.d[60];
acadoWorkspace.sbar[65] = acadoWorkspace.d[61];
acadoWorkspace.sbar[66] = acadoWorkspace.d[62];
acadoWorkspace.sbar[67] = acadoWorkspace.d[63];
acadoWorkspace.sbar[68] = acadoWorkspace.d[64];
acadoWorkspace.sbar[69] = acadoWorkspace.d[65];
acadoWorkspace.sbar[70] = acadoWorkspace.d[66];
acadoWorkspace.sbar[71] = acadoWorkspace.d[67];
acadoWorkspace.sbar[72] = acadoWorkspace.d[68];
acadoWorkspace.sbar[73] = acadoWorkspace.d[69];
acadoWorkspace.sbar[74] = acadoWorkspace.d[70];
acadoWorkspace.sbar[75] = acadoWorkspace.d[71];
acadoWorkspace.sbar[76] = acadoWorkspace.d[72];
acadoWorkspace.sbar[77] = acadoWorkspace.d[73];
acadoWorkspace.sbar[78] = acadoWorkspace.d[74];
acadoWorkspace.sbar[79] = acadoWorkspace.d[75];
acadoWorkspace.sbar[80] = acadoWorkspace.d[76];
acadoWorkspace.sbar[81] = acadoWorkspace.d[77];
acadoWorkspace.sbar[82] = acadoWorkspace.d[78];
acadoWorkspace.sbar[83] = acadoWorkspace.d[79];
acadoWorkspace.sbar[84] = acadoWorkspace.d[80];
acadoWorkspace.sbar[85] = acadoWorkspace.d[81];
acadoWorkspace.sbar[86] = acadoWorkspace.d[82];
acadoWorkspace.sbar[87] = acadoWorkspace.d[83];
acadoWorkspace.sbar[88] = acadoWorkspace.d[84];
acadoWorkspace.sbar[89] = acadoWorkspace.d[85];
acadoWorkspace.sbar[90] = acadoWorkspace.d[86];
acadoWorkspace.sbar[91] = acadoWorkspace.d[87];
acadoWorkspace.sbar[92] = acadoWorkspace.d[88];
acadoWorkspace.sbar[93] = acadoWorkspace.d[89];
acadoWorkspace.sbar[94] = acadoWorkspace.d[90];
acadoWorkspace.sbar[95] = acadoWorkspace.d[91];
acadoWorkspace.sbar[96] = acadoWorkspace.d[92];
acadoWorkspace.sbar[97] = acadoWorkspace.d[93];
acadoWorkspace.sbar[98] = acadoWorkspace.d[94];
acadoWorkspace.sbar[99] = acadoWorkspace.d[95];
acadoWorkspace.sbar[100] = acadoWorkspace.d[96];
acadoWorkspace.sbar[101] = acadoWorkspace.d[97];
acadoWorkspace.sbar[102] = acadoWorkspace.d[98];
acadoWorkspace.sbar[103] = acadoWorkspace.d[99];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[100];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[101];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[102];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[103];
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 184 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 184 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 368 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 92 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 176 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 176 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 352 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 88 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 160 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 152 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 152 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 136 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 136 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 104 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 104 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 88 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 88 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 56 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 40 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];
acadoWorkspace.sbar[44] = acadoWorkspace.d[40];
acadoWorkspace.sbar[45] = acadoWorkspace.d[41];
acadoWorkspace.sbar[46] = acadoWorkspace.d[42];
acadoWorkspace.sbar[47] = acadoWorkspace.d[43];
acadoWorkspace.sbar[48] = acadoWorkspace.d[44];
acadoWorkspace.sbar[49] = acadoWorkspace.d[45];
acadoWorkspace.sbar[50] = acadoWorkspace.d[46];
acadoWorkspace.sbar[51] = acadoWorkspace.d[47];
acadoWorkspace.sbar[52] = acadoWorkspace.d[48];
acadoWorkspace.sbar[53] = acadoWorkspace.d[49];
acadoWorkspace.sbar[54] = acadoWorkspace.d[50];
acadoWorkspace.sbar[55] = acadoWorkspace.d[51];
acadoWorkspace.sbar[56] = acadoWorkspace.d[52];
acadoWorkspace.sbar[57] = acadoWorkspace.d[53];
acadoWorkspace.sbar[58] = acadoWorkspace.d[54];
acadoWorkspace.sbar[59] = acadoWorkspace.d[55];
acadoWorkspace.sbar[60] = acadoWorkspace.d[56];
acadoWorkspace.sbar[61] = acadoWorkspace.d[57];
acadoWorkspace.sbar[62] = acadoWorkspace.d[58];
acadoWorkspace.sbar[63] = acadoWorkspace.d[59];
acadoWorkspace.sbar[64] = acadoWorkspace.d[60];
acadoWorkspace.sbar[65] = acadoWorkspace.d[61];
acadoWorkspace.sbar[66] = acadoWorkspace.d[62];
acadoWorkspace.sbar[67] = acadoWorkspace.d[63];
acadoWorkspace.sbar[68] = acadoWorkspace.d[64];
acadoWorkspace.sbar[69] = acadoWorkspace.d[65];
acadoWorkspace.sbar[70] = acadoWorkspace.d[66];
acadoWorkspace.sbar[71] = acadoWorkspace.d[67];
acadoWorkspace.sbar[72] = acadoWorkspace.d[68];
acadoWorkspace.sbar[73] = acadoWorkspace.d[69];
acadoWorkspace.sbar[74] = acadoWorkspace.d[70];
acadoWorkspace.sbar[75] = acadoWorkspace.d[71];
acadoWorkspace.sbar[76] = acadoWorkspace.d[72];
acadoWorkspace.sbar[77] = acadoWorkspace.d[73];
acadoWorkspace.sbar[78] = acadoWorkspace.d[74];
acadoWorkspace.sbar[79] = acadoWorkspace.d[75];
acadoWorkspace.sbar[80] = acadoWorkspace.d[76];
acadoWorkspace.sbar[81] = acadoWorkspace.d[77];
acadoWorkspace.sbar[82] = acadoWorkspace.d[78];
acadoWorkspace.sbar[83] = acadoWorkspace.d[79];
acadoWorkspace.sbar[84] = acadoWorkspace.d[80];
acadoWorkspace.sbar[85] = acadoWorkspace.d[81];
acadoWorkspace.sbar[86] = acadoWorkspace.d[82];
acadoWorkspace.sbar[87] = acadoWorkspace.d[83];
acadoWorkspace.sbar[88] = acadoWorkspace.d[84];
acadoWorkspace.sbar[89] = acadoWorkspace.d[85];
acadoWorkspace.sbar[90] = acadoWorkspace.d[86];
acadoWorkspace.sbar[91] = acadoWorkspace.d[87];
acadoWorkspace.sbar[92] = acadoWorkspace.d[88];
acadoWorkspace.sbar[93] = acadoWorkspace.d[89];
acadoWorkspace.sbar[94] = acadoWorkspace.d[90];
acadoWorkspace.sbar[95] = acadoWorkspace.d[91];
acadoWorkspace.sbar[96] = acadoWorkspace.d[92];
acadoWorkspace.sbar[97] = acadoWorkspace.d[93];
acadoWorkspace.sbar[98] = acadoWorkspace.d[94];
acadoWorkspace.sbar[99] = acadoWorkspace.d[95];
acadoWorkspace.sbar[100] = acadoWorkspace.d[96];
acadoWorkspace.sbar[101] = acadoWorkspace.d[97];
acadoWorkspace.sbar[102] = acadoWorkspace.d[98];
acadoWorkspace.sbar[103] = acadoWorkspace.d[99];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGu[ 88 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGu[ 104 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGu[ 136 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.evGu[ 152 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.evGu[ 176 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.evGu[ 184 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.x[88] += acadoWorkspace.sbar[88];
acadoVariables.x[89] += acadoWorkspace.sbar[89];
acadoVariables.x[90] += acadoWorkspace.sbar[90];
acadoVariables.x[91] += acadoWorkspace.sbar[91];
acadoVariables.x[92] += acadoWorkspace.sbar[92];
acadoVariables.x[93] += acadoWorkspace.sbar[93];
acadoVariables.x[94] += acadoWorkspace.sbar[94];
acadoVariables.x[95] += acadoWorkspace.sbar[95];
acadoVariables.x[96] += acadoWorkspace.sbar[96];
acadoVariables.x[97] += acadoWorkspace.sbar[97];
acadoVariables.x[98] += acadoWorkspace.sbar[98];
acadoVariables.x[99] += acadoWorkspace.sbar[99];
acadoVariables.x[100] += acadoWorkspace.sbar[100];
acadoVariables.x[101] += acadoWorkspace.sbar[101];
acadoVariables.x[102] += acadoWorkspace.sbar[102];
acadoVariables.x[103] += acadoWorkspace.sbar[103];
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[96] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[12];
acadoVariables.mu[97] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[13];
acadoVariables.mu[98] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[14];
acadoVariables.mu[99] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[15];
acadoVariables.mu[96] += acadoWorkspace.QDy[100];
acadoVariables.mu[97] += acadoWorkspace.QDy[101];
acadoVariables.mu[98] += acadoWorkspace.QDy[102];
acadoVariables.mu[99] += acadoWorkspace.QDy[103];
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[95] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoVariables.mu[ 92 ]), &(acadoVariables.mu[ 96 ]) );
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[89] = 0.0000000000000000e+00;
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[91] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 92 ]), &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.S1[ 184 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoVariables.mu[ 88 ]), &(acadoVariables.mu[ 92 ]) );
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[87] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 88 ]), &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.S1[ 176 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoVariables.mu[ 84 ]), &(acadoVariables.mu[ 88 ]) );
acadoVariables.mu[80] = 0.0000000000000000e+00;
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[83] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoVariables.mu[ 80 ]), &(acadoVariables.mu[ 84 ]) );
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[77] = 0.0000000000000000e+00;
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[79] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.S1[ 160 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoVariables.mu[ 76 ]), &(acadoVariables.mu[ 80 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[75] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.S1[ 152 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 76 ]) );
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[71] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoVariables.mu[ 68 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[67] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.S1[ 136 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoVariables.mu[ 64 ]), &(acadoVariables.mu[ 68 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[63] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 64 ]) );
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[59] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoVariables.mu[ 56 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[53] = 0.0000000000000000e+00;
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[55] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoVariables.mu[ 52 ]), &(acadoVariables.mu[ 56 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[51] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.S1[ 104 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 52 ]) );
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[47] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoVariables.mu[ 44 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[43] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.S1[ 88 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoVariables.mu[ 40 ]), &(acadoVariables.mu[ 44 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[39] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 40 ]) );
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[35] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 32 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[31] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoVariables.mu[ 28 ]), &(acadoVariables.mu[ 32 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[27] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.S1[ 56 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 28 ]) );
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[23] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoVariables.mu[ 20 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[17] = 0.0000000000000000e+00;
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[19] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.S1[ 40 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoVariables.mu[ 16 ]), &(acadoVariables.mu[ 20 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[15] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 16 ]) );
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[11] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoVariables.mu[ 8 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[7] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.S1[ 16 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoVariables.mu[ 4 ]), &(acadoVariables.mu[ 8 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[3] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 4 ]), &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.S1[ 8 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.evGx[ 16 ]), acadoVariables.mu, &(acadoVariables.mu[ 4 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_regularizeHessian(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.0000000000000000e+00;
acadoVariables.lbValues[1] = -1.0000000000000000e+00;
acadoVariables.lbValues[2] = -1.0000000000000000e+00;
acadoVariables.lbValues[3] = -1.0000000000000000e+00;
acadoVariables.lbValues[4] = -1.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+00;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e+00;
acadoVariables.lbValues[8] = -1.0000000000000000e+00;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = -1.0000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e+00;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+00;
acadoVariables.lbValues[14] = -1.0000000000000000e+00;
acadoVariables.lbValues[15] = -1.0000000000000000e+00;
acadoVariables.lbValues[16] = -1.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+00;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e+00;
acadoVariables.lbValues[20] = -1.0000000000000000e+00;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = -1.0000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e+00;
acadoVariables.lbValues[24] = -1.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+00;
acadoVariables.lbValues[26] = -1.0000000000000000e+00;
acadoVariables.lbValues[27] = -1.0000000000000000e+00;
acadoVariables.lbValues[28] = -1.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+00;
acadoVariables.lbValues[30] = -1.0000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e+00;
acadoVariables.lbValues[32] = -1.0000000000000000e+00;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = -1.0000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e+00;
acadoVariables.lbValues[36] = -1.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+00;
acadoVariables.lbValues[38] = -1.0000000000000000e+00;
acadoVariables.lbValues[39] = -1.0000000000000000e+00;
acadoVariables.lbValues[40] = -1.0000000000000000e+00;
acadoVariables.lbValues[41] = -1.0000000000000000e+00;
acadoVariables.lbValues[42] = -1.0000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e+00;
acadoVariables.lbValues[44] = -1.0000000000000000e+00;
acadoVariables.lbValues[45] = -1.0000000000000000e+00;
acadoVariables.lbValues[46] = -1.0000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e+00;
acadoVariables.lbValues[48] = -1.0000000000000000e+00;
acadoVariables.lbValues[49] = -1.0000000000000000e+00;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 1.0000000000000000e+00;
acadoVariables.ubValues[2] = 1.0000000000000000e+00;
acadoVariables.ubValues[3] = 1.0000000000000000e+00;
acadoVariables.ubValues[4] = 1.0000000000000000e+00;
acadoVariables.ubValues[5] = 1.0000000000000000e+00;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 1.0000000000000000e+00;
acadoVariables.ubValues[8] = 1.0000000000000000e+00;
acadoVariables.ubValues[9] = 1.0000000000000000e+00;
acadoVariables.ubValues[10] = 1.0000000000000000e+00;
acadoVariables.ubValues[11] = 1.0000000000000000e+00;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 1.0000000000000000e+00;
acadoVariables.ubValues[14] = 1.0000000000000000e+00;
acadoVariables.ubValues[15] = 1.0000000000000000e+00;
acadoVariables.ubValues[16] = 1.0000000000000000e+00;
acadoVariables.ubValues[17] = 1.0000000000000000e+00;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 1.0000000000000000e+00;
acadoVariables.ubValues[20] = 1.0000000000000000e+00;
acadoVariables.ubValues[21] = 1.0000000000000000e+00;
acadoVariables.ubValues[22] = 1.0000000000000000e+00;
acadoVariables.ubValues[23] = 1.0000000000000000e+00;
acadoVariables.ubValues[24] = 1.0000000000000000e+00;
acadoVariables.ubValues[25] = 1.0000000000000000e+00;
acadoVariables.ubValues[26] = 1.0000000000000000e+00;
acadoVariables.ubValues[27] = 1.0000000000000000e+00;
acadoVariables.ubValues[28] = 1.0000000000000000e+00;
acadoVariables.ubValues[29] = 1.0000000000000000e+00;
acadoVariables.ubValues[30] = 1.0000000000000000e+00;
acadoVariables.ubValues[31] = 1.0000000000000000e+00;
acadoVariables.ubValues[32] = 1.0000000000000000e+00;
acadoVariables.ubValues[33] = 1.0000000000000000e+00;
acadoVariables.ubValues[34] = 1.0000000000000000e+00;
acadoVariables.ubValues[35] = 1.0000000000000000e+00;
acadoVariables.ubValues[36] = 1.0000000000000000e+00;
acadoVariables.ubValues[37] = 1.0000000000000000e+00;
acadoVariables.ubValues[38] = 1.0000000000000000e+00;
acadoVariables.ubValues[39] = 1.0000000000000000e+00;
acadoVariables.ubValues[40] = 1.0000000000000000e+00;
acadoVariables.ubValues[41] = 1.0000000000000000e+00;
acadoVariables.ubValues[42] = 1.0000000000000000e+00;
acadoVariables.ubValues[43] = 1.0000000000000000e+00;
acadoVariables.ubValues[44] = 1.0000000000000000e+00;
acadoVariables.ubValues[45] = 1.0000000000000000e+00;
acadoVariables.ubValues[46] = 1.0000000000000000e+00;
acadoVariables.ubValues[47] = 1.0000000000000000e+00;
acadoVariables.ubValues[48] = 1.0000000000000000e+00;
acadoVariables.ubValues[49] = 1.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[53] = acadoVariables.u[index * 2];
acadoWorkspace.state[54] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[55] = acadoVariables.od[index * 13];
acadoWorkspace.state[56] = acadoVariables.od[index * 13 + 1];
acadoWorkspace.state[57] = acadoVariables.od[index * 13 + 2];
acadoWorkspace.state[58] = acadoVariables.od[index * 13 + 3];
acadoWorkspace.state[59] = acadoVariables.od[index * 13 + 4];
acadoWorkspace.state[60] = acadoVariables.od[index * 13 + 5];
acadoWorkspace.state[61] = acadoVariables.od[index * 13 + 6];
acadoWorkspace.state[62] = acadoVariables.od[index * 13 + 7];
acadoWorkspace.state[63] = acadoVariables.od[index * 13 + 8];
acadoWorkspace.state[64] = acadoVariables.od[index * 13 + 9];
acadoWorkspace.state[65] = acadoVariables.od[index * 13 + 10];
acadoWorkspace.state[66] = acadoVariables.od[index * 13 + 11];
acadoWorkspace.state[67] = acadoVariables.od[index * 13 + 12];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[100] = xEnd[0];
acadoVariables.x[101] = xEnd[1];
acadoVariables.x[102] = xEnd[2];
acadoVariables.x[103] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[100];
acadoWorkspace.state[1] = acadoVariables.x[101];
acadoWorkspace.state[2] = acadoVariables.x[102];
acadoWorkspace.state[3] = acadoVariables.x[103];
if (uEnd != 0)
{
acadoWorkspace.state[53] = uEnd[0];
acadoWorkspace.state[54] = uEnd[1];
}
else
{
acadoWorkspace.state[53] = acadoVariables.u[48];
acadoWorkspace.state[54] = acadoVariables.u[49];
}
acadoWorkspace.state[55] = acadoVariables.od[325];
acadoWorkspace.state[56] = acadoVariables.od[326];
acadoWorkspace.state[57] = acadoVariables.od[327];
acadoWorkspace.state[58] = acadoVariables.od[328];
acadoWorkspace.state[59] = acadoVariables.od[329];
acadoWorkspace.state[60] = acadoVariables.od[330];
acadoWorkspace.state[61] = acadoVariables.od[331];
acadoWorkspace.state[62] = acadoVariables.od[332];
acadoWorkspace.state[63] = acadoVariables.od[333];
acadoWorkspace.state[64] = acadoVariables.od[334];
acadoWorkspace.state[65] = acadoVariables.od[335];
acadoWorkspace.state[66] = acadoVariables.od[336];
acadoWorkspace.state[67] = acadoVariables.od[337];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[100] = acadoWorkspace.state[0];
acadoVariables.x[101] = acadoWorkspace.state[1];
acadoVariables.x[102] = acadoWorkspace.state[2];
acadoVariables.x[103] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[48] = uEnd[0];
acadoVariables.u[49] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49];
kkt = fabs( kkt );
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 13];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 13 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 13 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 13 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 13 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 13 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 13 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 13 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 13 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 13 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 13 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 13 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 13 + 12];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
return objVal;
}

