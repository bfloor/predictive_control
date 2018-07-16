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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
/* Vector of auxiliary variables; number of elements: 449. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt(((pow((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]),2))+(pow((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]),2)))));
a[1] = ((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0]);
a[2] = (a[1]*od[8]);
a[3] = ((((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]))));
a[4] = (a[2]*a[3]);
a[5] = ((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0]);
a[6] = (od[8]*((((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7])))));
a[7] = (a[5]*a[6]);
a[8] = (a[4]+a[7]);
a[9] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]));
a[10] = (a[9]*od[9]);
a[11] = ((((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]))*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]))));
a[12] = (a[10]*a[11]);
a[13] = ((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]));
a[14] = (od[9]*((((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]))*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7])))));
a[15] = (a[13]*a[14]);
a[16] = (a[12]+a[15]);
a[17] = (a[8]+a[16]);
a[18] = ((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]);
a[19] = (real_t)(-1.0000000000000000e+00);
a[20] = (a[18]*a[19]);
a[21] = (a[20]*od[8]);
a[22] = (a[21]*a[3]);
a[23] = ((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]);
a[24] = (real_t)(-1.0000000000000000e+00);
a[25] = (a[23]*a[24]);
a[26] = (a[25]*a[6]);
a[27] = (a[22]+a[26]);
a[28] = ((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0]);
a[29] = (real_t)(-1.0000000000000000e+00);
a[30] = (a[28]*a[29]);
a[31] = (a[30]*od[9]);
a[32] = (a[31]*a[11]);
a[33] = ((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0]);
a[34] = (real_t)(-1.0000000000000000e+00);
a[35] = (a[33]*a[34]);
a[36] = (a[35]*a[14]);
a[37] = (a[32]+a[36]);
a[38] = (a[27]+a[37]);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[41] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[42] = (a[40]*a[41]);
a[43] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[44] = (a[42]+a[43]);
a[45] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[46] = (a[44]*a[45]);
a[47] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[48] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[49] = (a[47]*a[48]);
a[50] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[51] = (a[49]+a[50]);
a[52] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[53] = (a[51]*a[52]);
a[54] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[55] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[56] = (a[54]*a[55]);
a[57] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[58] = (a[56]+a[57]);
a[59] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[60] = (a[58]*a[59]);
a[61] = (1.0/sqrt(((pow((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]),2))+(pow((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]),2)))));
a[62] = (a[61]*(real_t)(5.0000000000000000e-01));
a[63] = ((a[53]+a[60])*a[62]);
a[64] = (a[45]*a[45]);
a[65] = ((real_t)(-1.0000000000000000e+00)*a[64]);
a[66] = (a[65]*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[67] = (a[63]*a[66]);
a[68] = (xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3]));
a[69] = ((a[46]+a[67])*a[68]);
a[70] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[71] = (a[70]*od[0]);
a[72] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[73] = (a[72]*od[1]);
a[74] = (a[71]+a[73]);
a[75] = (a[74]+od[2]);
a[76] = (real_t)(-1.0000000000000000e+00);
a[77] = (a[75]*a[76]);
a[78] = (a[77]*a[1]);
a[79] = (a[69]+a[78]);
a[80] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[81] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[82] = (a[80]*a[81]);
a[83] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[84] = (a[82]+a[83]);
a[85] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[86] = (a[84]*a[85]);
a[87] = (a[85]*a[85]);
a[88] = ((real_t)(-1.0000000000000000e+00)*a[87]);
a[89] = (a[88]*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[90] = (a[63]*a[89]);
a[91] = (xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]));
a[92] = ((a[86]+a[90])*a[91]);
a[93] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[94] = (a[93]*od[4]);
a[95] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[96] = (a[95]*od[5]);
a[97] = (a[94]+a[96]);
a[98] = (a[97]+od[6]);
a[99] = (real_t)(-1.0000000000000000e+00);
a[100] = (a[98]*a[99]);
a[101] = (a[100]*a[18]);
a[102] = ((a[92]+a[101])*a[19]);
a[103] = ((a[79]+a[102])*od[8]);
a[104] = (a[103]*a[3]);
a[105] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[106] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[107] = (a[105]*a[106]);
a[108] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[109] = (a[107]+a[108]);
a[110] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[111] = (a[109]*a[110]);
a[112] = (a[110]*a[110]);
a[113] = ((real_t)(-1.0000000000000000e+00)*a[112]);
a[114] = (a[113]*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[115] = (a[63]*a[114]);
a[116] = (xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3]));
a[117] = ((a[111]+a[115])*a[116]);
a[118] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[119] = (a[118]*od[0]);
a[120] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[121] = (a[120]*od[1]);
a[122] = (a[119]+a[121]);
a[123] = (a[122]+od[2]);
a[124] = (real_t)(-1.0000000000000000e+00);
a[125] = (a[123]*a[124]);
a[126] = (a[125]*a[5]);
a[127] = (a[117]+a[126]);
a[128] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[129] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[130] = (a[128]*a[129]);
a[131] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[132] = (a[130]+a[131]);
a[133] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[134] = (a[132]*a[133]);
a[135] = (a[133]*a[133]);
a[136] = ((real_t)(-1.0000000000000000e+00)*a[135]);
a[137] = (a[136]*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[138] = (a[63]*a[137]);
a[139] = (xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]));
a[140] = ((a[134]+a[138])*a[139]);
a[141] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[142] = (a[141]*od[4]);
a[143] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[144] = (a[143]*od[5]);
a[145] = (a[142]+a[144]);
a[146] = (a[145]+od[6]);
a[147] = (real_t)(-1.0000000000000000e+00);
a[148] = (a[146]*a[147]);
a[149] = (a[148]*a[23]);
a[150] = ((a[140]+a[149])*a[24]);
a[151] = ((a[127]+a[150])*a[6]);
a[152] = (a[104]+a[151]);
a[153] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[154] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[155] = (a[153]*a[154]);
a[156] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[157] = (a[155]+a[156]);
a[158] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[159] = (a[157]*a[158]);
a[160] = (a[158]*a[158]);
a[161] = ((real_t)(-1.0000000000000000e+00)*a[160]);
a[162] = (a[161]*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[163] = (a[63]*a[162]);
a[164] = (real_t)(-1.0000000000000000e+00);
a[165] = ((a[159]+a[163])*a[164]);
a[166] = (xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3]));
a[167] = (a[165]*a[166]);
a[168] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[169] = (a[168]*od[0]);
a[170] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[171] = (a[170]*od[1]);
a[172] = (a[169]+a[171]);
a[173] = (a[172]+od[2]);
a[174] = (real_t)(-1.0000000000000000e+00);
a[175] = (a[173]*a[174]);
a[176] = (a[175]*a[9]);
a[177] = (a[167]+a[176]);
a[178] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[179] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[180] = (a[178]*a[179]);
a[181] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[182] = (a[180]+a[181]);
a[183] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[184] = (a[182]*a[183]);
a[185] = (a[183]*a[183]);
a[186] = ((real_t)(-1.0000000000000000e+00)*a[185]);
a[187] = (a[186]*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[188] = (a[63]*a[187]);
a[189] = (xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]));
a[190] = ((a[184]+a[188])*a[189]);
a[191] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[192] = (a[191]*od[4]);
a[193] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[194] = (a[193]*od[5]);
a[195] = (a[192]+a[194]);
a[196] = (a[195]+od[6]);
a[197] = (real_t)(-1.0000000000000000e+00);
a[198] = (a[196]*a[197]);
a[199] = (a[198]*a[28]);
a[200] = ((a[190]+a[199])*a[29]);
a[201] = ((a[177]+a[200])*od[9]);
a[202] = (a[201]*a[11]);
a[203] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[204] = ((real_t)(3.0000000000000000e+00)*od[0]);
a[205] = (a[203]*a[204]);
a[206] = ((real_t)(2.0000000000000000e+00)*od[1]);
a[207] = (a[205]+a[206]);
a[208] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[209] = (a[207]*a[208]);
a[210] = (a[208]*a[208]);
a[211] = ((real_t)(-1.0000000000000000e+00)*a[210]);
a[212] = (a[211]*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[213] = (a[63]*a[212]);
a[214] = (real_t)(-1.0000000000000000e+00);
a[215] = ((a[209]+a[213])*a[214]);
a[216] = (xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3]));
a[217] = (a[215]*a[216]);
a[218] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[219] = (a[218]*od[0]);
a[220] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[221] = (a[220]*od[1]);
a[222] = (a[219]+a[221]);
a[223] = (a[222]+od[2]);
a[224] = (real_t)(-1.0000000000000000e+00);
a[225] = (a[223]*a[224]);
a[226] = (a[225]*a[13]);
a[227] = (a[217]+a[226]);
a[228] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[229] = ((real_t)(3.0000000000000000e+00)*od[4]);
a[230] = (a[228]*a[229]);
a[231] = ((real_t)(2.0000000000000000e+00)*od[5]);
a[232] = (a[230]+a[231]);
a[233] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[234] = (a[232]*a[233]);
a[235] = (a[233]*a[233]);
a[236] = ((real_t)(-1.0000000000000000e+00)*a[235]);
a[237] = (a[236]*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[238] = (a[63]*a[237]);
a[239] = (xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]));
a[240] = ((a[234]+a[238])*a[239]);
a[241] = ((real_t)(3.0000000000000000e+00)*((xd[3])*(xd[3])));
a[242] = (a[241]*od[4]);
a[243] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[244] = (a[243]*od[5]);
a[245] = (a[242]+a[244]);
a[246] = (a[245]+od[6]);
a[247] = (real_t)(-1.0000000000000000e+00);
a[248] = (a[246]*a[247]);
a[249] = (a[248]*a[33]);
a[250] = ((a[240]+a[249])*a[34]);
a[251] = ((a[227]+a[250])*a[14]);
a[252] = (a[202]+a[251]);
a[253] = (a[152]+a[252]);
a[254] = (u[0]-(real_t)(1.0000000000000001e-01));
a[255] = (od[10]*a[254]);
a[256] = (od[10]*(u[0]-(real_t)(1.0000000000000001e-01)));
a[257] = (a[255]+a[256]);
a[258] = (od[11]*u[1]);
a[259] = (od[11]*u[1]);
a[260] = (a[258]+a[259]);
a[261] = (((a[2]*a[5])*(real_t)(2.0000000000000000e+00))+((a[10]*a[13])*(real_t)(2.0000000000000000e+00)));
a[262] = (((a[21]*a[5])+(a[25]*a[2]))+((a[31]*a[13])+(a[35]*a[10])));
a[263] = (real_t)(0.0000000000000000e+00);
a[264] = (a[3]*od[8]);
a[265] = (a[11]*od[9]);
a[266] = (((((a[103]*a[5])+((a[127]+a[150])*a[2]))+((a[46]+a[67])*a[264]))+((a[111]+a[115])*a[6]))+((((a[201]*a[13])+((a[227]+a[250])*a[10]))+(a[165]*a[265]))+(a[215]*a[14])));
a[267] = (((a[21]*a[25])*(real_t)(2.0000000000000000e+00))+((a[31]*a[35])*(real_t)(2.0000000000000000e+00)));
a[268] = (real_t)(0.0000000000000000e+00);
a[269] = (a[264]*a[19]);
a[270] = (a[6]*a[24]);
a[271] = (a[265]*a[29]);
a[272] = (a[14]*a[34]);
a[273] = (((((a[103]*a[25])+((a[127]+a[150])*a[21]))+((a[86]+a[90])*a[269]))+((a[134]+a[138])*a[270]))+((((a[201]*a[35])+((a[227]+a[250])*a[31]))+((a[184]+a[188])*a[271]))+((a[234]+a[238])*a[272])));
a[274] = (real_t)(0.0000000000000000e+00);
a[275] = (real_t)(0.0000000000000000e+00);
a[276] = (a[264]*a[68]);
a[277] = (a[45]*a[64]);
a[278] = (((real_t)(2.0000000000000000e+00)*a[277])*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[279] = (a[276]*a[278]);
a[280] = (a[276]*a[65]);
a[281] = (real_t)(2.0000000000000000e+00);
a[282] = (a[276]*a[45]);
a[283] = (a[282]*a[41]);
a[284] = (a[281]*a[283]);
a[285] = (pow(((pow((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]),2))+(pow((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]),2))),((real_t)(5.0000000000000000e-01)-(real_t)(2.0000000000000000e+00))));
a[286] = (((real_t)(-2.5000000000000000e-01))*a[285]);
a[287] = (real_t)(2.0000000000000000e+00);
a[288] = (a[62]*a[52]);
a[289] = (a[288]*a[48]);
a[290] = (a[287]*a[289]);
a[291] = (real_t)(2.0000000000000000e+00);
a[292] = (a[291]*a[62]);
a[293] = (real_t)(2.0000000000000000e+00);
a[294] = (a[62]*a[59]);
a[295] = (a[294]*a[55]);
a[296] = (a[293]*a[295]);
a[297] = (real_t)(2.0000000000000000e+00);
a[298] = (a[297]*a[62]);
a[299] = (((pow((a[53]+a[60]),2))*a[286])+((a[290]+(((a[51])*(a[51]))*a[292]))+(a[296]+(((a[58])*(a[58]))*a[298]))));
a[300] = (a[276]*a[66]);
a[301] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[302] = (a[264]*a[1]);
a[303] = (a[302]*a[76]);
a[304] = (a[303]*od[0]);
a[305] = (a[301]*a[304]);
a[306] = (real_t)(2.0000000000000000e+00);
a[307] = (a[303]*od[1]);
a[308] = (a[306]*a[307]);
a[309] = (a[269]*a[91]);
a[310] = (a[85]*a[87]);
a[311] = (((real_t)(2.0000000000000000e+00)*a[310])*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[312] = (a[309]*a[311]);
a[313] = (a[309]*a[88]);
a[314] = (real_t)(2.0000000000000000e+00);
a[315] = (a[309]*a[85]);
a[316] = (a[315]*a[81]);
a[317] = (a[314]*a[316]);
a[318] = (a[309]*a[89]);
a[319] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[320] = (a[269]*a[18]);
a[321] = (a[320]*a[99]);
a[322] = (a[321]*od[4]);
a[323] = (a[319]*a[322]);
a[324] = (real_t)(2.0000000000000000e+00);
a[325] = (a[321]*od[5]);
a[326] = (a[324]*a[325]);
a[327] = (a[6]*a[116]);
a[328] = (a[110]*a[112]);
a[329] = (((real_t)(2.0000000000000000e+00)*a[328])*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[330] = (a[327]*a[329]);
a[331] = (a[327]*a[113]);
a[332] = (real_t)(2.0000000000000000e+00);
a[333] = (a[327]*a[110]);
a[334] = (a[333]*a[106]);
a[335] = (a[332]*a[334]);
a[336] = (a[327]*a[114]);
a[337] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[338] = (a[6]*a[5]);
a[339] = (a[338]*a[124]);
a[340] = (a[339]*od[0]);
a[341] = (a[337]*a[340]);
a[342] = (real_t)(2.0000000000000000e+00);
a[343] = (a[339]*od[1]);
a[344] = (a[342]*a[343]);
a[345] = (a[270]*a[139]);
a[346] = (a[133]*a[135]);
a[347] = (((real_t)(2.0000000000000000e+00)*a[346])*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[348] = (a[345]*a[347]);
a[349] = (a[345]*a[136]);
a[350] = (real_t)(2.0000000000000000e+00);
a[351] = (a[345]*a[133]);
a[352] = (a[351]*a[129]);
a[353] = (a[350]*a[352]);
a[354] = (a[345]*a[137]);
a[355] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[356] = (a[270]*a[23]);
a[357] = (a[356]*a[147]);
a[358] = (a[357]*od[4]);
a[359] = (a[355]*a[358]);
a[360] = (real_t)(2.0000000000000000e+00);
a[361] = (a[357]*od[5]);
a[362] = (a[360]*a[361]);
a[363] = (a[265]*a[166]);
a[364] = (a[363]*a[164]);
a[365] = (a[158]*a[160]);
a[366] = (((real_t)(2.0000000000000000e+00)*a[365])*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[367] = (a[364]*a[366]);
a[368] = (a[364]*a[161]);
a[369] = (real_t)(2.0000000000000000e+00);
a[370] = (a[364]*a[158]);
a[371] = (a[370]*a[154]);
a[372] = (a[369]*a[371]);
a[373] = (a[364]*a[162]);
a[374] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[375] = (a[265]*a[9]);
a[376] = (a[375]*a[174]);
a[377] = (a[376]*od[0]);
a[378] = (a[374]*a[377]);
a[379] = (real_t)(2.0000000000000000e+00);
a[380] = (a[376]*od[1]);
a[381] = (a[379]*a[380]);
a[382] = (a[271]*a[189]);
a[383] = (a[183]*a[185]);
a[384] = (((real_t)(2.0000000000000000e+00)*a[383])*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[385] = (a[382]*a[384]);
a[386] = (a[382]*a[186]);
a[387] = (real_t)(2.0000000000000000e+00);
a[388] = (a[382]*a[183]);
a[389] = (a[388]*a[179]);
a[390] = (a[387]*a[389]);
a[391] = (a[382]*a[187]);
a[392] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[393] = (a[271]*a[28]);
a[394] = (a[393]*a[197]);
a[395] = (a[394]*od[4]);
a[396] = (a[392]*a[395]);
a[397] = (real_t)(2.0000000000000000e+00);
a[398] = (a[394]*od[5]);
a[399] = (a[397]*a[398]);
a[400] = (a[14]*a[216]);
a[401] = (a[400]*a[214]);
a[402] = (a[208]*a[210]);
a[403] = (((real_t)(2.0000000000000000e+00)*a[402])*(((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2]));
a[404] = (a[401]*a[403]);
a[405] = (a[401]*a[211]);
a[406] = (real_t)(2.0000000000000000e+00);
a[407] = (a[401]*a[208]);
a[408] = (a[407]*a[204]);
a[409] = (a[406]*a[408]);
a[410] = (a[401]*a[212]);
a[411] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[412] = (a[14]*a[13]);
a[413] = (a[412]*a[224]);
a[414] = (a[413]*od[0]);
a[415] = (a[411]*a[414]);
a[416] = (real_t)(2.0000000000000000e+00);
a[417] = (a[413]*od[1]);
a[418] = (a[416]*a[417]);
a[419] = (a[272]*a[239]);
a[420] = (a[233]*a[235]);
a[421] = (((real_t)(2.0000000000000000e+00)*a[420])*(((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6]));
a[422] = (a[419]*a[421]);
a[423] = (a[419]*a[236]);
a[424] = (real_t)(2.0000000000000000e+00);
a[425] = (a[419]*a[233]);
a[426] = (a[425]*a[229]);
a[427] = (a[424]*a[426]);
a[428] = (a[419]*a[237]);
a[429] = (((real_t)(6.0000000000000000e+00))*xd[3]);
a[430] = (a[272]*a[33]);
a[431] = (a[430]*a[247]);
a[432] = (a[431]*od[4]);
a[433] = (a[429]*a[432]);
a[434] = (real_t)(2.0000000000000000e+00);
a[435] = (a[431]*od[5]);
a[436] = (a[434]*a[435]);
a[437] = (((((a[103]*(a[127]+a[150]))*(real_t)(2.0000000000000000e+00))+(((((((a[46]+a[67])*a[77])*(real_t)(2.0000000000000000e+00))*a[264])+((((((a[63])*(a[63]))*a[279])+(((a[44]*a[63])*(real_t)(2.0000000000000000e+00))*a[280]))+a[284])+(a[299]*a[300])))+(a[305]+a[308]))+((((((a[86]+a[90])*a[100])*(real_t)(2.0000000000000000e+00))*a[269])+((((((a[63])*(a[63]))*a[312])+(((a[84]*a[63])*(real_t)(2.0000000000000000e+00))*a[313]))+a[317])+(a[299]*a[318])))+(a[323]+a[326]))))+(((((((a[111]+a[115])*a[125])*(real_t)(2.0000000000000000e+00))*a[6])+((((((a[63])*(a[63]))*a[330])+(((a[109]*a[63])*(real_t)(2.0000000000000000e+00))*a[331]))+a[335])+(a[299]*a[336])))+(a[341]+a[344]))+((((((a[134]+a[138])*a[148])*(real_t)(2.0000000000000000e+00))*a[270])+((((((a[63])*(a[63]))*a[348])+(((a[132]*a[63])*(real_t)(2.0000000000000000e+00))*a[349]))+a[353])+(a[299]*a[354])))+(a[359]+a[362]))))+((((a[201]*(a[227]+a[250]))*(real_t)(2.0000000000000000e+00))+((((((a[165]*a[175])*(real_t)(2.0000000000000000e+00))*a[265])+((((((a[63])*(a[63]))*a[367])+(((a[157]*a[63])*(real_t)(2.0000000000000000e+00))*a[368]))+a[372])+(a[299]*a[373])))+(a[378]+a[381]))+((((((a[184]+a[188])*a[198])*(real_t)(2.0000000000000000e+00))*a[271])+((((((a[63])*(a[63]))*a[385])+(((a[182]*a[63])*(real_t)(2.0000000000000000e+00))*a[386]))+a[390])+(a[299]*a[391])))+(a[396]+a[399]))))+((((((a[215]*a[225])*(real_t)(2.0000000000000000e+00))*a[14])+((((((a[63])*(a[63]))*a[404])+(((a[207]*a[63])*(real_t)(2.0000000000000000e+00))*a[405]))+a[409])+(a[299]*a[410])))+(a[415]+a[418]))+((((((a[234]+a[238])*a[248])*(real_t)(2.0000000000000000e+00))*a[272])+((((((a[63])*(a[63]))*a[422])+(((a[232]*a[63])*(real_t)(2.0000000000000000e+00))*a[423]))+a[427])+(a[299]*a[428])))+(a[433]+a[436])))));
a[438] = (real_t)(0.0000000000000000e+00);
a[439] = (real_t)(0.0000000000000000e+00);
a[440] = (real_t)(0.0000000000000000e+00);
a[441] = (real_t)(0.0000000000000000e+00);
a[442] = (real_t)(0.0000000000000000e+00);
a[443] = (real_t)(0.0000000000000000e+00);
a[444] = (real_t)(0.0000000000000000e+00);
a[445] = (real_t)(0.0000000000000000e+00);
a[446] = (od[10]*(real_t)(2.0000000000000000e+00));
a[447] = (real_t)(0.0000000000000000e+00);
a[448] = (od[11]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = (((((od[8]*((((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7])))))*((((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7])))))+((od[9]*((((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]))*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7])))))*((((real_t)(0.0000000000000000e+00)-((((((real_t)(3.0000000000000000e+00)*od[0])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[1])*xd[3]))+od[2])/a[0]))*(xd[0]-((((od[0]*(pow(xd[3],3)))+(od[1]*((xd[3])*(xd[3]))))+(od[2]*xd[3]))+od[3])))-(((((((real_t)(3.0000000000000000e+00)*od[4])*((xd[3])*(xd[3])))+(((real_t)(2.0000000000000000e+00)*od[5])*xd[3]))+od[6])/a[0])*(xd[1]-((((od[4]*(pow(xd[3],3)))+(od[5]*((xd[3])*(xd[3]))))+(od[6]*xd[3]))+od[7]))))))+((od[11]*u[1])*u[1]))+((od[10]*(u[0]-(real_t)(1.0000000000000001e-01)))*(u[0]-(real_t)(1.0000000000000001e-01))));
out[1] = a[17];
out[2] = a[38];
out[3] = a[39];
out[4] = a[253];
out[5] = a[257];
out[6] = a[260];
out[7] = a[261];
out[8] = a[262];
out[9] = a[263];
out[10] = a[266];
out[11] = a[262];
out[12] = a[267];
out[13] = a[268];
out[14] = a[273];
out[15] = a[263];
out[16] = a[268];
out[17] = a[274];
out[18] = a[275];
out[19] = a[266];
out[20] = a[273];
out[21] = a[275];
out[22] = a[437];
out[23] = a[438];
out[24] = a[439];
out[25] = a[440];
out[26] = a[441];
out[27] = a[442];
out[28] = a[443];
out[29] = a[444];
out[30] = a[445];
out[31] = a[446];
out[32] = a[447];
out[33] = a[447];
out[34] = a[448];
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
for (runObj = 0; runObj < 50; ++runObj)
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
acadoWorkspace.QDy[200] = 0.0000000000000000e+00;
acadoWorkspace.QDy[201] = 0.0000000000000000e+00;
acadoWorkspace.QDy[202] = 0.0000000000000000e+00;
acadoWorkspace.QDy[203] = 0.0000000000000000e+00;
}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 202] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + R11[0];
acadoWorkspace.H[iRow * 202 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + R11[1];
acadoWorkspace.H[iRow * 202 + 100] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + R11[2];
acadoWorkspace.H[iRow * 202 + 101] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + R11[3];
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
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] = acadoWorkspace.H[(iCol * 200) + (iRow * 2)];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 200 + 100) + (iRow * 2)];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] = acadoWorkspace.H[(iCol * 200) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 200 + 100) + (iRow * 2 + 1)];
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
for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 101)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 50; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (4)) * (4)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (4)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (4)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (50)) - (1)) * (4)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 49; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 8 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 8 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (2)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 16 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 8 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];


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
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 164 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 172 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.sbar[ 176 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 184 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.sbar[ 188 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 200 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[200];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[201];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[202];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[203];
acado_macBTw1( &(acadoWorkspace.evGu[ 392 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 98 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 392 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.g[ 98 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 784 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 196 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 384 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 768 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 192 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 376 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 94 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 376 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.g[ 94 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 752 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 188 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 368 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 368 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 736 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 184 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 360 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 352 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 352 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 704 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 176 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 344 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 86 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 344 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.g[ 86 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 688 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 172 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 336 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 672 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 328 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 82 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 328 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.g[ 82 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 656 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 164 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 320 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 640 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 160 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 312 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 312 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 304 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 304 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 608 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 152 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 296 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 74 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 296 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.g[ 74 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 148 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 70 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 280 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.g[ 70 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 272 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 272 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 544 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 136 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 264 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 528 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 256 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 128 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 248 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 62 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 248 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.g[ 62 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 496 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 124 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 232 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 232 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 464 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 116 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 224 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 112 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 208 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 208 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 416 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 200 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
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
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];

}

void acado_expand(  )
{
int lRun1;
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
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];

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
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.evGu[ 208 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGu[ 224 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.evGu[ 232 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.evGu[ 248 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGu[ 256 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.evGu[ 264 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.evGu[ 272 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.evGu[ 296 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.evGu[ 304 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.evGu[ 312 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 164 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 656 ]), &(acadoWorkspace.evGu[ 328 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 672 ]), &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 172 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 688 ]), &(acadoWorkspace.evGu[ 344 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.sbar[ 176 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.evGu[ 352 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 184 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 736 ]), &(acadoWorkspace.evGu[ 368 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.sbar[ 188 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 752 ]), &(acadoWorkspace.evGu[ 376 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.evGu[ 392 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 200 ]) );
for (lRun1 = 0; lRun1 < 204; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

acadoVariables.mu[196] = 0.0000000000000000e+00;
acadoVariables.mu[197] = 0.0000000000000000e+00;
acadoVariables.mu[198] = 0.0000000000000000e+00;
acadoVariables.mu[199] = 0.0000000000000000e+00;
acadoVariables.mu[196] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[12];
acadoVariables.mu[197] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[13];
acadoVariables.mu[198] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[14];
acadoVariables.mu[199] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[15];
acadoVariables.mu[196] += acadoWorkspace.QDy[200];
acadoVariables.mu[197] += acadoWorkspace.QDy[201];
acadoVariables.mu[198] += acadoWorkspace.QDy[202];
acadoVariables.mu[199] += acadoWorkspace.QDy[203];
acadoVariables.mu[192] = 0.0000000000000000e+00;
acadoVariables.mu[193] = 0.0000000000000000e+00;
acadoVariables.mu[194] = 0.0000000000000000e+00;
acadoVariables.mu[195] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 196 ]), &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.S1[ 392 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.evGx[ 784 ]), &(acadoVariables.mu[ 192 ]), &(acadoVariables.mu[ 196 ]) );
acadoVariables.mu[188] = 0.0000000000000000e+00;
acadoVariables.mu[189] = 0.0000000000000000e+00;
acadoVariables.mu[190] = 0.0000000000000000e+00;
acadoVariables.mu[191] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 192 ]), &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.S1[ 384 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.evGx[ 768 ]), &(acadoVariables.mu[ 188 ]), &(acadoVariables.mu[ 192 ]) );
acadoVariables.mu[184] = 0.0000000000000000e+00;
acadoVariables.mu[185] = 0.0000000000000000e+00;
acadoVariables.mu[186] = 0.0000000000000000e+00;
acadoVariables.mu[187] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 188 ]), &(acadoWorkspace.Q1[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.S1[ 376 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.evGx[ 752 ]), &(acadoVariables.mu[ 184 ]), &(acadoVariables.mu[ 188 ]) );
acadoVariables.mu[180] = 0.0000000000000000e+00;
acadoVariables.mu[181] = 0.0000000000000000e+00;
acadoVariables.mu[182] = 0.0000000000000000e+00;
acadoVariables.mu[183] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 184 ]), &(acadoWorkspace.Q1[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.S1[ 368 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.evGx[ 736 ]), &(acadoVariables.mu[ 180 ]), &(acadoVariables.mu[ 184 ]) );
acadoVariables.mu[176] = 0.0000000000000000e+00;
acadoVariables.mu[177] = 0.0000000000000000e+00;
acadoVariables.mu[178] = 0.0000000000000000e+00;
acadoVariables.mu[179] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 180 ]), &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.S1[ 360 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoVariables.mu[ 176 ]), &(acadoVariables.mu[ 180 ]) );
acadoVariables.mu[172] = 0.0000000000000000e+00;
acadoVariables.mu[173] = 0.0000000000000000e+00;
acadoVariables.mu[174] = 0.0000000000000000e+00;
acadoVariables.mu[175] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 176 ]), &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.S1[ 352 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.evGx[ 704 ]), &(acadoVariables.mu[ 172 ]), &(acadoVariables.mu[ 176 ]) );
acadoVariables.mu[168] = 0.0000000000000000e+00;
acadoVariables.mu[169] = 0.0000000000000000e+00;
acadoVariables.mu[170] = 0.0000000000000000e+00;
acadoVariables.mu[171] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 172 ]), &(acadoWorkspace.Q1[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.S1[ 344 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.evGx[ 688 ]), &(acadoVariables.mu[ 168 ]), &(acadoVariables.mu[ 172 ]) );
acadoVariables.mu[164] = 0.0000000000000000e+00;
acadoVariables.mu[165] = 0.0000000000000000e+00;
acadoVariables.mu[166] = 0.0000000000000000e+00;
acadoVariables.mu[167] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 168 ]), &(acadoWorkspace.Q1[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.S1[ 336 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.evGx[ 672 ]), &(acadoVariables.mu[ 164 ]), &(acadoVariables.mu[ 168 ]) );
acadoVariables.mu[160] = 0.0000000000000000e+00;
acadoVariables.mu[161] = 0.0000000000000000e+00;
acadoVariables.mu[162] = 0.0000000000000000e+00;
acadoVariables.mu[163] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 164 ]), &(acadoWorkspace.Q1[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.S1[ 328 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.evGx[ 656 ]), &(acadoVariables.mu[ 160 ]), &(acadoVariables.mu[ 164 ]) );
acadoVariables.mu[156] = 0.0000000000000000e+00;
acadoVariables.mu[157] = 0.0000000000000000e+00;
acadoVariables.mu[158] = 0.0000000000000000e+00;
acadoVariables.mu[159] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 160 ]), &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.S1[ 320 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.evGx[ 640 ]), &(acadoVariables.mu[ 156 ]), &(acadoVariables.mu[ 160 ]) );
acadoVariables.mu[152] = 0.0000000000000000e+00;
acadoVariables.mu[153] = 0.0000000000000000e+00;
acadoVariables.mu[154] = 0.0000000000000000e+00;
acadoVariables.mu[155] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 156 ]), &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.S1[ 312 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.evGx[ 624 ]), &(acadoVariables.mu[ 152 ]), &(acadoVariables.mu[ 156 ]) );
acadoVariables.mu[148] = 0.0000000000000000e+00;
acadoVariables.mu[149] = 0.0000000000000000e+00;
acadoVariables.mu[150] = 0.0000000000000000e+00;
acadoVariables.mu[151] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 152 ]), &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.S1[ 304 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.evGx[ 608 ]), &(acadoVariables.mu[ 148 ]), &(acadoVariables.mu[ 152 ]) );
acadoVariables.mu[144] = 0.0000000000000000e+00;
acadoVariables.mu[145] = 0.0000000000000000e+00;
acadoVariables.mu[146] = 0.0000000000000000e+00;
acadoVariables.mu[147] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 148 ]), &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.S1[ 296 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.evGx[ 592 ]), &(acadoVariables.mu[ 144 ]), &(acadoVariables.mu[ 148 ]) );
acadoVariables.mu[140] = 0.0000000000000000e+00;
acadoVariables.mu[141] = 0.0000000000000000e+00;
acadoVariables.mu[142] = 0.0000000000000000e+00;
acadoVariables.mu[143] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoVariables.mu[ 140 ]), &(acadoVariables.mu[ 144 ]) );
acadoVariables.mu[136] = 0.0000000000000000e+00;
acadoVariables.mu[137] = 0.0000000000000000e+00;
acadoVariables.mu[138] = 0.0000000000000000e+00;
acadoVariables.mu[139] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 140 ]), &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.S1[ 280 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.evGx[ 560 ]), &(acadoVariables.mu[ 136 ]), &(acadoVariables.mu[ 140 ]) );
acadoVariables.mu[132] = 0.0000000000000000e+00;
acadoVariables.mu[133] = 0.0000000000000000e+00;
acadoVariables.mu[134] = 0.0000000000000000e+00;
acadoVariables.mu[135] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 136 ]), &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.S1[ 272 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.evGx[ 544 ]), &(acadoVariables.mu[ 132 ]), &(acadoVariables.mu[ 136 ]) );
acadoVariables.mu[128] = 0.0000000000000000e+00;
acadoVariables.mu[129] = 0.0000000000000000e+00;
acadoVariables.mu[130] = 0.0000000000000000e+00;
acadoVariables.mu[131] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.evGx[ 528 ]), &(acadoVariables.mu[ 128 ]), &(acadoVariables.mu[ 132 ]) );
acadoVariables.mu[124] = 0.0000000000000000e+00;
acadoVariables.mu[125] = 0.0000000000000000e+00;
acadoVariables.mu[126] = 0.0000000000000000e+00;
acadoVariables.mu[127] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 128 ]), &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.S1[ 256 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoVariables.mu[ 124 ]), &(acadoVariables.mu[ 128 ]) );
acadoVariables.mu[120] = 0.0000000000000000e+00;
acadoVariables.mu[121] = 0.0000000000000000e+00;
acadoVariables.mu[122] = 0.0000000000000000e+00;
acadoVariables.mu[123] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 124 ]), &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.S1[ 248 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.evGx[ 496 ]), &(acadoVariables.mu[ 120 ]), &(acadoVariables.mu[ 124 ]) );
acadoVariables.mu[116] = 0.0000000000000000e+00;
acadoVariables.mu[117] = 0.0000000000000000e+00;
acadoVariables.mu[118] = 0.0000000000000000e+00;
acadoVariables.mu[119] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.evGx[ 480 ]), &(acadoVariables.mu[ 116 ]), &(acadoVariables.mu[ 120 ]) );
acadoVariables.mu[112] = 0.0000000000000000e+00;
acadoVariables.mu[113] = 0.0000000000000000e+00;
acadoVariables.mu[114] = 0.0000000000000000e+00;
acadoVariables.mu[115] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 116 ]), &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.S1[ 232 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.evGx[ 464 ]), &(acadoVariables.mu[ 112 ]), &(acadoVariables.mu[ 116 ]) );
acadoVariables.mu[108] = 0.0000000000000000e+00;
acadoVariables.mu[109] = 0.0000000000000000e+00;
acadoVariables.mu[110] = 0.0000000000000000e+00;
acadoVariables.mu[111] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 112 ]), &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.S1[ 224 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoVariables.mu[ 108 ]), &(acadoVariables.mu[ 112 ]) );
acadoVariables.mu[104] = 0.0000000000000000e+00;
acadoVariables.mu[105] = 0.0000000000000000e+00;
acadoVariables.mu[106] = 0.0000000000000000e+00;
acadoVariables.mu[107] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoVariables.mu[ 104 ]), &(acadoVariables.mu[ 108 ]) );
acadoVariables.mu[100] = 0.0000000000000000e+00;
acadoVariables.mu[101] = 0.0000000000000000e+00;
acadoVariables.mu[102] = 0.0000000000000000e+00;
acadoVariables.mu[103] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 104 ]), &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.S1[ 208 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.evGx[ 416 ]), &(acadoVariables.mu[ 100 ]), &(acadoVariables.mu[ 104 ]) );
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[99] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.S1[ 200 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoVariables.mu[ 96 ]), &(acadoVariables.mu[ 100 ]) );
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
acadoVariables.lbValues[0] = 0.0000000000000000e+00;
acadoVariables.lbValues[1] = -1.0000000000000000e+00;
acadoVariables.lbValues[2] = 0.0000000000000000e+00;
acadoVariables.lbValues[3] = -1.0000000000000000e+00;
acadoVariables.lbValues[4] = 0.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+00;
acadoVariables.lbValues[6] = 0.0000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e+00;
acadoVariables.lbValues[8] = 0.0000000000000000e+00;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = 0.0000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e+00;
acadoVariables.lbValues[12] = 0.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+00;
acadoVariables.lbValues[14] = 0.0000000000000000e+00;
acadoVariables.lbValues[15] = -1.0000000000000000e+00;
acadoVariables.lbValues[16] = 0.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+00;
acadoVariables.lbValues[18] = 0.0000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e+00;
acadoVariables.lbValues[20] = 0.0000000000000000e+00;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = 0.0000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e+00;
acadoVariables.lbValues[24] = 0.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+00;
acadoVariables.lbValues[26] = 0.0000000000000000e+00;
acadoVariables.lbValues[27] = -1.0000000000000000e+00;
acadoVariables.lbValues[28] = 0.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+00;
acadoVariables.lbValues[30] = 0.0000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e+00;
acadoVariables.lbValues[32] = 0.0000000000000000e+00;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = 0.0000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e+00;
acadoVariables.lbValues[36] = 0.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+00;
acadoVariables.lbValues[38] = 0.0000000000000000e+00;
acadoVariables.lbValues[39] = -1.0000000000000000e+00;
acadoVariables.lbValues[40] = 0.0000000000000000e+00;
acadoVariables.lbValues[41] = -1.0000000000000000e+00;
acadoVariables.lbValues[42] = 0.0000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e+00;
acadoVariables.lbValues[44] = 0.0000000000000000e+00;
acadoVariables.lbValues[45] = -1.0000000000000000e+00;
acadoVariables.lbValues[46] = 0.0000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e+00;
acadoVariables.lbValues[48] = 0.0000000000000000e+00;
acadoVariables.lbValues[49] = -1.0000000000000000e+00;
acadoVariables.lbValues[50] = 0.0000000000000000e+00;
acadoVariables.lbValues[51] = -1.0000000000000000e+00;
acadoVariables.lbValues[52] = 0.0000000000000000e+00;
acadoVariables.lbValues[53] = -1.0000000000000000e+00;
acadoVariables.lbValues[54] = 0.0000000000000000e+00;
acadoVariables.lbValues[55] = -1.0000000000000000e+00;
acadoVariables.lbValues[56] = 0.0000000000000000e+00;
acadoVariables.lbValues[57] = -1.0000000000000000e+00;
acadoVariables.lbValues[58] = 0.0000000000000000e+00;
acadoVariables.lbValues[59] = -1.0000000000000000e+00;
acadoVariables.lbValues[60] = 0.0000000000000000e+00;
acadoVariables.lbValues[61] = -1.0000000000000000e+00;
acadoVariables.lbValues[62] = 0.0000000000000000e+00;
acadoVariables.lbValues[63] = -1.0000000000000000e+00;
acadoVariables.lbValues[64] = 0.0000000000000000e+00;
acadoVariables.lbValues[65] = -1.0000000000000000e+00;
acadoVariables.lbValues[66] = 0.0000000000000000e+00;
acadoVariables.lbValues[67] = -1.0000000000000000e+00;
acadoVariables.lbValues[68] = 0.0000000000000000e+00;
acadoVariables.lbValues[69] = -1.0000000000000000e+00;
acadoVariables.lbValues[70] = 0.0000000000000000e+00;
acadoVariables.lbValues[71] = -1.0000000000000000e+00;
acadoVariables.lbValues[72] = 0.0000000000000000e+00;
acadoVariables.lbValues[73] = -1.0000000000000000e+00;
acadoVariables.lbValues[74] = 0.0000000000000000e+00;
acadoVariables.lbValues[75] = -1.0000000000000000e+00;
acadoVariables.lbValues[76] = 0.0000000000000000e+00;
acadoVariables.lbValues[77] = -1.0000000000000000e+00;
acadoVariables.lbValues[78] = 0.0000000000000000e+00;
acadoVariables.lbValues[79] = -1.0000000000000000e+00;
acadoVariables.lbValues[80] = 0.0000000000000000e+00;
acadoVariables.lbValues[81] = -1.0000000000000000e+00;
acadoVariables.lbValues[82] = 0.0000000000000000e+00;
acadoVariables.lbValues[83] = -1.0000000000000000e+00;
acadoVariables.lbValues[84] = 0.0000000000000000e+00;
acadoVariables.lbValues[85] = -1.0000000000000000e+00;
acadoVariables.lbValues[86] = 0.0000000000000000e+00;
acadoVariables.lbValues[87] = -1.0000000000000000e+00;
acadoVariables.lbValues[88] = 0.0000000000000000e+00;
acadoVariables.lbValues[89] = -1.0000000000000000e+00;
acadoVariables.lbValues[90] = 0.0000000000000000e+00;
acadoVariables.lbValues[91] = -1.0000000000000000e+00;
acadoVariables.lbValues[92] = 0.0000000000000000e+00;
acadoVariables.lbValues[93] = -1.0000000000000000e+00;
acadoVariables.lbValues[94] = 0.0000000000000000e+00;
acadoVariables.lbValues[95] = -1.0000000000000000e+00;
acadoVariables.lbValues[96] = 0.0000000000000000e+00;
acadoVariables.lbValues[97] = -1.0000000000000000e+00;
acadoVariables.lbValues[98] = 0.0000000000000000e+00;
acadoVariables.lbValues[99] = -1.0000000000000000e+00;
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
acadoVariables.ubValues[50] = 1.0000000000000000e+00;
acadoVariables.ubValues[51] = 1.0000000000000000e+00;
acadoVariables.ubValues[52] = 1.0000000000000000e+00;
acadoVariables.ubValues[53] = 1.0000000000000000e+00;
acadoVariables.ubValues[54] = 1.0000000000000000e+00;
acadoVariables.ubValues[55] = 1.0000000000000000e+00;
acadoVariables.ubValues[56] = 1.0000000000000000e+00;
acadoVariables.ubValues[57] = 1.0000000000000000e+00;
acadoVariables.ubValues[58] = 1.0000000000000000e+00;
acadoVariables.ubValues[59] = 1.0000000000000000e+00;
acadoVariables.ubValues[60] = 1.0000000000000000e+00;
acadoVariables.ubValues[61] = 1.0000000000000000e+00;
acadoVariables.ubValues[62] = 1.0000000000000000e+00;
acadoVariables.ubValues[63] = 1.0000000000000000e+00;
acadoVariables.ubValues[64] = 1.0000000000000000e+00;
acadoVariables.ubValues[65] = 1.0000000000000000e+00;
acadoVariables.ubValues[66] = 1.0000000000000000e+00;
acadoVariables.ubValues[67] = 1.0000000000000000e+00;
acadoVariables.ubValues[68] = 1.0000000000000000e+00;
acadoVariables.ubValues[69] = 1.0000000000000000e+00;
acadoVariables.ubValues[70] = 1.0000000000000000e+00;
acadoVariables.ubValues[71] = 1.0000000000000000e+00;
acadoVariables.ubValues[72] = 1.0000000000000000e+00;
acadoVariables.ubValues[73] = 1.0000000000000000e+00;
acadoVariables.ubValues[74] = 1.0000000000000000e+00;
acadoVariables.ubValues[75] = 1.0000000000000000e+00;
acadoVariables.ubValues[76] = 1.0000000000000000e+00;
acadoVariables.ubValues[77] = 1.0000000000000000e+00;
acadoVariables.ubValues[78] = 1.0000000000000000e+00;
acadoVariables.ubValues[79] = 1.0000000000000000e+00;
acadoVariables.ubValues[80] = 1.0000000000000000e+00;
acadoVariables.ubValues[81] = 1.0000000000000000e+00;
acadoVariables.ubValues[82] = 1.0000000000000000e+00;
acadoVariables.ubValues[83] = 1.0000000000000000e+00;
acadoVariables.ubValues[84] = 1.0000000000000000e+00;
acadoVariables.ubValues[85] = 1.0000000000000000e+00;
acadoVariables.ubValues[86] = 1.0000000000000000e+00;
acadoVariables.ubValues[87] = 1.0000000000000000e+00;
acadoVariables.ubValues[88] = 1.0000000000000000e+00;
acadoVariables.ubValues[89] = 1.0000000000000000e+00;
acadoVariables.ubValues[90] = 1.0000000000000000e+00;
acadoVariables.ubValues[91] = 1.0000000000000000e+00;
acadoVariables.ubValues[92] = 1.0000000000000000e+00;
acadoVariables.ubValues[93] = 1.0000000000000000e+00;
acadoVariables.ubValues[94] = 1.0000000000000000e+00;
acadoVariables.ubValues[95] = 1.0000000000000000e+00;
acadoVariables.ubValues[96] = 1.0000000000000000e+00;
acadoVariables.ubValues[97] = 1.0000000000000000e+00;
acadoVariables.ubValues[98] = 1.0000000000000000e+00;
acadoVariables.ubValues[99] = 1.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 50; ++index)
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
for (index = 0; index < 50; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[200] = xEnd[0];
acadoVariables.x[201] = xEnd[1];
acadoVariables.x[202] = xEnd[2];
acadoVariables.x[203] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[200];
acadoWorkspace.state[1] = acadoVariables.x[201];
acadoWorkspace.state[2] = acadoVariables.x[202];
acadoWorkspace.state[3] = acadoVariables.x[203];
if (uEnd != 0)
{
acadoWorkspace.state[53] = uEnd[0];
acadoWorkspace.state[54] = uEnd[1];
}
else
{
acadoWorkspace.state[53] = acadoVariables.u[98];
acadoWorkspace.state[54] = acadoVariables.u[99];
}
acadoWorkspace.state[55] = acadoVariables.od[650];
acadoWorkspace.state[56] = acadoVariables.od[651];
acadoWorkspace.state[57] = acadoVariables.od[652];
acadoWorkspace.state[58] = acadoVariables.od[653];
acadoWorkspace.state[59] = acadoVariables.od[654];
acadoWorkspace.state[60] = acadoVariables.od[655];
acadoWorkspace.state[61] = acadoVariables.od[656];
acadoWorkspace.state[62] = acadoVariables.od[657];
acadoWorkspace.state[63] = acadoVariables.od[658];
acadoWorkspace.state[64] = acadoVariables.od[659];
acadoWorkspace.state[65] = acadoVariables.od[660];
acadoWorkspace.state[66] = acadoVariables.od[661];
acadoWorkspace.state[67] = acadoVariables.od[662];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[200] = acadoWorkspace.state[0];
acadoVariables.x[201] = acadoWorkspace.state[1];
acadoVariables.x[202] = acadoWorkspace.state[2];
acadoVariables.x[203] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[98] = uEnd[0];
acadoVariables.u[99] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99];
kkt = fabs( kkt );
for (index = 0; index < 100; ++index)
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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
