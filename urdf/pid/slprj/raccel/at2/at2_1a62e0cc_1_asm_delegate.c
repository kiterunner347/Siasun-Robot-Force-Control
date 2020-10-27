/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'at2/Pos_control/Robot System/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"

void at2_1a62e0cc_1_setTargets(const RuntimeDerivedValuesBundle *rtdv, CTarget
  *targets)
{
  (void) rtdv;
  (void) targets;
}

void at2_1a62e0cc_1_resetAsmStateVector(const void *mech, double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
}

void at2_1a62e0cc_1_initializeTrackedAngleState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  (void) state;
  (void) neDiagMgr;
}

void at2_1a62e0cc_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void at2_1a62e0cc_1_adjustPosition(const void *mech, const double *dofDeltas,
  double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[4] = state[4] + dofDeltas[2];
  state[6] = state[6] + dofDeltas[3];
  state[8] = state[8] + dofDeltas[4];
  state[10] = state[10] + dofDeltas[5];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[8] = state[8] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[8] = state[8] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[10] = state[10] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[10] = state[10] + mag;
  state[11] = state[11] - 0.875 * mag;
}

void at2_1a62e0cc_1_perturbAsmJointPrimitiveState(const void *mech, size_t
  stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;
  }
}

void at2_1a62e0cc_1_computePosDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void at2_1a62e0cc_1_computeVelDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void at2_1a62e0cc_1_projectPartiallyTargetedPos(const void *mech, size_t
  stageIdx, size_t primIdx, const double *origState, int partialType, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void at2_1a62e0cc_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[144];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = - 0.9386421254436771;
  xx[1] = 0.2315998893487608;
  xx[2] = 0.2108008238792129;
  xx[3] = - 0.1444834393557381;
  xx[4] = 0.5;
  xx[5] = xx[4] * state[0];
  xx[6] = 0.3288083696074906;
  xx[7] = sin(xx[5]);
  xx[8] = 0.4956932808878663;
  xx[9] = 0.8038490078109737;
  xx[10] = cos(xx[5]);
  xx[11] = xx[6] * xx[7];
  xx[12] = - (xx[8] * xx[7]);
  xx[13] = xx[9] * xx[7];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 10, xx + 14);
  xx[0] = - 0.1807146198360494;
  xx[1] = 0.1469570455482168;
  xx[2] = - 0.09356719943576094;
  pm_math_Quaternion_xform_ra(xx + 14, xx + 0, xx + 10);
  xx[0] = 0.23333 - xx[12];
  xx[18] = - 0.8092409298579379;
  xx[19] = 0.519225986588831;
  xx[20] = - 0.2718902841604064;
  xx[21] = - 0.04011440729574299;
  xx[1] = xx[4] * state[2];
  xx[2] = 0.0519577871240773;
  xx[3] = sin(xx[1]);
  xx[5] = 0.04472479630072496;
  xx[7] = 0.9976472728139077;
  xx[22] = cos(xx[1]);
  xx[23] = - (xx[2] * xx[3]);
  xx[24] = - (xx[5] * xx[3]);
  xx[25] = xx[7] * xx[3];
  pm_math_Quaternion_compose_ra(xx + 18, xx + 22, xx + 26);
  xx[18] = - 0.03600982062520768;
  xx[19] = 0.3140398232702155;
  xx[20] = 0.2037136591958236;
  pm_math_Quaternion_xform_ra(xx + 26, xx + 18, xx + 21);
  xx[1] = 0.1475709962865888 - xx[21];
  xx[3] = - (0.04473686724564412 + xx[22]);
  xx[12] = 0.01884888413220301 - xx[23];
  xx[18] = - 0.9989961606562813;
  xx[19] = - 0.03266164201941988;
  xx[20] = 0.03051836112131727;
  xx[21] = 2.918521727498178e-3;
  xx[13] = xx[4] * state[4];
  xx[22] = 9.131635648929038e-3;
  xx[23] = sin(xx[13]);
  xx[24] = 0.02045388868948413;
  xx[25] = 0.9997490943571059;
  xx[30] = cos(xx[13]);
  xx[31] = xx[22] * xx[23];
  xx[32] = xx[24] * xx[23];
  xx[33] = xx[25] * xx[23];
  pm_math_Quaternion_compose_ra(xx + 18, xx + 30, xx + 34);
  xx[13] = 0.1632023270671031;
  xx[18] = 3.747757937505527e-3;
  xx[19] = 8.882932058271061e-3;
  xx[30] = - xx[13];
  xx[31] = - xx[18];
  xx[32] = - xx[19];
  pm_math_Quaternion_xform_ra(xx + 34, xx + 30, xx + 38);
  xx[20] = - (0.03621595018639374 + xx[38]);
  xx[21] = - (0.3519607515583694 + xx[39]);
  xx[23] = 0.1738459385161597 - xx[40];
  xx[30] = - 0.5454562156084232;
  xx[31] = 0.4526255143986982;
  xx[32] = - 0.439294578938647;
  xx[33] = 0.5519310948701553;
  xx[38] = xx[4] * state[6];
  xx[39] = 3.378667178765234e-6;
  xx[40] = sin(xx[38]);
  xx[41] = 0.2133729298191866;
  xx[42] = 0.9769708249528033;
  xx[43] = cos(xx[38]);
  xx[44] = xx[39] * xx[40];
  xx[45] = xx[41] * xx[40];
  xx[46] = xx[42] * xx[40];
  pm_math_Quaternion_compose_ra(xx + 30, xx + 43, xx + 47);
  xx[30] = - 2.5948037651771e-6;
  xx[31] = - 0.1634189552721184;
  xx[32] = - 0.7406063725104428;
  pm_math_Quaternion_xform_ra(xx + 47, xx + 30, xx + 43);
  xx[30] = - (xx[13] + xx[43]);
  xx[13] = - (xx[18] + xx[44]);
  xx[18] = - (xx[19] + xx[45]);
  xx[43] = - 0.7868727621261351;
  xx[44] = - 0.6171152697848129;
  xx[45] = 2.808287157690021e-6;
  xx[46] = 3.810261325298959e-6;
  xx[19] = xx[4] * state[8];
  xx[31] = 2.634604032000144e-6;
  xx[32] = sin(xx[19]);
  xx[33] = 0.02562841108668832;
  xx[38] = 0.9996715383255795;
  xx[51] = cos(xx[19]);
  xx[52] = xx[31] * xx[32];
  xx[53] = - (xx[33] * xx[32]);
  xx[54] = xx[38] * xx[32];
  pm_math_Quaternion_compose_ra(xx + 43, xx + 51, xx + 55);
  xx[19] = 4.468625262939415e-8;
  xx[32] = 0.01159652967198332;
  xx[40] = 2.408290401367426e-3;
  xx[43] = xx[19];
  xx[44] = - xx[32];
  xx[45] = xx[40];
  pm_math_Quaternion_xform_ra(xx + 55, xx + 43, xx + 51);
  xx[43] = 3.491303076964112e-7 - xx[51];
  xx[44] = 0.02249927966723341 - xx[52];
  xx[45] = 0.1106576163956834 - xx[53];
  xx[51] = - 0.7161082599400127;
  xx[52] = 0.6979892213883621;
  xx[53] = 4.801826128332003e-5;
  xx[54] = - 6.756923773239749e-5;
  xx[46] = xx[4] * state[10];
  xx[4] = 1.641806133273513e-4;
  xx[59] = sin(xx[46]);
  xx[60] = 2.655457763855013e-6;
  xx[61] = 0.9999999865188373;
  xx[62] = cos(xx[46]);
  xx[63] = xx[4] * xx[59];
  xx[64] = - (xx[60] * xx[59]);
  xx[65] = xx[61] * xx[59];
  pm_math_Quaternion_compose_ra(xx + 51, xx + 62, xx + 66);
  xx[51] = - 7.471347352210193e-5;
  xx[52] = 3.874231065075369e-6;
  xx[53] = - 0.1869499902435115;
  pm_math_Quaternion_xform_ra(xx + 66, xx + 51, xx + 62);
  xx[46] = xx[19] - xx[62];
  xx[19] = - (xx[32] + xx[63]);
  xx[32] = xx[40] - xx[64];
  xx[40] = - 0.923884715998796;
  xx[51] = - 3.264024093444298e-5;
  xx[52] = - 7.53338969475041e-5;
  xx[53] = - 0.3826709092722398;
  xx[54] = - 1.615517416663554e-5;
  xx[59] = 2.927108944441202e-6;
  xx[62] = 0.1697200049481622;
  pm_math_Quaternion_compose_ra(xx + 14, xx + 26, xx + 70);
  xx[63] = xx[1];
  xx[64] = xx[3];
  xx[65] = xx[12];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 63, xx + 74);
  xx[77] = xx[74] - xx[10];
  xx[78] = xx[75] - xx[11];
  xx[74] = xx[76] + xx[0];
  pm_math_Quaternion_compose_ra(xx + 70, xx + 34, xx + 79);
  xx[83] = xx[20];
  xx[84] = xx[21];
  xx[85] = xx[23];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 83, xx + 86);
  xx[75] = xx[86] + xx[77];
  xx[76] = xx[87] + xx[78];
  xx[86] = xx[88] + xx[74];
  pm_math_Quaternion_compose_ra(xx + 79, xx + 47, xx + 87);
  xx[91] = xx[30];
  xx[92] = xx[13];
  xx[93] = xx[18];
  pm_math_Quaternion_xform_ra(xx + 79, xx + 91, xx + 94);
  xx[97] = xx[94] + xx[75];
  xx[98] = xx[95] + xx[76];
  xx[94] = xx[96] + xx[86];
  pm_math_Quaternion_compose_ra(xx + 87, xx + 55, xx + 99);
  pm_math_Quaternion_xform_ra(xx + 87, xx + 43, xx + 103);
  xx[95] = xx[103] + xx[97];
  xx[96] = xx[104] + xx[98];
  xx[103] = xx[105] + xx[94];
  pm_math_Quaternion_compose_ra(xx + 99, xx + 66, xx + 104);
  xx[108] = xx[46];
  xx[109] = xx[19];
  xx[110] = xx[32];
  pm_math_Quaternion_xform_ra(xx + 99, xx + 108, xx + 111);
  xx[114] = xx[111] + xx[95];
  xx[115] = xx[112] + xx[96];
  xx[111] = xx[113] + xx[103];
  xx[116] = xx[40];
  xx[117] = xx[51];
  xx[118] = xx[52];
  xx[119] = xx[53];
  pm_math_Quaternion_compose_ra(xx + 104, xx + 116, xx + 120);
  xx[124] = xx[54];
  xx[125] = xx[59];
  xx[126] = xx[62];
  pm_math_Quaternion_xform_ra(xx + 104, xx + 124, xx + 127);
  xx[112] = xx[6] * state[1];
  xx[6] = - (xx[8] * state[1]);
  xx[8] = xx[9] * state[1];
  xx[9] = 0.07175064318296448 * state[1];
  xx[113] = 0.1145015895569342 * state[1];
  xx[130] = 0.04125831626189193 * state[1];
  xx[131] = xx[112];
  xx[132] = xx[6];
  xx[133] = xx[8];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 131, xx + 134);
  xx[137] = xx[134] - xx[2] * state[3];
  xx[2] = xx[135] - xx[5] * state[3];
  xx[5] = xx[136] + xx[7] * state[3];
  pm_math_Vector3_cross_ra(xx + 131, xx + 63, xx + 134);
  xx[63] = xx[134] + xx[9];
  xx[64] = xx[135] + xx[113];
  xx[65] = xx[136] + xx[130];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 63, xx + 131);
  xx[7] = xx[131] + 0.3224120251517006 * state[3];
  xx[63] = xx[132] + 0.025340588402493 * state[3];
  xx[64] = xx[133] + 0.01792734617824477 * state[3];
  xx[131] = xx[137];
  xx[132] = xx[2];
  xx[133] = xx[5];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 131, xx + 134);
  xx[65] = xx[134] + xx[22] * state[5];
  xx[22] = xx[135] + xx[24] * state[5];
  xx[24] = xx[136] + xx[25] * state[5];
  pm_math_Vector3_cross_ra(xx + 131, xx + 83, xx + 134);
  xx[83] = xx[134] + xx[7];
  xx[84] = xx[135] + xx[63];
  xx[85] = xx[136] + xx[64];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 83, xx + 131);
  xx[25] = xx[131] - 3.565127100334679e-3 * state[5];
  xx[83] = xx[132] + 0.1630802629832583 * state[5];
  xx[84] = xx[133] - 3.303899071709628e-3 * state[5];
  xx[131] = xx[65];
  xx[132] = xx[22];
  xx[133] = xx[24];
  pm_math_Quaternion_inverseXform_ra(xx + 47, xx + 131, xx + 134);
  xx[85] = xx[134] + xx[39] * state[7];
  xx[39] = xx[135] + xx[41] * state[7];
  xx[41] = xx[136] + xx[42] * state[7];
  pm_math_Vector3_cross_ra(xx + 131, xx + 91, xx + 134);
  xx[91] = xx[134] + xx[25];
  xx[92] = xx[135] + xx[83];
  xx[93] = xx[136] + xx[84];
  pm_math_Quaternion_inverseXform_ra(xx + 47, xx + 91, xx + 131);
  xx[42] = xx[131] - 1.630199999813731e-3 * state[7];
  xx[91] = xx[132] + 3.278513187029955e-8 * state[7];
  xx[92] = xx[133] - 1.522621115684168e-9 * state[7];
  xx[131] = xx[85];
  xx[132] = xx[39];
  xx[133] = xx[41];
  pm_math_Quaternion_inverseXform_ra(xx + 55, xx + 131, xx + 134);
  xx[93] = xx[134] + xx[31] * state[9];
  xx[31] = xx[135] - xx[33] * state[9];
  xx[33] = xx[136] + xx[38] * state[9];
  pm_math_Vector3_cross_ra(xx + 131, xx + 43, xx + 134);
  xx[131] = xx[134] + xx[42];
  xx[132] = xx[135] + xx[91];
  xx[133] = xx[136] + xx[92];
  pm_math_Quaternion_inverseXform_ra(xx + 55, xx + 131, xx + 134);
  xx[38] = xx[134] - 0.01153100000000742 * state[9];
  xx[131] = xx[135] - 3.832668330636206e-8 * state[9];
  xx[132] = xx[136] + 2.940702617870683e-8 * state[9];
  xx[133] = xx[93];
  xx[134] = xx[31];
  xx[135] = xx[33];
  pm_math_Quaternion_inverseXform_ra(xx + 66, xx + 133, xx + 138);
  xx[136] = xx[138] + xx[4] * state[11];
  xx[4] = xx[139] - xx[60] * state[11];
  xx[60] = xx[140] + xx[61] * state[11];
  pm_math_Vector3_cross_ra(xx + 133, xx + 108, xx + 138);
  xx[108] = xx[138] + xx[38];
  xx[109] = xx[139] + xx[131];
  xx[110] = xx[140] + xx[132];
  pm_math_Quaternion_inverseXform_ra(xx + 66, xx + 108, xx + 133);
  xx[61] = xx[133] + 3.377793209801478e-6 * state[11];
  xx[108] = xx[134] + 4.401990845515537e-5 * state[11];
  xx[109] = xx[135] - 4.376751591071101e-10 * state[11];
  xx[133] = xx[136];
  xx[134] = xx[4];
  xx[135] = xx[60];
  pm_math_Quaternion_inverseXform_ra(xx + 116, xx + 133, xx + 138);
  pm_math_Vector3_cross_ra(xx + 133, xx + 124, xx + 141);
  xx[124] = xx[141] + xx[61];
  xx[125] = xx[142] + xx[108];
  xx[126] = xx[143] + xx[109];
  pm_math_Quaternion_inverseXform_ra(xx + 116, xx + 124, xx + 133);
  motionData[0] = xx[14];
  motionData[1] = xx[15];
  motionData[2] = xx[16];
  motionData[3] = xx[17];
  motionData[4] = - xx[10];
  motionData[5] = - xx[11];
  motionData[6] = xx[0];
  motionData[7] = xx[26];
  motionData[8] = xx[27];
  motionData[9] = xx[28];
  motionData[10] = xx[29];
  motionData[11] = xx[1];
  motionData[12] = xx[3];
  motionData[13] = xx[12];
  motionData[14] = xx[34];
  motionData[15] = xx[35];
  motionData[16] = xx[36];
  motionData[17] = xx[37];
  motionData[18] = xx[20];
  motionData[19] = xx[21];
  motionData[20] = xx[23];
  motionData[21] = xx[47];
  motionData[22] = xx[48];
  motionData[23] = xx[49];
  motionData[24] = xx[50];
  motionData[25] = xx[30];
  motionData[26] = xx[13];
  motionData[27] = xx[18];
  motionData[28] = xx[55];
  motionData[29] = xx[56];
  motionData[30] = xx[57];
  motionData[31] = xx[58];
  motionData[32] = xx[43];
  motionData[33] = xx[44];
  motionData[34] = xx[45];
  motionData[35] = xx[66];
  motionData[36] = xx[67];
  motionData[37] = xx[68];
  motionData[38] = xx[69];
  motionData[39] = xx[46];
  motionData[40] = xx[19];
  motionData[41] = xx[32];
  motionData[42] = xx[40];
  motionData[43] = xx[51];
  motionData[44] = xx[52];
  motionData[45] = xx[53];
  motionData[46] = xx[54];
  motionData[47] = xx[59];
  motionData[48] = xx[62];
  motionData[49] = xx[70];
  motionData[50] = xx[71];
  motionData[51] = xx[72];
  motionData[52] = xx[73];
  motionData[53] = xx[77];
  motionData[54] = xx[78];
  motionData[55] = xx[74];
  motionData[56] = xx[79];
  motionData[57] = xx[80];
  motionData[58] = xx[81];
  motionData[59] = xx[82];
  motionData[60] = xx[75];
  motionData[61] = xx[76];
  motionData[62] = xx[86];
  motionData[63] = xx[87];
  motionData[64] = xx[88];
  motionData[65] = xx[89];
  motionData[66] = xx[90];
  motionData[67] = xx[97];
  motionData[68] = xx[98];
  motionData[69] = xx[94];
  motionData[70] = xx[99];
  motionData[71] = xx[100];
  motionData[72] = xx[101];
  motionData[73] = xx[102];
  motionData[74] = xx[95];
  motionData[75] = xx[96];
  motionData[76] = xx[103];
  motionData[77] = xx[104];
  motionData[78] = xx[105];
  motionData[79] = xx[106];
  motionData[80] = xx[107];
  motionData[81] = xx[114];
  motionData[82] = xx[115];
  motionData[83] = xx[111];
  motionData[84] = xx[120];
  motionData[85] = xx[121];
  motionData[86] = xx[122];
  motionData[87] = xx[123];
  motionData[88] = xx[127] + xx[114];
  motionData[89] = xx[128] + xx[115];
  motionData[90] = xx[129] + xx[111];
  motionData[91] = xx[112];
  motionData[92] = xx[6];
  motionData[93] = xx[8];
  motionData[94] = xx[9];
  motionData[95] = xx[113];
  motionData[96] = xx[130];
  motionData[97] = xx[137];
  motionData[98] = xx[2];
  motionData[99] = xx[5];
  motionData[100] = xx[7];
  motionData[101] = xx[63];
  motionData[102] = xx[64];
  motionData[103] = xx[65];
  motionData[104] = xx[22];
  motionData[105] = xx[24];
  motionData[106] = xx[25];
  motionData[107] = xx[83];
  motionData[108] = xx[84];
  motionData[109] = xx[85];
  motionData[110] = xx[39];
  motionData[111] = xx[41];
  motionData[112] = xx[42];
  motionData[113] = xx[91];
  motionData[114] = xx[92];
  motionData[115] = xx[93];
  motionData[116] = xx[31];
  motionData[117] = xx[33];
  motionData[118] = xx[38];
  motionData[119] = xx[131];
  motionData[120] = xx[132];
  motionData[121] = xx[136];
  motionData[122] = xx[4];
  motionData[123] = xx[60];
  motionData[124] = xx[61];
  motionData[125] = xx[108];
  motionData[126] = xx[109];
  motionData[127] = xx[138];
  motionData[128] = xx[139];
  motionData[129] = xx[140];
  motionData[130] = xx[133];
  motionData[131] = xx[134];
  motionData[132] = xx[135];
}

size_t at2_1a62e0cc_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const double *state,
  const int *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t at2_1a62e0cc_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t at2_1a62e0cc_1_computeFullAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

int at2_1a62e0cc_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv
    ;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

PmfMessageId at2_1a62e0cc_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState, void
  *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  (void) neDiagMgr;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  return NULL;
}
