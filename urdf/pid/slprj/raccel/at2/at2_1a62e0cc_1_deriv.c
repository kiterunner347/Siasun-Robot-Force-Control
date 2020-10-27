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
#include "at2_1a62e0cc_1_geometries.h"

PmfMessageId at2_1a62e0cc_1_compDerivs(const RuntimeDerivedValuesBundle *rtdv,
  const int *eqnEnableFlags, const double *state, const int *modeVector, const
  double *input, const double *inputDot, const double *inputDdot, const double
  *discreteState, double *deriv, double *errorResult, NeuDiagnosticManager
  *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) deriv;
  (void) neDiagMgr;
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId at2_1a62e0cc_1_numJacPerturbLoBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) bounds;
  (void) neDiagMgr;
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId at2_1a62e0cc_1_numJacPerturbHiBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) bounds;
  (void) neDiagMgr;
  errorResult[0] = 0.0;
  return NULL;
}
