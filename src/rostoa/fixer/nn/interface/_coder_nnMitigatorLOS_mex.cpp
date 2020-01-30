/*
 * _coder_nnMitigatorLOS_mex.cpp
 *
 * Code generation for function '_coder_nnMitigatorLOS_mex'
 *
 */

/* Include files */
#include "_coder_nnMitigatorLOS_api.h"
#include "_coder_nnMitigatorLOS_mex.h"

/* Function Declarations */
static void nnMitigatorLOS_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[2]);

/* Function Definitions */
static void nnMitigatorLOS_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[2])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        14, "nnMitigatorLOS");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 14,
                        "nnMitigatorLOS");
  }

  /* Call the function. */
  nnMitigatorLOS_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(nnMitigatorLOS_atexit);

  /* Module initialization. */
  nnMitigatorLOS_initialize();

  /* Dispatch the entry-point. */
  nnMitigatorLOS_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  nnMitigatorLOS_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_nnMitigatorLOS_mex.cpp) */
