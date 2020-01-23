/*
 * _coder_nnClassifier_mex.cpp
 *
 * Code generation for function '_coder_nnClassifier_mex'
 *
 */

/* Include files */
#include "_coder_nnClassifier_api.h"
#include "_coder_nnClassifier_mex.h"

/* Function Declarations */
static void nnClassifier_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[2]);

/* Function Definitions */
static void nnClassifier_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
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
                        12, "nnClassifier");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "nnClassifier");
  }

  /* Call the function. */
  nnClassifier_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(nnClassifier_atexit);

  /* Module initialization. */
  nnClassifier_initialize();

  /* Dispatch the entry-point. */
  nnClassifier_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  nnClassifier_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_nnClassifier_mex.cpp) */
