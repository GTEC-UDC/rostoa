/*
 * _coder_nnClassifier_api.h
 *
 * Code generation for function '_coder_nnClassifier_api'
 *
 */

#ifndef _CODER_NNCLASSIFIER_API_H
#define _CODER_NNCLASSIFIER_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_nnClassifier_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern boolean_T nnClassifier(real_T rss, real_T ranging);
extern void nnClassifier_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[1]);
extern void nnClassifier_atexit(void);
extern void nnClassifier_initialize(void);
extern void nnClassifier_terminate(void);
extern void nnClassifier_xil_terminate(void);

#endif

/* End of code generation (_coder_nnClassifier_api.h) */
