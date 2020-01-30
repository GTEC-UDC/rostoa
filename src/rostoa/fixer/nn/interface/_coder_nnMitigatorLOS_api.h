/*
 * _coder_nnMitigatorLOS_api.h
 *
 * Code generation for function '_coder_nnMitigatorLOS_api'
 *
 */

#ifndef _CODER_NNMITIGATORLOS_API_H
#define _CODER_NNMITIGATORLOS_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_nnMitigatorLOS_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern real_T nnMitigatorLOS(real_T rss, real_T ranging);
extern void nnMitigatorLOS_api(const mxArray * const prhs[2], int32_T nlhs,
  const mxArray *plhs[1]);
extern void nnMitigatorLOS_atexit(void);
extern void nnMitigatorLOS_initialize(void);
extern void nnMitigatorLOS_terminate(void);
extern void nnMitigatorLOS_xil_terminate(void);

#endif

/* End of code generation (_coder_nnMitigatorLOS_api.h) */
