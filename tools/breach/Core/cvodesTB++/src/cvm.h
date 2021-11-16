/*
 * -----------------------------------------------------------------
 * $Revision: 1.1 $
 * $Date: 2009-06-05 16:26:09 $
 * -----------------------------------------------------------------
 * Programmer: Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2005, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see sundials-x.y.z/src/cvodes/LICENSE.
 * -----------------------------------------------------------------
 * Header file for the CVODES Matlab interface.
 * -----------------------------------------------------------------
 */

#ifndef _CVM_H
#define _CVM_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif
  
#include <string.h>
#include <stdlib.h>
#include "mex.h"
#include <cvodes/cvodes.h>
#include <cvodes/cvodes_dense.h>
#include <cvodes/cvodes_diag.h>
#include <cvodes/cvodes_band.h>
#include <cvodes/cvodes_spgmr.h>
#include <cvodes/cvodes_spbcgs.h>
#include <cvodes/cvodes_sptfqmr.h>
#include <cvodes/cvodes_bandpre.h>
#include <cvodes/cvodes_bbdpre.h>

  /*
   * ---------------------------------------------------------------------------------
   * Constants
   * ---------------------------------------------------------------------------------
   */

  /* Linear solver types */

  enum {LS_NONE, LS_DENSE, LS_DIAG, LS_BAND, LS_SPGMR, LS_SPBCG, LS_SPTFQMR};

  /* Preconditioner modules */

  enum {PM_NONE, PM_BANDPRE, PM_BBDPRE};

  /*
   * ---------------------------------------------------------------------------------
   * Types for global data structures
   * ---------------------------------------------------------------------------------
   */

  typedef struct cvm_CVODESdataStruct {

    void *cvode_mem;   /* CVODES solver memory */
    void *bp_data;     /* Preconditioner memory (BandPre or BBDPre) */
    N_Vector y;        /* solution vector */
    N_Vector yQ;       /* quadratures vector */
    N_Vector *yS;      /* sensitivity vectors */

    /* - Added: now stores f_data and tolerances */

      void *f_data;
      int itol;
      realtype reltol;
      realtype Sabstol;
      N_Vector NV_abstol;
      N_Vector *yS0;
    
    /* - End Added ----------------------------- */

    int N;             /* problem dimension */
    int Nq;            /* number of quadratures */
    int Ng;            /* number of root functions */
    int Ns;            /* number of sensitivities */
    int Nd;            /* number of data points */
    int Nc;            /* number of check points */
    int ls;            /* linear solver type */
    int pm;            /* preconditioner module */
    int ism;           /* sensitivity method */
    
    void *cvadj_mem;   /* CVODES adjoint memory */
    int interp;
    N_Vector yB;
    N_Vector yQB;
    int NB;
    int NqB;
    int lsB;
    int pmB;   

    booleantype errmsg;

  } *cvm_CVODESdata;

  typedef struct cvm_MATLABdataStruct {

    mxArray *mx_RHSfct;
    mxArray *mx_QUADfct;
    mxArray *mx_JACfct;
    mxArray *mx_PSETfct;
    mxArray *mx_PSOLfct;
    mxArray *mx_GLOCfct;
    mxArray *mx_GCOMfct;
    
    mxArray *mx_Gfct;
    
    mxArray *mx_SRHSfct;
    
    mxArray *mx_RHSfctB;
    mxArray *mx_QUADfctB;
    mxArray *mx_JACfctB;
    mxArray *mx_PSETfctB;
    mxArray *mx_PSOLfctB;
    mxArray *mx_GLOCfctB;
    mxArray *mx_GCOMfctB;

    mxArray *mx_data;

    /* Monitor */
  
    mxArray *mx_MONfct;
    mxArray *mx_MONdata;
    
    mxArray *mx_MONfctB;
    mxArray *mx_MONdataB;

  } *cvm_MATLABdata;

  /*
   * ---------------------------------------------------------------------------------
   * Declarations for global variables (defined in cvmFun.cpp)
   * ---------------------------------------------------------------------------------
   */

  extern cvm_CVODESdata cvm_Cdata;  /* CVODES data */
  extern booleantype cvm_quad;      /* Forward quadratures? */
  extern booleantype cvm_quadB;     /* Backward quadratures? */
  extern booleantype cvm_asa;       /* Adjoint sensitivity? */
  extern booleantype cvm_fsa;       /* Forward sensitivity? */
  extern booleantype cvm_mon;       /* Forward monitoring? */ 
  extern booleantype cvm_monB;      /* Backward monitoring? */ 
  extern cvm_MATLABdata cvm_Mdata;  /* MATLAB data */

  /*
   * ---------------------------------------------------------------------------------
   * Wrapper functions
   * ---------------------------------------------------------------------------------
   */

  void mtlb_CVodeErrHandler(int error_code, 
                            const char *module, const char *function, 
                            char *msg, void *eh_data); 
  
  int mtlb_CVodeRhs(realtype t, N_Vector y, N_Vector yd, void *f_data);

  int mtlb_CVodeGfct(realtype t, N_Vector y, double *g, void *g_data);

  int mtlb_CVodeQUADfct(realtype t, N_Vector y, N_Vector yQd, void *fQ_data);


  int mtlb_CVodeSensRhs1(int Ns, realtype t,
                         N_Vector y, N_Vector ydot,
                         int iS, N_Vector yS, N_Vector ySdot,
                         void *fS_data,
                         N_Vector tmp1, N_Vector tmp2);
  int mtlb_CVodeSensRhs(int Ns, realtype t,
                        N_Vector y, N_Vector ydot,
                        N_Vector *yS, N_Vector *ySdot,
                        void *fS_data,
                        N_Vector tmp1, N_Vector tmp2);


  int mtlb_CVodeDenseJac(long int N, DenseMat J, realtype t,
                         N_Vector y, N_Vector fy, void *jac_data,
                         N_Vector tmp1, N_Vector tmp2, N_Vector tmp3);
  int mtlb_CVodeBandJac(long int N, long int mupper, long int mlower,
                        BandMat J, realtype t,
                        N_Vector y, N_Vector fy, void *jac_data,
                        N_Vector tmp1, N_Vector tmp2, N_Vector tmp3);
  int mtlb_CVodeSpilsJac(N_Vector v, N_Vector Jv, realtype t,
                         N_Vector y, N_Vector fy,
                         void *jac_data, N_Vector tmp);
  int mtlb_CVodeSpilsPset(realtype t, N_Vector y, N_Vector fy,
                          booleantype jok, booleantype *jcurPtr,
                          realtype gamma, void *P_data,
                          N_Vector tmp1, N_Vector tmp2,
                          N_Vector tmp3);
  int mtlb_CVodeSpilsPsol(realtype t, N_Vector y, N_Vector fy,
                          N_Vector r, N_Vector z,
                          realtype gamma, realtype delta,
                          int lr, void *P_data, N_Vector tmp);

  
  int mtlb_CVodeBBDgloc(long int Nlocal, realtype t, N_Vector y,
                        N_Vector g, void *f_data);
  int mtlb_CVodeBBDgcom(long int Nlocal, realtype t, N_Vector y,
                        void *f_data);



  int mtlb_CVodeRhsB(realtype t, N_Vector y, N_Vector yB, N_Vector yBdot, void *f_dataB);
  int mtlb_CVodeQUADfctB(realtype t, N_Vector y, N_Vector yB, N_Vector qBdot, void *fQ_dataB);
  int mtlb_CVodeDenseJacB(long int nB, DenseMat JB, realtype t,
                          N_Vector y, N_Vector yB, N_Vector fyB,
                          void *jac_dataB, N_Vector tmp1B,
                          N_Vector tmp2B, N_Vector tmp3B);
  int mtlb_CVodeBandJacB(long int nB, long int mupperB,
                         long int mlowerB, BandMat JB,
                         realtype t, N_Vector y,
                         N_Vector yB, N_Vector fyB,
                         void *jac_dataB, N_Vector tmp1B,
                         N_Vector tmp2B, N_Vector tmp3B);
  int mtlb_CVodeSpilsJacB(N_Vector vB, N_Vector JvB, realtype t,
                          N_Vector y, N_Vector yB, N_Vector fyB,
                          void *jac_dataB, N_Vector tmpB);
  int mtlb_CVodeSpilsPsetB(realtype t, N_Vector y,
                           N_Vector yB, N_Vector fyB,
                           booleantype jokB,
                           booleantype *jcurPtrB, realtype gammaB,
                           void *P_dataB,
                           N_Vector tmp1B, N_Vector tmp2B,
                           N_Vector tmp3B);
  int mtlb_CVodeSpilsPsolB(realtype t, N_Vector y,
                           N_Vector yB, N_Vector fyB,
                           N_Vector rB, N_Vector zB,
                           realtype gammaB, realtype deltaB,
                           int lrB, void *P_dataB, N_Vector tmpB);
  
  int mtlb_CVodeBBDglocB(long int NlocalB, realtype t, N_Vector y, 
                          N_Vector yB, N_Vector gB, void *f_dataB);

  int mtlb_CVodeBBDgcomB(long int NlocalB, realtype t, N_Vector y, 
                          N_Vector yB, void *f_dataB);

  void mtlb_CVodeMonitor(int call, double t, N_Vector y, N_Vector yQ, N_Vector *yS);
  void mtlb_CVodeMonitorB(int call, double tB, N_Vector yB, N_Vector yQB);

  /*
   * ---------------------------------------------------------------------------------
   * Option handling functions
   * ---------------------------------------------------------------------------------
   */

  int get_IntgrOptions(const mxArray *options, booleantype fwd,
                       int *lmm, int *iter, int *maxord, booleantype *sld,
                       long int *mxsteps,
                       int *itol, realtype *reltol, double *Sabstol, double **Vabstol,
                       double *hin, double *hmax, double *hmin, 
                       double *tstop, booleantype *tstopSet);

  int get_LinSolvOptions(const mxArray *options, booleantype fwd,
                         int *mupper, int *mlower,
                         int *mudq, int *mldq, double *dqrely,
                         int *ptype, int *gstype, int *maxl);

  int get_QuadOptions(const mxArray *options, booleantype fwd,
                      double **yQ0, booleantype *errconQ,
                      int *itolQ, double *reltolQ, double *SabstolQ, double **VabstolQ);

  int get_FSAOptions(const mxArray *options, 
                     char **pfield_name, int **plist, double **pbar,
                     booleantype *userSRHS, int *dqtype, double *rho,
                     booleantype *errconS, int *itolS, double *reltolS, 
                     double **SabstolS, double **VabstolS);

  /*
   * ---------------------------------------------------------------------------------
   * Interfacing functions with CVodes
   * ---------------------------------------------------------------------------------
   */

  void CVM_init();
  void CVM_makePersistent();
  void CVM_final();
  int CVM_Initialization(int action, int nlhs, mxArray *plhs[],
			 int nrhs, const mxArray *prhs[]);
  int CVM_SensInitialization(int action, int nlhs, mxArray *plhs[],
			     int nrhs, const mxArray *prhs[]);
  int CVM_SensToggleOff(int nlhs, mxArray *plhs[], int nrhs, 
			const mxArray *prhs[]);
  int CVM_AdjInitialization(int nlhs, mxArray *plhs[], int nrhs,
			    const mxArray *prhs[]);
  int CVM_InitializationB(int action, int nlhs, mxArray *plhs[], int nrhs,
			  const mxArray *prhs[]);
  int CVM_Solve(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_SolveB(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_Stats(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_StatsB(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_Get(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_Set(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  int CVM_Free(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
  

#ifdef __cplusplus
}
#endif

#endif
