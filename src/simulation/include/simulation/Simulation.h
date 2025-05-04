/*
 * Simulation.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Simulation".
 *
 * Model version              : 2.222
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Sat May  3 19:06:13 2025
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Simulation_h_
#define Simulation_h_
#include <cstdio>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros2_initialize.h"
#include "Simulation_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block states (default storage) for system '<Root>/Parse Thruster Data' */
struct DW_CoreSubsys_Simulation_T {
  int32_T sfEvent;                     /* '<S7>/MATLAB Function' */
  boolean_T doneDoubleBufferReInit;    /* '<S7>/MATLAB Function' */
};

/* Block signals (default storage) */
struct B_Simulation_T {
  SL_Bus_std_msgs_Float32MultiArray BusAssignment;/* '<Root>/Bus Assignment' */
  real_T x[36];
  SL_Bus_std_msgs_String In1;          /* '<S33>/In1' */
  SL_Bus_std_msgs_String rtb_SourceBlock_o2_m;
  char_T s1_data[130];
  char_T b_s1_data[130];
  char_T u[128];
  char_T u_data[128];
  char_T u_data_c[126];
  real_T R_tmp[9];
  real_T R_tmp_k[9];
  real_T R_tmp_c[9];
  real_T d[9];
  real_T duty_cycle_array[8];
  real_T split_values[8];
  real_T Sum3_i[6];                    /* '<S1>/Sum3' */
  real_T theta[3];                     /* '<S1>/Integrator2' */
  real_T nu[6];                        /* '<S1>/Integrator' */
  real_T Add[3];                       /* '<S5>/Add' */
  real_T Add1[3];                      /* '<S5>/Add1' */
  real_T Reshape[6];                   /* '<S5>/Reshape' */
  real_T ddtnu[6];                     /* '<S1>/Product' */
  real_T p[3];                         /* '<S1>/Integrator1' */
  real_T ddttheta[3];                  /* '<S1>/Product2' */
  real_T dpdt[3];                      /* '<S1>/Product1' */
  real_T Gain1[3];                     /* '<Root>/Gain1' */
  real_T Gain2[3];                     /* '<Root>/Gain2' */
  real_T net_force[3];              /* '<S21>/Thrust to Net Force and Moment' */
  real_T net_moment[3];             /* '<S21>/Thrust to Net Force and Moment' */
  real_T thruster_forces[8];           /* '<S21>/PWM to Thrust' */
  real_T global_force[3];              /* '<S21>/Body to Inertial' */
  real_T Sum_j[3];                     /* '<S14>/Sum' */
  real_T Sum3[3];                      /* '<S11>/Sum3' */
  real_T dv[3];
  creal_T temp_val;
  real_T dv1[2];
  int32_T b_s1_size[2];
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T yaw;
  real_T ImpAsg_InsertedFor_t1_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t2_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t3_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t4_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t5_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t6_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t7_at_in;  /* '<S7>/MATLAB Function' */
  real_T ImpAsg_InsertedFor_t8_at_in;  /* '<S7>/MATLAB Function' */
  real_T rtb_Sum3_b;
  real_T rtb_Sum3_i_idx_0;
  real_T rtb_Sum3_i_idx_2;
  real_T rtb_costheta_idx_2;
  real_T rtb_sintheta_idx_1;
  real_T rtb_sintheta_idx_2;
  real_T a_idx_1;
  real_T force_vector_idx_1;
  real_T a_idx_0;
  real_T rtb_Sum3_i_idx_1;
  real_T rtb_Sum3_i_idx_2_p;
  real_T rtb_Sum_j_idx_0;
  real_T rtb_Sum3_i_idx_0_c;
  real_T rtb_Sum3_idx_0;
  real_T rtb_Sum3_i_idx_0_f;
  real_T rtb_Sum_j_idx_1;
  real_T rtb_Sum3_i_idx_1_g;
  real_T rtb_Sum_j_idx_1_g;
  real_T rtb_Sum3_i_idx_1_m;
  real_T rtb_Sum3_idx_1;
  real_T rtb_Sum3_i_idx_1_n;
  real_T rtb_Sum3_idx_1_p;
  real_T rtb_Sum3_i_idx_1_l;
  real_T rtb_Sum3_i_idx_2_j;
  real_T rtb_Sum_j_idx_2;
  real_T rtb_Sum3_i_idx_2_d;
  real_T rtb_Sum3_idx_2;
  real_T net_moment_g;
  real_T rtb_Sum3_i_l;
  real_T rtb_Sum3_i_d;
  real_T rtb_Sum3_i_dy;
  real_T rtb_Sum3_i_lx;
  real_T absxk_tmp;
  real_T rtb_costheta_idx_0_tmp;
  real_T rtb_sintheta_idx_0_tmp;
  real_T rtb_costheta_idx_1_tmp;
  real_T smax;
  real_T s;
  real_T scanned1;
  real_T scanned2;
  real_T b_scanned1;
  int8_T ipiv[6];
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T b_low_ip1;
  int32_T b_mid_i;
  int32_T i;
  int32_T u_size;
  int32_T u_size_o;
  int32_T jj;
  int32_T c;
  int32_T c_b;
  int32_T kAcol;
  int32_T k;
  int32_T jA;
  int32_T i_n;
  int32_T ijA;
  int32_T x_tmp;
  int32_T k_b;
  int32_T idx;
  int32_T ntoread;
  int32_T b_nread;
  int32_T loop_ub;
  int32_T b_idx;
  int32_T b_k;
  int32_T kexp;
  int32_T b_k_l;
  int32_T b_k_h;
  int32_T j;
  int32_T ksaved;
  int32_T j_b;
  int32_T i1;
  char_T c_d[3];
  char_T c_e[3];
  int8_T ipiv_j;
  boolean_T b_varargout_1;
  boolean_T b;
  boolean_T success;
  boolean_T isimag1;
  boolean_T a__3;
  boolean_T b_finite;
  boolean_T b_a__2;
  boolean_T b_a__3;
  boolean_T f_success;
  boolean_T isneg;
  boolean_T b_success;
  boolean_T haspoint;
  boolean_T isneg_f;
  char_T c_bj;
  char_T c1;
  char_T c2;
  char_T c3;
  char_T c4;
  char_T c5;
};

/* Block states (default storage) for system '<Root>' */
struct DW_Simulation_T {
  ros_slros2_internal_block_Pub_T obj; /* '<S8>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_g;/* '<S9>/SourceBlock' */
  struct {
    void *LoggedData[6];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData[6];
  } Scope2_PWORK;                      /* '<Root>/Scope2' */

  struct {
    void *LoggedData[6];
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  struct {
    void *LoggedData[6];
  } Scope3_PWORK;                      /* '<Root>/Scope3' */

  int32_T sfEvent;                     /* '<Root>/MATLAB Function' */
  int32_T sfEvent_a;                /* '<S21>/Thrust to Net Force and Moment' */
  int32_T sfEvent_m;                   /* '<S21>/PWM to Thrust' */
  int32_T sfEvent_o;                   /* '<S21>/Body to Inertial' */
  int32_T sfEvent_as;                  /* '<S20>/Body to Inertial' */
  int8_T EnabledSubsystem_SubsysRanBC; /* '<S9>/Enabled Subsystem' */
  boolean_T objisempty;                /* '<S9>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S8>/SinkBlock' */
  boolean_T doneDoubleBufferReInit;    /* '<Root>/MATLAB Function' */
  boolean_T doneDoubleBufferReInit_h;
                                    /* '<S21>/Thrust to Net Force and Moment' */
  boolean_T doneDoubleBufferReInit_k;  /* '<S21>/PWM to Thrust' */
  boolean_T doneDoubleBufferReInit_i;  /* '<S21>/Body to Inertial' */
  boolean_T doneDoubleBufferReInit_l;  /* '<S20>/Body to Inertial' */
  DW_CoreSubsys_Simulation_T CoreSubsys[1];/* '<Root>/Parse Thruster Data' */
};

/* Continuous states (default storage) */
struct X_Simulation_T {
  real_T Integrator2_CSTATE[3];        /* '<S1>/Integrator2' */
  real_T Integrator_CSTATE[6];         /* '<S1>/Integrator' */
  real_T Integrator1_CSTATE[3];        /* '<S1>/Integrator1' */
};

/* State derivatives (default storage) */
struct XDot_Simulation_T {
  real_T Integrator2_CSTATE[3];        /* '<S1>/Integrator2' */
  real_T Integrator_CSTATE[6];         /* '<S1>/Integrator' */
  real_T Integrator1_CSTATE[3];        /* '<S1>/Integrator1' */
};

/* State disabled  */
struct XDis_Simulation_T {
  boolean_T Integrator2_CSTATE[3];     /* '<S1>/Integrator2' */
  boolean_T Integrator_CSTATE[6];      /* '<S1>/Integrator' */
  boolean_T Integrator1_CSTATE[3];     /* '<S1>/Integrator1' */
};

/* Invariant block signals (default storage) */
struct ConstB_Simulation_T {
  real_T M[36];                        /* '<S10>/transpose' */
  real_T Sum[36];                      /* '<S10>/Sum' */
  real_T u2MM[36];                     /* '<S10>/Gain' */
  real_T M11[9];                       /* '<S10>/M11' */
  real_T M12[9];                       /* '<S10>/M12' */
  real_T M22[9];                       /* '<S10>/M22' */
  real_T M21M12[9];                    /* '<S10>/transpose1' */
  real_T M_i[36];                      /* '<S11>/transpose' */
  real_T Sum_m[36];                    /* '<S11>/Sum' */
  real_T u2MM_n[36];                   /* '<S11>/Gain' */
  real_T M11_n[9];                     /* '<S11>/M11' */
  real_T M12_g[9];                     /* '<S11>/M12' */
  real_T M22_d[9];                     /* '<S11>/M22' */
  real_T M21M12_c[9];                  /* '<S11>/transpose1' */
  real_T M_c[36];                      /* '<S1>/Sum2' */
};

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
struct ODE3_IntgData {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
};

#endif

/* Constant parameters (default storage) */
struct ConstP_Simulation_T {
  /* Expression: pwm_values
   * Referenced by: '<S21>/Constant'
   */
  real_T Constant_Value_c[201];

  /* Expression: voltage_values
   * Referenced by: '<S21>/Constant1'
   */
  real_T Constant1_Value_h[6];

  /* Expression: force_values
   * Referenced by: '<S21>/Constant2'
   */
  real_T Constant2_Value[1206];

  /* Expression: thruster_pos
   * Referenced by: '<S21>/Constant4'
   */
  real_T Constant4_Value_l[24];

  /* Expression: thruster_dir
   * Referenced by: '<S21>/Constant5'
   */
  real_T Constant5_Value_h[24];
};

/* Real-time Model Data Structure */
struct tag_RTM_Simulation_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;
  X_Simulation_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_Simulation_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[12];
  real_T odeF[3][12];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_Simulation_T Simulation_B;

#ifdef __cplusplus

}

#endif

/* Continuous states (default storage) */
extern X_Simulation_T Simulation_X;

/* Disabled states (default storage) */
extern XDis_Simulation_T Simulation_XDis;

/* Block states (default storage) */
extern struct DW_Simulation_T Simulation_DW;
extern const ConstB_Simulation_T Simulation_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Simulation_T Simulation_ConstP;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void Simulation_initialize(void);
  extern void Simulation_step(void);
  extern void Simulation_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_Simulation_T *const Simulation_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Cast To Single' : Unused code path elimination
 * Block '<Root>/Cast To Single1' : Unused code path elimination
 * Block '<S3>/Diff' : Unused code path elimination
 * Block '<S3>/UD' : Unused code path elimination
 * Block '<S4>/Diff' : Unused code path elimination
 * Block '<S4>/UD' : Unused code path elimination
 * Block '<S26>/Constant' : Unused code path elimination
 * Block '<S26>/Constant1' : Unused code path elimination
 * Block '<S26>/Gain' : Unused code path elimination
 * Block '<S26>/Product' : Unused code path elimination
 * Block '<S26>/Sign' : Unused code path elimination
 * Block '<S26>/Square' : Unused code path elimination
 * Block '<S27>/Constant' : Unused code path elimination
 * Block '<S27>/Constant1' : Unused code path elimination
 * Block '<S27>/Gain' : Unused code path elimination
 * Block '<S27>/Product' : Unused code path elimination
 * Block '<S27>/Sign' : Unused code path elimination
 * Block '<S27>/Square' : Unused code path elimination
 * Block '<S28>/Constant' : Unused code path elimination
 * Block '<S28>/Constant1' : Unused code path elimination
 * Block '<S28>/Gain' : Unused code path elimination
 * Block '<S28>/Product' : Unused code path elimination
 * Block '<S28>/Sign' : Unused code path elimination
 * Block '<S28>/Square' : Unused code path elimination
 * Block '<Root>/Gain' : Unused code path elimination
 * Block '<Root>/M_A' : Unused code path elimination
 * Block '<Root>/Transpose' : Unused code path elimination
 * Block '<Root>/Transpose1' : Unused code path elimination
 * Block '<Root>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double1' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Simulation'
 * '<S1>'   : 'Simulation/6 DOF Eqs. of motion'
 * '<S2>'   : 'Simulation/Blank Message'
 * '<S3>'   : 'Simulation/Difference'
 * '<S4>'   : 'Simulation/Difference1'
 * '<S5>'   : 'Simulation/External Force'
 * '<S6>'   : 'Simulation/MATLAB Function'
 * '<S7>'   : 'Simulation/Parse Thruster Data'
 * '<S8>'   : 'Simulation/Publish3'
 * '<S9>'   : 'Simulation/Subscribe'
 * '<S10>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces'
 * '<S11>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces1'
 * '<S12>'  : 'Simulation/6 DOF Eqs. of motion/Euler angles to rotation matrix'
 * '<S13>'  : 'Simulation/6 DOF Eqs. of motion/Euler angles to attitude transformation matrix'
 * '<S14>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces/3x3 cross product'
 * '<S15>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces/3x3 cross product1'
 * '<S16>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces/3x3 cross product2'
 * '<S17>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces1/3x3 cross product'
 * '<S18>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces1/3x3 cross product1'
 * '<S19>'  : 'Simulation/6 DOF Eqs. of motion/6 DOF Coriolis forces1/3x3 cross product2'
 * '<S20>'  : 'Simulation/External Force/Hydrodynamic Forces'
 * '<S21>'  : 'Simulation/External Force/Thruster Forces'
 * '<S22>'  : 'Simulation/External Force/Hydrodynamic Forces/Body to Inertial'
 * '<S23>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag'
 * '<S24>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag1'
 * '<S25>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag2'
 * '<S26>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag3'
 * '<S27>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag4'
 * '<S28>'  : 'Simulation/External Force/Hydrodynamic Forces/Calculate Drag5'
 * '<S29>'  : 'Simulation/External Force/Thruster Forces/Body to Inertial'
 * '<S30>'  : 'Simulation/External Force/Thruster Forces/PWM to Thrust'
 * '<S31>'  : 'Simulation/External Force/Thruster Forces/Thrust to Net Force and Moment'
 * '<S32>'  : 'Simulation/Parse Thruster Data/MATLAB Function'
 * '<S33>'  : 'Simulation/Subscribe/Enabled Subsystem'
 */
#endif                                 /* Simulation_h_ */
