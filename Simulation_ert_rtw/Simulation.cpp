/*
 * Simulation.cpp
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

#include "Simulation.h"
#include "rtwtypes.h"
#include "Simulation_types.h"
#include <string.h>
#include <math.h>
#include <emmintrin.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "Simulation_private.h"
#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "Simulation_dt.h"

/* Named constants for MATLAB Function: '<S20>/Body to Inertial' */
const int32_T Simulation_CALL_EVENT = -1;

/* Block signals (default storage) */
B_Simulation_T Simulation_B;

/* Continuous states */
X_Simulation_T Simulation_X;

/* Disabled State Vector */
XDis_Simulation_T Simulation_XDis;

/* Block states (default storage) */
DW_Simulation_T Simulation_DW;

/* Real-time model */
RT_MODEL_Simulation_T Simulation_M_ = RT_MODEL_Simulation_T();
RT_MODEL_Simulation_T *const Simulation_M = &Simulation_M_;

/* Forward declaration for local functions */
static void Simulation_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj);
static void Simulation_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
  *obj);
static void rt_mldivide_U1d6x6_U2d_4sw8yi_p(const real_T u0[36], const real_T
  u1[6], real_T y[6]);
static boolean_T Simulation_isUnitImag(const char_T s_data[], int32_T k, int32_T
  n);
static void Simulation_readNonFinite(const char_T s_data[], int32_T *k, int32_T
  n, boolean_T *b_finite, real_T *fv);
static boolean_T Simulation_copydigits(char_T s1_data[], int32_T *idx, const
  char_T s_data[], int32_T *k, int32_T n, boolean_T allowpoint);
static boolean_T Simulation_copyexponent(char_T s1_data[], int32_T *idx, const
  char_T s_data[], int32_T *k, int32_T n);
static void Simulation_copysign(char_T s1_data[], int32_T *idx, const char_T
  s_data[], int32_T *k, int32_T n, boolean_T *foundsign, boolean_T *success);
static void Simulation_readfloat(char_T s1_data[], int32_T *idx, const char_T
  s_data[], int32_T *k, int32_T n, boolean_T *isimag, boolean_T *b_finite,
  real_T *nfv, boolean_T *foundsign, boolean_T *success);
static creal_T Simulation_str2double(const char_T s_data[], const int32_T
  *s_size);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Simulation_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  Simulation_step();
  Simulation_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  Simulation_step();
  Simulation_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void Simulation_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[17];
  static const char_T b_zeroDelimTopic_0[17] = "/thruster_output";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S9>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S9>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_Simulation_36.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

static void Simulation_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
  *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[17];
  static const char_T b_zeroDelimTopic_0[17] = "/offsetedSensors";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S8>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S8>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_Simulation_382.createPublisher(&b_zeroDelimTopic[0], qos_profile);
}

static void rt_mldivide_U1d6x6_U2d_4sw8yi_p(const real_T u0[36], const real_T
  u1[6], real_T y[6])
{
  memcpy(&Simulation_B.x[0], &u0[0], 36U * sizeof(real_T));
  for (Simulation_B.k = 0; Simulation_B.k < 6; Simulation_B.k++) {
    Simulation_B.ipiv[Simulation_B.k] = static_cast<int8_T>(Simulation_B.k + 1);
  }

  for (Simulation_B.kAcol = 0; Simulation_B.kAcol < 5; Simulation_B.kAcol++) {
    Simulation_B.c = Simulation_B.kAcol * 7 + 2;
    Simulation_B.jj = Simulation_B.kAcol * 7;
    Simulation_B.c_b = 6 - Simulation_B.kAcol;
    Simulation_B.jA = 1;
    Simulation_B.smax = fabs(Simulation_B.x[Simulation_B.jj]);
    for (Simulation_B.k = 2; Simulation_B.k <= Simulation_B.c_b; Simulation_B.k
         ++) {
      Simulation_B.s = fabs(Simulation_B.x[(Simulation_B.c + Simulation_B.k) - 3]);
      if (Simulation_B.s > Simulation_B.smax) {
        Simulation_B.jA = Simulation_B.k;
        Simulation_B.smax = Simulation_B.s;
      }
    }

    if (Simulation_B.x[(Simulation_B.c + Simulation_B.jA) - 3] != 0.0) {
      if (Simulation_B.jA - 1 != 0) {
        Simulation_B.c_b = Simulation_B.kAcol + Simulation_B.jA;
        Simulation_B.ipiv[Simulation_B.kAcol] = static_cast<int8_T>
          (Simulation_B.c_b);
        for (Simulation_B.k = 0; Simulation_B.k < 6; Simulation_B.k++) {
          Simulation_B.jA = Simulation_B.k * 6 + Simulation_B.kAcol;
          Simulation_B.smax = Simulation_B.x[Simulation_B.jA];
          Simulation_B.x_tmp = (Simulation_B.k * 6 + Simulation_B.c_b) - 1;
          Simulation_B.x[Simulation_B.jA] = Simulation_B.x[Simulation_B.x_tmp];
          Simulation_B.x[Simulation_B.x_tmp] = Simulation_B.smax;
        }
      }

      Simulation_B.k = Simulation_B.c - Simulation_B.kAcol;
      for (Simulation_B.c_b = Simulation_B.c; Simulation_B.c_b <= Simulation_B.k
           + 4; Simulation_B.c_b++) {
        Simulation_B.x[Simulation_B.c_b - 1] /= Simulation_B.x[Simulation_B.jj];
      }
    }

    Simulation_B.c_b = 4 - Simulation_B.kAcol;
    Simulation_B.jA = Simulation_B.jj;
    for (Simulation_B.x_tmp = 0; Simulation_B.x_tmp <= Simulation_B.c_b;
         Simulation_B.x_tmp++) {
      Simulation_B.smax = Simulation_B.x[(Simulation_B.x_tmp * 6 +
        Simulation_B.jj) + 6];
      if (Simulation_B.smax != 0.0) {
        Simulation_B.k = Simulation_B.jA + 8;
        Simulation_B.i_n = (Simulation_B.jA - Simulation_B.kAcol) + 12;
        for (Simulation_B.ijA = Simulation_B.k; Simulation_B.ijA <=
             Simulation_B.i_n; Simulation_B.ijA++) {
          Simulation_B.x[Simulation_B.ijA - 1] += Simulation_B.x
            [((Simulation_B.c + Simulation_B.ijA) - Simulation_B.jA) - 9] *
            -Simulation_B.smax;
        }
      }

      Simulation_B.jA += 6;
    }
  }

  for (Simulation_B.k = 0; Simulation_B.k < 6; Simulation_B.k++) {
    y[Simulation_B.k] = u1[Simulation_B.k];
  }

  for (Simulation_B.c_b = 0; Simulation_B.c_b < 5; Simulation_B.c_b++) {
    Simulation_B.ipiv_j = Simulation_B.ipiv[Simulation_B.c_b];
    if (Simulation_B.c_b + 1 != Simulation_B.ipiv_j) {
      Simulation_B.smax = y[Simulation_B.c_b];
      y[Simulation_B.c_b] = y[Simulation_B.ipiv_j - 1];
      y[Simulation_B.ipiv_j - 1] = Simulation_B.smax;
    }
  }

  for (Simulation_B.k = 0; Simulation_B.k < 6; Simulation_B.k++) {
    Simulation_B.kAcol = 6 * Simulation_B.k - 1;
    if (y[Simulation_B.k] != 0.0) {
      for (Simulation_B.c_b = Simulation_B.k + 2; Simulation_B.c_b < 7;
           Simulation_B.c_b++) {
        y[Simulation_B.c_b - 1] -= Simulation_B.x[Simulation_B.c_b +
          Simulation_B.kAcol] * y[Simulation_B.k];
      }
    }
  }

  for (Simulation_B.k = 5; Simulation_B.k >= 0; Simulation_B.k--) {
    Simulation_B.kAcol = 6 * Simulation_B.k;
    Simulation_B.smax = y[Simulation_B.k];
    if (Simulation_B.smax != 0.0) {
      y[Simulation_B.k] = Simulation_B.smax / Simulation_B.x[Simulation_B.k +
        Simulation_B.kAcol];
      for (Simulation_B.c_b = 0; Simulation_B.c_b < Simulation_B.k;
           Simulation_B.c_b++) {
        y[Simulation_B.c_b] -= Simulation_B.x[Simulation_B.c_b +
          Simulation_B.kAcol] * y[Simulation_B.k];
      }
    }
  }
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static boolean_T Simulation_isUnitImag(const char_T s_data[], int32_T k, int32_T
  n)
{
  boolean_T p;
  p = false;
  if (k <= n) {
    Simulation_B.c4 = s_data[k - 1];
    if (Simulation_B.c4 == 'j') {
      p = true;
    } else if (Simulation_B.c4 == 'i') {
      if (k >= n - 1) {
        p = true;
      } else {
        Simulation_B.b_k_h = k;
        for (Simulation_B.j = 0; Simulation_B.j < 3; Simulation_B.j++) {
          Simulation_B.c_d[Simulation_B.j] = '\x00';
          while ((Simulation_B.b_k_h <= n) && (s_data[Simulation_B.b_k_h - 1] ==
                  ',')) {
            Simulation_B.b_k_h++;
          }

          if (Simulation_B.b_k_h <= n) {
            Simulation_B.c_d[Simulation_B.j] = s_data[Simulation_B.b_k_h - 1];
          }

          Simulation_B.b_k_h++;
        }

        if ((((Simulation_B.c_d[0] == 'I') || (Simulation_B.c_d[0] == 'i')) &&
             ((Simulation_B.c_d[1] == 'N') || (Simulation_B.c_d[1] == 'n')) &&
             ((Simulation_B.c_d[2] == 'F') || (Simulation_B.c_d[2] == 'f'))) ||
            (((Simulation_B.c_d[0] == 'N') || (Simulation_B.c_d[0] == 'n')) &&
             ((Simulation_B.c_d[1] == 'A') || (Simulation_B.c_d[1] == 'a')) &&
             ((Simulation_B.c_d[2] == 'N') || (Simulation_B.c_d[2] == 'n')))) {
        } else {
          p = true;
        }
      }
    }
  }

  return p;
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static void Simulation_readNonFinite(const char_T s_data[], int32_T *k, int32_T
  n, boolean_T *b_finite, real_T *fv)
{
  Simulation_B.ksaved = *k;
  for (Simulation_B.j_b = 0; Simulation_B.j_b < 3; Simulation_B.j_b++) {
    Simulation_B.c_e[Simulation_B.j_b] = '\x00';
    while ((*k <= n) && (s_data[*k - 1] == ',')) {
      (*k)++;
    }

    if (*k <= n) {
      Simulation_B.c_e[Simulation_B.j_b] = s_data[*k - 1];
    }

    (*k)++;
  }

  if (((Simulation_B.c_e[0] == 'I') || (Simulation_B.c_e[0] == 'i')) &&
      ((Simulation_B.c_e[1] == 'N') || (Simulation_B.c_e[1] == 'n')) &&
      ((Simulation_B.c_e[2] == 'F') || (Simulation_B.c_e[2] == 'f'))) {
    *b_finite = false;
    *fv = (rtInf);
  } else if (((Simulation_B.c_e[0] == 'N') || (Simulation_B.c_e[0] == 'n')) &&
             ((Simulation_B.c_e[1] == 'A') || (Simulation_B.c_e[1] == 'a')) &&
             ((Simulation_B.c_e[2] == 'N') || (Simulation_B.c_e[2] == 'n'))) {
    *b_finite = false;
    *fv = (rtNaN);
  } else {
    *b_finite = true;
    *fv = 0.0;
    *k = Simulation_B.ksaved;
  }
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static boolean_T Simulation_copydigits(char_T s1_data[], int32_T *idx, const
  char_T s_data[], int32_T *k, int32_T n, boolean_T allowpoint)
{
  boolean_T exitg1;
  boolean_T success;
  success = (*k <= n);
  Simulation_B.haspoint = false;
  exitg1 = false;
  while ((!exitg1) && (success && (*k <= n))) {
    Simulation_B.c3 = s_data[*k - 1];
    if ((Simulation_B.c3 >= '0') && (Simulation_B.c3 <= '9')) {
      s1_data[*idx - 1] = Simulation_B.c3;
      (*idx)++;
      (*k)++;
    } else if (Simulation_B.c3 == '.') {
      success = (allowpoint && (!Simulation_B.haspoint));
      if (success) {
        s1_data[*idx - 1] = '.';
        (*idx)++;
        Simulation_B.haspoint = true;
      }

      (*k)++;
    } else if (Simulation_B.c3 == ',') {
      (*k)++;
    } else {
      exitg1 = true;
    }
  }

  return success;
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static boolean_T Simulation_copyexponent(char_T s1_data[], int32_T *idx, const
  char_T s_data[], int32_T *k, int32_T n)
{
  boolean_T success;
  success = true;
  if (*k <= n) {
    Simulation_B.c2 = s_data[*k - 1];
    if ((Simulation_B.c2 == 'E') || (Simulation_B.c2 == 'e')) {
      s1_data[*idx - 1] = 'e';
      (*idx)++;
      (*k)++;
      while ((*k <= n) && (s_data[*k - 1] == ',')) {
        (*k)++;
      }

      if (*k <= n) {
        Simulation_B.c2 = s_data[*k - 1];
        if (Simulation_B.c2 == '-') {
          s1_data[*idx - 1] = '-';
          (*idx)++;
          (*k)++;
        } else if (Simulation_B.c2 == '+') {
          (*k)++;
        }
      }

      Simulation_B.kexp = *k;
      Simulation_B.b_k_l = *k;
      Simulation_B.b_success = Simulation_copydigits(s1_data, idx, s_data,
        &Simulation_B.b_k_l, n, false);
      *k = Simulation_B.b_k_l;
      if ((!Simulation_B.b_success) || (Simulation_B.b_k_l <= Simulation_B.kexp))
      {
        success = false;
      }
    }
  }

  return success;
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static void Simulation_copysign(char_T s1_data[], int32_T *idx, const char_T
  s_data[], int32_T *k, int32_T n, boolean_T *foundsign, boolean_T *success)
{
  static const boolean_T b[128] = { false, false, false, false, false, false,
    false, false, false, true, true, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    true, true, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false };

  boolean_T exitg1;
  Simulation_B.isneg_f = false;
  *foundsign = false;
  exitg1 = false;
  while ((!exitg1) && (*k <= n)) {
    Simulation_B.c5 = s_data[*k - 1];
    if (Simulation_B.c5 == '-') {
      Simulation_B.isneg_f = !Simulation_B.isneg_f;
      *foundsign = true;
      (*k)++;
    } else if (Simulation_B.c5 == ',') {
      (*k)++;
    } else if (Simulation_B.c5 == '+') {
      *foundsign = true;
      (*k)++;
    } else if (!b[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c5) &
                127U)]) {
      exitg1 = true;
    } else {
      (*k)++;
    }
  }

  *success = (*k <= n);
  if ((*success) && Simulation_B.isneg_f) {
    boolean_T guard1;
    guard1 = false;
    if (*idx >= 2) {
      Simulation_B.i1 = *idx - 2;
      if (s1_data[Simulation_B.i1] == '-') {
        s1_data[Simulation_B.i1] = ' ';
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      s1_data[*idx - 1] = '-';
      (*idx)++;
    }
  }
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static void Simulation_readfloat(char_T s1_data[], int32_T *idx, const char_T
  s_data[], int32_T *k, int32_T n, boolean_T *isimag, boolean_T *b_finite,
  real_T *nfv, boolean_T *foundsign, boolean_T *success)
{
  static const boolean_T b[128] = { false, false, false, false, false, false,
    false, false, false, true, true, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    true, true, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false };

  boolean_T exitg1;
  *isimag = false;
  *b_finite = true;
  *nfv = 0.0;
  Simulation_B.b_idx = *idx;
  Simulation_B.b_k = *k;
  Simulation_copysign(s1_data, &Simulation_B.b_idx, s_data, &Simulation_B.b_k, n,
                      foundsign, success);
  *idx = Simulation_B.b_idx;
  *k = Simulation_B.b_k;
  if (*success) {
    if (Simulation_isUnitImag(s_data, Simulation_B.b_k, n)) {
      *success = false;
    } else {
      Simulation_readNonFinite(s_data, k, n, b_finite, nfv);
      if (*b_finite) {
        *success = Simulation_copydigits(s1_data, idx, s_data, k, n, true);
        if (*success) {
          *success = Simulation_copyexponent(s1_data, idx, s_data, k, n);
        }
      } else if (Simulation_B.b_idx >= 2) {
        Simulation_B.b_k = Simulation_B.b_idx - 2;
        if (s1_data[Simulation_B.b_k] == '-') {
          *idx = Simulation_B.b_idx - 1;
          s1_data[Simulation_B.b_k] = ' ';
          *nfv = -*nfv;
        }
      }

      exitg1 = false;
      while ((!exitg1) && (*k <= n)) {
        Simulation_B.c1 = s_data[*k - 1];
        if (b[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c1) & 127U)]
            || (Simulation_B.c1 == '\x00') || (Simulation_B.c1 == ',')) {
          (*k)++;
        } else {
          exitg1 = true;
        }
      }

      if ((*k <= n) && (s_data[*k - 1] == '*')) {
        (*k)++;
        exitg1 = false;
        while ((!exitg1) && (*k <= n)) {
          Simulation_B.c1 = s_data[*k - 1];
          if (b[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c1) &
               127U)] || (Simulation_B.c1 == '\x00') || (Simulation_B.c1 == ','))
          {
            (*k)++;
          } else {
            exitg1 = true;
          }
        }
      }

      if (*k <= n) {
        Simulation_B.c1 = s_data[*k - 1];
        if ((Simulation_B.c1 == 'i') || (Simulation_B.c1 == 'j')) {
          (*k)++;
          *isimag = true;
        }
      }
    }

    exitg1 = false;
    while ((!exitg1) && (*k <= n)) {
      Simulation_B.c1 = s_data[*k - 1];
      if (b[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c1) & 127U)] ||
          (Simulation_B.c1 == '\x00') || (Simulation_B.c1 == ',')) {
        (*k)++;
      } else {
        exitg1 = true;
      }
    }
  }
}

/* Function for MATLAB Function: '<S7>/MATLAB Function' */
static creal_T Simulation_str2double(const char_T s_data[], const int32_T
  *s_size)
{
  creal_T x;
  static const boolean_T c[128] = { false, false, false, false, false, false,
    false, false, false, true, true, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    true, true, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false };

  boolean_T exitg1;
  x.re = (rtNaN);
  x.im = 0.0;
  Simulation_B.b_nread = *s_size;
  if (*s_size >= 1) {
    Simulation_B.b_s1_size[1] = *s_size + 2;
    memset(&Simulation_B.b_s1_data[0], 0, static_cast<uint32_T>
           (Simulation_B.b_nread + 2) * sizeof(char_T));
    Simulation_B.ntoread = 0;
    Simulation_B.k_b = 1;
    exitg1 = false;
    while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
      Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
      if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) & 127U)]
          || (Simulation_B.c_bj == '\x00')) {
        Simulation_B.k_b++;
      } else {
        exitg1 = true;
      }
    }

    Simulation_B.isimag1 = false;
    Simulation_B.b_finite = true;
    Simulation_B.scanned1 = 0.0;
    Simulation_B.idx = 1;
    Simulation_B.isneg = false;
    exitg1 = false;
    while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
      Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
      if (Simulation_B.c_bj == '-') {
        Simulation_B.isneg = !Simulation_B.isneg;
        Simulation_B.k_b++;
      } else if ((Simulation_B.c_bj == ',') || (Simulation_B.c_bj == '+') || c[
                 static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
                  127U)]) {
        Simulation_B.k_b++;
      } else {
        exitg1 = true;
      }
    }

    Simulation_B.success = (Simulation_B.k_b <= *s_size);
    if (Simulation_B.success && Simulation_B.isneg) {
      Simulation_B.b_s1_data[0] = '-';
      Simulation_B.idx = 2;
    }

    if (Simulation_B.success) {
      if (Simulation_isUnitImag(s_data, Simulation_B.k_b, *s_size)) {
        Simulation_B.isimag1 = true;
        Simulation_B.k_b++;
        exitg1 = false;
        while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
          Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
          if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
               127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
               ',')) {
            Simulation_B.k_b++;
          } else {
            exitg1 = true;
          }
        }

        if ((Simulation_B.k_b <= *s_size) && (s_data[Simulation_B.k_b - 1] ==
             '*')) {
          Simulation_B.k_b++;
          Simulation_readfloat(Simulation_B.b_s1_data, &Simulation_B.idx, s_data,
                               &Simulation_B.k_b, *s_size, &Simulation_B.isneg,
                               &Simulation_B.b_finite, &Simulation_B.scanned1,
                               &Simulation_B.a__3, &Simulation_B.success);
        } else {
          Simulation_B.b_s1_data[Simulation_B.idx - 1] = '1';
          Simulation_B.idx++;
        }
      } else {
        Simulation_readNonFinite(s_data, &Simulation_B.k_b, *s_size,
          &Simulation_B.b_finite, &Simulation_B.scanned1);
        if (Simulation_B.b_finite) {
          Simulation_B.success = Simulation_copydigits(Simulation_B.b_s1_data,
            &Simulation_B.idx, s_data, &Simulation_B.k_b, *s_size, true);
          if (Simulation_B.success) {
            Simulation_B.success = Simulation_copyexponent
              (Simulation_B.b_s1_data, &Simulation_B.idx, s_data,
               &Simulation_B.k_b, *s_size);
          }
        } else if (Simulation_B.idx >= 2) {
          Simulation_B.loop_ub = 0;
          if (Simulation_B.b_s1_data[Simulation_B.loop_ub] == '-') {
            Simulation_B.idx = 1;
            Simulation_B.b_s1_data[Simulation_B.loop_ub] = ' ';
            Simulation_B.scanned1 = -Simulation_B.scanned1;
          }
        }

        exitg1 = false;
        while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
          Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
          if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
               127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
               ',')) {
            Simulation_B.k_b++;
          } else {
            exitg1 = true;
          }
        }

        if ((Simulation_B.k_b <= *s_size) && (s_data[Simulation_B.k_b - 1] ==
             '*')) {
          Simulation_B.k_b++;
          exitg1 = false;
          while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
            Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
            if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
                 127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
                 ',')) {
              Simulation_B.k_b++;
            } else {
              exitg1 = true;
            }
          }
        }

        if (Simulation_B.k_b <= *s_size) {
          Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
          if ((Simulation_B.c_bj == 'i') || (Simulation_B.c_bj == 'j')) {
            Simulation_B.k_b++;
            Simulation_B.isimag1 = true;
          }
        }
      }

      exitg1 = false;
      while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
        Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
        if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
             127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
             ',')) {
          Simulation_B.k_b++;
        } else {
          exitg1 = true;
        }
      }
    }

    Simulation_B.loop_ub = Simulation_B.b_s1_size[1];
    if (Simulation_B.loop_ub - 1 >= 0) {
      memcpy(&Simulation_B.s1_data[0], &Simulation_B.b_s1_data[0], static_cast<
             uint32_T>(Simulation_B.loop_ub) * sizeof(char_T));
    }

    if (Simulation_B.b_finite) {
      Simulation_B.ntoread = 1;
    }

    if (Simulation_B.success && (Simulation_B.k_b <= *s_size)) {
      Simulation_B.s1_data[Simulation_B.idx - 1] = ' ';
      Simulation_B.success = false;
      Simulation_B.a__3 = true;
      Simulation_B.scanned2 = 0.0;
      Simulation_B.idx++;
      Simulation_copysign(Simulation_B.s1_data, &Simulation_B.idx, s_data,
                          &Simulation_B.k_b, *s_size, &Simulation_B.isneg,
                          &Simulation_B.f_success);
      if (Simulation_B.f_success) {
        if (Simulation_isUnitImag(s_data, Simulation_B.k_b, *s_size)) {
          Simulation_B.success = true;
          Simulation_B.k_b++;
          exitg1 = false;
          while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
            Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
            if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
                 127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
                 ',')) {
              Simulation_B.k_b++;
            } else {
              exitg1 = true;
            }
          }

          if ((Simulation_B.k_b <= *s_size) && (s_data[Simulation_B.k_b - 1] ==
               '*')) {
            Simulation_B.k_b++;
            Simulation_readfloat(Simulation_B.s1_data, &Simulation_B.idx, s_data,
                                 &Simulation_B.k_b, *s_size,
                                 &Simulation_B.b_a__2, &Simulation_B.a__3,
                                 &Simulation_B.scanned2, &Simulation_B.b_a__3,
                                 &Simulation_B.f_success);
          } else {
            Simulation_B.s1_data[Simulation_B.idx - 1] = '1';
            Simulation_B.idx++;
          }
        } else {
          Simulation_readNonFinite(s_data, &Simulation_B.k_b, *s_size,
            &Simulation_B.a__3, &Simulation_B.scanned2);
          if (Simulation_B.a__3) {
            Simulation_B.f_success = Simulation_copydigits(Simulation_B.s1_data,
              &Simulation_B.idx, s_data, &Simulation_B.k_b, *s_size, true);
            if (Simulation_B.f_success) {
              Simulation_B.f_success = Simulation_copyexponent
                (Simulation_B.s1_data, &Simulation_B.idx, s_data,
                 &Simulation_B.k_b, *s_size);
            }
          } else if (Simulation_B.idx >= 2) {
            Simulation_B.loop_ub = Simulation_B.idx - 2;
            if (Simulation_B.s1_data[Simulation_B.loop_ub] == '-') {
              Simulation_B.idx--;
              Simulation_B.s1_data[Simulation_B.loop_ub] = ' ';
              Simulation_B.scanned2 = -Simulation_B.scanned2;
            }
          }

          exitg1 = false;
          while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
            Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
            if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
                 127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
                 ',')) {
              Simulation_B.k_b++;
            } else {
              exitg1 = true;
            }
          }

          if ((Simulation_B.k_b <= *s_size) && (s_data[Simulation_B.k_b - 1] ==
               '*')) {
            Simulation_B.k_b++;
            exitg1 = false;
            while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
              Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
              if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj)
                   & 127U)] || (Simulation_B.c_bj == '\x00') ||
                  (Simulation_B.c_bj == ',')) {
                Simulation_B.k_b++;
              } else {
                exitg1 = true;
              }
            }
          }

          if (Simulation_B.k_b <= *s_size) {
            Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
            if ((Simulation_B.c_bj == 'i') || (Simulation_B.c_bj == 'j')) {
              Simulation_B.k_b++;
              Simulation_B.success = true;
            }
          }
        }

        exitg1 = false;
        while ((!exitg1) && (Simulation_B.k_b <= Simulation_B.b_nread)) {
          Simulation_B.c_bj = s_data[Simulation_B.k_b - 1];
          if (c[static_cast<int32_T>(static_cast<uint8_T>(Simulation_B.c_bj) &
               127U)] || (Simulation_B.c_bj == '\x00') || (Simulation_B.c_bj ==
               ',')) {
            Simulation_B.k_b++;
          } else {
            exitg1 = true;
          }
        }
      }

      if (Simulation_B.a__3) {
        Simulation_B.ntoread++;
      }

      if (Simulation_B.f_success && (Simulation_B.k_b > *s_size) && ((
            static_cast<boolean_T>(Simulation_B.isimag1 ^ Simulation_B.success))
           && Simulation_B.isneg)) {
        Simulation_B.success = true;
      } else {
        Simulation_B.success = false;
      }
    } else {
      Simulation_B.scanned2 = 0.0;
    }

    if (Simulation_B.success) {
      Simulation_B.s1_data[Simulation_B.idx - 1] = '\x00';
      if (Simulation_B.ntoread == 2) {
        Simulation_B.b_nread = std::sscanf(&Simulation_B.s1_data[0], "%lf %lf",
          &Simulation_B.scanned1, &Simulation_B.scanned2);
        if (Simulation_B.b_nread != 2) {
          Simulation_B.scanned1 = (rtNaN);
          Simulation_B.scanned2 = (rtNaN);
        }
      } else if (Simulation_B.ntoread == 1) {
        Simulation_B.b_nread = std::sscanf(&Simulation_B.s1_data[0], "%lf",
          &Simulation_B.b_scanned1);
        if (Simulation_B.b_finite) {
          if (Simulation_B.b_nread == 1) {
            Simulation_B.scanned1 = Simulation_B.b_scanned1;
          } else {
            Simulation_B.scanned1 = (rtNaN);
          }
        } else if (Simulation_B.b_nread == 1) {
          Simulation_B.scanned2 = Simulation_B.b_scanned1;
        } else {
          Simulation_B.scanned2 = (rtNaN);
        }
      }

      if (Simulation_B.isimag1) {
        x.re = Simulation_B.scanned2;
        x.im = Simulation_B.scanned1;
      } else {
        x.re = Simulation_B.scanned1;
        x.im = Simulation_B.scanned2;
      }
    }
  }

  return x;
}

/* Model step function */
void Simulation_step(void)
{
  /* local scratch DWork variables */
  int32_T ForEach_itr;
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  __m128d tmp_5;
  static const int8_T d[3] = { 1, 0, 0 };

  if (rtmIsMajorTimeStep(Simulation_M)) {
    /* set solver stop time */
    if (!(Simulation_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Simulation_M->solverInfo,
                            ((Simulation_M->Timing.clockTickH0 + 1) *
        Simulation_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Simulation_M->solverInfo,
                            ((Simulation_M->Timing.clockTick0 + 1) *
        Simulation_M->Timing.stepSize0 + Simulation_M->Timing.clockTickH0 *
        Simulation_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Simulation_M)) {
    Simulation_M->Timing.t[0] = rtsiGetT(&Simulation_M->solverInfo);
  }

  /* Reset subsysRan breadcrumbs */
  srClearBC(Simulation_DW.EnabledSubsystem_SubsysRanBC);

  /* Integrator: '<S1>/Integrator2' */
  Simulation_B.theta[0] = Simulation_X.Integrator2_CSTATE[0];

  /* Trigonometry: '<S12>/sin(theta)' incorporates:
   *  Fcn: '<S13>/T21 '
   */
  Simulation_B.rtb_sintheta_idx_0_tmp = sin(Simulation_B.theta[0]);

  /* Trigonometry: '<S12>/cos(theta)' incorporates:
   *  Fcn: '<S13>/T31 '
   */
  Simulation_B.rtb_costheta_idx_0_tmp = cos(Simulation_B.theta[0]);

  /* Integrator: '<S1>/Integrator2' */
  Simulation_B.theta[1] = Simulation_X.Integrator2_CSTATE[1];

  /* Trigonometry: '<S12>/sin(theta)' */
  Simulation_B.rtb_sintheta_idx_1 = sin(Simulation_B.theta[1]);

  /* Trigonometry: '<S12>/cos(theta)' incorporates:
   *  Fcn: '<S13>/T23'
   */
  Simulation_B.rtb_costheta_idx_1_tmp = cos(Simulation_B.theta[1]);

  /* Integrator: '<S1>/Integrator2' */
  Simulation_B.theta[2] = Simulation_X.Integrator2_CSTATE[2];

  /* Trigonometry: '<S12>/sin(theta)' */
  Simulation_B.rtb_sintheta_idx_2 = sin(Simulation_B.theta[2]);

  /* Trigonometry: '<S12>/cos(theta)' */
  Simulation_B.rtb_costheta_idx_2 = cos(Simulation_B.theta[2]);
  Simulation_B.b = rtmIsMajorTimeStep(Simulation_M);
  if (Simulation_B.b) {
    /* MATLABSystem: '<S9>/SourceBlock' */
    Simulation_B.b_varargout_1 = Sub_Simulation_36.getLatestMessage
      (&Simulation_B.rtb_SourceBlock_o2_m);

    /* Outputs for Enabled SubSystem: '<S9>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S33>/Enable'
     */
    /* Start for MATLABSystem: '<S9>/SourceBlock' */
    if (Simulation_B.b_varargout_1) {
      /* SignalConversion generated from: '<S33>/In1' */
      Simulation_B.In1 = Simulation_B.rtb_SourceBlock_o2_m;
      if (rtsiIsModeUpdateTimeStep(&Simulation_M->solverInfo)) {
        srUpdateBC(Simulation_DW.EnabledSubsystem_SubsysRanBC);
      }
    }

    /* End of Start for MATLABSystem: '<S9>/SourceBlock' */
    /* End of Outputs for SubSystem: '<S9>/Enabled Subsystem' */

    /* Outputs for Iterator SubSystem: '<Root>/Parse Thruster Data' incorporates:
     *  ForEach: '<S7>/For Each'
     */
    for (ForEach_itr = 0; ForEach_itr < 1; ForEach_itr++) {
      /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
       *  ForEachSliceSelector generated from: '<S7>/In1'
       */
      Simulation_DW.CoreSubsys[ForEach_itr].sfEvent = Simulation_CALL_EVENT;
      for (Simulation_B.i = 0; Simulation_B.i < 128; Simulation_B.i++) {
        Simulation_B.u[Simulation_B.i] = static_cast<int8_T>
          (Simulation_B.In1.data[Simulation_B.i]);
      }

      memset(&Simulation_B.split_values[0], 0, sizeof(real_T) << 3U);
      Simulation_B.ImpAsg_InsertedFor_t1_at_in = 1.0;
      Simulation_B.low_i = 0;
      for (Simulation_B.i = 0; Simulation_B.i < 128; Simulation_B.i++) {
        if ((Simulation_B.u[Simulation_B.i] == ',') || (Simulation_B.i + 1 ==
             128)) {
          if (Simulation_B.i + 1 == 128) {
            if (Simulation_B.low_i + 1 > 128) {
              Simulation_B.low_i = 0;
              Simulation_B.high_i = 0;
            } else {
              Simulation_B.high_i = 128;
            }

            Simulation_B.low_ip1 = Simulation_B.high_i - Simulation_B.low_i;
            Simulation_B.u_size_o = Simulation_B.low_ip1;
            for (Simulation_B.high_i = 0; Simulation_B.high_i <
                 Simulation_B.low_ip1; Simulation_B.high_i++) {
              Simulation_B.u_data[Simulation_B.high_i] =
                Simulation_B.u[Simulation_B.low_i + Simulation_B.high_i];
            }

            Simulation_B.temp_val = Simulation_str2double(Simulation_B.u_data,
              &Simulation_B.u_size_o);
          } else {
            if (Simulation_B.low_i + 1 > Simulation_B.i) {
              Simulation_B.low_i = 0;
              Simulation_B.high_i = 0;
            } else {
              Simulation_B.high_i = Simulation_B.i;
            }

            Simulation_B.low_ip1 = Simulation_B.high_i - Simulation_B.low_i;
            Simulation_B.u_size = Simulation_B.low_ip1;
            for (Simulation_B.high_i = 0; Simulation_B.high_i <
                 Simulation_B.low_ip1; Simulation_B.high_i++) {
              Simulation_B.u_data_c[Simulation_B.high_i] =
                Simulation_B.u[Simulation_B.low_i + Simulation_B.high_i];
            }

            Simulation_B.temp_val = Simulation_str2double(Simulation_B.u_data_c,
              &Simulation_B.u_size);
          }

          if ((!rtIsNaN(Simulation_B.temp_val.re)) && (!rtIsNaN
               (Simulation_B.temp_val.im))) {
            Simulation_B.split_values[static_cast<int32_T>
              (Simulation_B.ImpAsg_InsertedFor_t1_at_in) - 1] =
              Simulation_B.temp_val.re;
          }

          Simulation_B.ImpAsg_InsertedFor_t1_at_in++;
          Simulation_B.low_i = Simulation_B.i + 1;
        }
      }

      /* ForEachSliceAssignment generated from: '<S7>/t8' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t8_at_in = Simulation_B.split_values[7];

      /* ForEachSliceAssignment generated from: '<S7>/t7' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t7_at_in = Simulation_B.split_values[6];

      /* ForEachSliceAssignment generated from: '<S7>/t6' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t6_at_in = Simulation_B.split_values[5];

      /* ForEachSliceAssignment generated from: '<S7>/t5' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t5_at_in = Simulation_B.split_values[4];

      /* ForEachSliceAssignment generated from: '<S7>/t4' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t4_at_in = Simulation_B.split_values[3];

      /* ForEachSliceAssignment generated from: '<S7>/t3' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t3_at_in = Simulation_B.split_values[2];

      /* ForEachSliceAssignment generated from: '<S7>/t2' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t2_at_in = Simulation_B.split_values[1];

      /* ForEachSliceAssignment generated from: '<S7>/t1' incorporates:
       *  MATLAB Function: '<S7>/MATLAB Function'
       */
      Simulation_B.ImpAsg_InsertedFor_t1_at_in = Simulation_B.split_values[0];
    }

    /* End of Outputs for SubSystem: '<Root>/Parse Thruster Data' */

    /* MATLAB Function: '<S21>/PWM to Thrust' incorporates:
     *  Constant: '<S21>/Constant'
     *  Constant: '<S21>/Constant1'
     *  Constant: '<S21>/Constant2'
     *  Constant: '<S21>/Constant3'
     */
    Simulation_DW.sfEvent_m = Simulation_CALL_EVENT;
    Simulation_B.split_values[0] = Simulation_B.ImpAsg_InsertedFor_t1_at_in;
    Simulation_B.split_values[1] = Simulation_B.ImpAsg_InsertedFor_t2_at_in;
    Simulation_B.split_values[2] = Simulation_B.ImpAsg_InsertedFor_t3_at_in;
    Simulation_B.split_values[3] = Simulation_B.ImpAsg_InsertedFor_t4_at_in;
    Simulation_B.split_values[4] = Simulation_B.ImpAsg_InsertedFor_t5_at_in;
    Simulation_B.split_values[5] = Simulation_B.ImpAsg_InsertedFor_t6_at_in;
    Simulation_B.split_values[6] = Simulation_B.ImpAsg_InsertedFor_t7_at_in;
    Simulation_B.split_values[7] = Simulation_B.ImpAsg_InsertedFor_t8_at_in;
    for (Simulation_B.i = 0; Simulation_B.i < 8; Simulation_B.i++) {
      Simulation_B.ImpAsg_InsertedFor_t1_at_in =
        Simulation_B.split_values[Simulation_B.i];
      Simulation_B.ImpAsg_InsertedFor_t1_at_in = ((static_cast<real_T>
        (Simulation_B.ImpAsg_InsertedFor_t1_at_in == 0.0) * 7.5 +
        static_cast<real_T>(!(Simulation_B.ImpAsg_InsertedFor_t1_at_in == 0.0)) *
        Simulation_B.ImpAsg_InsertedFor_t1_at_in) - 5.5) / 4.0 * 800.0 + 1100.0;
      if (!(Simulation_B.ImpAsg_InsertedFor_t1_at_in >= 1100.0)) {
        Simulation_B.ImpAsg_InsertedFor_t1_at_in = 1100.0;
      }

      Simulation_B.split_values[Simulation_B.i] =
        Simulation_B.ImpAsg_InsertedFor_t1_at_in;
      if (Simulation_B.ImpAsg_InsertedFor_t1_at_in <= 1900.0) {
        Simulation_B.duty_cycle_array[Simulation_B.i] =
          Simulation_B.ImpAsg_InsertedFor_t1_at_in;
      } else {
        Simulation_B.duty_cycle_array[Simulation_B.i] = 1900.0;
      }

      Simulation_B.high_i = 201;
      Simulation_B.low_i = 0;
      Simulation_B.low_ip1 = 2;
      while (Simulation_B.high_i > Simulation_B.low_ip1) {
        Simulation_B.b_low_ip1 = ((Simulation_B.low_i + Simulation_B.high_i) + 1)
          >> 1;
        if (Simulation_B.duty_cycle_array[Simulation_B.i] >=
            Simulation_ConstP.Constant_Value_c[Simulation_B.b_low_ip1 - 1]) {
          Simulation_B.low_i = Simulation_B.b_low_ip1 - 1;
          Simulation_B.low_ip1 = Simulation_B.b_low_ip1 + 1;
        } else {
          Simulation_B.high_i = Simulation_B.b_low_ip1;
        }
      }

      Simulation_B.high_i = 6;
      Simulation_B.low_ip1 = 1;
      Simulation_B.b_low_ip1 = 2;
      while (Simulation_B.high_i > Simulation_B.b_low_ip1) {
        Simulation_B.b_mid_i = (Simulation_B.low_ip1 + Simulation_B.high_i) >> 1;
        if (Simulation_ConstP.Constant1_Value_h[Simulation_B.b_mid_i - 1] <=
            16.0) {
          Simulation_B.low_ip1 = Simulation_B.b_mid_i;
          Simulation_B.b_low_ip1 = Simulation_B.b_mid_i + 1;
        } else {
          Simulation_B.high_i = Simulation_B.b_mid_i;
        }
      }

      Simulation_B.ImpAsg_InsertedFor_t1_at_in =
        Simulation_B.duty_cycle_array[Simulation_B.i];
      if (Simulation_B.ImpAsg_InsertedFor_t1_at_in ==
          Simulation_ConstP.Constant_Value_c[Simulation_B.low_i]) {
        Simulation_B.high_i = 6 * Simulation_B.low_i + Simulation_B.low_ip1;
        Simulation_B.ImpAsg_InsertedFor_t2_at_in =
          Simulation_ConstP.Constant2_Value[Simulation_B.high_i - 1];
        Simulation_B.ImpAsg_InsertedFor_t3_at_in =
          Simulation_ConstP.Constant2_Value[Simulation_B.high_i];
      } else {
        Simulation_B.scale =
          Simulation_ConstP.Constant_Value_c[Simulation_B.low_i + 1];
        if (Simulation_B.scale == Simulation_B.ImpAsg_InsertedFor_t1_at_in) {
          Simulation_B.high_i = (Simulation_B.low_i + 1) * 6 +
            Simulation_B.low_ip1;
          Simulation_B.ImpAsg_InsertedFor_t2_at_in =
            Simulation_ConstP.Constant2_Value[Simulation_B.high_i - 1];
          Simulation_B.ImpAsg_InsertedFor_t3_at_in =
            Simulation_ConstP.Constant2_Value[Simulation_B.high_i];
        } else {
          Simulation_B.ImpAsg_InsertedFor_t1_at_in =
            (Simulation_B.ImpAsg_InsertedFor_t1_at_in -
             Simulation_ConstP.Constant_Value_c[Simulation_B.low_i]) /
            (Simulation_B.scale -
             Simulation_ConstP.Constant_Value_c[Simulation_B.low_i]);
          Simulation_B.high_i = (Simulation_B.low_i + 1) * 6 +
            Simulation_B.low_ip1;
          Simulation_B.scale =
            Simulation_ConstP.Constant2_Value[Simulation_B.high_i - 1];
          Simulation_B.low_i = 6 * Simulation_B.low_i + Simulation_B.low_ip1;
          Simulation_B.ImpAsg_InsertedFor_t2_at_in =
            Simulation_ConstP.Constant2_Value[Simulation_B.low_i - 1];
          if (Simulation_B.scale == Simulation_B.ImpAsg_InsertedFor_t2_at_in) {
          } else {
            Simulation_B.ImpAsg_InsertedFor_t2_at_in = (1.0 -
              Simulation_B.ImpAsg_InsertedFor_t1_at_in) *
              Simulation_B.ImpAsg_InsertedFor_t2_at_in + Simulation_B.scale *
              Simulation_B.ImpAsg_InsertedFor_t1_at_in;
          }

          Simulation_B.scale =
            Simulation_ConstP.Constant2_Value[Simulation_B.high_i];
          Simulation_B.ImpAsg_InsertedFor_t3_at_in =
            Simulation_ConstP.Constant2_Value[Simulation_B.low_i];
          if (!(Simulation_B.scale == Simulation_B.ImpAsg_InsertedFor_t3_at_in))
          {
            Simulation_B.ImpAsg_InsertedFor_t3_at_in = (1.0 -
              Simulation_B.ImpAsg_InsertedFor_t1_at_in) *
              Simulation_B.ImpAsg_InsertedFor_t3_at_in + Simulation_B.scale *
              Simulation_B.ImpAsg_InsertedFor_t1_at_in;
          }
        }
      }

      Simulation_B.scale =
        Simulation_ConstP.Constant1_Value_h[Simulation_B.low_ip1 - 1];
      if ((Simulation_B.scale == 16.0) ||
          (Simulation_B.ImpAsg_InsertedFor_t2_at_in ==
           Simulation_B.ImpAsg_InsertedFor_t3_at_in)) {
        Simulation_B.thruster_forces[Simulation_B.i] =
          Simulation_B.ImpAsg_InsertedFor_t2_at_in;
      } else if (Simulation_ConstP.Constant1_Value_h[Simulation_B.low_ip1] ==
                 16.0) {
        Simulation_B.thruster_forces[Simulation_B.i] =
          Simulation_B.ImpAsg_InsertedFor_t3_at_in;
      } else {
        Simulation_B.ImpAsg_InsertedFor_t1_at_in = (16.0 - Simulation_B.scale) /
          (Simulation_ConstP.Constant1_Value_h[Simulation_B.low_ip1] -
           Simulation_B.scale);
        Simulation_B.thruster_forces[Simulation_B.i] = (1.0 -
          Simulation_B.ImpAsg_InsertedFor_t1_at_in) *
          Simulation_B.ImpAsg_InsertedFor_t2_at_in +
          Simulation_B.ImpAsg_InsertedFor_t1_at_in *
          Simulation_B.ImpAsg_InsertedFor_t3_at_in;
      }
    }

    /* End of MATLAB Function: '<S21>/PWM to Thrust' */
  }

  for (Simulation_B.i = 0; Simulation_B.i <= 4; Simulation_B.i += 2) {
    /* Integrator: '<S1>/Integrator' */
    tmp = _mm_loadu_pd(&Simulation_X.Integrator_CSTATE[Simulation_B.i]);
    _mm_storeu_pd(&Simulation_B.nu[Simulation_B.i], tmp);

    /* Gain: '<S1>/0 or 1   ' incorporates:
     *  Integrator: '<S1>/Integrator'
     */
    _mm_storeu_pd(&Simulation_B.Sum3_i[Simulation_B.i], _mm_mul_pd(_mm_set1_pd
      (0.0), tmp));
  }

  /* Product: '<S11>/Product' */
  Simulation_B.scale = Simulation_B.Sum3_i[1];
  Simulation_B.yaw = Simulation_B.Sum3_i[0];
  Simulation_B.rtb_Sum3_i_l = Simulation_B.Sum3_i[2];

  /* Product: '<S11>/Product1' */
  Simulation_B.rtb_Sum3_i_d = Simulation_B.Sum3_i[4];
  Simulation_B.rtb_Sum3_i_dy = Simulation_B.Sum3_i[3];
  Simulation_B.rtb_Sum3_i_lx = Simulation_B.Sum3_i[5];
  for (Simulation_B.high_i = 0; Simulation_B.high_i <= 0; Simulation_B.high_i +=
       2) {
    /* Product: '<S11>/Product' incorporates:
     *  Product: '<S11>/Product2'
     */
    tmp = _mm_set1_pd(Simulation_B.scale);
    tmp_0 = _mm_set1_pd(Simulation_B.yaw);
    tmp_1 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_l);

    /* Product: '<S11>/Product1' incorporates:
     *  Product: '<S11>/Product3'
     */
    tmp_2 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_d);
    tmp_3 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_dy);
    tmp_4 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_lx);

    /* Gain: '<S11>/Gain1' incorporates:
     *  Gain: '<S11>/Gain2'
     *  Product: '<S11>/Product'
     *  Product: '<S11>/Product1'
     *  Selector: '<S11>/M11'
     *  Selector: '<S11>/M12'
     *  Sum: '<S11>/Sum1'
     */
    tmp_5 = _mm_set1_pd(-1.0);
    _mm_storeu_pd(&Simulation_B.Sum_j[Simulation_B.high_i], _mm_mul_pd
                  (_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M11_n[Simulation_B.high_i + 3]), tmp), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M11_n[Simulation_B.high_i]), tmp_0)),
      _mm_mul_pd(_mm_loadu_pd(&Simulation_ConstB.M11_n[Simulation_B.high_i + 6]),
                 tmp_1)), _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M12_g[Simulation_B.high_i + 3]), tmp_2), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M12_g[Simulation_B.high_i]), tmp_3)),
      _mm_mul_pd(_mm_loadu_pd(&Simulation_ConstB.M12_g[Simulation_B.high_i + 6]),
                 tmp_4))), tmp_5));

    /* Gain: '<S11>/Gain2' incorporates:
     *  Gain: '<S11>/Gain1'
     *  Math: '<S11>/transpose1'
     *  Product: '<S11>/Product'
     *  Product: '<S11>/Product2'
     *  Product: '<S11>/Product3'
     *  Selector: '<S11>/M22'
     *  Sum: '<S11>/Sum1'
     *  Sum: '<S11>/Sum2'
     */
    _mm_storeu_pd(&Simulation_B.Sum3[Simulation_B.high_i], _mm_mul_pd(_mm_add_pd
      (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M21M12_c[Simulation_B.high_i + 3]), tmp), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M21M12_c[Simulation_B.high_i]), tmp_0)),
                  _mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M21M12_c[Simulation_B.high_i + 6]), tmp_1)),
       _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M22_d[Simulation_B.high_i + 3]), tmp_2), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M22_d[Simulation_B.high_i]), tmp_3)),
                  _mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M22_d[Simulation_B.high_i + 6]), tmp_4))), tmp_5));
  }

  for (Simulation_B.high_i = 2; Simulation_B.high_i < 3; Simulation_B.high_i++)
  {
    /* Gain: '<S11>/Gain1' incorporates:
     *  Product: '<S11>/Product'
     *  Product: '<S11>/Product1'
     *  Selector: '<S11>/M11'
     *  Selector: '<S11>/M12'
     *  Sum: '<S11>/Sum1'
     */
    Simulation_B.Sum_j[Simulation_B.high_i] =
      -(((Simulation_ConstB.M11_n[Simulation_B.high_i + 3] * Simulation_B.scale
          + Simulation_ConstB.M11_n[Simulation_B.high_i] * Simulation_B.yaw) +
         Simulation_ConstB.M11_n[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_l) +
        ((Simulation_ConstB.M12_g[Simulation_B.high_i + 3] *
          Simulation_B.rtb_Sum3_i_d +
          Simulation_ConstB.M12_g[Simulation_B.high_i] *
          Simulation_B.rtb_Sum3_i_dy) +
         Simulation_ConstB.M12_g[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_lx));

    /* Gain: '<S11>/Gain2' incorporates:
     *  Math: '<S11>/transpose1'
     *  Product: '<S11>/Product2'
     *  Product: '<S11>/Product3'
     *  Selector: '<S11>/M22'
     *  Sum: '<S11>/Sum2'
     */
    Simulation_B.Sum3[Simulation_B.high_i] =
      -(((Simulation_ConstB.M21M12_c[Simulation_B.high_i + 3] *
          Simulation_B.scale + Simulation_ConstB.M21M12_c[Simulation_B.high_i] *
          Simulation_B.yaw) + Simulation_ConstB.M21M12_c[Simulation_B.high_i + 6]
         * Simulation_B.rtb_Sum3_i_l) +
        ((Simulation_ConstB.M22_d[Simulation_B.high_i + 3] *
          Simulation_B.rtb_Sum3_i_d +
          Simulation_ConstB.M22_d[Simulation_B.high_i] *
          Simulation_B.rtb_Sum3_i_dy) +
         Simulation_ConstB.M22_d[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_lx));
  }

  /* Product: '<S17>/Product' incorporates:
   *  Product: '<S17>/Product1'
   */
  tmp = _mm_sub_pd(_mm_mul_pd(_mm_loadu_pd(&Simulation_B.Sum_j[1]), _mm_set_pd
    (Simulation_B.Sum3_i[3], Simulation_B.Sum3_i[5])), _mm_mul_pd(_mm_set_pd
    (Simulation_B.Sum_j[0], Simulation_B.Sum_j[2]), _mm_loadu_pd
    (&Simulation_B.Sum3_i[4])));
  _mm_storeu_pd(&Simulation_B.dv1[0], tmp);

  /* Sum: '<S17>/Sum' incorporates:
   *  Product: '<S17>/Product'
   *  Product: '<S17>/Product1'
   */
  Simulation_B.ImpAsg_InsertedFor_t1_at_in = Simulation_B.dv1[0];
  Simulation_B.ImpAsg_InsertedFor_t2_at_in = Simulation_B.dv1[1];
  Simulation_B.ImpAsg_InsertedFor_t3_at_in = Simulation_B.Sum_j[0] *
    Simulation_B.Sum3_i[4] - Simulation_B.Sum_j[1] * Simulation_B.Sum3_i[3];

  /* Product: '<S18>/Product' */
  Simulation_B.ImpAsg_InsertedFor_t4_at_in = Simulation_B.Sum_j[0];
  Simulation_B.ImpAsg_InsertedFor_t5_at_in = Simulation_B.Sum3_i[2];

  /* Product: '<S18>/Product1' */
  Simulation_B.ImpAsg_InsertedFor_t6_at_in = Simulation_B.Sum_j[2];
  Simulation_B.ImpAsg_InsertedFor_t7_at_in = Simulation_B.Sum3_i[0];

  /* Product: '<S19>/Product' */
  Simulation_B.ImpAsg_InsertedFor_t8_at_in = Simulation_B.Sum3[0];
  Simulation_B.rtb_Sum3_i_idx_0 = Simulation_B.Sum3_i[5];

  /* Product: '<S19>/Product1' */
  Simulation_B.rtb_Sum3_b = Simulation_B.Sum3[2];

  /* Product: '<S18>/Product' */
  Simulation_B.rtb_Sum_j_idx_0 = Simulation_B.Sum_j[1];
  Simulation_B.rtb_Sum3_i_idx_1_g = Simulation_B.Sum3_i[0];

  /* Product: '<S18>/Product1' */
  Simulation_B.rtb_Sum_j_idx_1_g = Simulation_B.Sum_j[0];
  Simulation_B.rtb_Sum3_i_idx_0_c = Simulation_B.Sum3_i[1];

  /* Product: '<S19>/Product' */
  Simulation_B.rtb_Sum3_idx_0 = Simulation_B.Sum3[1];
  Simulation_B.rtb_Sum3_i_idx_1_n = Simulation_B.Sum3_i[3];

  /* Product: '<S19>/Product1' */
  Simulation_B.rtb_Sum3_idx_1_p = Simulation_B.Sum3[0];
  Simulation_B.rtb_Sum3_i_idx_0_f = Simulation_B.Sum3_i[4];

  /* Product: '<S18>/Product' */
  Simulation_B.rtb_Sum_j_idx_1 = Simulation_B.Sum_j[2];
  Simulation_B.rtb_Sum3_i_idx_2_j = Simulation_B.Sum3_i[1];

  /* Product: '<S18>/Product1' */
  Simulation_B.rtb_Sum_j_idx_2 = Simulation_B.Sum_j[1];
  Simulation_B.rtb_Sum3_i_idx_1_m = Simulation_B.Sum3_i[2];

  /* Product: '<S19>/Product' */
  Simulation_B.rtb_Sum3_idx_1 = Simulation_B.Sum3[2];
  Simulation_B.rtb_Sum3_i_idx_2_d = Simulation_B.Sum3_i[4];

  /* Product: '<S19>/Product1' */
  Simulation_B.rtb_Sum3_idx_2 = Simulation_B.Sum3[1];
  Simulation_B.rtb_Sum3_i_idx_1_l = Simulation_B.Sum3_i[5];
  Simulation_B.rtb_Sum3_i_idx_2 = Simulation_B.Sum3_i[3];

  /* Gain: '<S1>/0 or 1' */
  for (Simulation_B.i = 0; Simulation_B.i <= 4; Simulation_B.i += 2) {
    tmp = _mm_loadu_pd(&Simulation_B.nu[Simulation_B.i]);
    _mm_storeu_pd(&Simulation_B.Sum3_i[Simulation_B.i], _mm_mul_pd(_mm_set1_pd
      (0.0), tmp));
  }

  /* End of Gain: '<S1>/0 or 1' */

  /* Product: '<S10>/Product' */
  Simulation_B.scale = Simulation_B.Sum3_i[1];
  Simulation_B.yaw = Simulation_B.Sum3_i[0];
  Simulation_B.rtb_Sum3_i_l = Simulation_B.Sum3_i[2];

  /* Product: '<S10>/Product1' */
  Simulation_B.rtb_Sum3_i_d = Simulation_B.Sum3_i[4];
  Simulation_B.rtb_Sum3_i_dy = Simulation_B.Sum3_i[3];
  Simulation_B.rtb_Sum3_i_lx = Simulation_B.Sum3_i[5];
  for (Simulation_B.high_i = 0; Simulation_B.high_i <= 0; Simulation_B.high_i +=
       2) {
    /* Product: '<S10>/Product' incorporates:
     *  Product: '<S10>/Product2'
     */
    tmp = _mm_set1_pd(Simulation_B.scale);
    tmp_0 = _mm_set1_pd(Simulation_B.yaw);
    tmp_1 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_l);

    /* Product: '<S10>/Product1' incorporates:
     *  Product: '<S10>/Product3'
     */
    tmp_2 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_d);
    tmp_3 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_dy);
    tmp_4 = _mm_set1_pd(Simulation_B.rtb_Sum3_i_lx);

    /* Gain: '<S10>/Gain1' incorporates:
     *  Gain: '<S10>/Gain2'
     *  Product: '<S10>/Product'
     *  Product: '<S10>/Product1'
     *  Selector: '<S10>/M11'
     *  Selector: '<S10>/M12'
     *  Sum: '<S10>/Sum1'
     */
    tmp_5 = _mm_set1_pd(-1.0);
    _mm_storeu_pd(&Simulation_B.Sum_j[Simulation_B.high_i], _mm_mul_pd
                  (_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M11[Simulation_B.high_i + 3]), tmp), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M11[Simulation_B.high_i]), tmp_0)),
      _mm_mul_pd(_mm_loadu_pd(&Simulation_ConstB.M11[Simulation_B.high_i + 6]),
                 tmp_1)), _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M12[Simulation_B.high_i + 3]), tmp_2), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M12[Simulation_B.high_i]), tmp_3)),
      _mm_mul_pd(_mm_loadu_pd(&Simulation_ConstB.M12[Simulation_B.high_i + 6]),
                 tmp_4))), tmp_5));

    /* Gain: '<S10>/Gain2' incorporates:
     *  Gain: '<S10>/Gain1'
     *  Math: '<S10>/transpose1'
     *  Product: '<S10>/Product'
     *  Product: '<S10>/Product2'
     *  Product: '<S10>/Product3'
     *  Selector: '<S10>/M22'
     *  Sum: '<S10>/Sum1'
     *  Sum: '<S10>/Sum2'
     */
    _mm_storeu_pd(&Simulation_B.Sum3[Simulation_B.high_i], _mm_mul_pd(_mm_add_pd
      (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M21M12[Simulation_B.high_i + 3]), tmp), _mm_mul_pd
      (_mm_loadu_pd(&Simulation_ConstB.M21M12[Simulation_B.high_i]), tmp_0)),
                  _mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M21M12[Simulation_B.high_i + 6]), tmp_1)), _mm_add_pd
       (_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M22[Simulation_B.high_i + 3]), tmp_2), _mm_mul_pd
                   (_mm_loadu_pd(&Simulation_ConstB.M22[Simulation_B.high_i]),
                    tmp_3)), _mm_mul_pd(_mm_loadu_pd
      (&Simulation_ConstB.M22[Simulation_B.high_i + 6]), tmp_4))), tmp_5));
  }

  for (Simulation_B.high_i = 2; Simulation_B.high_i < 3; Simulation_B.high_i++)
  {
    /* Gain: '<S10>/Gain1' incorporates:
     *  Product: '<S10>/Product'
     *  Product: '<S10>/Product1'
     *  Selector: '<S10>/M11'
     *  Selector: '<S10>/M12'
     *  Sum: '<S10>/Sum1'
     */
    Simulation_B.Sum_j[Simulation_B.high_i] =
      -(((Simulation_ConstB.M11[Simulation_B.high_i + 3] * Simulation_B.scale +
          Simulation_ConstB.M11[Simulation_B.high_i] * Simulation_B.yaw) +
         Simulation_ConstB.M11[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_l) +
        ((Simulation_ConstB.M12[Simulation_B.high_i + 3] *
          Simulation_B.rtb_Sum3_i_d + Simulation_ConstB.M12[Simulation_B.high_i]
          * Simulation_B.rtb_Sum3_i_dy) +
         Simulation_ConstB.M12[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_lx));

    /* Gain: '<S10>/Gain2' incorporates:
     *  Math: '<S10>/transpose1'
     *  Product: '<S10>/Product'
     *  Product: '<S10>/Product1'
     *  Product: '<S10>/Product2'
     *  Product: '<S10>/Product3'
     *  Selector: '<S10>/M22'
     *  Sum: '<S10>/Sum2'
     */
    Simulation_B.Sum3[Simulation_B.high_i] =
      -(((Simulation_ConstB.M21M12[Simulation_B.high_i + 3] * Simulation_B.scale
          + Simulation_ConstB.M21M12[Simulation_B.high_i] * Simulation_B.yaw) +
         Simulation_ConstB.M21M12[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_l) +
        ((Simulation_ConstB.M22[Simulation_B.high_i + 3] *
          Simulation_B.rtb_Sum3_i_d + Simulation_ConstB.M22[Simulation_B.high_i]
          * Simulation_B.rtb_Sum3_i_dy) +
         Simulation_ConstB.M22[Simulation_B.high_i + 6] *
         Simulation_B.rtb_Sum3_i_lx));
  }

  if (Simulation_B.b) {
    /* MATLAB Function: '<S21>/Thrust to Net Force and Moment' incorporates:
     *  Constant: '<S21>/Constant4'
     *  Constant: '<S21>/Constant5'
     *  Constant: '<S21>/Constant6'
     */
    Simulation_DW.sfEvent_a = Simulation_CALL_EVENT;
    Simulation_B.yaw = 0.0;
    Simulation_B.rtb_Sum3_i_l = 0.0;
    Simulation_B.rtb_Sum3_i_d = 0.0;
    Simulation_B.rtb_Sum3_i_dy = 0.0;
    Simulation_B.rtb_Sum3_i_lx = 0.0;
    Simulation_B.net_moment_g = 0.0;
    for (Simulation_B.i = 0; Simulation_B.i < 8; Simulation_B.i++) {
      Simulation_B.scale = 3.3121686421112381E-170;
      Simulation_B.absxk = fabs
        (Simulation_ConstP.Constant5_Value_h[Simulation_B.i]);
      if (Simulation_B.absxk > 3.3121686421112381E-170) {
        Simulation_B.y = 1.0;
        Simulation_B.scale = Simulation_B.absxk;
      } else {
        Simulation_B.t = Simulation_B.absxk / 3.3121686421112381E-170;
        Simulation_B.y = Simulation_B.t * Simulation_B.t;
      }

      Simulation_B.force_vector_idx_1 =
        Simulation_ConstP.Constant5_Value_h[Simulation_B.i + 8];
      Simulation_B.absxk = fabs(Simulation_B.force_vector_idx_1);
      if (Simulation_B.absxk > Simulation_B.scale) {
        Simulation_B.t = Simulation_B.scale / Simulation_B.absxk;
        Simulation_B.y = Simulation_B.y * Simulation_B.t * Simulation_B.t + 1.0;
        Simulation_B.scale = Simulation_B.absxk;
      } else {
        Simulation_B.t = Simulation_B.absxk / Simulation_B.scale;
        Simulation_B.y += Simulation_B.t * Simulation_B.t;
      }

      Simulation_B.absxk_tmp =
        Simulation_ConstP.Constant5_Value_h[Simulation_B.i + 16];
      Simulation_B.absxk = fabs(Simulation_B.absxk_tmp);
      if (Simulation_B.absxk > Simulation_B.scale) {
        Simulation_B.t = Simulation_B.scale / Simulation_B.absxk;
        Simulation_B.y = Simulation_B.y * Simulation_B.t * Simulation_B.t + 1.0;
        Simulation_B.scale = Simulation_B.absxk;
      } else {
        Simulation_B.t = Simulation_B.absxk / Simulation_B.scale;
        Simulation_B.y += Simulation_B.t * Simulation_B.t;
      }

      Simulation_B.y = Simulation_B.scale * sqrt(Simulation_B.y);
      Simulation_B.scale = Simulation_B.thruster_forces[Simulation_B.i];
      Simulation_B.absxk = Simulation_ConstP.Constant5_Value_h[Simulation_B.i] /
        Simulation_B.y * Simulation_B.scale;
      Simulation_B.t = Simulation_B.absxk;
      Simulation_B.yaw += Simulation_B.absxk;
      Simulation_B.a_idx_0 = Simulation_ConstP.Constant4_Value_l[Simulation_B.i]
        - 0.0425;
      Simulation_B.absxk = Simulation_B.force_vector_idx_1 / Simulation_B.y *
        Simulation_B.scale;
      Simulation_B.force_vector_idx_1 = Simulation_B.absxk;
      Simulation_B.rtb_Sum3_i_l += Simulation_B.absxk;
      Simulation_B.a_idx_1 = Simulation_ConstP.Constant4_Value_l[Simulation_B.i
        + 8];
      Simulation_B.absxk = Simulation_B.absxk_tmp / Simulation_B.y *
        Simulation_B.scale;
      Simulation_B.rtb_Sum3_i_d += Simulation_B.absxk;
      Simulation_B.scale = Simulation_ConstP.Constant4_Value_l[Simulation_B.i +
        16];
      Simulation_B.net_moment_g += Simulation_B.a_idx_0 *
        Simulation_B.force_vector_idx_1 - Simulation_B.t * Simulation_B.a_idx_1;
      Simulation_B.rtb_Sum3_i_dy += Simulation_B.a_idx_1 * Simulation_B.absxk -
        Simulation_B.force_vector_idx_1 * Simulation_B.scale;
      Simulation_B.rtb_Sum3_i_lx += Simulation_B.t * Simulation_B.scale -
        Simulation_B.a_idx_0 * Simulation_B.absxk;
    }

    Simulation_B.net_moment[2] = Simulation_B.net_moment_g;
    Simulation_B.net_moment[1] = Simulation_B.rtb_Sum3_i_lx;
    Simulation_B.net_moment[0] = Simulation_B.rtb_Sum3_i_dy;
    Simulation_B.net_force[2] = Simulation_B.rtb_Sum3_i_d;
    Simulation_B.net_force[1] = Simulation_B.rtb_Sum3_i_l;
    Simulation_B.net_force[0] = Simulation_B.yaw;

    /* End of MATLAB Function: '<S21>/Thrust to Net Force and Moment' */
  }

  /* MATLAB Function: '<S21>/Body to Inertial' */
  Simulation_DW.sfEvent_o = Simulation_CALL_EVENT;
  tmp = _mm_mul_pd(_mm_set1_pd(0.017453292519943295), _mm_loadu_pd
                   (&Simulation_B.theta[0]));
  _mm_storeu_pd(&Simulation_B.dv1[0], tmp);

  /* MATLAB Function: '<S21>/Body to Inertial' */
  Simulation_B.yaw = 0.017453292519943295 * Simulation_B.theta[2];
  Simulation_B.scale = sin(Simulation_B.yaw);
  Simulation_B.rtb_Sum3_i_d = cos(Simulation_B.yaw);
  Simulation_B.rtb_Sum3_i_dy = sin(Simulation_B.dv1[1]);
  Simulation_B.rtb_Sum3_i_lx = cos(Simulation_B.dv1[1]);
  Simulation_B.net_moment_g = sin(Simulation_B.dv1[0]);
  Simulation_B.yaw = cos(Simulation_B.dv1[0]);
  Simulation_B.R_tmp[0] = Simulation_B.rtb_Sum3_i_d;
  Simulation_B.R_tmp[3] = -Simulation_B.scale;
  Simulation_B.R_tmp[6] = 0.0;
  Simulation_B.R_tmp[1] = Simulation_B.scale;
  Simulation_B.R_tmp[4] = Simulation_B.rtb_Sum3_i_d;
  Simulation_B.R_tmp[7] = 0.0;
  Simulation_B.R_tmp_k[0] = Simulation_B.rtb_Sum3_i_lx;
  Simulation_B.R_tmp_k[3] = 0.0;
  Simulation_B.R_tmp_k[6] = Simulation_B.rtb_Sum3_i_dy;
  Simulation_B.R_tmp[2] = 0.0;
  Simulation_B.R_tmp_k[1] = 0.0;
  Simulation_B.R_tmp[5] = 0.0;
  Simulation_B.R_tmp_k[4] = 1.0;
  Simulation_B.R_tmp[8] = 1.0;
  Simulation_B.R_tmp_k[7] = 0.0;
  Simulation_B.R_tmp_k[2] = -Simulation_B.rtb_Sum3_i_dy;
  Simulation_B.R_tmp_k[5] = 0.0;
  Simulation_B.R_tmp_k[8] = Simulation_B.rtb_Sum3_i_lx;
  for (Simulation_B.high_i = 0; Simulation_B.high_i < 3; Simulation_B.high_i++)
  {
    Simulation_B.scale = Simulation_B.R_tmp_k[3 * Simulation_B.high_i + 1];
    Simulation_B.rtb_Sum3_i_d = Simulation_B.R_tmp_k[3 * Simulation_B.high_i];
    Simulation_B.rtb_Sum3_i_dy = Simulation_B.R_tmp_k[3 * Simulation_B.high_i +
      2];
    for (Simulation_B.low_i = 0; Simulation_B.low_i <= 0; Simulation_B.low_i +=
         2) {
      tmp = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.low_i + 3]);
      tmp_0 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.low_i]);
      tmp_1 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.low_i + 6]);
      _mm_storeu_pd(&Simulation_B.R_tmp_c[Simulation_B.low_i + 3 *
                    Simulation_B.high_i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(Simulation_B.scale), tmp), _mm_mul_pd(_mm_set1_pd
        (Simulation_B.rtb_Sum3_i_d), tmp_0)), _mm_mul_pd(_mm_set1_pd
        (Simulation_B.rtb_Sum3_i_dy), tmp_1)));
    }

    for (Simulation_B.low_i = 2; Simulation_B.low_i < 3; Simulation_B.low_i++) {
      Simulation_B.R_tmp_c[Simulation_B.low_i + 3 * Simulation_B.high_i] =
        (Simulation_B.R_tmp[Simulation_B.low_i + 3] * Simulation_B.scale +
         Simulation_B.rtb_Sum3_i_d * Simulation_B.R_tmp[Simulation_B.low_i]) +
        Simulation_B.R_tmp[Simulation_B.low_i + 6] * Simulation_B.rtb_Sum3_i_dy;
    }

    Simulation_B.d[3 * Simulation_B.high_i] = d[Simulation_B.high_i];
  }

  Simulation_B.d[1] = 0.0;
  Simulation_B.d[4] = Simulation_B.yaw;
  Simulation_B.d[7] = -Simulation_B.net_moment_g;
  Simulation_B.d[2] = 0.0;
  Simulation_B.d[5] = Simulation_B.net_moment_g;
  Simulation_B.d[8] = Simulation_B.yaw;

  /* MATLAB Function: '<S20>/Body to Inertial' */
  Simulation_DW.sfEvent_as = Simulation_CALL_EVENT;

  /* Signum: '<S23>/Sign' */
  if (rtIsNaN(Simulation_B.nu[0])) {
    Simulation_B.scale = (rtNaN);
  } else if (Simulation_B.nu[0] < 0.0) {
    Simulation_B.scale = -1.0;
  } else {
    Simulation_B.scale = (Simulation_B.nu[0] > 0.0);
  }

  /* SignalConversion generated from: '<S22>/ SFunction ' incorporates:
   *  Constant: '<S20>/Constant'
   *  Constant: '<S20>/Constant1'
   *  Constant: '<S23>/Constant'
   *  Constant: '<S23>/Constant1'
   *  Gain: '<S23>/Gain'
   *  MATLAB Function: '<S20>/Body to Inertial'
   *  Math: '<S23>/Square'
   *  Product: '<S23>/Product'
   *  Signum: '<S23>/Sign'
   */
  Simulation_B.dv[0] = Simulation_B.nu[0] * Simulation_B.nu[0] * 1500.0 * 0.1967
    * 0.5 * -Simulation_B.scale;

  /* Signum: '<S24>/Sign' */
  if (rtIsNaN(Simulation_B.nu[1])) {
    Simulation_B.scale = (rtNaN);
  } else if (Simulation_B.nu[1] < 0.0) {
    Simulation_B.scale = -1.0;
  } else {
    Simulation_B.scale = (Simulation_B.nu[1] > 0.0);
  }

  /* SignalConversion generated from: '<S22>/ SFunction ' incorporates:
   *  Constant: '<S20>/Constant3'
   *  Constant: '<S20>/Constant4'
   *  Constant: '<S24>/Constant'
   *  Constant: '<S24>/Constant1'
   *  Gain: '<S24>/Gain'
   *  MATLAB Function: '<S20>/Body to Inertial'
   *  Math: '<S24>/Square'
   *  Product: '<S24>/Product'
   *  Signum: '<S24>/Sign'
   */
  Simulation_B.dv[1] = Simulation_B.nu[1] * Simulation_B.nu[1] * 1500.0 * 0.17 *
    0.5 * -Simulation_B.scale;

  /* Signum: '<S25>/Sign' */
  if (rtIsNaN(Simulation_B.nu[2])) {
    Simulation_B.scale = (rtNaN);
  } else if (Simulation_B.nu[2] < 0.0) {
    Simulation_B.scale = -1.0;
  } else {
    Simulation_B.scale = (Simulation_B.nu[2] > 0.0);
  }

  /* SignalConversion generated from: '<S22>/ SFunction ' incorporates:
   *  Constant: '<S20>/Constant5'
   *  Constant: '<S20>/Constant6'
   *  Constant: '<S25>/Constant'
   *  Constant: '<S25>/Constant1'
   *  Gain: '<S25>/Gain'
   *  MATLAB Function: '<S20>/Body to Inertial'
   *  Math: '<S25>/Square'
   *  Product: '<S25>/Product'
   *  Signum: '<S25>/Sign'
   */
  Simulation_B.dv[2] = Simulation_B.nu[2] * Simulation_B.nu[2] * 2000.0 * 0.3 *
    0.5 * -Simulation_B.scale;
  for (Simulation_B.i = 0; Simulation_B.i < 3; Simulation_B.i++) {
    /* MATLAB Function: '<S21>/Body to Inertial' */
    Simulation_B.scale = Simulation_B.R_tmp_c[Simulation_B.i + 3];
    Simulation_B.rtb_Sum3_i_d = Simulation_B.R_tmp_c[Simulation_B.i];
    Simulation_B.rtb_Sum3_i_dy = Simulation_B.R_tmp_c[Simulation_B.i + 6];
    Simulation_B.rtb_Sum3_i_lx = 0.0;

    /* MATLAB Function: '<S20>/Body to Inertial' */
    Simulation_B.net_moment_g = 0.0;

    /* MATLAB Function: '<S21>/Body to Inertial' */
    Simulation_B.yaw = 0.0;

    /* MATLAB Function: '<S20>/Body to Inertial' */
    Simulation_B.rtb_Sum3_i_l = 0.0;
    for (Simulation_B.high_i = 0; Simulation_B.high_i < 3; Simulation_B.high_i++)
    {
      /* MATLAB Function: '<S21>/Body to Inertial' */
      tmp = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(Simulation_B.d[3 *
        Simulation_B.high_i + 1]), _mm_set1_pd(Simulation_B.scale)), _mm_mul_pd
        (_mm_set1_pd(Simulation_B.d[3 * Simulation_B.high_i]), _mm_set1_pd
         (Simulation_B.rtb_Sum3_i_d))), _mm_mul_pd(_mm_set1_pd(Simulation_B.d[3 *
        Simulation_B.high_i + 2]), _mm_set1_pd(Simulation_B.rtb_Sum3_i_dy)));

      /* MATLAB Function: '<S20>/Body to Inertial' incorporates:
       *  MATLAB Function: '<S21>/Body to Inertial'
       */
      _mm_storeu_pd(&Simulation_B.dv1[0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set_pd
        (Simulation_B.dv[Simulation_B.high_i],
         Simulation_B.net_force[Simulation_B.high_i])), _mm_set_pd
        (Simulation_B.net_moment_g, Simulation_B.rtb_Sum3_i_lx)));

      /* MATLAB Function: '<S21>/Body to Inertial' */
      Simulation_B.rtb_Sum3_i_lx = Simulation_B.dv1[0];

      /* MATLAB Function: '<S20>/Body to Inertial' */
      Simulation_B.net_moment_g = Simulation_B.dv1[1];

      /* MATLAB Function: '<S21>/Body to Inertial' incorporates:
       *  MATLAB Function: '<S20>/Body to Inertial'
       */
      _mm_storeu_pd(&Simulation_B.dv1[0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set_pd
        (0.0, Simulation_B.net_moment[Simulation_B.high_i])), _mm_set_pd
        (Simulation_B.rtb_Sum3_i_l, Simulation_B.yaw)));
      Simulation_B.yaw = Simulation_B.dv1[0];

      /* MATLAB Function: '<S20>/Body to Inertial' */
      Simulation_B.rtb_Sum3_i_l = Simulation_B.dv1[1];
    }

    /* MATLAB Function: '<S21>/Body to Inertial' */
    Simulation_B.global_force[Simulation_B.i] = Simulation_B.rtb_Sum3_i_lx;

    /* Sum: '<S5>/Add' incorporates:
     *  MATLAB Function: '<S20>/Body to Inertial'
     *  MATLAB Function: '<S21>/Body to Inertial'
     */
    Simulation_B.scale = Simulation_B.rtb_Sum3_i_lx + Simulation_B.net_moment_g;
    Simulation_B.Add[Simulation_B.i] = Simulation_B.scale;

    /* Sum: '<S5>/Add1' incorporates:
     *  MATLAB Function: '<S20>/Body to Inertial'
     *  MATLAB Function: '<S21>/Body to Inertial'
     */
    Simulation_B.yaw += Simulation_B.rtb_Sum3_i_l;
    Simulation_B.Add1[Simulation_B.i] = Simulation_B.yaw;

    /* Reshape: '<S5>/Reshape' */
    Simulation_B.Reshape[Simulation_B.i] = Simulation_B.scale;
    Simulation_B.Reshape[Simulation_B.i + 3] = Simulation_B.yaw;
  }

  /* Product: '<S14>/Product' */
  Simulation_B.scale = Simulation_B.Sum3_i[5];

  /* Product: '<S14>/Product1' */
  Simulation_B.yaw = Simulation_B.Sum3_i[3];

  /* Product: '<S15>/Product' */
  Simulation_B.rtb_Sum3_i_l = Simulation_B.Sum3_i[2];

  /* Product: '<S15>/Product1' */
  Simulation_B.rtb_Sum3_i_d = Simulation_B.Sum3_i[0];

  /* Product: '<S16>/Product' */
  Simulation_B.rtb_Sum3_i_dy = Simulation_B.Sum3_i[5];

  /* Product: '<S16>/Product1' */
  Simulation_B.rtb_Sum3_i_lx = Simulation_B.Sum3_i[3];

  /* Product: '<S14>/Product' */
  Simulation_B.net_moment_g = Simulation_B.Sum3_i[3];

  /* Product: '<S14>/Product1' */
  Simulation_B.y = Simulation_B.Sum3_i[4];

  /* Product: '<S15>/Product' */
  Simulation_B.absxk_tmp = Simulation_B.Sum3_i[0];

  /* Product: '<S15>/Product1' */
  Simulation_B.force_vector_idx_1 = Simulation_B.Sum3_i[1];

  /* Product: '<S16>/Product' incorporates:
   *  Product: '<S14>/Product'
   */
  Simulation_B.absxk = Simulation_B.Sum3_i[3];

  /* Product: '<S16>/Product1' incorporates:
   *  Product: '<S14>/Product1'
   */
  Simulation_B.t = Simulation_B.Sum3_i[4];

  /* Product: '<S14>/Product' */
  Simulation_B.a_idx_0 = Simulation_B.Sum3_i[4];

  /* Product: '<S14>/Product1' */
  Simulation_B.a_idx_1 = Simulation_B.Sum3_i[5];

  /* Product: '<S15>/Product' */
  Simulation_B.rtb_Sum3_i_idx_2_p = Simulation_B.Sum3_i[1];

  /* Product: '<S15>/Product1' */
  Simulation_B.rtb_Sum3_i_idx_1 = Simulation_B.Sum3_i[2];

  /* Sum: '<S1>/Sum3' incorporates:
   *  Product: '<S14>/Product'
   *  Product: '<S14>/Product1'
   *  Product: '<S15>/Product'
   *  Product: '<S15>/Product1'
   *  Product: '<S16>/Product'
   *  Product: '<S16>/Product1'
   *  Product: '<S18>/Product'
   *  Product: '<S18>/Product1'
   *  Product: '<S19>/Product'
   *  Product: '<S19>/Product1'
   *  Sum: '<S10>/Sum3'
   *  Sum: '<S11>/Sum3'
   *  Sum: '<S14>/Sum'
   *  Sum: '<S15>/Sum'
   *  Sum: '<S16>/Sum'
   *  Sum: '<S18>/Sum'
   *  Sum: '<S19>/Sum'
   *  Sum: '<S1>/Sum'
   */
  Simulation_B.Sum3_i[0] = (Simulation_B.Reshape[0] - (Simulation_B.Sum_j[1] *
    Simulation_B.scale - Simulation_B.Sum_j[2] * Simulation_B.y)) -
    Simulation_B.ImpAsg_InsertedFor_t1_at_in;
  Simulation_B.Sum3_i[3] = (Simulation_B.Reshape[3] - ((Simulation_B.Sum_j[1] *
    Simulation_B.rtb_Sum3_i_l - Simulation_B.Sum_j[2] *
    Simulation_B.force_vector_idx_1) + (Simulation_B.Sum3[1] *
    Simulation_B.rtb_Sum3_i_dy - Simulation_B.Sum3[2] * Simulation_B.t))) -
    ((Simulation_B.rtb_Sum_j_idx_0 * Simulation_B.ImpAsg_InsertedFor_t5_at_in -
      Simulation_B.ImpAsg_InsertedFor_t6_at_in * Simulation_B.rtb_Sum3_i_idx_0_c)
     + (Simulation_B.rtb_Sum3_idx_0 * Simulation_B.rtb_Sum3_i_idx_0 -
        Simulation_B.rtb_Sum3_b * Simulation_B.rtb_Sum3_i_idx_0_f));
  Simulation_B.Sum3_i[1] = (Simulation_B.Reshape[1] - (Simulation_B.Sum_j[2] *
    Simulation_B.net_moment_g - Simulation_B.Sum_j[0] * Simulation_B.a_idx_1)) -
    Simulation_B.ImpAsg_InsertedFor_t2_at_in;
  Simulation_B.Sum3_i[4] = (Simulation_B.Reshape[4] - ((Simulation_B.Sum_j[2] *
    Simulation_B.absxk_tmp - Simulation_B.Sum_j[0] *
    Simulation_B.rtb_Sum3_i_idx_1) + (Simulation_B.Sum3[2] * Simulation_B.absxk
    - Simulation_B.Sum3[0] * Simulation_B.a_idx_1))) -
    ((Simulation_B.rtb_Sum_j_idx_1 * Simulation_B.rtb_Sum3_i_idx_1_g -
      Simulation_B.rtb_Sum_j_idx_1_g * Simulation_B.rtb_Sum3_i_idx_1_m) +
     (Simulation_B.rtb_Sum3_idx_1 * Simulation_B.rtb_Sum3_i_idx_1_n -
      Simulation_B.rtb_Sum3_idx_1_p * Simulation_B.rtb_Sum3_i_idx_1_l));
  Simulation_B.Sum3_i[2] = (Simulation_B.Reshape[2] - (Simulation_B.Sum_j[0] *
    Simulation_B.a_idx_0 - Simulation_B.Sum_j[1] * Simulation_B.yaw)) -
    Simulation_B.ImpAsg_InsertedFor_t3_at_in;
  Simulation_B.Sum3_i[5] = (Simulation_B.Reshape[5] - ((Simulation_B.Sum_j[0] *
    Simulation_B.rtb_Sum3_i_idx_2_p - Simulation_B.Sum_j[1] *
    Simulation_B.rtb_Sum3_i_d) + (Simulation_B.Sum3[0] * Simulation_B.a_idx_0 -
    Simulation_B.Sum3[1] * Simulation_B.rtb_Sum3_i_lx))) -
    ((Simulation_B.ImpAsg_InsertedFor_t4_at_in * Simulation_B.rtb_Sum3_i_idx_2_j
      - Simulation_B.rtb_Sum_j_idx_2 * Simulation_B.ImpAsg_InsertedFor_t7_at_in)
     + (Simulation_B.ImpAsg_InsertedFor_t8_at_in *
        Simulation_B.rtb_Sum3_i_idx_2_d - Simulation_B.rtb_Sum3_idx_2 *
        Simulation_B.rtb_Sum3_i_idx_2));

  /* Product: '<S1>/Product' incorporates:
   *  Sum: '<S1>/Sum2'
   */
  rt_mldivide_U1d6x6_U2d_4sw8yi_p(Simulation_ConstB.M_c, Simulation_B.Sum3_i,
    Simulation_B.ddtnu);
  if (Simulation_B.b) {
  }

  /* Integrator: '<S1>/Integrator1' */
  Simulation_B.p[0] = Simulation_X.Integrator1_CSTATE[0];
  Simulation_B.p[1] = Simulation_X.Integrator1_CSTATE[1];
  Simulation_B.p[2] = Simulation_X.Integrator1_CSTATE[2];

  /* MATLAB Function: '<Root>/MATLAB Function' */
  Simulation_DW.sfEvent = Simulation_CALL_EVENT;
  if (Simulation_B.b) {
  }

  /* BusAssignment: '<Root>/Bus Assignment' */
  memset(&Simulation_B.BusAssignment, 0, sizeof
         (SL_Bus_std_msgs_Float32MultiArray));

  /* Fcn: '<S13>/T21 ' incorporates:
   *  Fcn: '<S13>/T31 '
   */
  Simulation_B.ImpAsg_InsertedFor_t1_at_in = tan(Simulation_B.theta[1]);

  /* Product: '<S1>/Product2' incorporates:
   *  Constant: '<S13>/Constant'
   *  Constant: '<S13>/Constant '
   *  Fcn: '<S13>/T21 '
   *  Fcn: '<S13>/T23'
   *  Fcn: '<S13>/T31 '
   *  Fcn: '<S13>/T32'
   *  Fcn: '<S13>/T33'
   *  Reshape: '<S13>/Reshape 9x1->3x3'
   */
  Simulation_B.R_tmp[0] = 1.0;
  Simulation_B.R_tmp[3] = Simulation_B.rtb_sintheta_idx_0_tmp *
    Simulation_B.ImpAsg_InsertedFor_t1_at_in;
  Simulation_B.R_tmp[6] = Simulation_B.rtb_costheta_idx_0_tmp *
    Simulation_B.ImpAsg_InsertedFor_t1_at_in;
  Simulation_B.R_tmp[1] = 0.0;
  Simulation_B.R_tmp[4] = Simulation_B.rtb_costheta_idx_0_tmp;
  Simulation_B.R_tmp[7] = -Simulation_B.rtb_sintheta_idx_0_tmp;
  Simulation_B.R_tmp[2] = 0.0;
  Simulation_B.R_tmp[5] = Simulation_B.rtb_sintheta_idx_0_tmp /
    Simulation_B.rtb_costheta_idx_1_tmp;
  Simulation_B.R_tmp[8] = Simulation_B.rtb_costheta_idx_0_tmp /
    Simulation_B.rtb_costheta_idx_1_tmp;
  Simulation_B.ImpAsg_InsertedFor_t1_at_in = Simulation_B.nu[4];
  Simulation_B.ImpAsg_InsertedFor_t2_at_in = Simulation_B.nu[3];
  Simulation_B.ImpAsg_InsertedFor_t3_at_in = Simulation_B.nu[5];
  for (Simulation_B.high_i = 0; Simulation_B.high_i <= 0; Simulation_B.high_i +=
       2) {
    /* Product: '<S1>/Product2' */
    tmp = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i + 3]);
    tmp_0 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i]);
    tmp_1 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i + 6]);

    /* Product: '<S1>/Product2' */
    _mm_storeu_pd(&Simulation_B.ddttheta[Simulation_B.high_i], _mm_add_pd
                  (_mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd
      (Simulation_B.ImpAsg_InsertedFor_t1_at_in)), _mm_mul_pd(tmp_0, _mm_set1_pd
      (Simulation_B.ImpAsg_InsertedFor_t2_at_in))), _mm_mul_pd(tmp_1,
      _mm_set1_pd(Simulation_B.ImpAsg_InsertedFor_t3_at_in))));
  }

  for (Simulation_B.high_i = 2; Simulation_B.high_i < 3; Simulation_B.high_i++)
  {
    /* Product: '<S1>/Product2' */
    Simulation_B.ddttheta[Simulation_B.high_i] =
      (Simulation_B.R_tmp[Simulation_B.high_i + 3] *
       Simulation_B.ImpAsg_InsertedFor_t1_at_in +
       Simulation_B.R_tmp[Simulation_B.high_i] *
       Simulation_B.ImpAsg_InsertedFor_t2_at_in) +
      Simulation_B.R_tmp[Simulation_B.high_i + 6] *
      Simulation_B.ImpAsg_InsertedFor_t3_at_in;
  }

  /* Product: '<S1>/Product1' incorporates:
   *  Fcn: '<S12>/R11'
   *  Fcn: '<S12>/R12'
   *  Fcn: '<S12>/R13'
   *  Fcn: '<S12>/R21 '
   *  Trigonometry: '<S12>/cos(theta)'
   *  Trigonometry: '<S12>/sin(theta)'
   */
  Simulation_B.R_tmp[0] = Simulation_B.rtb_costheta_idx_1_tmp *
    Simulation_B.rtb_costheta_idx_2;
  Simulation_B.R_tmp[3] = Simulation_B.rtb_sintheta_idx_1 *
    Simulation_B.rtb_costheta_idx_2 * Simulation_B.rtb_sintheta_idx_0_tmp -
    Simulation_B.rtb_costheta_idx_0_tmp * Simulation_B.rtb_sintheta_idx_2;
  Simulation_B.R_tmp[6] = Simulation_B.rtb_costheta_idx_0_tmp *
    Simulation_B.rtb_sintheta_idx_1 * Simulation_B.rtb_costheta_idx_2 +
    Simulation_B.rtb_sintheta_idx_0_tmp * Simulation_B.rtb_sintheta_idx_2;
  Simulation_B.R_tmp[1] = Simulation_B.rtb_costheta_idx_1_tmp *
    Simulation_B.rtb_sintheta_idx_2;

  /* Fcn: '<S12>/R22' incorporates:
   *  Fcn: '<S12>/R23'
   */
  Simulation_B.rtb_sintheta_idx_2 *= Simulation_B.rtb_sintheta_idx_1;

  /* Product: '<S1>/Product1' incorporates:
   *  Fcn: '<S12>/R22'
   *  Fcn: '<S12>/R23'
   *  Fcn: '<S12>/R31 '
   *  Fcn: '<S12>/R32'
   *  Fcn: '<S12>/R33'
   *  Trigonometry: '<S12>/cos(theta)'
   *  Trigonometry: '<S12>/sin(theta)'
   */
  Simulation_B.R_tmp[4] = Simulation_B.rtb_sintheta_idx_2 *
    Simulation_B.rtb_sintheta_idx_0_tmp + Simulation_B.rtb_costheta_idx_0_tmp *
    Simulation_B.rtb_costheta_idx_2;
  Simulation_B.R_tmp[7] = Simulation_B.rtb_sintheta_idx_2 *
    Simulation_B.rtb_costheta_idx_0_tmp - Simulation_B.rtb_sintheta_idx_0_tmp *
    Simulation_B.rtb_costheta_idx_2;
  Simulation_B.R_tmp[2] = -Simulation_B.rtb_sintheta_idx_1;
  Simulation_B.R_tmp[5] = Simulation_B.rtb_sintheta_idx_0_tmp *
    Simulation_B.rtb_costheta_idx_1_tmp;
  Simulation_B.R_tmp[8] = Simulation_B.rtb_costheta_idx_0_tmp *
    Simulation_B.rtb_costheta_idx_1_tmp;
  Simulation_B.ImpAsg_InsertedFor_t1_at_in = Simulation_B.nu[1];
  Simulation_B.ImpAsg_InsertedFor_t2_at_in = Simulation_B.nu[0];
  Simulation_B.ImpAsg_InsertedFor_t3_at_in = Simulation_B.nu[2];
  for (Simulation_B.high_i = 0; Simulation_B.high_i <= 0; Simulation_B.high_i +=
       2) {
    /* Product: '<S1>/Product1' */
    tmp = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i + 3]);
    tmp_0 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i]);
    tmp_1 = _mm_loadu_pd(&Simulation_B.R_tmp[Simulation_B.high_i + 6]);

    /* Product: '<S1>/Product1' */
    _mm_storeu_pd(&Simulation_B.dpdt[Simulation_B.high_i], _mm_add_pd(_mm_add_pd
      (_mm_mul_pd(tmp, _mm_set1_pd(Simulation_B.ImpAsg_InsertedFor_t1_at_in)),
       _mm_mul_pd(tmp_0, _mm_set1_pd(Simulation_B.ImpAsg_InsertedFor_t2_at_in))),
      _mm_mul_pd(tmp_1, _mm_set1_pd(Simulation_B.ImpAsg_InsertedFor_t3_at_in))));
  }

  for (Simulation_B.high_i = 2; Simulation_B.high_i < 3; Simulation_B.high_i++)
  {
    /* Product: '<S1>/Product1' */
    Simulation_B.dpdt[Simulation_B.high_i] =
      (Simulation_B.R_tmp[Simulation_B.high_i + 3] *
       Simulation_B.ImpAsg_InsertedFor_t1_at_in +
       Simulation_B.R_tmp[Simulation_B.high_i] *
       Simulation_B.ImpAsg_InsertedFor_t2_at_in) +
      Simulation_B.R_tmp[Simulation_B.high_i + 6] *
      Simulation_B.ImpAsg_InsertedFor_t3_at_in;
  }

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  Simulation_B.BusAssignment.data[0] = static_cast<real32_T>(Simulation_B.p[2]);
  Simulation_B.BusAssignment.data[1] = static_cast<real32_T>(Simulation_B.theta
    [0]);
  Simulation_B.BusAssignment.data[2] = static_cast<real32_T>(Simulation_B.theta
    [1]);
  Simulation_B.BusAssignment.data[3] = static_cast<real32_T>(Simulation_B.theta
    [2]);

  /* MATLABSystem: '<S8>/SinkBlock' */
  Pub_Simulation_382.publish(&Simulation_B.BusAssignment);
  if (rtmIsMajorTimeStep(Simulation_M)) {
    /* External mode */
    rtExtModeUploadCheckTrigger(2);

    {                                  /* Sample time: [0.0s, 0.0s] */
      rtExtModeUpload(0, (real_T)Simulation_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(Simulation_M)) {/* Sample time: [0.025s, 0.0s] */
      rtExtModeUpload(1, (real_T)(((Simulation_M->Timing.clockTick1+
        Simulation_M->Timing.clockTickH1* 4294967296.0)) * 0.025));
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Simulation_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Simulation_M)!=-1) &&
          !((rtmGetTFinal(Simulation_M)-(((Simulation_M->Timing.clockTick1+
               Simulation_M->Timing.clockTickH1* 4294967296.0)) * 0.025)) >
            (((Simulation_M->Timing.clockTick1+Simulation_M->Timing.clockTickH1*
               4294967296.0)) * 0.025) * (DBL_EPSILON))) {
        rtmSetErrorStatus(Simulation_M, "Simulation finished");
      }

      if (rtmGetStopRequested(Simulation_M)) {
        rtmSetErrorStatus(Simulation_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&Simulation_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Simulation_M->Timing.clockTick0)) {
      ++Simulation_M->Timing.clockTickH0;
    }

    Simulation_M->Timing.t[0] = rtsiGetSolverStopTime(&Simulation_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.025s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.025, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      Simulation_M->Timing.clockTick1++;
      if (!Simulation_M->Timing.clockTick1) {
        Simulation_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Simulation_derivatives(void)
{
  XDot_Simulation_T *_rtXdot;
  int32_T i;
  _rtXdot = ((XDot_Simulation_T *) Simulation_M->derivs);

  /* Derivatives for Integrator: '<S1>/Integrator2' */
  _rtXdot->Integrator2_CSTATE[0] = Simulation_B.ddttheta[0];
  _rtXdot->Integrator2_CSTATE[1] = Simulation_B.ddttheta[1];
  _rtXdot->Integrator2_CSTATE[2] = Simulation_B.ddttheta[2];

  /* Derivatives for Integrator: '<S1>/Integrator' */
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = Simulation_B.ddtnu[i];
  }

  /* End of Derivatives for Integrator: '<S1>/Integrator' */

  /* Derivatives for Integrator: '<S1>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[0] = Simulation_B.dpdt[0];
  _rtXdot->Integrator1_CSTATE[1] = Simulation_B.dpdt[1];
  _rtXdot->Integrator1_CSTATE[2] = Simulation_B.dpdt[2];
}

/* Model initialize function */
void Simulation_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Simulation_M->solverInfo,
                          &Simulation_M->Timing.simTimeStep);
    rtsiSetTPtr(&Simulation_M->solverInfo, &rtmGetTPtr(Simulation_M));
    rtsiSetStepSizePtr(&Simulation_M->solverInfo,
                       &Simulation_M->Timing.stepSize0);
    rtsiSetdXPtr(&Simulation_M->solverInfo, &Simulation_M->derivs);
    rtsiSetContStatesPtr(&Simulation_M->solverInfo, (real_T **)
                         &Simulation_M->contStates);
    rtsiSetNumContStatesPtr(&Simulation_M->solverInfo,
      &Simulation_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Simulation_M->solverInfo,
      &Simulation_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Simulation_M->solverInfo,
      &Simulation_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Simulation_M->solverInfo,
      &Simulation_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&Simulation_M->solverInfo, (boolean_T**)
      &Simulation_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&Simulation_M->solverInfo, (&rtmGetErrorStatus
      (Simulation_M)));
    rtsiSetRTModelPtr(&Simulation_M->solverInfo, Simulation_M);
  }

  rtsiSetSimTimeStep(&Simulation_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&Simulation_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&Simulation_M->solverInfo, false);
  Simulation_M->intgData.y = Simulation_M->odeY;
  Simulation_M->intgData.f[0] = Simulation_M->odeF[0];
  Simulation_M->intgData.f[1] = Simulation_M->odeF[1];
  Simulation_M->intgData.f[2] = Simulation_M->odeF[2];
  Simulation_M->contStates = ((X_Simulation_T *) &Simulation_X);
  Simulation_M->contStateDisabled = ((XDis_Simulation_T *) &Simulation_XDis);
  Simulation_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&Simulation_M->solverInfo, static_cast<void *>
                    (&Simulation_M->intgData));
  rtsiSetSolverName(&Simulation_M->solverInfo,"ode3");
  rtmSetTPtr(Simulation_M, &Simulation_M->Timing.tArray[0]);
  rtmSetTFinal(Simulation_M, -1);
  Simulation_M->Timing.stepSize0 = 0.025;

  /* External mode info */
  Simulation_M->Sizes.checksums[0] = (1106676841U);
  Simulation_M->Sizes.checksums[1] = (1305328841U);
  Simulation_M->Sizes.checksums[2] = (1809502106U);
  Simulation_M->Sizes.checksums[3] = (2973264343U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[12];
    Simulation_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    systemRan[10] = (sysRanDType *)&Simulation_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[11] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Simulation_M->extModeInfo,
      &Simulation_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Simulation_M->extModeInfo, Simulation_M->Sizes.checksums);
    rteiSetTPtr(Simulation_M->extModeInfo, rtmGetTPtr(Simulation_M));
  }

  /* block I/O */
  (void) memset((static_cast<void *>(&Simulation_B)), 0,
                sizeof(B_Simulation_T));

  {
    Simulation_B.Gain1[0] = 0.0;
    Simulation_B.Gain1[1] = 0.0;
    Simulation_B.Gain1[2] = 0.0;
    Simulation_B.Gain2[0] = 0.0;
    Simulation_B.Gain2[1] = 0.0;
    Simulation_B.Gain2[2] = 0.0;
  }

  /* states (continuous) */
  {
    (void) memset(static_cast<void *>(&Simulation_X), 0,
                  sizeof(X_Simulation_T));
  }

  /* disabled states */
  {
    (void) memset(static_cast<void *>(&Simulation_XDis), 0,
                  sizeof(XDis_Simulation_T));
  }

  /* states (dwork) */
  (void) memset(static_cast<void *>(&Simulation_DW), 0,
                sizeof(DW_Simulation_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    Simulation_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 26;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;
  }

  /* Start for MATLABSystem: '<S9>/SourceBlock' */
  Simulation_DW.obj_g.QOSAvoidROSNamespaceConventions = false;
  Simulation_DW.obj_g.matlabCodegenIsDeleted = false;
  Simulation_DW.objisempty = true;
  Simulation_DW.obj_g.isSetupComplete = false;
  Simulation_DW.obj_g.isInitialized = 1;
  Simulation_Subscriber_setupImpl(&Simulation_DW.obj_g);
  Simulation_DW.obj_g.isSetupComplete = true;

  /* Start for MATLABSystem: '<S8>/SinkBlock' */
  Simulation_DW.obj.QOSAvoidROSNamespaceConventions = false;
  Simulation_DW.obj.matlabCodegenIsDeleted = false;
  Simulation_DW.objisempty_g = true;
  Simulation_DW.obj.isSetupComplete = false;
  Simulation_DW.obj.isInitialized = 1;
  Simulation_Publisher_setupImpl(&Simulation_DW.obj);
  Simulation_DW.obj.isSetupComplete = true;

  /* ConstCode for Gain: '<Root>/Gain1' */
  Simulation_B.Gain1[0] = 0.0;

  /* ConstCode for Gain: '<Root>/Gain2' */
  Simulation_B.Gain2[0] = 0.0;

  /* ConstCode for Gain: '<Root>/Gain1' */
  Simulation_B.Gain1[1] = 0.0;

  /* ConstCode for Gain: '<Root>/Gain2' */
  Simulation_B.Gain2[1] = 0.0;

  /* ConstCode for Gain: '<Root>/Gain1' */
  Simulation_B.Gain1[2] = 0.0;

  /* ConstCode for Gain: '<Root>/Gain2' */
  Simulation_B.Gain2[2] = 0.0;

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;

    /* InitializeConditions for Integrator: '<S1>/Integrator2' */
    Simulation_X.Integrator2_CSTATE[0] = 0.0;
    Simulation_X.Integrator2_CSTATE[1] = 0.0;
    Simulation_X.Integrator2_CSTATE[2] = 0.0;

    /* InitializeConditions for Integrator: '<S1>/Integrator' */
    for (int32_T i = 0; i < 6; i++) {
      Simulation_X.Integrator_CSTATE[i] = 0.0;
    }

    /* End of InitializeConditions for Integrator: '<S1>/Integrator' */

    /* InitializeConditions for Integrator: '<S1>/Integrator1' */
    Simulation_X.Integrator1_CSTATE[0] = 0.0;
    Simulation_X.Integrator1_CSTATE[1] = 0.0;
    Simulation_X.Integrator1_CSTATE[2] = 0.0;

    /* SystemInitialize for SignalConversion generated from: '<S33>/In1' */
    memset(&Simulation_B.In1, 0, sizeof(SL_Bus_std_msgs_String));

    /* SystemInitialize for Iterator SubSystem: '<Root>/Parse Thruster Data' */
    for (ForEach_itr = 0; ForEach_itr < 1; ForEach_itr++) {
      /* SystemInitialize for MATLAB Function: '<S7>/MATLAB Function' */
      Simulation_DW.CoreSubsys[ForEach_itr].doneDoubleBufferReInit = false;
      Simulation_DW.CoreSubsys[ForEach_itr].sfEvent = Simulation_CALL_EVENT;
    }

    /* End of SystemInitialize for SubSystem: '<Root>/Parse Thruster Data' */

    /* SystemInitialize for MATLAB Function: '<S21>/PWM to Thrust' */
    Simulation_DW.doneDoubleBufferReInit_k = false;
    Simulation_DW.sfEvent_m = Simulation_CALL_EVENT;

    /* SystemInitialize for MATLAB Function: '<S21>/Thrust to Net Force and Moment' */
    Simulation_DW.doneDoubleBufferReInit_h = false;
    Simulation_DW.sfEvent_a = Simulation_CALL_EVENT;

    /* SystemInitialize for MATLAB Function: '<S21>/Body to Inertial' */
    Simulation_DW.doneDoubleBufferReInit_i = false;
    Simulation_DW.sfEvent_o = Simulation_CALL_EVENT;

    /* SystemInitialize for MATLAB Function: '<S20>/Body to Inertial' */
    Simulation_DW.doneDoubleBufferReInit_l = false;
    Simulation_DW.sfEvent_as = Simulation_CALL_EVENT;

    /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function' */
    Simulation_DW.doneDoubleBufferReInit = false;
    Simulation_DW.sfEvent = Simulation_CALL_EVENT;
  }
}

/* Model terminate function */
void Simulation_terminate(void)
{
  /* Terminate for MATLABSystem: '<S9>/SourceBlock' */
  if (!Simulation_DW.obj_g.matlabCodegenIsDeleted) {
    Simulation_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((Simulation_DW.obj_g.isInitialized == 1) &&
        Simulation_DW.obj_g.isSetupComplete) {
      Sub_Simulation_36.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S9>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S8>/SinkBlock' */
  if (!Simulation_DW.obj.matlabCodegenIsDeleted) {
    Simulation_DW.obj.matlabCodegenIsDeleted = true;
    if ((Simulation_DW.obj.isInitialized == 1) &&
        Simulation_DW.obj.isSetupComplete) {
      Pub_Simulation_382.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S8>/SinkBlock' */
}
