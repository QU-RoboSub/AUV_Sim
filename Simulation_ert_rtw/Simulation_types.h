/*
 * Simulation_types.h
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

#ifndef Simulation_types_h_
#define Simulation_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayDimension_

struct SL_Bus_std_msgs_MultiArrayDimension
{
  uint8_T label[128];
  SL_Bus_ROSVariableLengthArrayInfo label_SL_Info;
  uint32_T size;
  uint32_T stride;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_MultiArrayLayout_

struct SL_Bus_std_msgs_MultiArrayLayout
{
  SL_Bus_std_msgs_MultiArrayDimension dim[16];
  SL_Bus_ROSVariableLengthArrayInfo dim_SL_Info;
  uint32_T data_offset;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float32MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float32MultiArray_

struct SL_Bus_std_msgs_Float32MultiArray
{
  SL_Bus_std_msgs_MultiArrayLayout layout;
  real32_T data[4];
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_String_

struct SL_Bus_std_msgs_String
{
  uint8_T data[128];
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
};

#endif

/* Custom Type definition for MATLABSystem: '<S9>/SourceBlock' */
#include "rmw/qos_profiles.h"
#ifndef struct_sJ4ih70VmKcvCeguWN0mNVF
#define struct_sJ4ih70VmKcvCeguWN0mNVF

struct sJ4ih70VmKcvCeguWN0mNVF
{
  real_T sec;
  real_T nsec;
};

#endif                                 /* struct_sJ4ih70VmKcvCeguWN0mNVF */

#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Pub_T */

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Sub_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_Simulation_T RT_MODEL_Simulation_T;

#endif                                 /* Simulation_types_h_ */
