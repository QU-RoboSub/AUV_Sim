/*
 * Simulation_dt.h
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

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_std_msgs_MultiArrayDimension),
  sizeof(SL_Bus_std_msgs_MultiArrayLayout),
  sizeof(SL_Bus_std_msgs_Float32MultiArray),
  sizeof(SL_Bus_std_msgs_String),
  sizeof(ros_slros2_internal_block_Pub_T),
  sizeof(ros_slros2_internal_block_Sub_T),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_std_msgs_MultiArrayDimension",
  "SL_Bus_std_msgs_MultiArrayLayout",
  "SL_Bus_std_msgs_Float32MultiArray",
  "SL_Bus_std_msgs_String",
  "ros_slros2_internal_block_Pub_T",
  "ros_slros2_internal_block_Sub_T",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&Simulation_B.In1), 19, 0, 1 },

  { (char_T *)(&Simulation_B.theta[0]), 0, 0, 59 }
  ,

  { (char_T *)(&Simulation_DW.obj), 20, 0, 1 },

  { (char_T *)(&Simulation_DW.obj_g), 21, 0, 1 },

  { (char_T *)(&Simulation_DW.Scope_PWORK.LoggedData[0]), 11, 0, 24 },

  { (char_T *)(&Simulation_DW.sfEvent), 6, 0, 5 },

  { (char_T *)(&Simulation_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 1 },

  { (char_T *)(&Simulation_DW.objisempty), 8, 0, 7 },

  { (char_T *)(&Simulation_DW.CoreSubsys[0].sfEvent), 6, 0, 1 },

  { (char_T *)(&Simulation_DW.CoreSubsys[0].doneDoubleBufferReInit), 8, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  10U,
  rtBTransitions
};

/* [EOF] Simulation_dt.h */
