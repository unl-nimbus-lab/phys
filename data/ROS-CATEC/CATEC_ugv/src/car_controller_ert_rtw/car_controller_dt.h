/*
 * car_controller_dt.h
 *
 * Code generation for model "car_controller".
 *
 * Model version              : 1.218
 * Simulink Coder version : 8.5 (R2013b) 08-Aug-2013
 * C source code generated on : Wed Mar 25 12:48:14 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
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
  2*sizeof(uint32_T)
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
  "timer_uint32_pair_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&car_controller_B.RateLimiter), 0, 0, 22 },

  { (char_T *)(&car_controller_B.Add3), 1, 0, 7 }
  ,

  { (char_T *)(&car_controller_DW.DiscreteTimeIntegrator_DSTATE), 0, 0, 4 },

  { (char_T *)(&car_controller_DW.DirectionOut_PWORK.LoggedData), 11, 0, 11 },

  { (char_T *)(&car_controller_DW.sfEvent), 6, 0, 1 },

  { (char_T *)(&car_controller_DW.DiscreteTimeIntegrator2_IC_LOAD), 3, 0, 3 },

  { (char_T *)(&car_controller_DW.isStable), 8, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  7U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&car_controller_P.Constant4_Value), 0, 0, 35 },

  { (char_T *)(&car_controller_P.Constant5_Value_g), 1, 0, 6 },

  { (char_T *)(&car_controller_P.PWSwitch_CurrentSetting), 3, 0, 2 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  3U,
  rtPTransitions
};

/* [EOF] car_controller_dt.h */
