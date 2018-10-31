/*
 * File: car_controller.h
 *
 * Code generated for Simulink model 'car_controller'.
 *
 * Model version                  : 1.218
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Wed Mar 25 12:48:14 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_car_controller_h_
#define RTW_HEADER_car_controller_h_
#ifndef car_controller_COMMON_INCLUDES_
# define car_controller_COMMON_INCLUDES_
#include <math.h>
#include <float.h>
#include <string.h>
#include "rtwtypes.h"
#include "multiword_types.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#endif                                 /* car_controller_COMMON_INCLUDES_ */

#include "car_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
# define rtmGetRTWExtModeInfo(rtm)     ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T Switch1;                      /* '<S1>/Switch1' */
  real_T Add1;                         /* '<Root>/Add1' */
  real_T DirSwitch;                    /* '<Root>/DirSwitch' */
  real_T Saturation2;                  /* '<Root>/Saturation2' */
  real_T ENPhi_0[3];                   /* '<Root>/ENPhi_0' */
  real_T phi_dot;                      /* '<S4>/Product1' */
  real_T Flight_plan[12];              /* '<Root>/Flight_plan' */
  real_T k;                            /* '<S2>/Chart' */
  real32_T Add3;                       /* '<Root>/Add3' */
  real32_T TrigonometricFunction1;     /* '<Root>/Trigonometric Function1' */
  real32_T Add5;                       /* '<Root>/Add5' */
  real32_T TmpSignalConversionAtSFunctionI[2];/* '<S2>/Chart' */
  real32_T q1ref;                      /* '<S2>/Chart' */
  real32_T q2ref;                      /* '<S2>/Chart' */
} B_car_controller_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S4>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S4>/Discrete-Time Integrator1' */
  real_T DiscreteTimeIntegrator2_DSTATE;/* '<S4>/Discrete-Time Integrator2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  struct {
    void *LoggedData;
  } DirectionOut_PWORK;                /* '<Root>/DirectionOut' */

  struct {
    void *LoggedData;
  } DirectionOut1_PWORK;               /* '<Root>/DirectionOut1' */

  struct {
    void *LoggedData;
  } Error_PWORK;                       /* '<Root>/Error ' */

  struct {
    void *LoggedData;
  } MyC_PWORK;                         /* '<Root>/MyC ' */

  struct {
    void *LoggedData;
  } Power2_PWORK;                      /* '<Root>/Power2 ' */

  struct {
    void *LoggedData;
  } Sat_PWORK;                         /* '<Root>/Sat ' */

  struct {
    void *LoggedData;
  } Yaw_PWORK;                         /* '<Root>/Yaw ' */

  struct {
    void *LoggedData;
  } YawRef_PWORK;                      /* '<Root>/YawRef' */

  struct {
    void *LoggedData;
  } YawRef1_PWORK;                     /* '<Root>/YawRef1' */

  struct {
    void *LoggedData;
  } l_PWORK;                           /* '<Root>/l ' */

  struct {
    void *LoggedData;
  } y_comp_PWORK;                      /* '<Root>/y_comp' */

  int32_T sfEvent;                     /* '<S2>/Chart' */
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;/* '<S4>/Discrete-Time Integrator2' */
  uint8_T is_active_c1_car_controller; /* '<S2>/Chart' */
  uint8_T is_c1_car_controller;        /* '<S2>/Chart' */
  boolean_T isStable;                  /* '<S2>/Chart' */
} DW_car_controller_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T PosY;                       /* '<Root>/PosY' */
  real32_T Yaw;                        /* '<Root>/Yaw' */
  real32_T PosX;                       /* '<Root>/PosX' */
  real32_T XRef;                       /* '<Root>/XRef' */
  real32_T YRef;                       /* '<Root>/YRef' */
  real32_T VelRef;                     /* '<Root>/VelRef' */
} ExtU_car_controller_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T Power;                        /* '<Root>/Power' */
  real_T Direction;                    /* '<Root>/Direction' */
  real32_T EN_road[2];                 /* '<Root>/EN_road' */
  real_T EN_vehicle[2];                /* '<Root>/EN_vehicle' */
  real_T phi_vehicle;                  /* '<Root>/phi_vehicle' */
} ExtY_car_controller_T;

/* Parameters (auto storage) */
struct P_car_controller_T_ {
  real_T Constant4_Value;              /* Expression: 2*pi
                                        * Referenced by: '<S1>/Constant4'
                                        */
  real_T Constant5_Value;              /* Expression: 2*pi
                                        * Referenced by: '<S1>/Constant5'
                                        */
  real_T RateLimiter_RisingLim;        /* Computed Parameter: RateLimiter_RisingLim
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Computed Parameter: RateLimiter_FallingLim
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 2.0
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -2.0
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Constant_Value;               /* Expression: pi
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Constant2_Value;              /* Expression: -pi
                                        * Referenced by: '<S1>/Constant2'
                                        */
  real_T Gain7_Gain;                   /* Expression: 0.75
                                        * Referenced by: '<Root>/Gain7'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 0.5
                                        * Referenced by: '<Root>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.5
                                        * Referenced by: '<Root>/Saturation2'
                                        */
  real_T ENPhi_0_Value[3];             /* Expression: [ 0 0 0]
                                        * Referenced by: '<Root>/ENPhi_0'
                                        */
  real_T DiscreteTimeIntegrator_gainval;/* Computed Parameter: DiscreteTimeIntegrator_gainval
                                         * Referenced by: '<S4>/Discrete-Time Integrator'
                                         */
  real_T DiscreteTimeIntegrator1_gainval;/* Computed Parameter: DiscreteTimeIntegrator1_gainval
                                          * Referenced by: '<S4>/Discrete-Time Integrator1'
                                          */
  real_T Gain4_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T DiscreteTimeIntegrator2_gainval;/* Computed Parameter: DiscreteTimeIntegrator2_gainval
                                          * Referenced by: '<S4>/Discrete-Time Integrator2'
                                          */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Constant_Value_f;             /* Expression: -55
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T L_Value;                      /* Expression: 180
                                        * Referenced by: '<S3>/L'
                                        */
  real_T Flight_plan_Value[12];        /* Expression: [0.6 2.1 0.1; 1.5 1.4 0.1; 1.7 1.0 0.1; 1.85 2.3 0.1]
                                        * Referenced by: '<Root>/Flight_plan'
                                        */
  real_T Constant_Value_h;             /* Expression: -1
                                        * Referenced by: '<S4>/Constant'
                                        */
  real32_T Constant5_Value_g;          /* Computed Parameter: Constant5_Value_g
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real32_T Gain6_Gain;                 /* Computed Parameter: Gain6_Gain
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real32_T Constant6_Value;            /* Computed Parameter: Constant6_Value
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real32_T Constant1_Value;            /* Computed Parameter: Constant1_Value
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real32_T Constant2_Value_n;          /* Computed Parameter: Constant2_Value_n
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real32_T Constant4_Value_p;          /* Computed Parameter: Constant4_Value_p
                                        * Referenced by: '<Root>/Constant4'
                                        */
  uint8_T PWSwitch_CurrentSetting;     /* Computed Parameter: PWSwitch_CurrentSetting
                                        * Referenced by: '<Root>/PWSwitch'
                                        */
  uint8_T DirSwitch_CurrentSetting;    /* Computed Parameter: DirSwitch_CurrentSetting
                                        * Referenced by: '<Root>/DirSwitch'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_car_controller_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
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
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern P_car_controller_T car_controller_P;

/* Block signals (auto storage) */
extern B_car_controller_T car_controller_B;

/* Block states (auto storage) */
extern DW_car_controller_T car_controller_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_car_controller_T car_controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_car_controller_T car_controller_Y;

/* Model entry point functions */
extern void car_controller_initialize(void);
extern void car_controller_step(void);
extern void car_controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_car_controller_T *const car_controller_M;

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
 * '<Root>' : 'car_controller'
 * '<S1>'   : 'car_controller/-PItoPI'
 * '<S2>'   : 'car_controller/Subsystem'
 * '<S3>'   : 'car_controller/pure pursuit'
 * '<S4>'   : 'car_controller/tricicle model'
 * '<S5>'   : 'car_controller/Subsystem/Chart'
 */
#endif                                 /* RTW_HEADER_car_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
