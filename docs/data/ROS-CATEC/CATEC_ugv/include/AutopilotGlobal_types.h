#ifndef AUTOPILOTGLOBAL_TYPES_H
#define AUTOPILOTGLOBAL_TYPES_H
/*
 * File: AutopilotGlobal_types.h
 *
 * Real-Time Workshop code generated for Simulink model AutopilotGlobal.
 *
 * Model version                        : 1.13
 * Real-Time Workshop file version      : 7.6.1  (R2010bSP1)  28-Jan-2011
 * Real-Time Workshop file generated on : Wed May 09 11:35:02 2012
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Wed May 09 11:35:02 2012
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_AutopilotGlobal_types_h_
#define RTW_HEADER_AutopilotGlobal_types_h_
#include "rtwtypes.h"
#ifndef _DEFINED_TYPEDEF_FOR_TState_
#define _DEFINED_TYPEDEF_FOR_TState_

typedef struct {
  uint32_T uiIdInterface;
  uint32_T uiDataOk;
} TState;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TAutonomOn_
#define _DEFINED_TYPEDEF_FOR_TAutonomOn_

typedef struct {
  boolean_T bAutonomOn;
} TAutonomOn;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TALocalOn_
#define _DEFINED_TYPEDEF_FOR_TALocalOn_

typedef struct {
  boolean_T bALocalOn;
} TALocalOn;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IAutonomous_
#define _DEFINED_TYPEDEF_FOR_IAutonomous_

typedef struct {
  TState state;
  TAutonomOn autonomOn;
  TALocalOn aLocalOn;
} IAutonomous;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TCmdFlags_
#define _DEFINED_TYPEDEF_FOR_TCmdFlags_

typedef struct {
  boolean_T bFlag1;
  boolean_T bFlag2;
  boolean_T bFlag3;
  boolean_T bFlag4;
  boolean_T bFlag5;
  boolean_T bFlag6;
  boolean_T bFlag7;
  boolean_T bFlag8;
} TCmdFlags;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_ICmdFlagsFw_
#define _DEFINED_TYPEDEF_FOR_ICmdFlagsFw_

typedef struct {
  TState state;
  TCmdFlags cmdFlagsFw;
} ICmdFlagsFw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_ICmdFlagsMr_
#define _DEFINED_TYPEDEF_FOR_ICmdFlagsMr_

typedef struct {
  TState state;
  TCmdFlags cmdFlagsMr;
} ICmdFlagsMr;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_ICmdFlagsRw_
#define _DEFINED_TYPEDEF_FOR_ICmdFlagsRw_

typedef struct {
  TState state;
  TCmdFlags cmdFlagsRw;
} ICmdFlagsRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TDetect_
#define _DEFINED_TYPEDEF_FOR_TDetect_

typedef struct {
  boolean_T bDetect;
} TDetect;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IDetectWp_
#define _DEFINED_TYPEDEF_FOR_IDetectWp_

typedef struct {
  TState state;
  TDetect detect;
} IDetectWp;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TAccel_
#define _DEFINED_TYPEDEF_FOR_TAccel_

typedef struct {
  real_T dAx;
  real_T dAy;
  real_T dAz;
} TAccel;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IAccel_
#define _DEFINED_TYPEDEF_FOR_IAccel_

typedef struct {
  TState state;
  TAccel accel;
} IAccel;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TAttitude_
#define _DEFINED_TYPEDEF_FOR_TAttitude_

typedef struct {
  real_T dPhiEuler;
  real_T dThetaEuler;
  real_T dPsiEuler;
} TAttitude;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IAttitude_
#define _DEFINED_TYPEDEF_FOR_IAttitude_

typedef struct {
  TState state;
  TAttitude attitude;
} IAttitude;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TBattery_
#define _DEFINED_TYPEDEF_FOR_TBattery_

typedef struct {
  real32_T sVoltage;
  real32_T sCurrent;
} TBattery;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IBattery_
#define _DEFINED_TYPEDEF_FOR_IBattery_

typedef struct {
  TState state;
  TBattery battery;
} IBattery;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPosRel_
#define _DEFINED_TYPEDEF_FOR_TPosRel_

typedef struct {
  real_T dXRel;
  real_T dYRel;
  real_T dZRel;
} TPosRel;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TOrigin_
#define _DEFINED_TYPEDEF_FOR_TOrigin_

typedef struct {
  real_T dLat0;
  real_T dLon0;
  real_T dHWgs0;
} TOrigin;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THAgl_
#define _DEFINED_TYPEDEF_FOR_THAgl_

typedef struct {
  real_T dHAgl;
} THAgl;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THBarom_
#define _DEFINED_TYPEDEF_FOR_THBarom_

typedef struct {
  real_T dHBarom;
} THBarom;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IPosRel_
#define _DEFINED_TYPEDEF_FOR_IPosRel_

typedef struct {
  TState state;
  TPosRel posRel;
  TOrigin origin;
  THAgl hAgl;
  THBarom hBarom;
} IPosRel;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPosWgs_
#define _DEFINED_TYPEDEF_FOR_TPosWgs_

typedef struct {
  real_T dLatWgs;
  real_T dLonWgs;
  real_T dHWgs;
} TPosWgs;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IPosWgs_
#define _DEFINED_TYPEDEF_FOR_IPosWgs_

typedef struct {
  TState state;
  TPosWgs posWgs;
  THAgl hAgl;
} IPosWgs;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRates_
#define _DEFINED_TYPEDEF_FOR_TRates_

typedef struct {
  real_T dRateP;
  real_T dRateQ;
  real_T dRateR;
} TRates;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IRates_
#define _DEFINED_TYPEDEF_FOR_IRates_

typedef struct {
  TState state;
  TRates rates;
} IRates;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TVelEarth_
#define _DEFINED_TYPEDEF_FOR_TVelEarth_

typedef struct {
  real_T dVelNorth;
  real_T dVelWest;
  real_T dVelUp;
} TVelEarth;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IVelEarth_
#define _DEFINED_TYPEDEF_FOR_IVelEarth_

typedef struct {
  TState state;
  TVelEarth velEarth;
} IVelEarth;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_ITestbedMonitor_
#define _DEFINED_TYPEDEF_FOR_ITestbedMonitor_

typedef struct {
  uint32_T quad_ID;
  IPosRel iPosRel;
  IVelEarth iVelEarth;
  IAttitude iAttitude;
  IBattery iBattery;
} ITestbedMonitor;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TVelAerodyn_
#define _DEFINED_TYPEDEF_FOR_TVelAerodyn_

typedef struct {
  real_T dVelIas;
  real_T dVelTas;
} TVelAerodyn;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IVelAerodyn_
#define _DEFINED_TYPEDEF_FOR_IVelAerodyn_

typedef struct {
  TState state;
  TVelAerodyn velAerodyn;
} IVelAerodyn;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TVelBody_
#define _DEFINED_TYPEDEF_FOR_TVelBody_

typedef struct {
  real_T dVelU;
  real_T dVelV;
  real_T dVelW;
} TVelBody;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IVelBody_
#define _DEFINED_TYPEDEF_FOR_IVelBody_

typedef struct {
  TState state;
  TVelBody velBody;
} IVelBody;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TMode_
#define _DEFINED_TYPEDEF_FOR_TMode_

typedef struct {
  uint8_T uiMode;
} TMode;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TWaypoints_
#define _DEFINED_TYPEDEF_FOR_TWaypoints_

typedef struct {
  real_T dWp1Lat;
  real_T dWp1Lon;
  real_T dWp1H;
  real_T dWp2Lat;
  real_T dWp2Lon;
  real_T dWp2H;
  real_T dWp3Lat;
  real_T dWp3Lon;
  real_T dWp3H;
  real_T dTypeWp;
  real_T dRadius;
} TWaypoints;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TVelFlightPlan_
#define _DEFINED_TYPEDEF_FOR_TVelFlightPlan_

typedef struct {
  real_T dVelFlightPlan;
} TVelFlightPlan;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THFlightPlan_
#define _DEFINED_TYPEDEF_FOR_THFlightPlan_

typedef struct {
  real_T dHFlightPlan;
} THFlightPlan;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TCourseFlightPlan_
#define _DEFINED_TYPEDEF_FOR_TCourseFlightPlan_

typedef struct {
  real_T dCourseFlightPlan;
} TCourseFlightPlan;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THeadingFlightPlan_
#define _DEFINED_TYPEDEF_FOR_THeadingFlightPlan_

typedef struct {
  real_T dHeadingFlightPlan;
} THeadingFlightPlan;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IFlightPlan_
#define _DEFINED_TYPEDEF_FOR_IFlightPlan_

typedef struct {
  TState state;
  TMode modeFp;
  TWaypoints waypoints;
  TVelFlightPlan velFlightPlan;
  THFlightPlan hFlightPlan;
  TCourseFlightPlan courseFlightPlan;
  THeadingFlightPlan headingFlightPlan;
} IFlightPlan;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefFwMode_
#define _DEFINED_TYPEDEF_FOR_TRefFwMode_

typedef struct {
  uint8_T uiRefFwModeThrot;
  uint8_T uiRefFwModeLon;
  uint8_T uiRefFwModeLat;
} TRefFwMode;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefFw_
#define _DEFINED_TYPEDEF_FOR_TRefFw_

typedef struct {
  real_T dRefFw1;
  real_T dRefFw2;
  real_T dRefFw3;
} TRefFw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IRefFw_
#define _DEFINED_TYPEDEF_FOR_IRefFw_

typedef struct {
  TState state;
  TRefFwMode refFwMode;
  TRefFw refFw;
} IRefFw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefRwMode_
#define _DEFINED_TYPEDEF_FOR_TRefRwMode_

typedef struct {
  uint8_T uiRefRwMode;
} TRefRwMode;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefPosRw_
#define _DEFINED_TYPEDEF_FOR_TRefPosRw_

typedef struct {
  real_T dRefPosRw1;
  real_T dRefPosRw2;
  real_T dRefPosRw3;
  real_T dRefPosRw4;
} TRefPosRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefVelRw_
#define _DEFINED_TYPEDEF_FOR_TRefVelRw_

typedef struct {
  real_T dRefVelRw1;
  real_T dRefVelRw2;
  real_T dRefVelRw3;
  real_T dRefVelRw4;
} TRefVelRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRefRpmRw_
#define _DEFINED_TYPEDEF_FOR_TRefRpmRw_

typedef struct {
  real_T dRefRpmRw;
} TRefRpmRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IRefRw_
#define _DEFINED_TYPEDEF_FOR_IRefRw_

typedef struct {
  TState state;
  TRefRwMode refRwMode;
  TRefPosRw refPosRw;
  TRefVelRw refVelRw;
  TRefRpmRw refRpmRw;
} IRefRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPStaAds_
#define _DEFINED_TYPEDEF_FOR_TPStaAds_

typedef struct {
  real_T dPStaAds;
} TPStaAds;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPTotAds_
#define _DEFINED_TYPEDEF_FOR_TPTotAds_

typedef struct {
  real_T dPTotAds;
} TPTotAds;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IAds_
#define _DEFINED_TYPEDEF_FOR_IAds_

typedef struct {
  TState state;
  TPStaAds pStaAds;
  TPTotAds pTotAds;
} IAds;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THAglAltim_
#define _DEFINED_TYPEDEF_FOR_THAglAltim_

typedef struct {
  real_T dHAglAltim;
} THAglAltim;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IAltim_
#define _DEFINED_TYPEDEF_FOR_IAltim_

typedef struct {
  TState state;
  THAglAltim hAglAltim;
} IAltim;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPosGps_
#define _DEFINED_TYPEDEF_FOR_TPosGps_

typedef struct {
  real_T dLatGps;
  real_T dLonGps;
  real_T dHWgsGps;
} TPosGps;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TVelGps_
#define _DEFINED_TYPEDEF_FOR_TVelGps_

typedef struct {
  real_T dVelNorthGps;
  real_T dVelEastGps;
  real_T dVelDownGps;
} TVelGps;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IGps_
#define _DEFINED_TYPEDEF_FOR_IGps_

typedef struct {
  TState state;
  TPosGps posGps;
  TVelGps velGps;
} IGps;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TRatesImu_
#define _DEFINED_TYPEDEF_FOR_TRatesImu_

typedef struct {
  real_T dPImu;
  real_T dQImu;
  real_T dRImu;
} TRatesImu;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TAccelImu_
#define _DEFINED_TYPEDEF_FOR_TAccelImu_

typedef struct {
  real_T dAccelXImu;
  real_T dAccelYImu;
  real_T dAccelZImu;
} TAccelImu;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IImu_
#define _DEFINED_TYPEDEF_FOR_IImu_

typedef struct {
  TState state;
  TRatesImu ratesImu;
  TAccelImu accelImu;
} IImu;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TMagField_
#define _DEFINED_TYPEDEF_FOR_TMagField_

typedef struct {
  real_T dMagFieldX;
  real_T dMagFieldY;
  real_T dMagFieldZ;
} TMagField;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IMagnet_
#define _DEFINED_TYPEDEF_FOR_IMagnet_

typedef struct {
  TState state;
  TMagField magField;
} IMagnet;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPosVicon_
#define _DEFINED_TYPEDEF_FOR_TPosVicon_

typedef struct {
  real_T dXVicon;
  real_T dYVicon;
  real_T dZVicon;
} TPosVicon;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TAttVicon_
#define _DEFINED_TYPEDEF_FOR_TAttVicon_

typedef struct {
  real_T dPhiVicon;
  real_T dThetaVicon;
  real_T dPsiVicon;
} TAttVicon;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_THeartBeat_
#define _DEFINED_TYPEDEF_FOR_THeartBeat_

typedef struct {
  uint32_T uiHeartBeat;
} THeartBeat;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IVicon_
#define _DEFINED_TYPEDEF_FOR_IVicon_

typedef struct {
  TState state;
  TPosVicon posVicon;
  TAttVicon attVicon;
  THeartBeat heartBeat;
} IVicon;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TTestbedObjects_
#define _DEFINED_TYPEDEF_FOR_TTestbedObjects_

typedef struct {
  IVicon object1;
  IVicon object2;
  IVicon object3;
  IVicon object4;
  IVicon object5;
  IVicon object6;
  IVicon object7;
  IVicon object8;
  IVicon object9;
  IVicon object10;
  IVicon object11;
  IVicon object12;
  IVicon object13;
  IVicon object14;
  IVicon object15;
  IVicon object16;
  IVicon object17;
  IVicon object18;
  IVicon object19;
  IVicon object20;
} TTestbedObjects;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IViconMulti_
#define _DEFINED_TYPEDEF_FOR_IViconMulti_

typedef struct {
  TState state;
  TTestbedObjects objects;
} IViconMulti;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TContFw_
#define _DEFINED_TYPEDEF_FOR_TContFw_

typedef struct {
  real_T dFlap;
  real_T dElev;
  real_T dAileron;
  real_T dRudder;
  real_T dThrottle;
} TContFw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IContFw_
#define _DEFINED_TYPEDEF_FOR_IContFw_

typedef struct {
  TState state;
  TContFw contFw;
} IContFw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TContQuad_
#define _DEFINED_TYPEDEF_FOR_TContQuad_

typedef struct {
  real_T dRollQuad;
  real_T dPitchQuad;
  real_T dYawQuad;
  real_T dThrustQuad;
} TContQuad;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IContQuad_
#define _DEFINED_TYPEDEF_FOR_IContQuad_

typedef struct {
  TState state;
  TContQuad contQuad;
} IContQuad;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TContRw_
#define _DEFINED_TYPEDEF_FOR_TContRw_

typedef struct {
  real_T dT1MR;
  real_T dT2MR;
  real_T dF3MR;
  real_T dF2TR;
  real_T dRpmCtrl;
} TContRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IContRw_
#define _DEFINED_TYPEDEF_FOR_IContRw_

typedef struct {
  TState state;
  TContRw contRw;
} IContRw;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TServoTicks_
#define _DEFINED_TYPEDEF_FOR_TServoTicks_

typedef struct {
  uint16_T uiServo1;
  uint16_T uiServo2;
  uint16_T uiServo3;
  uint16_T uiServo4;
  uint16_T uiServo5;
  uint16_T uiServo6;
  uint16_T uiServo7;
  uint16_T uiServo8;
  uint16_T uiServo9;
} TServoTicks;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IPcToServos_
#define _DEFINED_TYPEDEF_FOR_IPcToServos_

typedef struct {
  TState state;
  TServoTicks servoTicks;
} IPcToServos;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_TPilotChannels_
#define _DEFINED_TYPEDEF_FOR_TPilotChannels_

typedef struct {
  uint16_T uiChannel1;
  uint16_T uiChannel2;
  uint16_T uiChannel3;
  uint16_T uiChannel4;
  uint16_T uiChannel5;
  uint16_T uiChannel6;
  uint16_T uiChannel7;
  uint16_T uiChannel8;
  uint16_T uiChannel9;
} TPilotChannels;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_IStick_
#define _DEFINED_TYPEDEF_FOR_IStick_

typedef struct {
  TState state;
  TPilotChannels pilotChannels;
} IStick;

#endif

/* Forward declaration for rtModel */
typedef struct RT_MODEL_AutopilotGlobal RT_MODEL_AutopilotGlobal;

#endif                                 /* RTW_HEADER_AutopilotGlobal_types_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
#endif // AUTOPILOTGLOBAL_TYPES_H
