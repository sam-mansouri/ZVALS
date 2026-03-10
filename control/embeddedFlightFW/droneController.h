/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: droneController.h
 *
 * Code generated for Simulink model 'droneController'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Mar  2 14:11:41 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef droneController_h_
#define droneController_h_
#ifndef droneController_COMMON_INCLUDES_
#define droneController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* droneController_COMMON_INCLUDES_ */

#include "droneController_types.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator2_DSTATE;/* '<S1>/Discrete-Time Integrator2' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S1>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S1>/Discrete-Time Integrator1' */
  boolean_T Relay_Mode;                /* '<S3>/Relay' */
} DW_droneController_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T x1;                           /* '<Root>/x1' */
  real_T In2;                          /* '<Root>/In2' */
  real_T y1;                           /* '<Root>/y1' */
  real_T In4;                          /* '<Root>/In4' */
  real_T In5;                          /* '<Root>/In5' */
  real_T x2;                           /* '<Root>/x2' */
  real_T y2;                           /* '<Root>/y2' */
  real_T x3;                           /* '<Root>/x3' */
  real_T y3;                           /* '<Root>/y3' */
} ExtU_droneController_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T xdrone;                       /* '<Root>/xdrone' */
  real_T ydrone;                       /* '<Root>/ydrone' */
  real_T zdrone;                       /* '<Root>/zdrone' */
  real_T vx;                           /* '<Root>/vx' */
  real_T vy;                           /* '<Root>/vy' */
  real_T enableZ;                      /* '<Root>/enableZ' */
  real_T vz;                           /* '<Root>/vz' */
} ExtY_droneController_T;

/* Parameters (default storage) */
struct P_droneController_T_ {
  real_T k11;                          /* Variable: k11
                                        * Referenced by: '<S3>/Gain'
                                        */
  real_T k12;                          /* Variable: k12
                                        * Referenced by: '<S3>/Gain1'
                                        */
  real_T k21;                          /* Variable: k21
                                        * Referenced by: '<S3>/Gain2'
                                        */
  real_T k22;                          /* Variable: k22
                                        * Referenced by: '<S3>/Gain3'
                                        */
  real_T Zero_Value;                   /* Expression: 0
                                        * Referenced by: '<S4>/Zero'
                                        */
  real_T DiscreteTimeIntegrator2_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator2_gainval
                           * Referenced by: '<S1>/Discrete-Time Integrator2'
                           */
  real_T DiscreteTimeIntegrator2_IC;   /* Expression: 0
                                        * Referenced by: '<S1>/Discrete-Time Integrator2'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S1>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S1>/Discrete-Time Integrator'
                                        */
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Relay_OnVal;                  /* Expression: 0.95
                                        * Referenced by: '<S3>/Relay'
                                        */
  real_T Relay_OffVal;                 /* Expression: 0.92
                                        * Referenced by: '<S3>/Relay'
                                        */
  real_T Relay_YOn;                    /* Expression: 1
                                        * Referenced by: '<S3>/Relay'
                                        */
  real_T Relay_YOff;                   /* Expression: 0
                                        * Referenced by: '<S3>/Relay'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<S3>/Gain4'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<S3>/Gain5'
                                        */
  real_T DiscreteTimeIntegrator1_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator1_gainval
                           * Referenced by: '<S1>/Discrete-Time Integrator1'
                           */
  real_T DiscreteTimeIntegrator1_IC;   /* Expression: 0
                                        * Referenced by: '<S1>/Discrete-Time Integrator1'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S4>/Switch'
                                        */
  real_T z_final_Value;                /* Expression: 0
                                        * Referenced by: '<S4>/z_final'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.5
                                        * Referenced by: '<S4>/Gain'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<S4>/Gain1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_droneController_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_droneController_T droneController_P;

/* Block states (default storage) */
extern DW_droneController_T droneController_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_droneController_T droneController_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_droneController_T droneController_Y;

/* Model entry point functions */
extern void droneController_initialize(void);
extern void droneController_step(void);
extern void droneController_terminate(void);

/* Real-time Model object */
extern RT_MODEL_droneController_T *const droneController_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('controlMain/droneController')    - opens subsystem controlMain/droneController
 * hilite_system('controlMain/droneController/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'controlMain'
 * '<S1>'   : 'controlMain/droneController'
 * '<S2>'   : 'controlMain/droneController/MATLAB Function'
 * '<S3>'   : 'controlMain/droneController/XY_convergence_controller'
 * '<S4>'   : 'controlMain/droneController/Z_convergence_controller'
 */
#endif                                 /* droneController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
