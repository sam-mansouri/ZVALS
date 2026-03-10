/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: droneController.c
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

#include "droneController.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_droneController_T droneController_DW;

/* External inputs (root inport signals with default storage) */
ExtU_droneController_T droneController_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_droneController_T droneController_Y;

/* Real-time model */
static RT_MODEL_droneController_T droneController_M_;
RT_MODEL_droneController_T *const droneController_M = &droneController_M_;

/* Model step function */
void droneController_step(void)
{
  real_T b_a;
  real_T c_a;
  real_T d_a;
  real_T f_a;
  real_T rtb_d1;

  /* Sum: '<S1>/Add' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   *  Inport: '<Root>/In2'
   */
  droneController_Y.xdrone = droneController_DW.DiscreteTimeIntegrator2_DSTATE +
    droneController_U.In2;

  /* Sum: '<S1>/Add1' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   *  Inport: '<Root>/In4'
   */
  droneController_Y.ydrone = droneController_U.In4 +
    droneController_DW.DiscreteTimeIntegrator_DSTATE;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/x1'
   *  Inport: '<Root>/x2'
   *  Inport: '<Root>/x3'
   *  Inport: '<Root>/y1'
   *  Inport: '<Root>/y2'
   *  Inport: '<Root>/y3'
   */
  rtb_d1 = droneController_Y.xdrone - droneController_U.x1;
  b_a = droneController_Y.ydrone - droneController_U.y1;
  rtb_d1 = sqrt(rtb_d1 * rtb_d1 + b_a * b_a);
  c_a = droneController_Y.xdrone - droneController_U.x2;
  d_a = droneController_Y.ydrone - droneController_U.y2;
  b_a = droneController_Y.xdrone - droneController_U.x3;
  f_a = droneController_Y.ydrone - droneController_U.y3;

  /* Sum: '<S1>/Subtract' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Sum: '<S3>/Sum1'
   */
  droneController_Y.zdrone = rtb_d1 - sqrt(c_a * c_a + d_a * d_a);

  /* Sum: '<S3>/Sum2' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtb_d1 -= sqrt(b_a * b_a + f_a * f_a);

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S3>/Constant'
   *  Math: '<S3>/Square'
   *  Math: '<S3>/Square1'
   *  Sqrt: '<S3>/Sqrt'
   *  Sum: '<S3>/Add'
   *  Sum: '<S3>/Subtract'
   */
  droneController_Y.enableZ = droneController_P.Constant_Value - sqrt
    (droneController_Y.zdrone * droneController_Y.zdrone + rtb_d1 * rtb_d1);

  /* Relay: '<S3>/Relay' */
  droneController_DW.Relay_Mode = ((droneController_Y.enableZ >=
    droneController_P.Relay_OnVal) || ((!(droneController_Y.enableZ <=
    droneController_P.Relay_OffVal)) && droneController_DW.Relay_Mode));
  if (droneController_DW.Relay_Mode) {
    /* Switch: '<S4>/Switch' */
    droneController_Y.enableZ = droneController_P.Relay_YOn;
  } else {
    /* Switch: '<S4>/Switch' */
    droneController_Y.enableZ = droneController_P.Relay_YOff;
  }

  /* End of Relay: '<S3>/Relay' */

  /* Gain: '<S3>/Gain4' incorporates:
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  Sum: '<S3>/Sum3'
   */
  droneController_Y.vx = (droneController_P.k11 * droneController_Y.zdrone +
    droneController_P.k12 * rtb_d1) * droneController_P.Gain4_Gain;

  /* Gain: '<S3>/Gain5' incorporates:
   *  Gain: '<S3>/Gain2'
   *  Gain: '<S3>/Gain3'
   *  Sum: '<S3>/Sum4'
   */
  droneController_Y.vy = (droneController_P.k21 * droneController_Y.zdrone +
    droneController_P.k22 * rtb_d1) * droneController_P.Gain5_Gain;

  /* Sum: '<S1>/Subtract' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   *  Inport: '<Root>/In5'
   */
  droneController_Y.zdrone = droneController_U.In5 -
    droneController_DW.DiscreteTimeIntegrator1_DSTATE;

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S4>/Zero'
   */
  if (droneController_Y.enableZ > droneController_P.Switch_Threshold) {
    rtb_d1 = droneController_Y.zdrone;
  } else {
    rtb_d1 = droneController_P.Zero_Value;
  }

  /* Gain: '<S4>/Gain1' incorporates:
   *  Constant: '<S4>/z_final'
   *  Gain: '<S4>/Gain'
   *  Sum: '<S4>/Subtract'
   *  Switch: '<S4>/Switch'
   */
  droneController_Y.vz = (droneController_P.z_final_Value - rtb_d1) *
    droneController_P.Gain_Gain * droneController_P.Gain1_Gain;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' */
  droneController_DW.DiscreteTimeIntegrator2_DSTATE +=
    droneController_P.DiscreteTimeIntegrator2_gainval * droneController_Y.vx;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
  droneController_DW.DiscreteTimeIntegrator_DSTATE +=
    droneController_P.DiscreteTimeIntegrator_gainval * droneController_Y.vy;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  droneController_DW.DiscreteTimeIntegrator1_DSTATE +=
    droneController_P.DiscreteTimeIntegrator1_gainval * droneController_Y.vz;
}

/* Model initialize function */
void droneController_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' */
  droneController_DW.DiscreteTimeIntegrator2_DSTATE =
    droneController_P.DiscreteTimeIntegrator2_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
  droneController_DW.DiscreteTimeIntegrator_DSTATE =
    droneController_P.DiscreteTimeIntegrator_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  droneController_DW.DiscreteTimeIntegrator1_DSTATE =
    droneController_P.DiscreteTimeIntegrator1_IC;
}

/* Model terminate function */
void droneController_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
