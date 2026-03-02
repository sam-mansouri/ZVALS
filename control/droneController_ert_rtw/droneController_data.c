/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: droneController_data.c
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

/* Block parameters (default storage) */
P_droneController_T droneController_P = {
  /* Variable: k11
   * Referenced by: '<S3>/Gain'
   */
  0.5,

  /* Variable: k12
   * Referenced by: '<S3>/Gain1'
   */
  0.0,

  /* Variable: k21
   * Referenced by: '<S3>/Gain2'
   */
  0.0,

  /* Variable: k22
   * Referenced by: '<S3>/Gain3'
   */
  0.5,

  /* Expression: 0
   * Referenced by: '<S4>/Zero'
   */
  0.0,

  /* Computed Parameter: DiscreteTimeIntegrator2_gainval
   * Referenced by: '<S1>/Discrete-Time Integrator2'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S1>/Discrete-Time Integrator2'
   */
  0.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<S1>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S1>/Discrete-Time Integrator'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S3>/Constant'
   */
  1.0,

  /* Expression: 0.95
   * Referenced by: '<S3>/Relay'
   */
  0.95,

  /* Expression: 0.92
   * Referenced by: '<S3>/Relay'
   */
  0.92,

  /* Expression: 1
   * Referenced by: '<S3>/Relay'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S3>/Relay'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S3>/Gain4'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<S3>/Gain5'
   */
  -1.0,

  /* Computed Parameter: DiscreteTimeIntegrator1_gainval
   * Referenced by: '<S1>/Discrete-Time Integrator1'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S1>/Discrete-Time Integrator1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S4>/Switch'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S4>/z_final'
   */
  0.0,

  /* Expression: 0.5
   * Referenced by: '<S4>/Gain'
   */
  0.5,

  /* Expression: -1
   * Referenced by: '<S4>/Gain1'
   */
  -1.0
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
