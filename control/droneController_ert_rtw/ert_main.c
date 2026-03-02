/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
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
#include "rtwtypes.h"
#include "MW_target_hardware_resources.h"

volatile int IsrOverrun = 0;
static boolean_T OverrunFlag = 0;
void rt_OneStep(void)
{
  /* Check for overrun. Protect OverrunFlag against preemption */
  if (OverrunFlag++) {
    IsrOverrun = 1;
    OverrunFlag--;
    return;
  }

  __enable_irq();
  droneController_step();

  /* Get model outputs here */
  __disable_irq();
  OverrunFlag--;
}

volatile boolean_T stopRequested;
volatile boolean_T runModel;
int main(int argc, char **argv)
{
  float modelBaseRate = 0.001;
  float systemClock = 25;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;
  SystemCoreClockUpdate();
  rtmSetErrorStatus(droneController_M, 0);
  droneController_initialize();
  __disable_irq();
  ARMCM_SysTick_Config(modelBaseRate);
  runModel =
    rtmGetErrorStatus(droneController_M) == (NULL);
  __enable_irq();
  __enable_irq();
  while (runModel) {
    stopRequested = !(
                      rtmGetErrorStatus(droneController_M) == (NULL));
    runModel = !(stopRequested);
  }

  /* Terminate model */
  droneController_terminate();
  __disable_irq();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
