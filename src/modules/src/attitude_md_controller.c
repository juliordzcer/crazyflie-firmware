/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>

#include "stabilizer_types.h"
#include "get_rate.h"  // Proporciona la velocidad deseada angular
#include "attitude_md_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"
#include "math3d.h"





static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
static float yawMaxDelta = YAW_MAX_DELTA;

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject MDRollRate = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject MDPitchRate = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject MDYawRate = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};


static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static bool isInit;

void attitudeControllerInitMD(const float updateDt)
{
  if(isInit)
    return;

  pidInit(&MDRollRate,  0, MDRollRate.kp,  MDRollRate.ki,  MDRollRate.kd,
       MDRollRate.kff,  updateDt, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&MDPitchRate, 0, MDPitchRate.kp, MDPitchRate.ki, MDPitchRate.kd,
       MDPitchRate.kff, updateDt, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&MDYawRate,   0, MDYawRate.kp,   MDYawRate.ki,   MDYawRate.kd,
       MDYawRate.kff,   updateDt, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&MDRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&MDPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&MDYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  isInit = true;
}

bool attitudeControllerTestMD()
{
  return isInit;
}

void ControllerAtitudeMD(float eulerRollActual , float eulerPitchActual , float eulerYawActual ,
                         float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
                         float rollRateActual  , float pitchRateActual  , float yawRateActual  ,
                         float rollRateDesired , float pitchRateDesired , float yawRateDesired ,
                         int16_t* roll         , int16_t* pitch         , int16_t* yaw          )
{

  
  //Modificar por el controlador 
  pidSetDesired(&MDRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&MDRollRate, rollRateActual, true));

  pidSetDesired(&MDPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&MDPitchRate, pitchRateActual, true));

  pidSetDesired(&MDYawRate, yawRateDesired);

  yawOutput = saturateSignedInt16(pidUpdate(&MDYawRate, yawRateActual, true));

  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}



void attitudeControllerResetAllMD(void)
{
  pidReset(&MDRollRate);
  pidReset(&MDPitchRate);
  pidReset(&MDYawRate);
}


float attitudeControllerGetYawMaxDeltaMD(void)
{
  return yawMaxDelta;
}

