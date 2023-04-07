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

#include "attitude_tc_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"


static bool attFiltEnable = ATTITUDE_LPF_ENABLE;
static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;
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

PidObject TCRollRate = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject TCPitchRate = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject TCYawRate = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

PidObject TCRoll = {
  .kp = PID_ROLL_KP,
  .ki = PID_ROLL_KI,
  .kd = PID_ROLL_KD,
  .kff = PID_ROLL_KFF,
};

PidObject TCPitch = {
  .kp = PID_PITCH_KP,
  .ki = PID_PITCH_KI,
  .kd = PID_PITCH_KD,
  .kff = PID_PITCH_KFF,
};

PidObject TCYaw = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static bool isInit;

void attitudeControllerInitTC(const float updateDt)
{
  if(isInit)
    return;

  //TODO: get parameters from configuration manager instead - now (partly) implemented
  pidInit(&TCRollRate,  0, TCRollRate.kp,  TCRollRate.ki,  TCRollRate.kd,
       TCRollRate.kff,  updateDt, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&TCPitchRate, 0, TCPitchRate.kp, TCPitchRate.ki, TCPitchRate.kd,
       TCPitchRate.kff, updateDt, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&TCYawRate,   0, TCYawRate.kp,   TCYawRate.ki,   TCYawRate.kd,
       TCYawRate.kff,   updateDt, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&TCRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&TCPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&TCYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&TCRoll,  0, TCRoll.kp,  TCRoll.ki,  TCRoll.kd,  TCRoll.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&TCPitch, 0, TCPitch.kp, TCPitch.ki, TCPitch.kd, TCPitch.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&TCYaw,   0, TCYaw.kp,   TCYaw.ki,   TCYaw.kd,   TCYaw.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&TCRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&TCPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&TCYaw,   PID_YAW_INTEGRATION_LIMIT);

  isInit = true;
}

bool attitudeControllerTestTC()
{
  return isInit;
}

void attitudeControllerCorrectRateTC(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&TCRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&TCRollRate, rollRateActual, true));

  pidSetDesired(&TCPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&TCPitchRate, pitchRateActual, true));

  pidSetDesired(&TCYawRate, yawRateDesired);

  yawOutput = saturateSignedInt16(pidUpdate(&TCYawRate, yawRateActual, true));
}

void attitudeControllerCorrectAttitudeTC(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&TCRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&TCRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&TCPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&TCPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&TCYaw, yawError);
  *yawRateDesired = pidUpdate(&TCYaw, eulerYawActual, false);
}

void attitudeControllerResetRollAttitudeTC(void)
{
    pidReset(&TCRoll);
}

void attitudeControllerResetPitchAttitudeTC(void)
{
    pidReset(&TCPitch);
}

void attitudeControllerResetAllTC(void)
{
  pidReset(&TCRoll);
  pidReset(&TCPitch);
  pidReset(&TCYaw);
  pidReset(&TCRollRate);
  pidReset(&TCPitchRate);
  pidReset(&TCYawRate);
}

void attitudeControllerGetActuatorOutputTC(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

float attitudeControllerGetYawMaxDeltaTC(void)
{
  return yawMaxDelta;
}

/**
 *  Log variables of attitude PID controller
 */ 
LOG_GROUP_START(tc_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &TCRoll.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &TCRoll.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &TCRoll.outD)
/**
 * @brief Feedforward output roll
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &TCRoll.outFF)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &TCPitch.outP)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &TCPitch.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &TCPitch.outD)
/**
 * @brief Feedforward output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &TCPitch.outFF)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &TCYaw.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &TCYaw.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &TCYaw.outD)
/**TCYawRate
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &TCYaw.outFF)
LOG_GROUP_STOP(tc_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &TCRollRate.outP)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &TCRollRate.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &TCRollRate.outD)
/**
 * @brief Feedforward output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &TCRollRate.outFF)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &TCPitchRate.outP)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &TCPitchRate.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &TCPitchRate.outD)
/**
 * @brief Feedforward output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &TCPitchRate.outFF)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &TCYawRate.outP)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &TCYawRate.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &TCYawRate.outD)
/**
 * @brief Feedforward output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &TCYawRate.outFF)
LOG_GROUP_STOP(pid_rate)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(tc_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &TCRoll.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &TCRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &TCRoll.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &TCRoll.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &TCPitch.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &TCPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &TCPitch.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &TCPitch.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &TCYaw.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &TCYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &TCYaw.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &TCYaw.kff)
/**
 * @brief If nonzero, yaw setpoint can only be set within +/- yawMaxDelta from the current yaw
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawMaxDelta, &yawMaxDelta)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attFiltEn, &attFiltEnable)
/**
 * @brief Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attFiltCut, &attFiltCutoff)
PARAM_GROUP_STOP(tc_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates 
 */
PARAM_GROUP_START(pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &TCRollRate.kp)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &TCRollRate.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &TCRollRate.kd)
/**
 * @brief Feedforward gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &TCRollRate.kff)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &TCPitchRate.kp)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &TCPitchRate.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &TCPitchRate.kd)
/**
 * @brief Feedforward gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &TCPitchRate.kff)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &TCYawRate.kp)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &TCYawRate.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &TCYawRate.kd)
/**
 * @brief Feedforward gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &TCYawRate.kff)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, rateFiltEn, &rateFiltEnable)
/**
 * @brief Low pass filter cut-off frequency, roll axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omxFiltCut, &omxFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, pitch axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omyFiltCut, &omyFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, yaw axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omzFiltCut, &omzFiltCutoff)
PARAM_GROUP_STOP(pid_rate)
