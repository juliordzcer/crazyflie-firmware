#include <stdbool.h>

#include "stabilizer_types.h"

#include "get_rate.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"


static bool attFiltEnable = ATTITUDE_LPF_ENABLE;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;


static inline int16_t saturateSignedInt16(float in)
{
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject gainRoll = {
  .kp = PID_ROLL_KP,
  .ki = PID_ROLL_KI,
  .kd = PID_ROLL_KD,
  .kff = PID_ROLL_KFF,
};

PidObject gainPitch = {
  .kp = PID_PITCH_KP,
  .ki = PID_PITCH_KI,
  .kd = PID_PITCH_KD,
  .kff = PID_PITCH_KFF,
};

PidObject gainYaw = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

static bool isInit;

void GetAttitudeRateInit(const float updateDt)
{
  if(isInit)
    return;

  pidInit(&gainRoll,  0, gainRoll.kp,  gainRoll.ki,  gainRoll.kd,  gainRoll.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&gainPitch, 0, gainPitch.kp, gainPitch.ki, gainPitch.kd, gainPitch.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&gainYaw,   0, gainYaw.kp,   gainYaw.ki,   gainYaw.kd,   gainYaw.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&gainRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&gainPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&gainYaw,   PID_YAW_INTEGRATION_LIMIT);

  isInit = true;
}

bool GetAttitudeRateTest()
{
  return isInit;
}

void GetAttitudeRate(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&gainRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&gainRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&gainPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&gainPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&gainYaw, yawError);
  *yawRateDesired = pidUpdate(&gainYaw, eulerYawActual, false);
}

void GetAttitudeRateResetAll(void)
{
  pidReset(&gainRoll);
  pidReset(&gainPitch);
  pidReset(&gainYaw);
}

void GetAttitudeRateResetRoll(void)
{
  pidReset(&gainRoll);
}

void GetAttitudeRateResetPitch(void)
{
  pidReset(&gainPitch);
}

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(get_rate)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &gainRoll.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &gainRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &gainRoll.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &gainRoll.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &gainPitch.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &gainPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &gainPitch.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &gainPitch.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &gainYaw.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &gainYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &gainYaw.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &gainYaw.kff)
PARAM_GROUP_STOP(get_rate)

