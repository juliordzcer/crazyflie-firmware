
#include "stabilizer_types.h"

#include "attitude_tc_controller.h"
#include "position_controller.h"
#include "controller_tc.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllertcInit(void)
{
  attitudeControllerInitTC(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllertcTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTestTC();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllertc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
       
      float yawMaxDelta = attitudeControllerGetYawMaxDeltaTC();
      if (yawMaxDelta != 0.0f)
      {
      float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
        if (delta > yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
        }
        else if (delta < -yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
        }
      }
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudeTC(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudeTC();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudeTC();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRateTC(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutputTC(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllTC();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}





























// /**
//  * Code developed by Ing. Julio Cesar Rodriguez Cervantes in January 2023
//  * More information see https://github.com/juliordzcer
//  * TC.c - implementation of the Twisting Continuo
//  */
// #include "stabilizer_types.h"
// #include "position_controller.h"
// #include "controller_pid.h"
// #include "attitude_controller.h"
// #include "controller_mellinger.h"
// #include "physicalConstants.h"
// #include "commander.h"
// #include "platform_defaults.h"

// #include "log.h"
// #include "param.h"
// #include "math3d.h"

// #define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// static attitude_t attitudeDesired;
// static attitude_t rateDesired;
// static float actuatorThrust;

// static float cmd_thrust;
// static float cmd_roll;
// static float cmd_pitch;
// static float cmd_yaw;


// // Maximum roll/pitch angle permited
// static float rLimit  = 20.0f;
// static float pLimit  = 20.0f;
// static float yLimit  = 360.0f;

// static float i_error_roll;
// static float i_error_pitch;
// static float i_error_yaw;
// static float error_roll;
// static float error_pitch;
// static float error_yaw;
// static float error_vroll;
// static float error_vpitch;
// static float error_vyaw;

// static float roll_control;
// static float pitch_control;
// static float yaw_control;

// static float sign_error_roll;
// static float sign_error_pitch;
// static float sign_error_yaw;
// static float sign_errorp_roll;
// static float sign_errorp_pitch;
// static float sign_errorp_yaw;

// static bool isInit;

// static float Rollzeta  = 0.06f;
// static float roll_kff  = 1.0f;

// static float Pitchzeta = 0.0f;
// static float pitch_kff = 0.0f;

// static float Yawzeta   = 0.0f;
// static float yaw_kff   = 0.0f;

// void controllerMellingerReset(void)
// {
//   roll_control = 0;
//   i_error_roll = 0;
//   i_error_pitch = 0;
//   i_error_yaw = 0;
//   error_roll = 0;
//   error_pitch = 0;
//   error_yaw = 0;
//   error_vroll = 0;
//   error_vpitch = 0;
//   error_vyaw = 0;
//   attitudeControllerResetAllPID();
//   positionControllerResetAllPID();
// }

// void controllerMellingerInit(void)
// {
//   attitudeControllerInit(ATTITUDE_UPDATE_DT);
//   controllerMellingerReset();
//   positionControllerInit();
// }

// bool controllerMellingerTest(void)
// {
//   return true;
//   return isInit;
// }
// static float capAngle(float angle) 
// {
//   float result = fmodf(angle + 180.0f, 360.0f) - 180.0f;
//   if (result > 180.0f) 
//   {
//     result -= 360.0f;
//   }
//   return result;
// }

// float sign(float x) {
//     if (x < 0.0f) {
//         return -1.0f;
//     } else if (x > 0.0f) {
//         return 1.0f;
//     } else {
//         return 0.0f;
//     }
// }

// float Integral(float input, float integralLimit, float dt) 
// {
//     static float integralValue = 0.0f; // initialize to 0 only once

//     float deltaIntegral = input * dt;
//     integralValue += deltaIntegral;

//     if (integralLimit != 0.0f) {
//         // Limit integral value if required
//         if (integralValue > integralLimit) {
//             integralValue = integralLimit;
//         } else if (integralValue < -integralLimit) {
//             integralValue = -integralLimit;
//         }
//     } else {
//         // Reset integral value to 0 if limit is 0
//         if (input == 0.0f) {
//             integralValue = 0.0f;
//         }
//     }

//     return integralValue;
// }


// float calcularRpm(float in) 
// {
//     const float A = 0.109e-6f, B = -210.59e-6f, C = 0.1517f, G = 9.80665f;
//     float in_gramos, in_rpm, in_abs;

//     in_abs = (in == 0) ? 0 : fabsf(in);
//     in_gramos = in_abs / (G / 1000.0f);
//     in_rpm = (sqrtf(4.0f * A * (in_gramos - C) + powf(B, 2.0f)) - B) / (2.0f * A) * ((in >= 0) ? 1.0f : -1.0f);
    
//     return in_rpm;
// }

// typedef struct {
//   float k1;
//   float k2;
//   float k3;
//   float k4;
// } Ganancias;

// void controllerMellinger(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick)
// {
//   const float dt = ATTITUDE_UPDATE_DT;
//   if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) 
//   {
//     // Rate-controled YAW is moving YAW angle setpoint
//     if (setpoint->mode.yaw == modeVelocity) {
//       attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
       
//       float yawMaxDelta = attitudeControllerGetYawMaxDelta();
//       if (yawMaxDelta != 0.0f)
//       {
//       float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
//       // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
//         if (delta > yawMaxDelta)
//         {
//           attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
//         }
//         else if (delta < -yawMaxDelta)
//         {
//           attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
//         }
//       }
//     } else if (setpoint->mode.yaw == modeAbs) {
//       attitudeDesired.yaw = setpoint->attitude.yaw;
//     } else if (setpoint->mode.quat == modeAbs) {
//       struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
//       struct vec rpy = quat2rpy(setpoint_quat);
//       attitudeDesired.yaw = degrees(rpy.z);
//     }

//     attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
//   }

//   if (RATE_DO_EXECUTE(POSITION_RATE, tick)) 
//   {
//     positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
//   }

//   if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
//     // Switch between manual and automatic position control
//     if (setpoint->mode.z == modeDisable) {
//       actuatorThrust = clamp(setpoint->thrust,0,60000);
//     }
//     if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
//       attitudeDesired.roll = setpoint->attitude.roll;
//       attitudeDesired.pitch = setpoint->attitude.pitch;
//     }

//     attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
//                                 attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
//                                 &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);


//     if (setpoint->mode.roll == modeVelocity) {
//       rateDesired.roll = setpoint->attitudeRate.roll;
//       attitudeControllerResetRollAttitudePID();
//     }
//     if (setpoint->mode.pitch == modeVelocity) {
//       rateDesired.pitch = setpoint->attitudeRate.pitch;
//       attitudeControllerResetPitchAttitudePID();
//     }

//     Ganancias calcularGanancias(float zeta) 
//     {
//       Ganancias ganancias;
//       if (zeta <= 0.0f) {
//         ganancias.k1 = 0.0f;
//         ganancias.k2 = 0.0f;
//         ganancias.k3 = 0.0f;
//         ganancias.k4 = 0.0f;
//         return ganancias;
//       }
      
//       ganancias.k1 = 7.0f * powf(zeta, 2.0f/3.0f);
//       ganancias.k2 = 5.0f * powf(zeta, 1.0f/2.0f);
//       ganancias.k3 = 2.3f * zeta;
//       ganancias.k4 = 1.1f * zeta;

//       return ganancias;
//     }

//     error_roll   = (setpoint->attitude.roll - state->attitude.roll) ;
//     error_pitch  = (setpoint->attitude.pitch - state->attitude.pitch);
//     error_yaw    = (setpoint->attitude.yaw - state->attitude.yaw);

//     if (error_yaw > 180.0f)
//       error_yaw -= 360.0f;
//     else if (error_yaw < -180.0f)
//       error_yaw += 360.0f;

//     error_vroll  = (setpoint->attitudeRate.roll - sensors->gyro.x);
//     error_vpitch = (setpoint->attitudeRate.pitch -(-sensors->gyro.y));
//     error_vyaw   = (setpoint->attitudeRate.yaw - sensors->gyro.z);

//     i_error_roll  = Integral(error_roll  ,rLimit,dt);
//     i_error_pitch = Integral(error_pitch ,pLimit,dt);
//     i_error_yaw =   Integral(error_yaw   ,yLimit,dt);

//     // Roll
//     sign_error_roll = sign(-error_roll);
//     sign_errorp_roll = sign(-error_vroll);

//     // Pitch
//     sign_error_pitch = sign(-error_pitch);
//     sign_errorp_pitch = sign(-error_vpitch);

//     // Yaw
//     sign_error_yaw = sign(-error_yaw);
//     sign_errorp_yaw = sign(-error_vyaw);

//     Ganancias r = calcularGanancias(Rollzeta);
//     Ganancias p = calcularGanancias(Pitchzeta);
//     Ganancias y = calcularGanancias(Yawzeta);

//     float rollp_control = -r.k3  * sign_error_roll  - r.k4  * sign_errorp_roll;
//     roll_control += rollp_control * dt;
//     roll_control = clamp(roll_control,-10,10);
//     float rollOutput  = (-r.k1 * powf(fabs(error_roll),1.0f/3.0f)  * sign_error_roll   - r.k2  * powf(fabs(error_vroll),1.0f/2.0f)  * sign_errorp_roll + roll_control)*roll_kff;
//     rollOutput  = calcularRpm(rollOutput);

//     float pitchp_control = -p.k3 * sign_error_pitch - p.k4 * sign_errorp_pitch;
//     pitch_control += pitchp_control * dt;
//     pitch_control = clamp(pitch_control,-10,10);
//     float pitchOutput = (-p.k1 * powf(fabs(error_pitch),1.0f/3.0f) * sign_error_pitch - p.k2 * powf(fabs(error_vpitch),1.0f/2.0f) * sign_errorp_pitch + pitch_control)*pitch_kff;
//     pitchOutput = calcularRpm(pitchOutput);

//     float yawp_control = -y.k3 * sign_error_yaw - y.k4 * sign_errorp_yaw;
//     yaw_control += yawp_control * dt;
//     yaw_control = clamp(yaw_control,-10,10);
//     float yawOutput   = (-y.k1* powf(fabs(error_yaw),1.0f/3.0f) * sign_error_yaw - y.k2 * powf(fabs(error_vyaw),1.0f/2.0f) * sign_errorp_yaw + yaw_control)*yaw_kff;
//     yawOutput   = calcularRpm(yawOutput);

//     control->roll = clamp(rollOutput, -32000, 32000);
//     control->pitch = clamp(pitchOutput, -32000, 32000);
//     control->yaw = clamp(-yawOutput, -32000, 32000);
//     control->thrust = clamp(actuatorThrust,0,60000);

//     cmd_thrust = control->thrust;
//     cmd_roll = control->roll;
//     cmd_pitch = control->pitch;
//     cmd_yaw = control->yaw;

//   }

//   control->thrust = clamp(actuatorThrust,0,60000);

//   if (control->thrust == 0)
//   {
//     control->thrust = 0;
//     control->roll = 0;
//     control->pitch = 0;
//     control->yaw = 0;

//     controllerMellingerReset();

//     // Reset the calculated YAW angle for rate control
//     attitudeDesired.yaw = state->attitude.yaw;
//   }
// }

// /**
//  * Tunning variables for the full state Mellinger Controller
//  */
// PARAM_GROUP_START(ctrlMel)
// // roll
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Rollzeta, &Rollzeta)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff,  &roll_kff)
// // pitch
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Pitchzeta, &Pitchzeta)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff,  &pitch_kff)
// // yaw
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Yawzeta, &Yawzeta)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff,  &yaw_kff)

// PARAM_GROUP_STOP(ctrlMel)


// LOG_GROUP_START(ctrlMel)
// LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
// LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
// LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
// LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
// LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
// LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
// LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
// LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
// LOG_ADD(LOG_FLOAT, rollr,      &rateDesired.roll)
// LOG_ADD(LOG_FLOAT, pitchr,     &rateDesired.pitch)
// LOG_ADD(LOG_FLOAT, yawr,       &rateDesired.yaw)

// LOG_ADD(LOG_FLOAT, error_roll,&error_roll)
// LOG_ADD(LOG_FLOAT, error_pitch,&error_pitch)
// LOG_ADD(LOG_FLOAT, error_yaw,&error_yaw)

// LOG_ADD(LOG_FLOAT, i_error_roll,&i_error_roll)
// LOG_ADD(LOG_FLOAT, i_error_pitch,&i_error_pitch)
// LOG_ADD(LOG_FLOAT, i_error_yaw,&i_error_yaw)

// LOG_ADD(LOG_FLOAT, error_vroll,&error_vroll)
// LOG_ADD(LOG_FLOAT, error_vpitch,&error_vpitch)
// LOG_ADD(LOG_FLOAT, error_vyaw,&error_vyaw)

// LOG_ADD(LOG_FLOAT, sign_errorp_roll ,&sign_errorp_roll)
// LOG_ADD(LOG_FLOAT, sign_errorp_pitch,&sign_errorp_pitch)
// LOG_ADD(LOG_FLOAT, sign_errorp_yaw  ,&sign_errorp_yaw)

// LOG_ADD(LOG_FLOAT, sign_error_roll ,&sign_error_roll)
// LOG_ADD(LOG_FLOAT, sign_error_pitch,&sign_error_pitch)
// LOG_ADD(LOG_FLOAT, sign_error_yaw  ,&sign_error_yaw)

// LOG_GROUP_STOP(ctrlMel)
