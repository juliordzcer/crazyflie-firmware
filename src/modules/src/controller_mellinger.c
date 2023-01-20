/**
 * Code developed by Ing. Julio Cesar Rodriguez Cervantes in January 2023
 * More information see https://github.com/juliordzcer
 * TC.c - implementation of the Twisting Continuo
 */

#include <math.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_mellinger.h"
#include "attitude_controller.h"
#include "pid.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"
#include "commander.h"
#include "platform_defaults.h"

// Maximum roll/pitch angle permited
static float rLimit  = 20.0f;
static float pLimit  = 20.0f;
static float yLimit  = 360.0f;
static float tol  = 2.0f;

static float thrustBase = 36000.0f;
static float thrustMin  = 20000.0f;
static const float thrustScale = 1000.0f;

static float i_range_x = 2.0f;
static float i_range_y = 2.0f;
static float i_range_z  = 0.5f;

static float i_error_x;
static float i_error_y;
static float i_error_z;
static float error_x;
static float error_y;
static float error_z;
static float error_vx;
static float error_vy;
static float error_vz;

static float i_error_roll;
static float i_error_pitch;
static float i_error_yaw;
static float error_roll;
static float error_pitch;
static float error_yaw;
static float error_vroll;
static float error_vpitch;
static float error_vyaw;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float* thrust;
static float rollOutput;
static float pitchOutput;
static float yawOutput;
static bool isInit;

static float kp_x = 25.0f;
static float ki_x = 15.0f;
static float kd_x = 0.0f;

static float kp_y = 25.0f;
static float ki_y = 15.0f;
static float kd_y = 0.0f;

static float kp_z = 25.0f;
static float ki_z = 15.0f;
static float kd_z = 0.0f;


static float Rollzeta  = 600.0f;
static float roll_kff  = 10.0f;

static float Pitchzeta = 600.0f;
static float pitch_kff = 10.0f;

static float Yawzeta   = 0.0f;
static float yaw_kff   = 0.0f;

void controllerMellingerReset(void)
{
  error_x = 0;
  error_y = 0;
  error_z = 0;
  error_vx = 0;
  error_vy = 0;
  error_vz = 0;
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;

  i_error_roll = 0;
  i_error_pitch = 0;
  i_error_yaw = 0;
  error_roll = 0;
  error_pitch = 0;
  error_yaw = 0;
  error_vroll = 0;
  error_vpitch = 0;
  error_vyaw = 0;

}

void controllerMellingerInit(void)
{

  controllerMellingerReset();
}

bool controllerMellingerTest(void)
{
  return true;
  return isInit;

}


float Integral(const float in, const float iLimit , const float dt)
{
    float integ = 0.01;
     integ += in * dt;

    if(iLimit != 0)
    {
    	integ = clamp(integ, -iLimit, iLimit);
    }
    return integ;
}


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

void controllerMellinger(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  float dt;

  float Rollk1;
  float Rollk2;
  float Rollk3;
  float Rollk4;

  float Pitchk1;
  float Pitchk2;
  float Pitchk3;
  float Pitchk4;

  float Yawk1;
  float Yawk2;
  float Yawk3;
  float Yawk4;

  if (Rollzeta > 0)
    {
         Rollk3 =   7 *pow(Rollzeta ,(double) 2/3 );
         Rollk3 =   5 *pow(Rollzeta ,(double) 1/2 );
         Rollk3 = 2.3 * pow(Rollzeta , (double) 1 );
         Rollk4 = 1.1 * pow(Rollzeta , (double) 1 );
    }
  else 
    {
         Rollk1 = 0.0f;
         Rollk2 = 0.0f;
         Rollk3 = 0.0f;
         Rollk4 = 0.0f;
    }
  if (Pitchzeta > 0)
    {
         Pitchk1 =  7 * pow(Pitchzeta , (double) 2/3 );
         Pitchk2 =  5 * pow(Pitchzeta , (double) 1/2 );
         Pitchk3 = 2.3 * pow(Pitchzeta , (double) 1 );
         Pitchk4 = 1.1 * pow(Pitchzeta , (double) 1 );
    }
  else 
    {
         Pitchk1 = 0.0f;
         Pitchk2 = 0.0f;
         Pitchk3 = 0.0f;
         Pitchk4 = 0.0f;
    }

  if (Yawzeta  > 0)
    {
         Yawk1 =   7 * pow(Yawzeta , (double) 2/3 );
         Yawk2 =   5 * pow(Yawzeta , (double) 1/2 );
         Yawk3 = 2.3 * pow(Yawzeta , (double) 1 );
         Yawk4 = 1.1 * pow(Yawzeta , (double) 1 );
    }
    else 
    {
         Yawk1 = 0.0f;
         Yawk2 = 0.0f;
         Yawk3 = 0.0f;
         Yawk4 = 0.0f;
    }


  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) 
  {
    return;
  }

  
  dt = (float)(1.0f/ATTITUDE_RATE);

  error_x  =  state->position.x - setpoint->position.x;
  error_y  =  state->position.y - setpoint->position.y;
  error_z  =  state->position.z - setpoint->position.z;

  error_vx  =  state->velocity.x - setpoint->velocity.x;
  error_vy  =  state->velocity.y - setpoint->velocity.y;
  error_vz  =  state->velocity.z - setpoint->velocity.z;


  i_error_x += (error_x) * dt;
  i_error_x = clamp(i_error_x, -i_range_x, i_range_x);
  i_error_y += (error_y) * dt;
  i_error_y = clamp(i_error_y, -i_range_y, i_range_y);
  i_error_z += (error_z) * dt;
  i_error_z = clamp(i_error_z, -i_range_z, i_range_z);


  // Pitch
  // float pitch;
  // pitch = -(kp_x * error_x + ki_x * i_error_x + kd_x * error_vx) / CF_MASS;
  // setpoint->attitude.pitch = clamp(setpoint->attitude.pitch, -pLimit, pLimit);
  // Roll
  // float roll;
  // roll = (kp_y * error_y + ki_y * i_error_y + kd_y * error_vy) / CF_MASS;    
  // setpoint->attitude.roll = clamp(setpoint->attitude.roll, -rLimit, rLimit);

  // float stateAttitudeRateRoll = sensors->gyro.x;
  // float stateAttitudeRatePitch = -sensors->gyro.y;
  // float stateAttitudeRateYaw = sensors->gyro.z;

  error_roll   = state->attitude.roll  - setpoint->attitude.roll;
  error_pitch  = state->attitude.pitch - setpoint->attitude.pitch;
  error_yaw    = state->attitude.yaw   - setpoint->attitude.yaw;
  //   if (error_yaw > 180.0f)
  //   error_yaw += 360.0f;
  // else if (error_yaw < -180.0f)
  //   error_yaw -= 360.0f;

  error_vroll  = sensors->gyro.x  - setpoint->attitude.roll;
  error_vpitch = -sensors->gyro.y - setpoint->attitudeRate.pitch;
  error_vyaw   = sensors->gyro.z   - setpoint->attitude.yaw;

  i_error_roll  = Integral(error_roll  ,rLimit,dt);
  i_error_pitch = Integral(error_pitch ,pLimit,dt);
  i_error_yaw =   Integral(error_yaw   ,yLimit,dt);

  // Thrustk
  float thrustRaw = (kp_z * error_z + ki_y * i_error_z + kd_y * error_vz);    
  
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + thrustBase;
  // Check for minimum thrust
  if (*thrust < thrustMin) 
  {
    *thrust = thrustMin;
  }
  // saturate
  *thrust = clamp(*thrust, 0, UINT16_MAX);

// Sign Function
  // Roll
  float sign_error_roll = 0.0;
  float sign_errorp_roll = 0.0;
  if (error_roll < -tol)
  {
    sign_error_roll = -1.0;
  }
  else if (error_roll > tol)
  {
    sign_error_roll = 1.0;
  }
  else
  {
    sign_error_roll = 0.0;
  }  

  if (error_vroll < -tol)
  {
    sign_errorp_roll = -1.0;
  }
  else if (error_vroll > tol)
  {
    sign_errorp_roll = 1.0;
  }
  else 
  {
    sign_errorp_roll = 0.0;
  }

  // Pitch
  float sign_error_pitch = 0.0;
  float sign_errorp_pitch = 0.0;
  if (error_pitch < -tol)
  {
    sign_error_pitch = -1.0;
  }
  else if (error_pitch > tol)
  {
    sign_error_pitch = 1.0;
  }
  else
  {
    sign_error_pitch = 0.0;
  }
  if (error_vpitch < -tol)
  {
    sign_errorp_pitch = -1.0;
  }
  else if (error_vpitch > tol)
  {
    sign_errorp_pitch = 1.0;
  }
  else 
  {
    sign_errorp_pitch = 0.0;
  }
  // Yaw
  float sign_error_yaw = 0.0;
  float sign_errorp_yaw = 0.0;
  if (error_yaw < -tol)
  {
    sign_error_yaw = -1.0;
  }
  else if (error_yaw >tol)
  {
    sign_error_yaw = 1.0;
  }
  else
  {
    sign_error_yaw = 0.0;
  }
  if (error_vyaw < -tol)
  {
    sign_errorp_yaw = -1.0;
  }
  else if (error_vyaw > tol)
  {
    sign_errorp_yaw = 1.0;
  }
  else
  {
    sign_errorp_yaw = 0.0;
  }

  rollOutput = (float)(-Rollk1   * (float)pow((float)fabs(error_roll),(double)1/3)  * sign_error_roll  - (float)Rollk2  * (float)pow((float)fabs(error_vroll),(double)1/2)  * sign_errorp_roll +  Integral(-(float)Rollk3  * sign_error_roll  - (float)Rollk4  * sign_errorp_roll , 32000, dt))  * roll_kff;
  rollOutput = saturateSignedInt16(rollOutput);

  pitchOutput = (float)(-Pitchk1 * (float)pow((float)fabs(error_pitch),(double)1/3) * sign_error_pitch - (float)Pitchk2 * (float)pow((float)fabs(error_vpitch),(double)1/2) * sign_errorp_pitch + Integral(-(float)Pitchk3 * sign_error_pitch - (float)Pitchk4 * sign_errorp_pitch, 32000, dt))  * pitch_kff ;
  pitchOutput = 0;//saturateSignedInt16(pitchOutput);

  yawOutput = (float)(-Yawk1* (float)pow((float)fabs(error_yaw),(double)1/3) * sign_error_yaw - (float)Yawk2 * (float)pow((float)fabs(error_vyaw),(double)1/2) * sign_errorp_yaw + Integral( -(float)Yawk3 * sign_error_yaw - (float)Yawk4 * sign_errorp_yaw , 32000, dt)) * yaw_kff;
  yawOutput = 0;//saturateSignedInt16(yawOutput);


  if (setpoint->mode.z == modeDisable) 
  {
    control->thrust = setpoint->thrust;
  } 
  else 
  {
    control->thrust = *thrust;
  }

  cmd_thrust = control->thrust;

  if (control->thrust > 0) {
    control->roll = rollOutput;
    control->pitch = pitchOutput;
    control->yaw = yawOutput;

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    controllerMellingerReset();
  }

}

/**
 * Tunning variables for the full state Mellinger Controller
 */
PARAM_GROUP_START(ctrlMel)
// x
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, x_kp, &kp_x)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, x_ki, &ki_x)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, x_kd, &kd_x)
// y
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, y_kp, &kp_y)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, y_ki, &ki_y)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, y_kd, &kd_y)
// z
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, z_kp, &kp_z)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, z_ki, &ki_z)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, z_kd, &kd_z)
// roll
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_zeta, &Rollzeta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &roll_kff)
// pitch
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_zeta, &Pitchzeta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pitch_kff)
// yaw
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_zeta, &Yawzeta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &yaw_kff)
// tolerancia 
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, tol, &tol)
PARAM_GROUP_STOP(ctrlMel)

/**
 * Logging variables for the command and reference signals for the
 * Mellinger controller
 */
LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, i_error_roll, &i_error_roll)
LOG_ADD(LOG_FLOAT, i_error_pitch, &i_error_pitch)
LOG_ADD(LOG_FLOAT, i_error_yaw, &i_error_yaw)
LOG_ADD(LOG_FLOAT, error_roll, &error_roll)
LOG_ADD(LOG_FLOAT, error_pitch, &error_pitch)
LOG_ADD(LOG_FLOAT, error_yaw, &error_yaw)
LOG_ADD(LOG_FLOAT, error_vroll, &error_vroll)
LOG_ADD(LOG_FLOAT, error_vpitch, &error_vpitch)
LOG_ADD(LOG_FLOAT, error_vyaw, &error_vyaw)
LOG_GROUP_STOP(ctrlMel)

