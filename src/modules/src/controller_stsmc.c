#include <math.h>

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "position_controller.h"
#include "controller_stsmc.h"


#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Ganancias del controlador de orientacion.

// static float k0_phi = 0.20f; 
// static float k1_phi = 8.0f;
// static float k2_phi = 8.50f;

// static float k0_theta = 0.20f; 
// static float k1_theta = 8.0f;
// static float k2_theta = 8.50f;

// static float k0_psi = 0.10f;
// static float k1_psi = 10.0f;
// static float k2_psi = 8.0f;

static float k0_phi = 0.0035f;
static float zeta_phi = 0.875f; 

static float k0_theta = 0.0035f;
static float zeta_theta = 0.875f; 

static float k0_psi = 0.0027f;
static float zeta_psi = 0.76f; 

static float iephi = 0;
static float ietheta = 0;
static float iepsi = 0;

static float nu_phi = 0;
static float nu_theta = 0;
static float nu_psi = 0;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

void controllerstsmcReset(void)
{
  iephi = 0;
  ietheta = 0;
  iepsi = 0;

  nu_phi = 0;
  nu_theta = 0;
  nu_psi = 0;
  
  attitudeControllerResetAllPID();
  positionControllerResetAllPID();
}

void controllerstsmcInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  controllerstsmcReset();
}

bool controllerstsmcTest(void)
{
  bool pass = true;
  pass &= attitudeControllerTest();
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

void controllerstsmc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  control->controlMode = controlModeLegacy;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
       
      float yawMaxDelta = attitudeControllerGetYawMaxDelta();
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

  // Control de posicion
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

    // Se obtiene la velocidad deseada en grados

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    float dt = ATTITUDE_UPDATE_DT;

    // float k0_theta = k0_phi;
    // float k1_theta = k1_phi;
    // float k2_theta = k2_phi;

    float k1_phi = 1.5f*powf(zeta_phi,1.0f/2.0f);
    float k2_phi = 1.1f*zeta_phi;

    float k1_theta = 1.5f*powf(zeta_theta,1.0f/2.0f);
    float k2_theta = 1.1f*zeta_theta;

    float k1_psi = 1.5f*powf(zeta_psi,1.0f/2.0f);
    float k2_psi = 1.1f*zeta_psi;

    // Conversion de a radianes
    float phid   = radians(attitudeDesired.roll);
    float thetad = radians(attitudeDesired.pitch);
    float psid   = radians(attitudeDesired.yaw);

    float phidp   = radians(rateDesired.roll);
    float thetadp = radians(rateDesired.pitch);
    float psidp   = radians(rateDesired.yaw);

    float phi   = radians(state->attitude.roll);
    float theta = radians(state->attitude.pitch);
    float psi   = radians(state->attitude.yaw);

    float phip   = radians(sensors->gyro.x);
    float thetap = -radians(sensors->gyro.y);
    float psip   = radians(sensors->gyro.z);

    // Error de orientacion.
    float ephi   = phid - phi;
    float etheta = thetad - theta;
    float epsi   = psid - psi;    
    
    // Error de velocidad angular
    float ephip   = phidp - phip;
    float ethetap = thetadp - thetap;
    float epsip   = psidp - psip;

    // Control de Phi 
    float S_phi = ephip + k0_phi * powf(fabsf(ephi), 2.0f / 3.0f) * sign(ephi);
    nu_phi += (k2_phi * sign(S_phi)) * dt;
    float tau_bar_phi = nu_phi + k1_phi * powf(fabsf(S_phi), 1.0f/2.0f) * sign(S_phi);
    float tau_phi   =  tau_bar_phi * 10000.0f;

    // Control de Theta
    float S_theta = ethetap + k0_theta * powf(fabsf(etheta), 2.0f / 3.0f) * sign(etheta);
    nu_theta += (k2_theta * sign(S_theta)) * dt;
    float tau_bar_theta = k1_theta * powf(fabsf(S_theta), 1.0f/2.0f) * sign(S_theta) + nu_theta;
    float tau_theta = tau_bar_theta  * 10000.0f;

    // Control de Psi
    float S_psi = epsip + k0_psi * powf(fabsf(epsi), 2.0f / 3.0f) * sign(epsi);
    nu_psi += (k2_psi * sign(S_psi)) * dt;
    float tau_bar_psi = k1_psi * powf(fabsf(S_psi), 1.0f/2.0f) * sign(S_psi) + nu_psi;
    float tau_psi   = tau_bar_psi * 10000.0f;

    control->roll = clamp((tau_phi), -32000, 32000);
    control->pitch = clamp((tau_theta), -32000, 32000);
    control->yaw = clamp((tau_psi), -32000, 32000);
    
    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

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

    controllerstsmcReset();

    attitudeDesired.yaw = state->attitude.yaw;
  }
}
PARAM_GROUP_START(STSMC)
// PARAM_ADD(PARAM_FLOAT, k0_phi, &k0_phi)
// PARAM_ADD(PARAM_FLOAT, k1_phi, &k1_phi)
// PARAM_ADD(PARAM_FLOAT, k2_phi, &k2_phi)


// PARAM_ADD(PARAM_FLOAT, k0_theta, &k0_theta)
// PARAM_ADD(PARAM_FLOAT, zeta_theta, &zeta_theta)

// PARAM_ADD(PARAM_FLOAT, k0_psi, &k0_psi)
// PARAM_ADD(PARAM_FLOAT, k1_psi, &k1_psi)
// PARAM_ADD(PARAM_FLOAT, k2_psi, &k2_psi)

PARAM_ADD(PARAM_FLOAT, zeta_phi, &zeta_phi)
PARAM_ADD(PARAM_FLOAT, k0_phi, &k0_phi)
PARAM_ADD(PARAM_FLOAT, zeta_theta, &zeta_theta)
PARAM_ADD(PARAM_FLOAT, k0_theta, &k0_theta)
PARAM_ADD(PARAM_FLOAT, zeta_psi, &zeta_psi)
PARAM_ADD(PARAM_FLOAT, k0_psi, &k0_psi)
PARAM_GROUP_STOP(STSMC)

LOG_GROUP_START(STSMC)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(STSMC)