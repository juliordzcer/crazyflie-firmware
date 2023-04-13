#include <math.h>

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "position_controller.h"
#include "controller_tc.h"


#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Ganancias TC

static float zeta_phi   = 0.000261;  //0.00165f;
static float zeta_theta = 0.000261;  //0.00165f;
static float zeta_psi   = 0.00028;  //0.0018f;

static int gain = 2;

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

static float cmd_roll_n;
static float cmd_pitch_n;
static float cmd_yaw_n;



void controllertcReset(void)
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

void controllertcInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  controllertcReset();
}

bool controllertcTest(void)
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

void controllertc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
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
    float k0_phi, k1_phi, k2_phi, k3_phi;
    float k0_theta, k1_theta, k2_theta, k3_theta;
    float k0_psi, k1_psi, k2_psi, k3_psi;
    
     switch(gain) {
      case 1:
        k0_phi = 25.0f*powf(zeta_phi,2.0f/3.0f);
        k1_phi = 15.0f*powf(zeta_phi,1.0f/2.0f);
        k2_phi = 2.3f*zeta_phi;
        k3_phi = 1.1f*zeta_phi;

        k0_theta = 25.0f*powf(zeta_theta,2.0f/3.0f);
        k1_theta = 15.0f*powf(zeta_theta,1.0f/2.0f);
        k2_theta = 2.3f*zeta_theta;
        k3_theta = 1.1f*zeta_theta;

        k0_psi = 25.0f*powf(zeta_psi,2.0f/3.0f);
        k1_psi = 15.0f*powf(zeta_psi,1.0f/2.0f);
        k2_psi = 2.3f*zeta_psi;
        k3_psi = 1.1f*zeta_psi;
        break;
      case 2:
        k0_phi = 19.0f*powf(zeta_phi,2.0f/3.0f);
        k1_phi = 10.0f*powf(zeta_phi,1.0f/2.0f);
        k2_phi = 2.3f*zeta_phi;
        k3_phi = 1.1f*zeta_phi;

        k0_theta = 19.0f*powf(zeta_theta,2.0f/3.0f);
        k1_theta = 10.0f*powf(zeta_theta,1.0f/2.0f);
        k2_theta = 2.3f*zeta_theta;
        k3_theta = 1.1f*zeta_theta;

        k0_psi = 19.0f*powf(zeta_psi,2.0f/3.0f);
        k1_psi = 10.0f*powf(zeta_psi,1.0f/2.0f);
        k2_psi = 2.3f*zeta_psi;
        k3_psi = 1.1f*zeta_psi;
        break;
      case 3:
        k0_phi = 13.0f*powf(zeta_phi,2.0f/3.0f);
        k1_phi = 10.0f*powf(zeta_phi,1.0f/2.0f);
        k2_phi = 2.3f*zeta_phi;
        k3_phi = 1.1f*zeta_phi;

        k0_theta = 13.0f*powf(zeta_theta,2.0f/3.0f);
        k1_theta = 10.0f*powf(zeta_theta,1.0f/2.0f);
        k2_theta = 2.3f*zeta_theta;
        k3_theta = 1.1f*zeta_theta;

        k0_psi = 7.0f*powf(zeta_psi,2.0f/3.0f);
        k1_psi = 5.0f*powf(zeta_psi,1.0f/2.0f);
        k2_psi = 2.3f*zeta_psi;
        k3_psi = 1.1f*zeta_psi;
        break;
      case 4:
        k0_phi = 7.0f*powf(zeta_phi,2.0f/3.0f);
        k1_phi = 5.0f*powf(zeta_phi,1.0f/2.0f);
        k2_phi = 2.3f*zeta_phi;
        k3_phi = 1.1f*zeta_phi;

        k0_theta = 7.0f*powf(zeta_theta,2.0f/3.0f);
        k1_theta = 5.0f*powf(zeta_theta,1.0f/2.0f);
        k2_theta = 2.3f*zeta_theta;
        k3_theta = 1.1f*zeta_theta;

        k0_psi = 7.0f*powf(zeta_psi,2.0f/3.0f);
        k1_psi = 5.0f*powf(zeta_psi,1.0f/2.0f);
        k2_psi = 2.3f*zeta_psi;
        k3_psi = 1.1f*zeta_psi;
        break;
      default:
        k0_phi = 0.0f;
        k1_phi = 0.0f;
        k2_phi = 0.0f;
        k3_phi = 0.0f;

        k0_theta = 0.0f;
        k1_theta = 0.0f;
        k2_theta = 0.0f;
        k3_theta = 0.0f;

        k0_psi = 0.0f;
        k1_psi = 0.0f;
        k2_psi = 0.0f;
        k3_psi = 0.0f;
        break;
   }

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
    float thetap = radians(-sensors->gyro.y);
    float psip   = radians(sensors->gyro.z);

    // Errores de orientacion [Rad].

    // Error de orientacion.
    float ephi   = phid - phi;
    float etheta = thetad - theta;
    float epsi   = psid - psi;    
    
    // Error de velocidad angular
    float ephip   = phidp - phip;
    float ethetap = thetadp - thetap;
    float epsip   = psidp - psip; 

    // Control de Phi 
    nu_phi += (k2_phi * sign(ephi) + k3_phi * sign(ephip)) * dt;
    float tau_phi_n = nu_phi + k0_phi * powf(fabsf(ephi), 1.0f/3.0f) * sign(ephi) + k1_phi * powf(fabsf(ephip), 1.0f/2.0f) * sign(ephip);

    // Control de theta 
    nu_theta += (k2_theta * sign(etheta) + k3_theta * sign(ethetap)) * dt;
    float tau_theta_n = nu_theta + k0_theta * powf(fabsf(etheta), 1.0f/3.0f) * sign(etheta) + k1_theta * powf(fabsf(ethetap), 1.0f/2.0f) * sign(ethetap);

    // Control de psi 
    nu_psi += (k2_psi * sign(epsi) + k3_psi * sign(epsip)) * dt;
    float tau_psi_n = nu_psi + k0_psi * powf(fabsf(epsi), 1.0f/3.0f) * sign(epsi) + k1_psi * powf(fabsf(epsip), 1.0f/2.0f) * sign(epsip);

    control->roll = clamp(calculate_rpm(tau_phi_n), -32000, 32000);
    control->pitch = clamp(calculate_rpm(tau_theta_n), -32000, 32000);
    control->yaw = clamp(calculate_rpm(tau_psi_n), -32000, 32000);
    
    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    cmd_roll_n = tau_phi_n;
    cmd_pitch_n = tau_theta_n;
    cmd_yaw_n = tau_psi_n;

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


    controllertcReset();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

PARAM_GROUP_START(Twisting)
PARAM_ADD(PARAM_FLOAT, zeta_phi, &zeta_phi)
PARAM_ADD(PARAM_FLOAT, zeta_theta, &zeta_theta)
PARAM_ADD(PARAM_FLOAT, zeta_psi, &zeta_psi)
PARAM_ADD_CORE(PARAM_UINT8, gain, &gain)
PARAM_GROUP_STOP(Twisting)

LOG_GROUP_START(Twisting)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, cmd_roll_n, &cmd_roll_n)
LOG_ADD(LOG_FLOAT, cmd_pitch_n, &cmd_pitch_n)
LOG_ADD(LOG_FLOAT, cmd_yaw_n, &cmd_yaw_n)
LOG_GROUP_STOP(Twisting)