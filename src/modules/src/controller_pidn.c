#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pidn.h"


#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
#include "num.h"
#include <math.h>
#include <float.h>

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static float kp_phi = 8.0f;
static float ki_phi = 2.0f;
static float kd_phi = 8.0f;

static float kp_theta = 8.0f;
static float ki_theta = 2.0f;
static float kd_theta = 8.0f;

static float kp_psi = 4.0f;
static float ki_psi = 1.0f;
static float kd_psi = 12.0f;

static float ks = 1000.0f;

static float iephi = 0.0f;
static float ietheta = 0.0f;
static float iepsi = 0.0f;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float o_roll;
static float o_pitch;
static float o_yaw;

static float cmd_roll_n;
static float cmd_pitch_n;
static float cmd_yaw_n;

// Observadore

static float L_phi = 3.0f;
static float L_theta = 3.0f;
static float L_psi = 3.0f;

static float phihat1 = 0.0f;
static float phihat2 = 0.0f;
static float phihat3 = 0.0f;

static float thetahat1 = 0.0f;
static float thetahat2 = 0.0f;
static float thetahat3 = 0.0f;

static float psihat1 = 0.0f;
static float psihat2 = 0.0f;
static float psihat3 = 0.0f;



void controllerpidnReset(void)
{
  iephi = 0;
  ietheta = 0;
  iepsi = 0;

  phihat1 = 0.0f;
  phihat2 = 0.0f;
  phihat3 = 0.0f;

  thetahat1 = 0.0f;
  thetahat2 = 0.0f;
  thetahat3 = 0.0f;

  psihat1 = 0.0f;
  psihat2 = 0.0f;
  psihat3 = 0.0f;

  attitudeControllerResetAllPID();
  positionControllerResetAllPID();
}

void controllerpidnInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  controllerpidnReset();
}

bool controllerpidnTest(void)
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

void controllerpidn(control_t *control, const setpoint_t *setpoint,
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

    float dt = ATTITUDE_UPDATE_DT;

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

    // Observador
    float k1_phi = 2.0f * cbrtf(L_phi);
    float k2_phi = 1.5f * sqrtf(L_phi);
    float k3_phi = 1.1f * L_phi;

    float k1_theta = 2.0f * cbrtf(L_theta);
    float k2_theta = 1.5f * sqrtf(L_theta);
    float k3_theta = 1.1f * L_theta;

    float k1_psi = 2.0f * cbrtf(L_psi);
    float k2_psi = 1.5f * sqrtf(L_psi);
    float k3_psi = 1.1f * L_psi;

    // Cálculos para phi
    float ephi1 = phi - phihat1;
    phihat1 = phihat1 + (phihat2 + k1_phi * powf(fabsf(ephi1), 2.0f / 3.0f) * (ephi1 < 0 ? -1.0f : 1.0f)) * dt;
    phihat2 = phihat2 + (phihat3 + k2_phi * powf(fabsf(ephi1), 1.0f / 3.0f) * (ephi1 < 0 ? -1.0f : 1.0f)) * dt;
    phihat3 = phihat3 + (k3_phi * (ephi1 < 0 ? -1.0f : 1.0f)) * dt;

    // Cálculos para theta
    float etheta1 = theta - thetahat1;
    thetahat1 = thetahat1 + (thetahat2 + k1_theta * powf(fabsf(etheta1), 2.0f / 3.0f) * (etheta1 < 0 ? -1.0f : 1.0f)) * dt;
    thetahat2 = thetahat2 + (thetahat3 + k2_theta * powf(fabsf(etheta1), 1.0f / 3.0f) * (etheta1 < 0 ? -1.0f : 1.0f)) * dt;
    thetahat3 = thetahat3 + (k3_theta * (etheta1 < 0 ? -1.0f : 1.0f)) * dt;

    // Cálculos para psi
    float epsi1 = psi - psihat1;
    psihat1 = psihat1 + (psihat2 + k1_psi * powf(fabsf(epsi1), 2.0f / 3.0f) * (epsi1 < 0 ? -1.0f : 1.0f)) * dt;
    psihat2 = psihat2 + (psihat3 + k2_psi * powf(fabsf(epsi1), 1.0f / 3.0f) * (epsi1 < 0 ? -1.0f : 1.0f)) * dt;
    psihat3 = psihat3 + (k3_psi * (epsi1 < 0 ? -1.0f : 1.0f)) * dt;

    // attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
    //                             attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
    //                             &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

 
    rateDesired.roll  = phihat2;
    rateDesired.pitch = thetahat2;
    rateDesired.yaw   = psihat2;


    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }


    // Errores de orientacion [Rad].
    float ephi   = phid - phi;
    float etheta = thetad - theta;
    float epsi   = psid - psi;    
       
    // Integral del error de orientacion.
    iephi   = iephi + ephi * dt;
    iephi = clamp(iephi, -1,1);
    ietheta = ietheta + etheta * dt;
    ietheta = clamp(ietheta, -1,1);
    iepsi   = iepsi + epsi * dt;
    iepsi = clamp(iepsi, -1.5,1.5);
    
    // Error de velocidad angular
    float ephip   = phidp - phip;
    float ethetap = thetadp - thetap;
    float epsip   = psidp - psip; 

    // float Jx = 16.6e-6f;
    // float Jy = 16.6e-6f;
    // float Jz = 29.3e-6f;

    // Controlador Phi
    float tau_bar_phi   = kp_phi * ephi + ki_phi * iephi + kd_phi * ephip;
    float tau_phi   = tau_bar_phi;
    // float tau_phi   = (Jx * ( tau_bar_phi - ((Jy-Jz)/Jx) * thetap * psip)) * ks;

    // Controlador Theta
    float tau_bar_theta = kp_theta * etheta + ki_theta * ietheta + kd_theta * ethetap;
    float tau_theta = tau_bar_theta;
    // float tau_theta = (Jy * ( tau_bar_theta - ((Jz-Jx)/Jy) * phip * psip))* ks;

    // Controlador Psi
    float tau_bar_psi   = kp_psi * epsi + ki_psi * iepsi + kd_psi * epsip;
    float tau_psi = tau_bar_psi;
    // float tau_psi   = (Jz * ( tau_bar_psi - ((Jx-Jy)/Jz) * thetap * phip))* ks;

    control->roll = clamp(calculate_rpm(tau_phi), -32000, 32000);
    control->pitch = clamp(calculate_rpm(tau_theta), -32000, 32000);
    control->yaw = clamp(calculate_rpm(tau_psi), -32000, 32000);
    
    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    cmd_roll_n  = tau_phi;
    cmd_pitch_n = tau_theta;
    cmd_yaw_n   = tau_psi;

    o_roll = degrees(phihat2);
    o_pitch = degrees(thetahat2);
    o_yaw = degrees(psihat2);

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

    controllerpidnReset();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

PARAM_GROUP_START(PIDN)
PARAM_ADD(PARAM_FLOAT, kp_phi, &kp_phi)
PARAM_ADD(PARAM_FLOAT, ki_phi, &ki_phi)
PARAM_ADD(PARAM_FLOAT, kd_phi, &kd_phi)

PARAM_ADD(PARAM_FLOAT, kp_theta, &kp_theta)
PARAM_ADD(PARAM_FLOAT, ki_theta, &ki_theta)
PARAM_ADD(PARAM_FLOAT, kd_theta, &kd_theta)

PARAM_ADD(PARAM_FLOAT, kp_psi, &kp_psi)
PARAM_ADD(PARAM_FLOAT, ki_psi, &ki_psi)
PARAM_ADD(PARAM_FLOAT, kd_psi, &kd_psi)
PARAM_GROUP_STOP(PIDN)

LOG_GROUP_START(PIDN)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, cmd_roll_n, &cmd_roll_n)
LOG_ADD(LOG_FLOAT, cmd_pitch_n, &cmd_pitch_n)
LOG_ADD(LOG_FLOAT, cmd_yaw_n, &cmd_yaw_n)
LOG_GROUP_STOP(PIDN)


 
LOG_GROUP_START(FTSMO)

LOG_ADD(LOG_FLOAT, o_roll, &o_roll)
LOG_ADD(LOG_FLOAT, o_pitch, &o_pitch)
LOG_ADD(LOG_FLOAT, o_yaw, &o_yaw)

LOG_ADD(LOG_FLOAT, phihat1, &phihat1)
LOG_ADD(LOG_FLOAT, phihat2, &phihat2)
LOG_ADD(LOG_FLOAT, phihat3, &phihat3)

LOG_ADD(LOG_FLOAT, thetahat1, &thetahat1)
LOG_ADD(LOG_FLOAT, thetahat2, &thetahat2)
LOG_ADD(LOG_FLOAT, thetahat3, &thetahat3)

LOG_ADD(LOG_FLOAT, psihat1, &psihat1)
LOG_ADD(LOG_FLOAT, psihat2, &psihat2)
LOG_ADD(LOG_FLOAT, psihat3, &psihat3)
LOG_GROUP_STOP(FTSMO)

PARAM_GROUP_START(FTSMO)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_phi, &L_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_theta, &L_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_psi, &L_psi)

PARAM_GROUP_STOP(FTSMO)
