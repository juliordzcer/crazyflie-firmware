/**
 * position_estimator_JCRC.c: Controlador de posicion PID 
 */

#include <math.h>
#include <float.h>

#include "stabilizer_types.h"
#include "controller_JCRC.h"
#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
#include "num.h"
#include "physicalConstants.h"



#define DT (float)(1.0f/POSITION_RATE)


static float kp_xy = 14.95f;
static float ki_xy = 3.0f;
static float kd_xy = 10.5f;

static float kp_z = 12.15f;
static float ki_z = 2.0f;
static float kd_z = 7.0f;

static float ie_x = 0.0f;
static float ie_y = 0.0f;
static float ie_z = 0.0f;

static float i_range_xy = 2.0;
static float i_range_z  = 0.4;

void controllerJCRCReset(void)
{
  ie_x = 0.0f;
  ie_y = 0.0f;
  ie_z = 0.0f;
}

void controllerJCRCInit(void)
{
  controllerJCRCReset();
}

void controllerJCRC(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state)
{

  struct vec e_xi;
  struct vec e_xip;
  struct vec nu;
  float dt;

  dt = DT;
  struct vec xid = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec xidp = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec xi = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec xip = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  // Position Error (ep)
  e_xi = vsub(xid, xi);

  // Velocity Error (ev)
  e_xip = vsub(xidp, xip);

  // Integral Error
  ie_x += e_xi.x * dt;
  ie_x = clamp(ie_x, -i_range_xy, i_range_xy);

  ie_y += e_xi.y * dt;
  ie_y = clamp(ie_y, -i_range_xy, i_range_xy);

  ie_z += e_xi.z * dt;
  ie_z = clamp(ie_z, -i_range_z, i_range_z);


  if (setpoint->mode.x == modeAbs) 
  {
    nu.x = kp_xy * e_xi.x + kd_xy * e_xip.x + ki_xy * ie_x + setpoint->acceleration.x;
    nu.y = kp_xy * e_xi.y + kd_xy * e_xip.y + ki_xy * ie_y + setpoint->acceleration.y;
    nu.z = kp_z  * e_xi.z + kd_z  * e_xip.z + ki_z  * ie_z + setpoint->acceleration.z;
  } 
  else 
  {
    nu.x = 0;
    nu.y = 0;
    nu.z = 0;
  }
  
  float cosyawd = cosf(setpoint->attitude.yaw * (float)M_PI / 180.0f);
  float sinyawd = sinf(setpoint->attitude.yaw * (float)M_PI / 180.0f);

  float u = sqrtf(powf(nu.x, 2.0f) + powf(nu.y, 2.0f) + powf((nu.z + 9.81f), 2.0f)) * 0.32f;
  float phi = asinf((nu.x * sinyawd - nu.y * cosyawd)*( 0.32f / u )) ;
  float theta = atanf((nu.x * cosyawd + nu.y * sinyawd) / (nu.z + 9.81f));      

  float u_rpm = clamp(calculate_rpm(u),10000.0f,60000.0f)*0.4f;

  attitude->roll  = clamp(phi,  -10, 10);
  attitude->pitch = clamp(theta, -10, 10);

  *thrust = u_rpm;

}

PARAM_GROUP_START(JCRC)
PARAM_ADD(PARAM_FLOAT, kp_xy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, ki_xy, &ki_xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &kd_xy)

PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, ki_z, &ki_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_GROUP_STOP(JCRC)
