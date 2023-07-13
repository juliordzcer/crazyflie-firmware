#include "stabilizer_types.h"
#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
#include "num.h"
#include <math.h>
#include <float.h>

#include "FTSMO.h"

static float k1_phi = 5.1299f;
static float k2_phi = 58.3541f;
static float k3_phi = 55.5f;

static float k1_theta = 5.1299f;
static float k2_theta = 58.3541;
static float k3_theta = 55.5f;

static float k1_psi = 0.0f;
static float k2_psi = 0.0f;
static float k3_psi = 0.0f;

static float dt_obs = 0.001f;


static float phihat1 = 0.0f;
static float phihat2 = 0.0f;
static float phihat3 = 0.0f;

static float thetahat1 = 0.0f;
static float thetahat2 = 0.0f;
static float thetahat3 = 0.0f;

static float psihat1 = 0.0f;
static float psihat2 = 0.0f;
static float psihat3 = 0.0f;


void FTSMOReset(void)
{
  phihat1 = 0.0f;
  phihat2 = 0.0f;
  phihat3 = 0.0f;

  thetahat1 = 0.0f;
  thetahat2 = 0.0f;
  thetahat3 = 0.0f;

  psihat1 = 0.0f;
  psihat2 = 0.0f;
  psihat3 = 0.0f;
}

void FTSMOInit(void)
{
  FTSMOReset();
}

bool FTSMOTest(void)
{
  bool pass = true;
  return pass;
}

void FTSMO( float eulerRollActual, float eulerPitchActual, float eulerYawActual,
            float* phi2, float* theta2, float* psi2)
{
    float phi   = radians(eulerRollActual);
    float theta = radians(eulerPitchActual);
    float psi   = radians(eulerYawActual);

    // Cálculos para phi
    float ephi1 = (phi - phihat1);
    phihat1 = phihat1 + (phihat2 + k1_phi * powf(fabsf(ephi1), 2.0f / 3.0f) * (ephi1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    phihat2 = phihat2 + (phihat3 + k2_phi * powf(fabsf(ephi1), 1.0f / 3.0f) * (ephi1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    phihat3 = phihat3 + (k3_phi * (ephi1 < 0 ? -1.0f : 1.0f)) * dt_obs;

    // Cálculos para theta
    float etheta1 = theta - thetahat1;
    thetahat1 = thetahat1 + (thetahat2 + k1_theta * powf(fabsf(etheta1), 2.0f / 3.0f) * (etheta1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    thetahat2 = thetahat2 + (thetahat3 + k2_theta * powf(fabsf(etheta1), 1.0f / 3.0f) * (etheta1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    thetahat3 = thetahat3 + (k3_theta * (etheta1 < 0 ? -1.0f : 1.0f)) * dt_obs;

    // Cálculos para psi
    float epsi1 = psi - psihat1;
    psihat1 = psihat1 + (psihat2 + k1_psi * powf(fabsf(epsi1), 2.0f / 3.0f) * (epsi1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    psihat2 = psihat2 + (psihat3 + k2_psi * powf(fabsf(epsi1), 1.0f / 3.0f) * (epsi1 < 0 ? -1.0f : 1.0f)) * dt_obs;
    psihat3 = psihat3 + (k3_psi * (epsi1 < 0 ? -1.0f : 1.0f)) * dt_obs;

    *phi2 = phihat2;
    *theta2 = thetahat2;
    *psi2 = psihat2;
}

LOG_GROUP_START(FTSMO)

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

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_phi, &k1_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_phi, &k2_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_phi, &k3_phi)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_theta, &k1_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_theta, &k2_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_theta, &k3_theta)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_psi, &k1_psi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_psi, &k2_psi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_psi, &k3_psi)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, dt_obs, &dt_obs)

PARAM_GROUP_STOP(FTSMO)
