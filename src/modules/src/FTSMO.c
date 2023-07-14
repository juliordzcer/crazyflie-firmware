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

// static float D_phi = 0;
// // static float D_theta = 0;
// static float D_psi = 0;

static float k1_phi = 8.0f;
static float k2_phi = 800.0f;
static float k3_phi = 400.0f;

static float k1_theta = 8.0f;
static float k2_theta = 800.0f;
static float k3_theta = 400.0f;

static float k1_psi = 0.0f;
static float k2_psi = 0.0f;
static float k3_psi = 0.0f;

static float dt_obs = 0.01f;

static float phihat1 = 0.0f;
static float phihat2 = 0.0f;
static float phihat3 = 0.0f;

static float thetahat1 = 0.0f;
static float thetahat2 = 0.0f;
static float thetahat3 = 0.0f;

static float psihat1 = 0.0f;
static float psihat2 = 0.0f;
static float psihat3 = 0.0f;

static float ks = 1.0f;


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

    // float k1_phi = 2.0f*powf(D_phi, 1.0f/3.0f);
    // float k2_phi = 1.5f*powf(D_phi, 1.0f/2.0f);
    // float k3_phi = 1.1f*D_phi;

    // float k1_theta = 2.0f*powf(D_phi, 1.0f/3.0f);
    // float k2_theta = 1.5f*powf(D_phi, 1.0f/2.0f);
    // float k3_theta = 1.1f*D_phi;

    // // float k1_theta = 2.0f*powf(D_theta, 1.0f/3.0f);
    // // float k2_theta = 1.5f*powf(D_theta, 1.0f/2.0f);
    // // float k3_theta = 1.1f*D_theta;

    // float k1_psi = 2.0f*powf(D_psi, 1.0f/3.0f);
    // float k2_psi = 1.5f*powf(D_psi, 1.0f/2.0f);
    // float k3_psi = 1.1f*D_psi;


    float phi   = (eulerRollActual);
    float theta = (eulerPitchActual);
    float psi   = (eulerYawActual);

    // Cálculos para phi
    float ephi1 = (phi - phihat1);
    phihat1 = (phihat1 + (phihat2 + k1_phi * powf(fabsf(ephi1), 2.0f / 3.0f) * sign(ephi1)) * dt_obs);
    phihat2 = (phihat2 + (phihat3 + k2_phi * powf(fabsf(ephi1), 1.0f / 3.0f) * sign(ephi1)) * dt_obs);
    phihat3 = (phihat3 + (k3_phi * sign(ephi1)) * dt_obs);

    // Cálculos para theta
    float etheta1 = (theta - thetahat1);
    thetahat1 = (thetahat1 + (thetahat2 + k1_theta * powf(fabsf(etheta1), 2.0f / 3.0f) * sign(etheta1)) * dt_obs);
    thetahat2 = (thetahat2 + (thetahat3 + k2_theta * powf(fabsf(etheta1), 1.0f / 3.0f) * sign(etheta1)) * dt_obs);
    thetahat3 = (thetahat3 + (k3_theta * sign(etheta1)) * dt_obs);

    // Cálculos para psi
    float epsi1 = (psi - psihat1);
    psihat1 = (psihat1 + (psihat2 + k1_psi * powf(fabsf(epsi1), 2.0f / 3.0f) * sign(epsi1)) * dt_obs);
    psihat2 = (psihat2 + (psihat3 + k2_psi * powf(fabsf(epsi1), 1.0f / 3.0f) * sign(epsi1)) * dt_obs);
    psihat3 = (psihat3 + (k3_psi * sign(epsi1)) * dt_obs);

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


// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, D_phi, &D_phi)
// // PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, D_theta, &D_theta)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, D_psi, &D_psi)


PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_phi, &k1_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_phi, &k2_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_phi, &k3_phi)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_theta, &k1_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_theta, &k2_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_theta, &k3_theta)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k1_psi, &k1_psi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k2_psi, &k2_psi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k3_psi, &k3_psi)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, ks, &ks)


PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, dt_obs, &dt_obs)

PARAM_GROUP_STOP(FTSMO)
