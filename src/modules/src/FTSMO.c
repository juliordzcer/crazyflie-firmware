#include "stabilizer_types.h"

#include "commander.h"
#include "platform_defaults.h"
#include "log.h"
#include "param.h"
#include "math3d.h"

#define DT (float)(1.0f/ATTITUDE_RATE)

// Ganancias de los Observadores
static float L_x = 2.5f;
static float L_x = 2.5f;
static float L_x = 2.5f;

static float L_phi = 3.0f;
static float L_theta = 3.0f;
static float L_psi = 3.0f;

// Condiciones inciciales.
static float xhat1 = 0.0f;
static float xhat2 = 0.0f;
static float xhat3 = 0.0f;

static float yhat1 = 0.0f;
static float yhat2 = 0.0f;
static float yhat3 = 0.0f;

static float zhat1 = 0.0f;
static float zhat2 = 0.0f;
static float zhat3 = 0.0f;

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
    xhat1 = 0;
    xhat2 = 0;
    xhat3 = 0;

    yhat1 = 0;
    yhat2 = 0;
    yhat3 = 0;

    zhat1 = 0;
    zhat2 = 0;
    zhat3 = 0;

    phihat1 = 0;
    phihat2 = 0;
    phihat3 = 0;

    thetahat1 = 0;
    thetahat2 = 0;
    thetahat3 = 0;

    psihat1 = 0;
    psihat2 = 0;
    psihat3 = 0;
}

void FTSMOInit(void)
{
    FTSMOReset();
}

void FTSMOAttitude(
       float phi,      float theta,      float psi,
       float* phihat1, float* thetahat1, float* psihat1
       float* phihat2, float* thetahat2, float* psihat2
       float* phihat3, float* thetahat3, float* psihat3)
{
    // float k1_x = 2 * powf(L_x, 1.0 / 3.0);
    // float k2_x = 1.5 * sqrtf(L_x);
    // float k3_x = 1.1 * L_x;

    // float k1_y = 2 * powf(L_y, 1.0 / 3.0);
    // float k2_y = 1.5 * sqrtf(L_y);
    // float k3_y = 1.1 * L_y;

    // float k1_z = 2 * powf(L_z, 1.0 / 3.0);
    // float k2_z = 1.5 * sqrtf(L_z);
    // float k3_z = 1.1 * L_z;

    // // Cálculos para x
    // ex1 = x - xhat1;
    // xhat1 = xhat1 + (xhat2 + k1_x * powf(fabs(ex1), 2.0 / 3.0) * sign(ex1)) * dt;
    // xhat2 = xhat2 + (xhat3 + k2_x * powf(fabs(ex1), 1.0 / 3.0) * sign(ex1)) * dt;
    // xhat3 = xhat3 + (k3_x * sign(ex1)) * dt;

    // ey1 = y - yhat1;
    // yhat1 = yhat1 + (yhat2 + k1_y * powf(fabs(ey1), 2.0 / 3.0) * sign(ey1)) * dt;
    // yhat2 = yhat2 + (yhat3 + k2_y * powf(fabs(ey1), 1.0 / 3.0) * sign(ey1)) * dt;
    // yhat3 = yhat3 + (k3_y * sign(ey1)) * dt;

    // ez1 = z - zhat1;
    // zhat1 = zhat1 + (zhat2 + k1_z * powf(fabs(ez1), 2.0 / 3.0) * sign(ez1)) * dt;
    // zhat2 = zhat2 + (zhat3 + k2_z * powf(fabs(ez1), 1.0 / 3.0) * sign(ez1)) * dt;
    // zhat3 = zhat3 + (k3_z * sign(ez1)) * dt;

    float k1_phi = 2 * powf(L_phi, 1.0 / 3.0);
    float k2_phi = 1.5 * sqrtf(L_phi);
    float k3_phi = 1.1 * L_phi;

    float k1_theta = 2 * powf(L_theta, 1.0 / 3.0);
    float k2_theta = 1.5 * sqrtf(L_theta);
    float k3_theta = 1.1 * L_theta;

    float k1_psi = 2 * powf(L_psi, 1.0 / 3.0);
    float k2_psi = 1.5 * sqrtf(L_psi);
    float k3_psi = 1.1 * L_psi;

       // Variables phi
    float ephi1, phihat1, phihat2, phihat3;
    float k1_phi, k2_phi, k3_phi;
    float phi, phihat2_prev, thetap, psip, dt, Jx, Jy, Jz, tau_phi;
    
    // Variables theta
    float etheta1, thetahat1, thetahat2, thetahat3;
    float k1_theta, k2_theta, k3_theta;
    float theta, thetahat2_prev, phip, psip, Jy, Jz, Jx, tau_theta;
    
    // Variables psi
    float epsi1, psihat1, psihat2, psihat3;
    float k1_psi, k2_psi, k3_psi;
    float psi, psihat2_prev, thetap, phip, Jx, Jy, Jz, tau_psi;

    // Cálculos para phi
    ephi1 = phi - phihat1;
    phihat1 = phihat1 + (phihat2 + k1_phi * powf(fabs(ephi1), 2.0 / 3.0) * sign(ephi1)) * dt;
    phihat2 = phihat2 + (phihat3 + k2_phi * powf(fabs(ephi1), 1.0 / 3.0) * sign(ephi1)) * dt;
    phihat3 = phihat3 + (k3_phi * sign(ephi1)) * dt;

    // Cálculos para theta
    etheta1 = theta - thetahat1;
    thetahat1 = thetahat1 + (thetahat2 + k1_theta * powf(fabs(etheta1), 2.0 / 3.0) * sign(etheta1)) * dt;
    thetahat2 = thetahat2 + (thetahat3 + k2_theta * powf(fabs(etheta1), 1.0 / 3.0) * sign(etheta1)) * dt;
    thetahat3 = thetahat3 + (k3_theta * sign(etheta1)) * dt;

    // Cálculos para psi
    epsi1 = psi - psihat1;
    psihat1 = psihat1 + (psihat2 + k1_psi * powf(fabs(epsi1), 2.0 / 3.0) * sign(epsi1)) * dt;
    psihat2 = psihat2 + (psihat3 + k2_psi * powf(fabs(epsi1), 1.0 / 3.0) * sign(epsi1)) * dt;
    psihat3 = psihat3 + (k3_psi * sign(epsi1)) * dt;

}


void FTSMOPosition(
       float  x,     float  y,     float  z,
       float* xhat1, float* yhat1, float* zhat1
       float* xhat2, float* yhat2, float* zhat2
       float* xhat3, float* yhat3, float* zhat3)
{
    float k1_x = 2 * powf(L_x, 1.0 / 3.0);
    float k2_x = 1.5 * sqrtf(L_x);
    float k3_x = 1.1 * L_x;

    float k1_y = 2 * powf(L_y, 1.0 / 3.0);
    float k2_y = 1.5 * sqrtf(L_y);
    float k3_y = 1.1 * L_y;

    float k1_z = 2 * powf(L_z, 1.0 / 3.0);
    float k2_z = 1.5 * sqrtf(L_z);
    float k3_z = 1.1 * L_z;

    // Cálculos para x
    ex1 = x - xhat1;
    xhat1 = xhat1 + (xhat2 + k1_x * powf(fabs(ex1), 2.0 / 3.0) * sign(ex1)) * dt;
    xhat2 = xhat2 + (xhat3 + k2_x * powf(fabs(ex1), 1.0 / 3.0) * sign(ex1)) * dt;
    xhat3 = xhat3 + (k3_x * sign(ex1)) * dt;

    ey1 = y - yhat1;
    yhat1 = yhat1 + (yhat2 + k1_y * powf(fabs(ey1), 2.0 / 3.0) * sign(ey1)) * dt;
    yhat2 = yhat2 + (yhat3 + k2_y * powf(fabs(ey1), 1.0 / 3.0) * sign(ey1)) * dt;
    yhat3 = yhat3 + (k3_y * sign(ey1)) * dt;

    ez1 = z - zhat1;
    zhat1 = zhat1 + (zhat2 + k1_z * powf(fabs(ez1), 2.0 / 3.0) * sign(ez1)) * dt;
    zhat2 = zhat2 + (zhat3 + k2_z * powf(fabs(ez1), 1.0 / 3.0) * sign(ez1)) * dt;
    zhat3 = zhat3 + (k3_z * sign(ez1)) * dt;

}

 
LOG_GROUP_START(FTSMO)
LOG_ADD(LOG_FLOAT, xhat1, &xhat1)
LOG_ADD(LOG_FLOAT, xhat2, &xhat2)
LOG_ADD(LOG_FLOAT, xhat3, &xhat3)

LOG_ADD(LOG_FLOAT, yhat1, &yhat1)
LOG_ADD(LOG_FLOAT, yhat2, &yhat2)
LOG_ADD(LOG_FLOAT, yhat3, &yhat3)

LOG_ADD(LOG_FLOAT, zhat1, &zhat1)
LOG_ADD(LOG_FLOAT, zhat2, &zhat2)
LOG_ADD(LOG_FLOAT, zhat3, &zhat3)

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

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_x, &L_x)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_y, &L_y)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_z, &L_z)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_phi, &L_phi)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_theta, &L_theta)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, L_psi, &L_psi)

PARAM_GROUP_STOP(pid_attitude)
