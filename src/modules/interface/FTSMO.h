#include <stdbool.h>
#include <stdint.h>

void FTSMOInit(void);

void FTSMOPosition( float  x,      float  y,      float  z,
                    float  xhat1,  float yhat1,   float zhat1,
                    float  xhat2,  float yhat2,   float zhat2,
                    float  xhat3,  float yhat3,   float zhat3,
                    float* xhat1p, float* yhat1p, float* zhat1p,
                    float* xhat2p, float* yhat2p, float* zhat2p,
                    float* xhat3p, float* yhat3p, float* zhat3p);

void FTSMOAttitude( float phi,       float theta,       float psi,
                    float phihat1,   float thetahat1,    float psihat1,
                    float phihat2,   float thetahat2,    float psihat2,
                    float phihat3,   float thetahat3,    float psihat3,
                    float* phihat1p, float* thetahat1p, float* psihat1p,
                    float* phihat2p, float* thetahat2p, float* psihat2p,
                    float* phihat3p, float* thetahat3p, float* psihat3p);