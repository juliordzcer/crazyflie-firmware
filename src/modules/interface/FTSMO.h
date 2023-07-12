#include <stdbool.h>
#include <stdint.h>

void FTSMOInit(void);

void FTSMO( float eulerRollActual, float eulerPitchActual, float eulerYawActual,
            float* phi2, float* theta2, float* psi2);