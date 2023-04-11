#include <stdbool.h>
#include <stdint.h>


void GetAttitudeRateInit(const float updateDt);

bool GetAttitudeRateTest(void);

void GetAttitudeRate(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

void GetAttitudeRateResetAll(void);

void GetAttitudeRateResetRoll(void);

void GetAttitudeRateResetPitch(void);

