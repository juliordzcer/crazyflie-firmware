
#include "stabilizer_types.h"

void controllerJCRCInit();

void controllerJCRCReset();
void controllerJCRC(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state);