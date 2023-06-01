#include "stabilizer_types.h"

void controllerstsmcInit(void);
bool controllerstsmcTest(void);
void controllerstsmc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);