#include "stabilizer_types.h"

void controllerpidnInit(void);
bool controllerpidnTest(void);
void controllerpidn(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);