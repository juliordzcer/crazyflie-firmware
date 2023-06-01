#include "stabilizer_types.h"

void controllerntsmcInit(void);
bool controllerntsmcTest(void);
void controllerntsmc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);