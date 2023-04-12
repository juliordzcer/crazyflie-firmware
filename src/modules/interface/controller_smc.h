#include "stabilizer_types.h"

void controllersmcInit(void);
bool controllersmcTest(void);
void controllersmc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


