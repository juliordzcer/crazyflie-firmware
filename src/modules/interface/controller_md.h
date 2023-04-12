#include "stabilizer_types.h"

void controllermdInit(void);
bool controllermdTest(void);
void controllermd(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


