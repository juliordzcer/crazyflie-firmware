#include "stabilizer_types.h"

void controllerbcInit(void);
bool controllerbcTest(void);
void controllerbc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);