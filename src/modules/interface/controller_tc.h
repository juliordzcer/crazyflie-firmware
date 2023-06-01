#include "stabilizer_types.h"

void controllertcInit(void);
bool controllertcTest(void);
void controllertc(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);