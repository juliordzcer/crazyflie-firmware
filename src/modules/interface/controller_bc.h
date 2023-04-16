#include "stabilizer_types.h"

void controllerbcInit(void);
bool controllerbcTest(void);
void controllerbc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

void setgainsbc(float new_k1_phi, float new_k2_phi, float new_k1_theta, float new_k2_theta, float new_k1_psi, float new_k2_psi);
