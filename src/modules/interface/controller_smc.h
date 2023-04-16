#include "stabilizer_types.h"

void controllersmcInit(void);
bool controllersmcTest(void);
void controllersmc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


void setgainssmc(float new_k1_phi, float new_k2_phi, float new_k1_theta, float new_k2_theta, float new_k1_psi, float new_k2_psi);