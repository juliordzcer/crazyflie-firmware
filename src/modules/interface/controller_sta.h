
#include "stabilizer_types.h"

void controllerstaInit(void);
bool controllerstaTest(void);
void controllersta(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


void setgainssta(float new_k0_phi, float new_zeta_phi, float new_k0_theta, float new_zeta_theta, float new_k0_psi, float new_zeta_psi);
