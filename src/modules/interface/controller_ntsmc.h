#include "stabilizer_types.h"

void controllerntsmcInit(void);
bool controllerntsmcTest(void);
void controllerntsmc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


void setgainsntsmc(int new_gain, float new_zeta_phi, float new_zeta_theta, float new_zeta_psi);