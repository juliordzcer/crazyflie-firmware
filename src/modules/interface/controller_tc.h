#include "stabilizer_types.h"

void controllertcInit(void);
bool controllertcTest(void);
void controllertc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


void setgainstc(int new_gain, float new_zeta_phi, float new_zeta_theta, float new_zeta_psi);