#include "stabilizer_types.h"

void controllerpidnInit(void);
bool controllerpidnTest(void);
void controllerpidn(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);


void setgainspidn(float new_kp_phi, float new_ki_phi, float new_kd_phi, float new_kp_theta, float new_ki_theta, float new_kd_theta, float new_kp_psi, float new_ki_psi, float new_kd_psi);
