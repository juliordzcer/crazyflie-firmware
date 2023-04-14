#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_tc.h"
#include "controller_smc.h"
#include "controller_bc.h"
#include "controller_sta.h"
#include "controller_ntsmc.h"
#include "controller_stsmc.h"
#include "controller_mellinger.h"
#include "controller_indi.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypeSTSMC
static ControllerType currentController = ControllerTypeAny;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
  {.init = 0, .test = 0, .update = 0, .name = "None"}, // Any
  {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid, .name = "PID"},
  {.init = controllerMellingerInit, .test = controllerMellingerTest, .update = controllerMellinger, .name = "Mellinger"},
  {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI"},
  {.init = controllersmcInit, .test = controllersmcTest, .update = controllersmc, .name = "Sliding Mode"},
  {.init = controllerbcInit, .test = controllerbcTest, .update = controllerbc, .name = "Backstepping"},
  {.init = controllertcInit, .test = controllertcTest, .update = controllertc, .name = "Twisting"},
  {.init = controllerstaInit, .test = controllerstaTest, .update = controllersta, .name = "Super-Twisting"},
  {.init = controllerntsmcInit, .test = controllerntsmcTest, .update = controllerntsmc, .name = "Nonsingular-Terminal"},
  {.init = controllerstsmcInit, .test = controllerstsmcTest, .update = controllerstsmc, .name = "Singular-Terminal"},
};


void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  currentController = controller;

  if (ControllerTypeAny == currentController) {
    currentController = DEFAULT_CONTROLLER;
  }

  #if defined(CONFIG_CONTROLLER_PID)
    #define CONTROLLER ControllerTypePID
  #elif defined(CONFIG_CONTROLLER_INDI)
    #define CONTROLLER ControllerTypeINDI
  #elif defined(CONFIG_CONTROLLER_MELLINGER)
    #define CONTROLLER ControllerTypeMellinger
  #elif defined(CONFIG_CONTROLLER_SMC)
    #define CONTROLLER ControllerTypeSMC
  #elif defined(CONFIG_CONTROLLER_BC)
    #define CONTROLLER ControllerTypeBC    
  #elif defined(CONFIG_CONTROLLER_TC)
    #define CONTROLLER ControllerTypeTC
  #elif defined(CONFIG_CONTROLLER_STA)
    #define CONTROLLER ControllerTypeSTA
  #elif defined(CONFIG_CONTROLLER_NTSMC)
    #define CONTROLLER ControllerTypeNTSMC
  #elif defined(CONFIG_CONTROLLER_STSMC)
    #define CONTROLLER ControllerTypeSTSMC
  #else
    #define CONTROLLER ControllerTypeAny
  #endif

  ControllerType forcedController = CONTROLLER;
  if (forcedController != ControllerTypeAny) {
    DEBUG_PRINT("Controller type forced\n");
    currentController = forcedController;
  }

  initController();

  DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}

ControllerType getControllerType(void) {
  return currentController;
}

static void initController() {
  controllerFunctions[currentController].init();
}

bool controllerTest(void) {
  return controllerFunctions[currentController].test();
}

void controller(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, tick);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
