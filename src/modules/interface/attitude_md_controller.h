/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * attitude_controller.h: PID-based attitude controller
 */


#include <stdbool.h>
#include <stdint.h>


void attitudeControllerInitMD(const float updateDt);
bool attitudeControllerTestMD(void);


void ControllerAtitudeMD(float eulerRollActual , float eulerPitchActual , float eulerYawActual ,
                         float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
                         float rollRateActual  , float pitchRateActual  , float yawRateActual  ,
                         float rollRateDesired , float pitchRateDesired , float yawRateDesired ,
                         int16_t* roll         , int16_t* pitch         , int16_t* yaw          );

/**
 * Reset controller roll, pitch and yaw MD.
 */
void attitudeControllerResetAllMD(void);
/**
 * Get yaw max delta
 */
float attitudeControllerGetYawMaxDeltaMD(void);

