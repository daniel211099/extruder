/*
 * signallight_control.h
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

#ifndef INC_HMI_SIGNALLIGHT_CONTROL_H_
#define INC_HMI_SIGNALLIGHT_CONTROL_H_

#include "stm32f4xx_hal.h"
//#include "TSC2046.h"
#include "HMI/MY_ILI9341.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "HMI/TSC2046.h"
#include "main.h"
#include "Management/global_state_machine.h"
#include "SensorActuators/sensor.h"
#include "Regelung/pid_controller.h"




void HMI_SignalLigth_init(StateMachine* stateMachine);

void HMI_SetSignalLigth(StateMachine *state);

#endif /* INC_HMI_SIGNALLIGHT_CONTROL_H_ */
