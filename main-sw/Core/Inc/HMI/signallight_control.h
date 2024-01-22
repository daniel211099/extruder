/*
 * signallight_control.h
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

#ifndef INC_HMI_SIGNALLIGHT_CONTROL_H_
#define INC_HMI_SIGNALLIGHT_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "Management/global_state_machine.h"

void HMI_signallight_check_blob(StateMachine* stateMachine, uint16_t pin);

#endif /* INC_HMI_SIGNALLIGHT_CONTROL_H_ */
