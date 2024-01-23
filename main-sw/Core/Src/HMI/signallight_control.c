/*
 * signallight_control.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

// TODO: ADD IMPLEMENTATION FOR SIGNALLIGHT

// HMI_signallight_check_blob.c
#include "HMI/signallight_control.h"

void HMI_signallight_check_blob(StateMachine* stateMachine, uint16_t pin) {
	if (stateMachine->getBlobDetected(stateMachine) == 1) {
        HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
    }
	else {
		HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
    }
}
