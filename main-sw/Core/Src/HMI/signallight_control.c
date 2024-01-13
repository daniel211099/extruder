/*
 * signallight_control.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

//TODO A: Chose Pins for Ligth in main.h:






#include <HMI/signallight_control.h>

void HMI_SignalLigth(StateMachine *state)//SignalLeuchte
{
	//TODO A: Add pin Port and activate in HMI_SignalLigth
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

	HMI_SetSignalLigth(stateMachine);
}

void HMI_SetSignalLigth(StateMachine *state)//SignalLeuchte
{
	//TODO A: Add pin Port and activate in HMI_SetSignalLigth
	if(StateMachine.getBlobDetected()==1)
		{
		HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		}
	else
		{
		HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
		}
}


