/*
 * signallight_control.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

//TODO A: Chose Pins for Ligth in main.h:

//TODO A: Add pins to MX_GPIO_Init in main.c via STM Visualitation

//TODO A: Add HMI_SignalLigth(&sensorExtruder) to Initialize in main.c

//TODO A: Add HMI_SetSignalLigthto while-loop in main.c

//TODO A: Add pin Port and activate in HMI_SignalLigth

//TODO A: Add pin Port and activate in HMI_SetSignalLigth


#include <HMI/signallight_control.h>

void HMI_SignalLigth(StateMachine *state){

	//TODO A: Add pin Port and activate in HMI_SignalLigth
	//HAL_GPIO_WritePin(GPIO, Signallight_RED, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIO, Signallight_YELLOW, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIO, Signallight_GREEN, GPIO_PIN_RESET);

	HMI_SetSignalLigth(stateMachine);


}

void HMI_SetSignalLigth(StateMachine *state)
{
	//TODO A: Add pin Port and activate in HMI_SetSignalLigth
	if(StateMachine.getBlobDetected()==1)
		{
		//HAL_GPIO_WritePin(GPIO, Signallight_RED, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIO, Signallight_YELLOW, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIO, Signallight_GREEN, GPIO_PIN_RESET);
		}
	else if (StateMachine.getState()=0  )
		{
		//HAL_GPIO_WritePin(GPIO, Signallight_RED, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIO, Signallight_YELLOW, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIO, Signallight_GREEN, GPIO_PIN_RESET);
		}
	else if (StateMachine.getState()=1  )
		{
		//HAL_GPIO_WritePin(GPIO, Signallight_RED, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIO, Signallight_YELLOW, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIO, Signallight_GREEN, GPIO_PIN_RESET);
		}
	else if (StateMachine.getState()=2  )
		{
		//HAL_GPIO_WritePin(GPIO, Signallight_RED, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIO, Signallight_YELLOW, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIO, Signallight_GREEN, GPIO_PIN_SET);
		}
	else
		{
		}
}


