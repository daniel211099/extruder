/*
 * hmi_display.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

// TODO: ADD IMPLEMENTATION FOR HMI-DISPLAY

/*
 * hmi_display.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

// TODO: ADD IMPLEMENTATION FOR HMI-DISPLAY

#include <HMI/hmi_display.h>


Hmi HMI_init(StateMachine* stateMachine, Sensor* sensorExtruder, Sensor* sensorBack, PIDController *pidController)
{
	Hmi hmi;
	hmi.HmiInformation.stateMachine = stateMachine;
	hmi.HmiInformation.sensorExtruder = sensorExtruder;
	hmi.HmiInformation.sensorBack = sensorBack;

	TSC2046_Calibrate();
	ILI9341_Fill(COLOR_WHITE);

	ILI9341_Fill_Rect(5, 10, 315, 50, COLOR_ORANGE);
	ILI9341_printText("Durchmesser: 1.75mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);

	char buf[20];
	sprintf(buf, "Soll: %.2f mm", pidController->get_setPoint(pidController));

	ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
	ILI9341_printText(buf, 50, 70, COLOR_WHITE, COLOR_BLUE, 2);

	ILI9341_Fill_Rect(30, 110, 70, 150, COLOR_BLUE);
	ILI9341_printText("UP", 45,  130, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(80, 110, 120, 150, COLOR_BLUE);
	ILI9341_printText("DOWN", 90,  130, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
	ILI9341_printText("AUS", 230,  155, COLOR_WHITE, COLOR_RED, 2);

	ILI9341_printText("REGELUNG", 195,  120, COLOR_BLACK, COLOR_WHITE, 2);

	return hmi;
}


void HMI_getTouch(Hmi *hmi, TS_TOUCH_DATA_Def myTS_Handle, StateMachine *state, PIDController *pidController)
{
	if(myTS_Handle.isPressed)
	{
		//Draw a point


		if(myTS_Handle.X >=30 && myTS_Handle.X<=70 && myTS_Handle.Y>=110 && myTS_Handle.Y<=150)
		{
			float Soll = pidController->get_setPoint(pidController);
			Soll = Soll + 0.05;
			pidController->set_setPoint(pidController,Soll);
			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
			ILI9341_printText(buf, 50, 80, COLOR_WHITE, COLOR_BLUE, 2);



			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
		}

		if(myTS_Handle.X >=80 && myTS_Handle.X<=120 && myTS_Handle.Y>=110 && myTS_Handle.Y<=150)
		{
			float Soll = pidController->get_setPoint(pidController);
			Soll = Soll - 0.05;
			pidController->set_setPoint(pidController,Soll);

			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
			ILI9341_printText(buf, 50, 80, COLOR_WHITE, COLOR_BLUE, 2);



			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		}

		if(myTS_Handle.X >=180 && myTS_Handle.X<=300 && myTS_Handle.Y>=140 && myTS_Handle.Y<=180)
		{
			if(hmi->HmiInformation.stateMachine->getState(hmi->HmiInformation.stateMachine) == STATE_IDLE){
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_MANUAL_CONTROL);
  				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_GREEN);
  				ILI9341_printText("Manual", 230,  155, COLOR_WHITE, COLOR_GREEN, 2);
			}
			else if(hmi->HmiInformation.stateMachine->getState(hmi->HmiInformation.stateMachine) == STATE_MANUAL_CONTROL){
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_AUTOMATIC_MODE);
  				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_GREEN);
  				ILI9341_printText("Auto", 230,  155, COLOR_WHITE, COLOR_GREEN, 2);
			}
			else{
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_IDLE);
  				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_GREEN);
  				ILI9341_printText("Idle", 230,  155, COLOR_WHITE, COLOR_GREEN, 2);
			}
		}

		if(hmi->HmiInformation.stateMachine->getBlobDetected(hmi->HmiInformation.stateMachine)==1)
		{
			ILI9341_Fill(COLOR_RED);
			ILI9341_Fill_Rect(110, 70, 210, 170, COLOR_WHITE);
			ILI9341_printText("BLOB erkannt!", 90, 20, COLOR_WHITE, COLOR_RED, 2);
			ILI9341_printText("OK", 145, 110, COLOR_BLACK, COLOR_WHITE, 3);
			if(myTS_Handle.X >=110 && myTS_Handle.X<=210 && myTS_Handle.Y>=70 && myTS_Handle.Y<=140)
				{
				hmi->HmiInformation.stateMachine->setBlobDetected(hmi->HmiInformation.stateMachine,0);
				ILI9341_Fill(COLOR_WHITE);

				ILI9341_Fill_Rect(5, 10, 315, 50, COLOR_ORANGE);
				ILI9341_printText("Durchmesser: 1.75mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);
				char buf[20];
				sprintf(buf, "Soll: %.2f mm", pidController->get_setPoint(pidController));
				ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
				ILI9341_printText(buf, 50, 70, COLOR_WHITE, COLOR_BLUE, 2);

				ILI9341_Fill_Rect(30, 110, 70, 150, COLOR_BLUE);
				ILI9341_printText("UP", 45,  130, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_Fill_Rect(80, 110, 120, 150, COLOR_BLUE);
				ILI9341_printText("DOWN", 90,  130, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_printText("REGELUNG", 195,  120, COLOR_BLACK, COLOR_WHITE, 2);


				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
				ILI9341_printText("IDLE", 230,  155, COLOR_WHITE, COLOR_RED, 2);
				}
		}


	}


}
