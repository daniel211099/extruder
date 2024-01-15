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
	ILI9341_printText("Sensor1 : 0 mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);

	ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_ORANGE);
	ILI9341_printText("Sensor2 : 0 mm", 50, 75, COLOR_WHITE, COLOR_ORANGE, 2);

	char buf[20];
	sprintf(buf, "Soll: %.2f mm", pidController->get_setPoint(pidController));

	ILI9341_Fill_Rect(5, 110, 315, 150, COLOR_BLUE);
	ILI9341_printText(buf, 50, 120, COLOR_WHITE, COLOR_BLUE, 2);

	ILI9341_Fill_Rect(30, 160, 70, 200, COLOR_BLUE);
	ILI9341_printText("UP", 45,  180, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(80, 160, 120, 200, COLOR_BLUE);
	ILI9341_printText("DOWN", 90,  180, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(180, 190, 300, 230, COLOR_RED);
	ILI9341_printText("IDLE", 230,  205, COLOR_WHITE, COLOR_RED, 2);

	ILI9341_printText("REGELUNG", 195,  170, COLOR_BLACK, COLOR_WHITE, 2);

	return hmi;
}


void HMI_getTouch(Hmi *hmi, TS_TOUCH_DATA_Def myTS_Handle, StateMachine *state, PIDController *pidController)
{
	if(myTS_Handle.isPressed)
	{
		//Draw a point


		if(myTS_Handle.X >=30 && myTS_Handle.X<=70 && myTS_Handle.Y>=160 && myTS_Handle.Y<=200)
		{
			float Soll = pidController->get_setPoint(pidController);
			Soll = Soll + 0.05;
			pidController->set_setPoint(pidController,Soll);
			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 110, 315, 150, COLOR_BLUE);
			ILI9341_printText(buf, 50, 120, COLOR_WHITE, COLOR_BLUE, 2);



		}

		if(myTS_Handle.X >=80 && myTS_Handle.X<=120 && myTS_Handle.Y>=160 && myTS_Handle.Y<=200)
		{
			float Soll = pidController->get_setPoint(pidController);
			Soll = Soll - 0.05;
			pidController->set_setPoint(pidController,Soll);

			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 110, 315, 150, COLOR_BLUE);
			ILI9341_printText(buf, 50, 120, COLOR_WHITE, COLOR_BLUE, 2);



			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		}

		if(myTS_Handle.X >=180 && myTS_Handle.X<=300 && myTS_Handle.Y>=190 && myTS_Handle.Y<=230)
		{
			if(hmi->HmiInformation.stateMachine->getState(hmi->HmiInformation.stateMachine) == STATE_IDLE){
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_MANUAL_CONTROL);
  				ILI9341_Fill_Rect(180, 190, 300, 230, COLOR_ORANGE);
  				ILI9341_printText("Manual", 230,  205, COLOR_WHITE, COLOR_ORANGE, 2);
			}
			else if(hmi->HmiInformation.stateMachine->getState(hmi->HmiInformation.stateMachine) == STATE_MANUAL_CONTROL){
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_AUTOMATIC_MODE);
  				ILI9341_Fill_Rect(180, 190, 300, 230, COLOR_GREEN);
  				ILI9341_printText("Auto", 230,  205, COLOR_WHITE, COLOR_GREEN, 2);
			}
			else{
				hmi->HmiInformation.stateMachine->changeState(hmi->HmiInformation.stateMachine, STATE_IDLE);
  				ILI9341_Fill_Rect(180, 190, 300, 230, COLOR_RED);
  				ILI9341_printText("Idle", 230,  205, COLOR_WHITE, COLOR_RED, 2);
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
				ILI9341_printText("Sensor1: 0 mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);

				ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_ORANGE);
				ILI9341_printText("Sensor2 : 0 mm", 50, 75, COLOR_WHITE, COLOR_ORANGE, 2);

				char buf[20];
				sprintf(buf, "Soll: %.2f mm", pidController->get_setPoint(pidController));
				ILI9341_Fill_Rect(5, 110, 315, 150, COLOR_BLUE);
				ILI9341_printText(buf, 50, 120, COLOR_WHITE, COLOR_BLUE, 2);

				ILI9341_Fill_Rect(30, 160, 70, 200, COLOR_BLUE);
				ILI9341_printText("UP", 45,  180, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_Fill_Rect(80, 160, 120, 200, COLOR_BLUE);
				ILI9341_printText("DOWN", 90,  180, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_printText("REGELUNG", 195,  170, COLOR_BLACK, COLOR_WHITE, 2);


				ILI9341_Fill_Rect(180, 190, 300, 230, COLOR_RED);
				ILI9341_printText("IDLE", 230,  205, COLOR_WHITE, COLOR_RED, 2);
				}
		}


	}


}

void HMI_updateDisplaySensor(Sensor* sensorExtruder, Sensor* sensorBack)
{
	float value1 = sensorExtruder->getDiameter(sensorExtruder);
	char buf1[20];
	sprintf(buf1, "Sensor1: %.2f mm", value1);


	ILI9341_Fill_Rect(5, 10, 315, 50, COLOR_ORANGE);
	ILI9341_printText(buf1, 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);

	float value2 = sensorBack->getDiameter(sensorBack);
	char buf2[20];
	sprintf(buf2, "Sensor2: %.2f mm", value2);

	ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_ORANGE);
	ILI9341_printText(buf2, 50, 75, COLOR_WHITE, COLOR_ORANGE, 2);
}
