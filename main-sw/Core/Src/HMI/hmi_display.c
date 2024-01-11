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


void HMI_init(float Soll)
{

	TSC2046_Calibrate();
	ILI9341_Fill(COLOR_WHITE);

	ILI9341_Fill_Rect(5, 10, 315, 50, COLOR_ORANGE);
	ILI9341_printText("Durchmesser: 1.75mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);

	char buf[20];
	sprintf(buf, "Soll: %.2f mm", Soll);

	ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
	ILI9341_printText(buf, 50, 70, COLOR_WHITE, COLOR_BLUE, 2);

	ILI9341_Fill_Rect(30, 110, 70, 150, COLOR_BLUE);
	ILI9341_printText("UP", 45,  130, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(80, 110, 120, 150, COLOR_BLUE);
	ILI9341_printText("DOWN", 90,  130, COLOR_WHITE, COLOR_BLUE, 1);

	ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
	ILI9341_printText("AUS", 230,  155, COLOR_WHITE, COLOR_RED, 2);

	ILI9341_printText("REGELUNG", 195,  120, COLOR_BLACK, COLOR_WHITE, 2);
}


void HMI_getTouch(TS_TOUCH_DATA_Def myTS_Handle, float Soll, bool Reg_aktiv, int Blob)
{
	if(myTS_Handle.isPressed)
	{
		//Draw a point


		if(myTS_Handle.X >=30 && myTS_Handle.X<=70 && myTS_Handle.Y>=110 && myTS_Handle.Y<=150)
		{
			Soll = Soll + 0.05;

			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
			ILI9341_printText(buf, 50, 80, COLOR_WHITE, COLOR_BLUE, 2);



			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
		}

		if(myTS_Handle.X >=80 && myTS_Handle.X<=120 && myTS_Handle.Y>=110 && myTS_Handle.Y<=150)
		{
			Soll = Soll - 0.05;

			char buf[20];
			sprintf(buf, "Soll: %.2f mm", Soll);

			ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
			ILI9341_printText(buf, 50, 80, COLOR_WHITE, COLOR_BLUE, 2);



			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		}

		if(myTS_Handle.X >=180 && myTS_Handle.X<=300 && myTS_Handle.Y>=140 && myTS_Handle.Y<=180)
		{

			if(Reg_aktiv ==0)
			{
  				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_GREEN);
  				ILI9341_printText("AN", 230,  155, COLOR_WHITE, COLOR_GREEN, 2);
  				Reg_aktiv = 1;



  				HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			}
			elseif(Reg_aktiv ==1)
			{
				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
				ILI9341_printText("AUS", 230,  155, COLOR_WHITE, COLOR_RED, 2);
				Reg_aktiv = 0;



				HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
			}
			elseif(Reg_aktiv ==2)
			{
				ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
				ILI9341_printText("AUS", 230,  155, COLOR_WHITE, COLOR_RED, 2);
				Reg_aktiv = 0;

				HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
						}

		}

		if(Blob==1)
		{
			ILI9341_Fill(COLOR_RED);
			ILI9341_Fill_Rect(110, 70, 210, 170, COLOR_WHITE);
			ILI9341_printText("BLOB erkannt!", 90, 20, COLOR_WHITE, COLOR_RED, 2);
			ILI9341_printText("OK", 145, 110, COLOR_BLACK, COLOR_WHITE, 3);
			if(myTS_Handle.X >=110 && myTS_Handle.X<=210 && myTS_Handle.Y>=70 && myTS_Handle.Y<=140)
				{
				Blob = 0;
				ILI9341_Fill(COLOR_WHITE);

				ILI9341_Fill_Rect(5, 10, 315, 50, COLOR_ORANGE);
				ILI9341_printText("Durchmesser: 1.75mm", 50, 25, COLOR_WHITE, COLOR_ORANGE, 2);
				char buf[20];
				sprintf(buf, "Soll: %.2f mm", Soll);
				ILI9341_Fill_Rect(5, 60, 315, 100, COLOR_BLUE);
				ILI9341_printText(buf, 50, 70, COLOR_WHITE, COLOR_BLUE, 2);

				ILI9341_Fill_Rect(30, 110, 70, 150, COLOR_BLUE);
				ILI9341_printText("UP", 45,  130, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_Fill_Rect(80, 110, 120, 150, COLOR_BLUE);
				ILI9341_printText("DOWN", 90,  130, COLOR_WHITE, COLOR_BLUE, 1);

				ILI9341_printText("REGELUNG", 195,  120, COLOR_BLACK, COLOR_WHITE, 2);

				if(Reg_aktiv = 0)
				{
					ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_RED);
					ILI9341_printText("AUS", 230,  155, COLOR_WHITE, COLOR_RED, 2);
				}
				if(Reg_aktiv = 1)
				{
					ILI9341_Fill_Rect(180, 140, 300, 180, COLOR_GREEN);
					ILI9341_printText("AN", 230,  155, COLOR_WHITE, COLOR_GREEN, 2);
				}

				}
		}


	}


}
