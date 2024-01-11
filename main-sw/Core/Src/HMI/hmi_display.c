/*
 * hmi_display.c
 *
 *  Created on: Jan 3, 2024
 *      Author:
 */

// TODO: ADD IMPLEMENTATION FOR HMI-DISPLAY

#include <HMI/hmi_display.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "HMI/MY_ILI9341.h"
#include "HMI/TSC2046.h"



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
