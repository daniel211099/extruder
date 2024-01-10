/*
 * senser_recieve_handler.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */
#include "kommunikation/sensor_recieve_handler.h"
#include <kommunikation/uart_processor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

SensorReceiveHandler gHandler;

CommandEntry sensor_interface_dictionary[] = {
		{"!diam",      handleDiamRecieved},
        {NULL, NULL}
};


// Konstruktor
SensorReceiveHandler createSensorReceiveHandler(Sensor* sensorExtruder,
												Sensor* sensorBack)
{
	SensorReceiveHandler handler;
    handler.sensorExtruder = sensorExtruder;
    handler.sensorBack = sensorBack;

    // Zuweisung der globalen Variable
    gHandler = handler;

    handler.uartProcessor = createUartProcessor(sensor_interface_dictionary);
    return handler;
}

// Methoden
void handleDiamRecieved(uint8_t uartNr, const char* value){
	uint8_t ext = gHandler.sensorExtruder->getUartPort(gHandler.sensorExtruder);
	uint8_t back = gHandler.sensorBack->getUartPort(gHandler.sensorBack);
	if(ext == uartNr){
		float diam = getFloatFromMessage(value);
		gHandler.sensorExtruder->setDiameter(gHandler.sensorExtruder,diam);
	}
	else if(back == uartNr){
		float diam = getFloatFromMessage(value);
		gHandler.sensorBack->setDiameter(gHandler.sensorBack,diam);
	}
}
float getFloatFromMessage(const char* value) {
    int length = strlen(value);
    int startIndex = -1;  // Index des ersten Leerzeichens
    int endIndex = -1;    // Index des zweiten Leerzeichens

    for (int i = 0; i < length; i++) {
        if (value[i] == ' ' || value[i] == '\r') {
            if (startIndex == -1) {
                startIndex = i + 1;
            } else {
                endIndex = i;
                break;
            }
        }
    }
    if (startIndex != -1 && endIndex != -1 && startIndex < endIndex) {
        char floatStr[endIndex - startIndex + 1];
        strncpy(floatStr, value + startIndex, endIndex - startIndex);
        floatStr[endIndex - startIndex] = '\0';
        return strtof(floatStr, NULL);
    }
    return 0.00;
}


void processSensorInterfaceMessage(uint8_t uartNr,SensorReceiveHandler* handler,uint8_t* receivedData, uint8_t receivedDataIndex){
	processCommand(uartNr, &(handler->uartProcessor), receivedData, receivedDataIndex);
}
