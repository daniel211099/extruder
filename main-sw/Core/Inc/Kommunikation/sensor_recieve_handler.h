/*
 * sensor_recieve_handler.h
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_KOMMUNIKATION_SENSOR_RECIEVE_HANDLER_H_
#define INC_KOMMUNIKATION_SENSOR_RECIEVE_HANDLER_H_

#include <SensorActuators/sensor.h>
#include "kommunikation/uart_processor.h"
#include "HMI/hmi_display.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

typedef struct {
    Sensor* sensorExtruder;  // Sensor am Extruder
    Sensor* sensorBack;
    UartProcessor uartProcessor;
} SensorReceiveHandler;

// Konstruktor
SensorReceiveHandler createSensorReceiveHandler(Sensor* sensorExtruder,
												Sensor* sensorBack);

// Methoden
void handleDiamRecieved(uint8_t uartNr, const char* value);

void processSensorInterfaceMessage(uint8_t uartNr, SensorReceiveHandler* handler,uint8_t* receivedData, uint8_t receivedDataIndex);
float getFloatFromMessage(const char* value);



#endif /* INC_KOMMUNIKATION_SENSOR_RECIEVE_HANDLER_H_ */
