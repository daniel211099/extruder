/*
 * pc_recieve_handler.h
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_KOMMUNIKATION_PC_RECIEVE_HANDLER_H_
#define INC_KOMMUNIKATION_PC_RECIEVE_HANDLER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <Management/global_state_machine.h>
#include <SensorActuators/motor.h>
#include <Kommunikation/pc_send_handler.h>
#include <Kommunikation/uart_processor.h>
#include <Regelung/pid_controller.h>

typedef struct {
    StateMachine* stateMachine;
    PIDController* pidController;
	Motor*		  motor;
	PcSendHandler* sender;
    UART_HandleTypeDef* uart3;  // Externer Zugriff auf UART
    UartProcessor uartProcessor;

} PcReceiveHandler;

// Konstruktor
PcReceiveHandler createPcReceiveHandler(StateMachine* stateMachine,
										PIDController* controller,
										Motor* motor,
										PcSendHandler* sender,
										UART_HandleTypeDef* uart3);

// Methoden
void handleSetState(uint8_t uartNr, const char* value);
void handleSetSpeed(uint8_t uartNr, const char* value);
void handleSetCP(uint8_t uartNr, const char* value);
void handleSetCI(uint8_t uartNr, const char* value);
void handleSetCD(uint8_t uartNr, const char* value);

void handleGetState(uint8_t uartNr, const char* value);
void handleGetSpeed(uint8_t uartNr, const char* value);
void handleGetCP(uint8_t uartNr, const char* value);
void handleGetCI(uint8_t uartNr, const char* value);
void handleGetCD(uint8_t uartNr, const char* value);

void processPcInterfaceMessage(PcReceiveHandler* handler,uint8_t* receivedData, uint8_t receivedDataIndex);

float extractFloatValue(uint8_t uartNr, const char* value);
int extractIntValue(uint8_t uartNr, const char* value);

#endif /* INC_KOMMUNIKATION_PC_RECIEVE_HANDLER_H_ */
