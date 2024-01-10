/*
 * pc_recieve_handler.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */
#include <Kommunikation/pc_recieve_handler.h>
#include <kommunikation/uart_processor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PcReceiveHandler gPcHandler;

CommandEntry pc_interface_dictionary[] = {
		{"!STATE",	handleSetState},
		{"!SPEED",  handleSetSpeed},
		{"!CP",     handleSetCP},
		{"!CI",     handleSetCI},
		{"!CD",     handleSetCD},
		{"?STATE",  handleGetState},
		{"?SPEED",  handleGetSpeed},
		{"?CP",     handleGetCP},
		{"?CI",		handleGetCI},
		{"?CD",		handleGetCD},
        {NULL, NULL}
};


// Konstruktor
PcReceiveHandler createPcReceiveHandler(StateMachine*  stateMachine,
										PIDController* controller,
		   								Motor* motor,
		   								PcSendHandler* sender,
		   								UART_HandleTypeDef* uart3)
{
    PcReceiveHandler handler;
    handler.stateMachine	= stateMachine;
    handler.pidController 	= controller;
    handler.motor 			= motor;
    handler.sender 			= sender;
    handler.uart3 			= uart3;

    // Zuweisung der globalen Variable
    gPcHandler = handler;

    handler.uartProcessor = createUartProcessor(pc_interface_dictionary);
    return handler;
}

// Handle-Methods
void handleSetSpeed(uint8_t uartNr, const char* value){
	if(gPcHandler.stateMachine->getState(gPcHandler.stateMachine) == STATE_MANUAL_CONTROL){
		gPcHandler.motor->setSpeed(gPcHandler.motor,extractFloatValue(3,value));
	}

}


void handleSetState(uint8_t uartNr, const char* value){
	int state = extractIntValue(3, value);
	State s = STATE_IDLE;
	if(state == 1){
		s = STATE_MANUAL_CONTROL;
	}else if(state ==2){
		s = STATE_AUTOMATIC_MODE;
	}

	gPcHandler.stateMachine->changeState(gPcHandler.stateMachine,s);
}

void handleSetCP(uint8_t uartNr, const char* value){
	float kp = extractFloatValue(3, value);
	gPcHandler.pidController->set_kp(gPcHandler.pidController,kp);
}
void handleSetCI(uint8_t uartNr, const char* value){
	float ki = extractFloatValue(3, value);
	gPcHandler.pidController->set_ki(gPcHandler.pidController,ki);
}
void handleSetCD(uint8_t uartNr, const char* value){
	float kd = extractFloatValue(3, value);
	gPcHandler.pidController->set_kd(gPcHandler.pidController,kd);
}

void handleGetState(uint8_t uartNr, const char* value){
	gPcHandler.sender->interface.sendState(gPcHandler.uart3,gPcHandler.stateMachine->getState(gPcHandler.stateMachine));
}
void handleGetSpeed(uint8_t uartNr, const char* value){
	gPcHandler.sender->interface.sendSpeed(gPcHandler.uart3,gPcHandler.motor->getSpeed(gPcHandler.motor));
}
void handleGetCP(uint8_t uartNr, const char* value){
	float kp = gPcHandler.pidController->get_kp(gPcHandler.pidController);
	gPcHandler.sender->interface.sendCPvalue(gPcHandler.uart3,kp);
}
void handleGetCI(uint8_t uartNr, const char* value){
	float ki = gPcHandler.pidController->get_ki(gPcHandler.pidController);
	gPcHandler.sender->interface.sendCIvalue(gPcHandler.uart3,ki);
}
void handleGetCD(uint8_t uartNr, const char* value){
	float kd = gPcHandler.pidController->get_kd(gPcHandler.pidController);
	gPcHandler.sender->interface.sendCDvalue(gPcHandler.uart3,kd);
}

void processPcInterfaceMessage(PcReceiveHandler* handler,uint8_t* receivedData, uint8_t receivedDataIndex){
	processCommand(3, &(handler->uartProcessor), receivedData, receivedDataIndex);
}

float extractFloatValue(uint8_t uartNr, const char* value) {
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
int extractIntValue(uint8_t uartNr, const char* value) {
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
        char intStr[endIndex - startIndex + 1];
        strncpy(intStr, value + startIndex, endIndex - startIndex);
        intStr[endIndex - startIndex] = '\0';
        return atoi(intStr);
    }
    return 0;
}
