/*
 * pc_send_handler.h
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_KOMMUNIKATION_PC_SEND_HANDLER_H_
#define INC_KOMMUNIKATION_PC_SEND_HANDLER_H_

#include "stm32f4xx_hal.h"
#include "Management/global_state_machine.h"


typedef struct {
    void (*sendDiamExt)	(UART_HandleTypeDef *huart, float diameter);
    void (*sendDiamBack)(UART_HandleTypeDef *huart, float diameter);
    void (*sendState)	(UART_HandleTypeDef *huart, State state);
    void (*sendSpeed)	(UART_HandleTypeDef *huart, float speed);
    void (*sendCPvalue)	(UART_HandleTypeDef *huart, float cp);
    void (*sendCIvalue)	(UART_HandleTypeDef *huart, float ci);
    void (*sendCDvalue)	(UART_HandleTypeDef *huart, float cd);

    uint32_t timeout;
} PcSendInterface;

typedef struct {
    PcSendInterface interface;
    // Weitere interne Daten oder Konfigurationen, die benötigt werden
} PcSendHandler;

PcSendHandler createPcSendHandler();



#endif /* INC_KOMMUNIKATION_PC_SEND_HANDLER_H_ */
