/*
 * pc_send_handler.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */


#include <Kommunikation/pc_send_handler.h>
#include <stdio.h>
#include <string.h>


// Implementierung der Interface methoden
void sendDiamExt_impl(UART_HandleTypeDef *huart,float value) {
	uint8_t tag[] = "DIAMEXT ";
	uint8_t diameter[10];
	uint8_t endTag[] = " /r/n";

	sprintf((char *)diameter, "%.2f", value);

	HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
   	HAL_UART_Transmit(huart, diameter, strlen((char*)diameter), 10);
	HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}
void sendDiamBack_impl(UART_HandleTypeDef *huart,float value) {
	uint8_t tag[] = "DIAMBACK ";
	uint8_t diameter[10];
	uint8_t endTag[] = " /r/n";

	sprintf((char *)diameter, "%.2f", value);

	HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
   	HAL_UART_Transmit(huart, diameter, strlen((char*)diameter), 10);
	HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}

void sendState_impl(UART_HandleTypeDef *huart, State state) {
    uint8_t tag[] = "STATE ";
    uint8_t stateStr[10];
    uint8_t endTag[] = "\r\n";

    sprintf((char *)stateStr, "%d", state);

    HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
    HAL_UART_Transmit(huart, stateStr, strlen((char*)stateStr), 10);
    HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}
void sendSpeed_impl(UART_HandleTypeDef *huart, float speed){
    uint8_t tag[] = "SPEED ";
    uint8_t value[10];
    uint8_t endTag[] = "\r\n";

    sprintf((char *)value, "%.2f", speed);

    HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
    HAL_UART_Transmit(huart, value, strlen((char*)value), 10);
    HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);

}
void sendCPvalue_impl(UART_HandleTypeDef *huart, float cp){
    uint8_t tag[] = "CP ";
    uint8_t value[10];
    uint8_t endTag[] = "\r\n";

    sprintf((char *)value, "%.2f", cp);

    HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
    HAL_UART_Transmit(huart, value, strlen((char*)value), 10);
    HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}
void sendCIvalue_impl(UART_HandleTypeDef *huart, float ci){
    uint8_t tag[] = "CI ";
    uint8_t value[10];
    uint8_t endTag[] = "\r\n";

    sprintf((char *)value, "%.2f", ci);

    HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
    HAL_UART_Transmit(huart, value, strlen((char*)value), 10);
    HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}
void sendCDvalue_impl(UART_HandleTypeDef *huart, float cd){
    uint8_t tag[] = "CD ";
    uint8_t value[10];
    uint8_t endTag[] = "\r\n";

    sprintf((char *)value, "%.2f", cd);

    HAL_UART_Transmit(huart, tag, strlen((char*)tag), 10);
    HAL_UART_Transmit(huart, value, strlen((char*)value), 10);
    HAL_UART_Transmit(huart, endTag, strlen((char*)endTag), 10);
}
PcSendHandler createPcSendHandler(uint32_t timeout) {
	PcSendHandler handler;
    handler.interface.sendDiamExt	= sendDiamExt_impl;
    handler.interface.sendDiamBack	= sendDiamBack_impl;
    handler.interface.sendState		= sendState_impl;
    handler.interface.sendSpeed		= sendSpeed_impl;
    handler.interface.sendCPvalue	= sendCPvalue_impl;
    handler.interface.sendCIvalue	= sendCIvalue_impl;
    handler.interface.sendCDvalue	= sendCDvalue_impl;

    handler.interface.timeout = timeout;
    return handler;
}
