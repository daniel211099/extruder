/*
 * motor_control.h
 *
 *  Created on: Jan 5, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_MOTOR_MOTOR_CONTROL_H_
#define INC_MOTOR_MOTOR_CONTROL_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct Motor Motor; // Vorwärtsdeklaration

typedef struct {
	int speed;
	TIM_HandleTypeDef* timer;
} MotorInformation;


// Definition der Motor "Klasse"
struct Motor {
    MotorInformation info;
    // Getter-Methoden
    int (*getSpeed)(const Motor *motor);
    // Setter-Methoden
    void (*setSpeed)(Motor *motor, int speed);
};

// Konstruktor
Motor initMotor(TIM_HandleTypeDef *timer);
void generatePWM(Motor* motor, int frequency);
void calculate_Periode_Motor(Motor *motor,float f);
#endif /* INC_MOTOR_MOTOR_CONTROL_H_ */
