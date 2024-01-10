/*
 * motor_control.c
 *
 *  Created on: Jan 5, 2024
 *      Author: Daniel Alf
 */

#include <SensorActuators/motor.h>
#include "stm32f4xx_hal.h"
// Implementierung der Getter-Methode für die Geschwindigkeit des Motors
int getSpeed(const Motor *motor) {
    return motor->info.speed;
}

// Implementierung der Setter-Methode für die Geschwindigkeit des Motors
void setSpeed(Motor *motor, int speed) {
    motor->info.speed = speed;

    if(speed <= 0){
    	return;
    }
    // Geschwindigkeit (u/min) in Frequenz umrechnen
    float frequency = speed * 3200.0 * 60.0;

    //calculate_Periode_Motor(motor,frequency);

}
/*
void generatePWM(Motor *motor, int frequency) {
    // Frequenz in Timer-Wert umrechnen
    uint16_t timerValue = HAL_RCC_GetHCLKFreq() / frequency;

    // Timer-Wert setzen
    __HAL_TIM_SET_AUTORELOAD(motor->info.timer, timerValue);
}*/

void calculate_Periode_Motor(Motor *motor,float f) {

	HAL_TIM_Base_Stop(motor->info.timer);
    // Grundfrequenz des TIM3-Timers
    uint32_t tim3Grundfrequenz = HAL_RCC_GetSysClockFreq();
    uint32_t prescaler = motor->info.timer->Init.Prescaler;
    // Berechnung der Periode
    uint32_t periode = (tim3Grundfrequenz / (prescaler * f)) - 1;

    // Überprüfung, ob die Periode im gültigen Bereich liegt
    if (periode > 0xFFFF) {
        // Periode ist zu groß, setze den Maximalwert
        periode = 0xFFFF;
    }
    motor->info.timer->Init.Period = periode;

    //init_Timer(periode);
	HAL_TIM_Base_Init(motor->info.timer);
    HAL_TIM_Base_Start_IT(motor->info.timer);
}


// Implementierung des Konstruktors für den Motor
Motor initMotor(TIM_HandleTypeDef *timer) {
    Motor newMotor;
    newMotor.info.speed = 0;
    newMotor.info.timer = timer;
    newMotor.getSpeed = &getSpeed;
    newMotor.setSpeed = &setSpeed;
    return newMotor;
}
