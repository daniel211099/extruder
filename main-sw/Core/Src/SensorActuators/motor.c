/*
 * motor_control.c
 *
 *  Created on: Jan 5, 2024
 *      Author: Daniel Alf
 */

#include <SensorActuators/motor.h>
#include "stm32f4xx_hal.h"
// Implementierung der Getter-Methode für die Geschwindigkeit des Motors
float getSpeed(const Motor *motor) {
    return motor->info.speed;
}

// Implementierung der Setter-Methode für die Geschwindigkeit des Motors
void setSpeed(Motor *motor, float speed) {
	if(speed <= 0.5){
		HAL_TIM_PWM_Stop(motor->info.timer, 0);
	    motor->info.speed = 0;
		return;
	}

	if(speed > 100){
		return;
	}
    motor->info.speed = speed;
	  // Geschwindigkeit in Frequenz umrechnen
	int frequency = (speed * 3200) / 60;

	      // PWM-Signal generieren
	generatePWM(motor,frequency);
	HAL_TIM_PWM_Start(motor->info.timer, 0);

}

void generatePWM(Motor *motor, int frequency) {
	  uint32_t period = (HAL_RCC_GetHCLKFreq() / ((motor->info.timer->Init.Prescaler + 1)* frequency)) - 1;

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};

	  motor->info.timer->Init.Period = period; //26250-1
	  HAL_TIM_Base_Init(motor->info.timer);
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(motor->info.timer, &sClockSourceConfig);
	  HAL_TIM_PWM_Init(motor->info.timer);
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  HAL_TIMEx_MasterConfigSynchronization(motor->info.timer, &sMasterConfig);
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = motor->info.timer->Init.Period / 2; //13125-1
	  HAL_TIM_PWM_ConfigChannel(motor->info.timer, &sConfigOC, TIM_CHANNEL_1);
	  HAL_TIM_MspPostInit(motor->info.timer);


}

// Implementierung des Konstruktors für den Motor
Motor initMotor(TIM_HandleTypeDef *timer) {
    Motor newMotor;
    newMotor.info.speed = 0;
    newMotor.info.timer = timer;
    newMotor.getSpeed = &getSpeed;
    newMotor.setSpeed = &setSpeed;

    newMotor.setSpeed(&newMotor, 0);

    return newMotor;
}
