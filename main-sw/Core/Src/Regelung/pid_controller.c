/*
 * pid_controller.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */
#include "Regelung/pid_controller.h"

void set_kp(PIDController *pid, float kp) {
    pid->data.kp = kp;
}

float get_kp(const PIDController *pid) {
    return pid->data.kp;
}

void set_ki(PIDController *pid, float ki) {
	pid->data.ki = ki;
}

float get_ki(const PIDController *pid) {
    return pid->data.ki;
}

void set_kd(PIDController *pid, float kd) {
	pid->data.kd = kd;
}

float get_kd(const PIDController *pid) {
    return pid->data.kd;
}

PIDController pid_init(float kp, float ki, float kd) {
    PIDController pidController;
    pidController.data.kp = kp;
    pidController.data.ki = ki;
    pidController.data.kd = kd;
    pidController.data.setpoint = 0.0;
    pidController.data.last_error = 0.0;
    pidController.data.integral = 0.0;

    //Initialisierung der getter FUnktionen
    pidController.get_kp = get_kp;
    pidController.get_ki = get_ki;
    pidController.get_kd = get_kd;

    //Initialisierung der setter Funktionen
    pidController.set_kp = set_kp;
    pidController.set_ki = set_ki;
    pidController.set_kd = set_kd;

    return pidController;
}

float pid_update(PIDController *pid, float current_value) {
    // Berechne den Fehler
    float error = pid->data.setpoint - current_value;

    // Berechne den Proportionalanteil
    float p_term = pid->data.kp * error;

    // Berechne den Integralanteil
    pid->data.integral += error;
    float i_term = pid->data.ki * pid->data.integral;

    // Berechne den Derivativanteil
    float d_term = pid->data.kd * (error - pid->data.last_error);
    pid->data.last_error = error;

    // Summe der Anteile für die Ausgabe
    float output = p_term + i_term + d_term;

    return output;
}
