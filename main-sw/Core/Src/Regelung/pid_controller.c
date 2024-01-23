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
void set_setPoint(PIDController *pid, float setPoint) {
	pid->data.setpoint = setPoint;
}

float get_setPoint(const PIDController *pid) {
    return pid->data.setpoint;
}
float pid_update(PIDController *pid, float current_value) {
    // Berechne den Fehler
    float error =  current_value - pid->data.setpoint;

    // Berechne den Proportionalanteil
    float p_term = pid->data.kp * error;

    // Summe der Anteile für die Ausgabe
    float output = p_term;

    return output;
}

PIDController pid_init(float kp, float ki, float kd, float setpoint) {
    PIDController pidController;
    pidController.data.kp = kp;
    pidController.data.ki = ki;
    pidController.data.kd = kd;
    pidController.data.setpoint = 0.0;
    pidController.data.last_error = 0.0;
    pidController.data.integral = 0.0;
    pidController.data.setpoint = setpoint;

    //Initialisierung der getter FUnktionen
    pidController.get_kp = get_kp;
    pidController.get_ki = get_ki;
    pidController.get_kd = get_kd;
    pidController.get_setPoint = get_setPoint;

    //Initialisierung der setter Funktionen
    pidController.set_kp = set_kp;
    pidController.set_ki = set_ki;
    pidController.set_kd = set_kd;
    pidController.set_setPoint = set_setPoint;

    pidController.pid_update = pid_update;

    return pidController;
}

