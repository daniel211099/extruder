/*
 * pid_controller.h
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_REGELUNG_PID_CONTROLLER_H_
#define INC_REGELUNG_PID_CONTROLLER_H_

typedef struct PIDController PIDController;

typedef struct {
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain

    float setpoint;  // Zielwert
    float last_error; // Letzter Fehlerwert für den D-Anteil

    float integral;   // Summe der Fehler für den I-Anteil
} PIDControllerData;

struct PIDController{
	PIDControllerData data;

	// Getter-Methoden
	float	(*get_kp)(const PIDController *pid);
	float	(*get_ki)(const PIDController *pid);
	float	(*get_kd)(const PIDController *pid);
	float   (*get_setPoint)(const PIDController *pid);

	// Setter Methoden
	void	(*set_kp)(PIDController *pid, float kp);
	void	(*set_ki)(PIDController *pid, float ki);
	void	(*set_kd)(PIDController *pid, float kd);
	void	(*set_setPoint)(PIDController *pid, float kd);

	// Funktionen für die PID-Regelung

	float  (*pid_update)(PIDController *pid, float current_value);
};

PIDController pid_init(float kp, float ki, float kd, float setPoint);


#endif /* INC_REGELUNG_PID_CONTROLLER_H_ */
