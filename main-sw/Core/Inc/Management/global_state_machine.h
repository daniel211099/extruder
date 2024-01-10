/*
 * global_state_machine.h
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#ifndef INC_MANAGEMENT_GLOBAL_STATE_MACHINE_H_
#define INC_MANAGEMENT_GLOBAL_STATE_MACHINE_H_

#include <SensorActuators/motor.h>

typedef struct StateMachine StateMachine;

// Definition der Zustände
typedef enum {
    STATE_IDLE = 0,
    STATE_MANUAL_CONTROL = 1,
    STATE_AUTOMATIC_MODE = 2
} State;


typedef struct {
    State currentState;
    Motor* motor;
}StateMachineInformation;

// Definition der State Machine "Klasse"
struct StateMachine {
	StateMachineInformation info;

    // Methoden für die State Machine
    void  (*init)		(struct StateMachine *machine);
    void  (*changeState)(struct StateMachine *machine, State newState);
    State (*getState)	(const struct StateMachine *machine);
};

StateMachine initStateMachine(Motor* motor);

#endif /* INC_MANAGEMENT_GLOBAL_STATE_MACHINE_H_ */
