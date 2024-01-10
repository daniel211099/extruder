/*
 * global_state_machine.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Daniel Alf
 */

#include <Management/global_state_machine.h>

// Change the state of the State Machine
static void changeState(StateMachine *machine, State newState) {
    switch (newState) {
        case STATE_IDLE:
            machine->info.motor->setSpeed(machine->info.motor, 0);
            break;
        case STATE_MANUAL_CONTROL:
            // TODO: Logik für manuellen Betrieb implementieren
            break;
        case STATE_AUTOMATIC_MODE:
            // TODO: Logik für Regelung implementieren
            break;
        default:
            // Handle unknown state
            break;
    }
    machine->info.currentState = newState;
}
// Get the current state of the State Machine
static State getState(const StateMachine *machine) {
    return machine->info.currentState;
}

// Initialization function for the State Machine
StateMachine initStateMachine(Motor* motor) {
    StateMachine machine;

    machine.info.motor = motor;

    machine.changeState = changeState;
    machine.getState = getState;

    // Set initial state and initialize motor
    machine.info.currentState = STATE_IDLE;
    return machine;
}