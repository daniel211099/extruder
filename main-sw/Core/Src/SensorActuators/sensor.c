/*
 * sensor.c
 *
 *  Created on: Nov 29, 2023
 *      Author: Daniel Alf
 */


#include <stdlib.h>
#include <stdint.h>
#include <SensorActuators/sensor.h>

// Getter-Methoden
float getDiameter(const Sensor* sensor) {
    return sensor->info.diameter;
}
char getPosition(const Sensor* sensor) {
	return sensor->info.position;
}
uint8_t getUartPort(const Sensor* sensor) {
	return sensor->info.uartPort;
}

// Setter-Methoden
void setDiameter(Sensor* sensor, float diameter) {
    sensor->info.diameter = diameter;
}
void setPosition(Sensor* sensor, char position) {
	sensor->info.position = position;
}
void setUartPort(Sensor* sensor, uint8_t port){
	sensor->info.uartPort = port;
}
// Konstruktor
Sensor initSensor(float diameter, uint8_t port) {
    Sensor sensor;
    sensor.info.diameter = diameter;
    sensor.info.uartPort = port;

    // Initialisiere die Getter-Funktionen
    sensor.getDiameter = getDiameter;
    sensor.getUartPort = getUartPort;

    // Initialisiere die Setter-Funktionen
    sensor.setDiameter = setDiameter;

    return sensor;
}
