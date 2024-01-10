/*
 * sensor_information.h
 *
 *  Created on: Nov 29, 2023
 *      Author: Daniel Alf
 */

#ifndef INC_SENSORACTUATORS_SENSOR_H_
#define INC_SENSORACTUATORS_SENSOR_H_

#include <stdint.h>

typedef struct Sensor Sensor;  // Vorwärtsdeklaration

typedef struct {
    float diameter;
    char  position;
    uint8_t uartPort;
} SensorInformation;

struct Sensor {
    SensorInformation info;

    // Getter-Methoden
    float   (*getDiameter)(const Sensor* sensor);
    char    (*getPosition)(const Sensor* sensor);
    uint8_t (*getUartPort)(const Sensor* sensor);

    // Setter-Methoden
    void (*setDiameter)(Sensor* sensor, float diameter);
    void (*setPosition)(Sensor* sensor, char position);
    void (*setUartPort)(Sensor* sensor, uint8_t uartPort);
};

// Konstruktor
Sensor initSensor(float diameter, uint8_t port);

#endif /* INC_SENSORACTUATORS_SENSOR_H_ */
