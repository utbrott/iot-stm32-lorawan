#ifndef _BME280_H
#define _BME280_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define SDA_PIN PB7
#define SCL_PIN PB6
#define SENSOR_ADDR 0x77

typedef struct
{
    uint16_t temperature;
    uint16_t pressure;
    uint16_t altitude;
    uint16_t humidity;
} DataRead_t;

typedef struct
{
    float temperature;
    float pressure;
    float altitude;
    float humidity;
} DataReceived_t;

namespace BME280
{
    /* Inits BME280 sensor*/
    extern void HardwareInit(void);
    extern void DataInit(DataRead_t *data);
};

#endif /* _BME_280 */
