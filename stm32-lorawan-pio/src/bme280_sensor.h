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

void SensorInit(void);

#endif /* _BME_280 */