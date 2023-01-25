#include "bme280_sensor.h"

Adafruit_BME280 sensor;

void SensorInit(void)
{
    while (!Serial.available())
    {
        /* Wait for Serial to init */
    }

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);

    if (!sensor.begin(SENSOR_ADDR))
    {
        Serial.println("ERR: BME280 sensor not found!");
        while (true)
        {
            /* Infinite loop, do not continue if no sensor */
        }
    }

    Serial.println("BME280 sensor connected.");
}