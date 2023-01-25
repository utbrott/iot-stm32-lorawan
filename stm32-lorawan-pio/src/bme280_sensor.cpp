#include "bme280_sensor.h"

Adafruit_BME280 sensor;

void BME280::HardwareInit(void)
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

void BME280::DataInit(DataRead_t *data)
{
    data->temperature = 0;
    data->pressure = 0;
    data->altitude = 0;
    data->humidity = 0;
}
