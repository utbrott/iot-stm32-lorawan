#include "bme280_sensor.h"

Adafruit_BME280 sensor;

void BME280::HardwareInit(void)
{
    Serial.println("[INFO] Trying to connect BME280 sensor...");

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);

    if (!sensor.begin(SENSOR_ADDR))
    {
        Serial.println("[ERR] BME280 sensor not found!");
        while (true)
        {
            /* Infinite loop, do not continue if no sensor */
        }
    }

    Serial.println("[INFO] BME280 sensor connected.");
}

void BME280::DataInit(DataRead_t *data)
{
    data->temperature = 0;
    data->pressure = 0;
}

void BME280::ReadData(DataRead_t *data)
{
    /* Magnitude change for easier sending */
    data->temperature = (sensor.readTemperature() * 100);
    data->pressure = (sensor.readPressure() / 100.00f);
}
