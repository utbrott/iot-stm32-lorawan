#include "lora.h"

HardwareSerial SerialLora(PA10, PA9);

void LoRa::ShieldInit(ModuleType_t moduleType)
{
    Serial.begin(115200);
    Serial.println("IOT::LoRa");

    while (!loraRadio.begin(&SerialLora))
    {
        Serial.println("LoRa Shield not ready!");
        delay(1000); /* Give module 1s to init */
    }

    Serial.print("LoRa module ready!");

    if (moduleType == SLAVE)
    {
        BME280::HardwareInit();
    }
}

void LoRa::DataInit(DataReceived_t *data)
{
    data->temperature = 0;
    data->pressure = 0;
    data->altitude = 0;
    data->humidity = 0;
}

void LoRa::SendMessage(DataRead_t *data, uint8_t message[])
{
    /* Split each 16-bit data to 2x8-bit ones, with bit masking */
    message[0] = (data->temperature & 0xFF00) >> 8;
    message[1] = (data->temperature & 0x00FF);

    message[2] = (data->pressure & 0xFF00) >> 8;
    message[3] = (data->pressure & 0x00FF);

    message[4] = (data->altitude & 0xFF00) >> 8;
    message[5] = (data->altitude & 0x00FF);

    message[6] = (data->humidity & 0xFF00) >> 8;
    message[7] = (data->humidity & 0x00FF);

    for (uint8_t idx = 0; idx < 8; idx++)
    {
        /* Print each package element to Serial */
        Serial.println(message[idx]);
    }

    Serial.println();

    loraRadio.write(message, 8);
}

void LoRa::ReceiveMessage(DataReceived_t *data, uint8_t message[])
{
    /* Merge each 2x8-bit packs to 16-bit ones, fix floats */
    data->temperature = (float)(message[0] << 8 + message[1]);
    data->pressure = (float)(message[2] << 8 + message[3]);
    data->altitude = (float)(message[4] << 8 + message[5]);
    data->humidity = (float)(message[6] << 8 + message[7]);

    memset(message, 0, 8);

    /* Message strings array */
    String serialMessage[4] = {
        "Temperature: " + String(message[0]) + " \u00b0C",
        "Pressure: " + String(message[1]) + " hPa",
        "Altitude: " + String(message[2]) + " m",
        "Humidity: " + String(message[3]) + " %",
    };

    for (uint8_t idx = 0; idx < 4; idx++)
    {
        Serial.println(serialMessage[idx]);
    }

    Serial.println();
}
