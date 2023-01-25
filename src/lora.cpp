#include "lora.h"

HardwareSerial SerialLora(PA10, PA9);

void LoRa::ShieldInit(ModuleType_t moduleType)
{
    Serial.begin(115200);

    Serial.println("**** IOT ****");
    switch (moduleType)
    {
    case SLAVE:
        Serial.println("LoRa SLAVE Module");
        break;

    case MASTER:
        Serial.println("LoRa MASTER Module");
        break;

    default:
        break;
    }

    while (!loraRadio.begin(&SerialLora))
    {
        Serial.println("[INFO] LoRa Shield not ready!");
        delay(1000); /* Give module 1s to init */
    }

    Serial.println("[INFO] Shield ready!");

    if (moduleType == SLAVE)
    {
        BME280::HardwareInit();
    }
}

void LoRa::DataInit(DataReceived_t *data)
{
    data->temperature = 0;
    data->pressure = 0;
}

void LoRa::SendRequest(void)
{

    uint8_t message[1];
    message[0] = 0xFF;

    Serial.println("[INFO] Sending new request");

    loraRadio.write(message, 1);
}

void LoRa::SendResponse(DataRead_t *data)
{
    uint8_t message[8];

    /* Split each 16-bit data to 2x8-bit ones, with bit masking */
    message[0] = (data->temperature & 0xFF00) >> 8;
    message[1] = (data->temperature & 0x00FF);

    message[2] = (data->pressure & 0xFF00) >> 8;
    message[3] = (data->pressure & 0x00FF);

    Serial.println("[INFO] Sending new packet");

    loraRadio.write(message, 8);
}

void LoRa::ReadResponse(DataReceived_t *data, uint8_t message[])
{
    /* Merge each 2x8-bit packs to 16-bit ones, fix floats */
    data->temperature = (float)((message[0] << 8) + message[1]) / 100;
    data->pressure = (float)((message[2] << 8) + message[3]);

    memset(message, 0, 8);

    /* Message strings array */
    String serialMessage[3] = {
        "[INFO] New data packet received",
        "Temperature: " + String(data->temperature) + " \u00b0C",
        "Pressure: " + String(data->pressure) + " hPa",
    };

    for (uint8_t idx = 0; idx < (sizeof(serialMessage) / sizeof(serialMessage[0])); idx++)
    {
        Serial.println(serialMessage[idx]);
    }

    Serial.println();
}
