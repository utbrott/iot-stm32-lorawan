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

    if (moduleType)
    {
        BME280::SensorInit();
    }
}
