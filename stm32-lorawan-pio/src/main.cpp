#include "main.h"

/**
 * Defines how the module behaves
 * Options: MASTER, SLAVE
 */
#define MODULE_TYPE LoRa::SLAVE

uint8_t loraMessage[8];
uint8_t receivedMessage[8];

bool nextMessage = true;
uint32_t currentTime = 0;

LoRa::ModuleType_t moduleType = MODULE_TYPE;
DataRead_t readData;
DataReceived_t receivedData;

void setup()
{
  /* Init LoRa shield */
  LoRa::ShieldInit(MODULE_TYPE);

  /* Init data container structs */
  BME280::DataInit(&readData);
  LoRa::DataInit(&receivedData);
}

void loop()
{
  switch (moduleType)
  {
  case LoRa::SLAVE:
    if (nextMessage)
    {
      BME280::ReadData(&readData);
      LoRa::SendMessage(&readData, loraMessage);

      currentTime = millis();
      nextMessage = false;
    }

    if (millis() - currentTime >= SEND_PERIOD_MS)
    {
      nextMessage = true;
    }
    break;

  case LoRa::MASTER:
    if (loraRadio.read(receivedMessage) > 0)
    {
      LoRa::ReceiveMessage(&receivedData, receivedMessage);
    }
    break;

  default:
    break;
  }
}
