#include "main.h"

/**
 * Defines how the module behaves
 * Options: MASTER, SLAVE
 */
#define MODULE_TYPE LoRa::SLAVE

uint8_t requestMessage[1];
uint8_t receivedMessage[8];

bool nextMessage = true;
uint8_t interruptState = 0;
uint32_t currentTime = 0;

LoRa::ModuleType_t moduleType = MODULE_TYPE;
DataRead_t sensorData;
DataReceived_t receivedData;

void ButtonClickInterrupt(void);

void setup()
{
  /* Init LoRa shield */
  LoRa::ShieldInit(MODULE_TYPE);

  /* Init data container structs */
  BME280::DataInit(&sensorData);
  LoRa::DataInit(&receivedData);

  if (moduleType == LoRa::MASTER)
  {
    attachInterrupt(digitalPinToInterrupt(BOARD_BTN), ButtonClickInterrupt, RISING);
  }
}

void loop()
{

  switch (moduleType)
  {
  case LoRa::SLAVE:
    if (loraRadio.read(requestMessage) == 0x04)
    {
      memset(requestMessage, 0, 1);

      BME280::ReadData(&sensorData);
      LoRa::SendResponse(&sensorData);
      // currentTime = millis();
      // nextMessage = false;
    }

    // if (millis() - currentTime >= SEND_PERIOD_MS)
    // {
    //   nextMessage = true;
    // }

    break;

  case LoRa::MASTER:
    if (interruptState)
    {
      LoRa::SendRequest();
      BOOL(interruptState);
    }

    if (loraRadio.read(receivedMessage) > 0)
    {
      LoRa::ReadResponse(&receivedData, receivedMessage);
    }

    break;

  default:
    break;
  }
}

void ButtonClickInterrupt(void)
{
  BOOL(interruptState);
}
