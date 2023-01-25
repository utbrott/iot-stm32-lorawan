#ifndef _MAIN_H
#define _MAIN_H

#include <Arduino.h>
#include "lora.h"
#include "bme280_sensor.h"

extern DataRead_t readData;
extern DataReceived_t receivedData;
extern LoRa::ModuleType_t moduleType;

extern uint8_t loraMessage[8];
extern uint8_t receivedMessage[8];

extern bool nextMessage;
extern uint32_t currentTime;

#endif /* _MAIN_H */
