#ifndef _LORA_H
#define _LORA_H

#include <Arduino.h>
#include <Wire.h>
#include "LoRaRadio.h"

#include "bme280_sensor.h"

#define SEND_PERIOD_MS 5000

namespace LoRa
{

    typedef enum
    {
        SLAVE,
        MASTER,
    } ModuleType_t;

    /**
     * Init Arduino Serial and LoRa module
     * Init BME280 sensor if LoRa is specififed as SLAVE
     */
    void ShieldInit(ModuleType_t moduleType);
    void DataInit(DataReceived_t *data);

    void SendMessage(DataRead_t *data);
    void ReceiveMessage(DataReceived_t *data, uint8_t message[]);
};

#endif /* _LORA_H */
