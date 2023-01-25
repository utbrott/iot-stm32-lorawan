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
        MASTER = 0,
        SLAVE = 1,
    } ModuleType_t;

    /**
     * Init Arduino Serial and LoRa module
     * Init BME280 sensor if LoRa is specififed as SLAVE
     */
    void
    ShieldInit(ModuleType_t moduleType);
};

#endif /* _LORA_H */
