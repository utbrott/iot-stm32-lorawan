# STM32 LoRa WAN

Embedded Systems IOT project that utilizes STM32 L152 Nucleo and USI STM32 LoRa
expansion boardto create a two-way LoRa message exchange.

Developed for IOT: Internet of Things course @ WUT Warsaw University of
Technology.

<h6>
  Note: this repositiory is a shallow copy of the original repo, where the
  project was originally developed, created only for future development
  possibilites. Purpose of this shallow copy is to remove old branches, code
  and unstandarized commits. If you wish to view the orignal repo, follow
  <a href="https://github.com/piotrs112/stm32-lorawan.git">this link</a>.
</h6>

# Features

- BME280 Sensor to read surrouding's general temperature and pressure
- Request to measuring SLAVE is sent only when button on MASTER board is pressed,

# Built with

- STM32 L152 Nucleo-64 development board,
- USI STM32 Nucleo expansion board for LoRa,
- BME280 I2C/SPI breakout board,

# File structure:

```
|
|-- src
|   |
|   |- main.h
|   |- main.cpp
|   |- lora.h (interfacing with LoRa WAN Shield)
|   |- lora.cpp
|   |- bme280_sensor.h (interfacing, using Adafruit BME280 sensor)
|   |- bme280_sensor.cpp
|
|- platformio.ini (project config for PlatformIO)
|- README.md --> This file
```

## Future development note

Project was built using PlatformIO extension for VSCode.
