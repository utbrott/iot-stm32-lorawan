# STM32 LoraWan

Embedded Systems IOT project that utilizes STM32 L152 Nucleo, OpenWeatherMap to
create a simple home weather station.

Developed for IOT: Internet of Things course @ WUT Warsaw University of
Technology.

Visit the [Wiki](https://github.com/piotrs112/stm32-lorawan/wiki) to read more
about the project.

<h6>
  Warning: Currently project's Wiki is only in Polish, as the course is in Polish
</h6>

# Features

- Requesting data from OpenWeatherMap for outside weather data,
- BME280 Sensor to read surrouding's general temperature, humidity and pressure
- MCP9808 Sensor to read surrouding's accurate temperature.

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
|- LICENSE
|- platformio.ini
|- README.md --> This file
```

## Future development note

Project was built using PlatformIO extension for VSCode.
