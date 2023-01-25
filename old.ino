
#include <Wire.h>
#include "LoRaRadio.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEND_PERIOD_MS 5000                                                  //Odczyt danych i wysyłka co 5s
#define SEALEVELPRESSURE_HPA (1013.25)                            

#define MASTER  true
#define SLAVE   false

Adafruit_BME280 bme;

// Serial port use to communicate with the USI shield.
// By default, use D0 (Rx) and D1(Tx).
// For Nucleo64, see "Known limitations" chapter in the README.md
  
HardwareSerial SerialLora(PA10, PA9);                                        //Ustawienie pinów dla UART LoRa

uint8_t lora_message[8];                                                    //Wiadomość wysyłana do mastera przez slave
uint8_t recived_message[8];                                                 //Wiadomość otrzymana przez mastera

/******************** WAŻNE !!! *********************************/
bool master_or_slave = MASTER;                                               //ZMIENIĆ NA MASTER LUB SLAVE (false), w zależności od tego na jaką płytkę chcesz wgrać kod
/****************************************************************/

bool next = true;
int timer = 0;

/********ZMIENNE DO OBSŁUGI CZUJNIKA*******/
uint16_t temperature = 0;
uint16_t pressure = 0;
uint16_t altitude = 0;
uint16_t humidity = 0;

float temperature_received = 0;
float pressure_received = 0;
float altitude_received = 0;
float humidity_received = 0;
/******************************************/
  
void setup()
{  
  Serial.begin(9600);
  Serial.println("********** INTERNET RZECZY **********");

  while(!loraRadio.begin(&SerialLora))                                          //Połączenie z LoRa
  {
    Serial.println("LoRa moduł nie gotowy");
    delay(1000);
  }
  Serial.println("LoRa moduł gotowy\n");

  if(master_or_slave == SLAVE)
  {
      Wire.setSCL(PB6);                                                         //Ustawienie pinów dla I2C czujnika
      Wire.setSDA(PB7);                                                         //Ustawienie pinów dla I2C czujnika
    
      if(!bme.begin(0x77))                                                      //Połączenie z czujnikiem
      {
          Serial.println("Sprawdź połączenie nie znajduje modułu");
          while (1);
      }
      Serial.println("Połączono z czujnikiem");
   }
}


void loop()
{
  
  if((master_or_slave == SLAVE) && (next == true))      //Odczyt danych z czujnika i wysyłka paczki przez SLAVE  (KOD TYLKO DLA SLAVE)                   
  {
      //Odczyt parametrów z czujnika
      temperature = (bme.readTemperature()*100);                                  //Temperatura w stopniach Celsjusza, pomnożona potem razy 100 do łatwiejszej wysyłki, bo jest to liczba po przecinku np 24,56 -> 2456                            
      pressure = (bme.readPressure() / 100.0F);                                   //Ciśnienie w hPa
      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA)*100;                      //Wysokość w metrach, pomnożona potem razy 100 do łatwiejszej wysyłki, bo jest to liczba po przecinku np 24,56 -> 2456 
      humidity = (bme.readHumidity()*100);                                        //Wilgotność w procentach, pomnożona potem razy 100 do łatwiejszej wysyłki, bo jest to liczba po przecinku np 24,56 -> 2456  

      //Przygotowanie paczki do wysyłki, dzielenie każdej zmiennej 16bitowej na dwie 8bitowe
      lora_message[0] = (temperature & 0xFF00)>>8;
      lora_message[1] = (temperature & 0x00FF);
      lora_message[2] = (pressure & 0xFF00)>>8;
      lora_message[3] = (pressure & 0x00FF);
      lora_message[4] = (altitude & 0xFF00)>>8;
      lora_message[5] = (altitude & 0x00FF);
      lora_message[6] = (humidity & 0xFF00)>>8;
      lora_message[7] = (humidity & 0x00FF);

      Serial.println(lora_message[0]);      //Sprawdzenie
      Serial.println(lora_message[1]);
      Serial.println(lora_message[2]);
      Serial.println(lora_message[3]);
      Serial.println(lora_message[4]);
      Serial.println(lora_message[5]);
      Serial.println(lora_message[6]);
      Serial.println(lora_message[7]);
     
      loraRadio.write(lora_message, 8);
      timer = millis();
      next = false;
  }


  if((master_or_slave == MASTER) && (loraRadio.read(recived_message) > 0))      //(KOD TYLKO DLA MASTER)  
  {
      //Zamiana odebranych danych 8bitowych na zmienne 16bitowe i zamiana na poprawne wartości (liczby po przecinki np 25,45)
      temperature_received = (float)((recived_message[0]<<8) + recived_message[1]) / 100;
      pressure_received = (float)(recived_message[2]<<8) + recived_message[3];
      altitude_received = (float)((recived_message[4]<<8) + recived_message[5]) / 100;
      humidity_received = (float)((recived_message[6]<<8) + recived_message[7]) / 100;
           
      memset(recived_message, 0, 8);

      Serial.print("Temperature = ");
      Serial.print(temperature_received);
      Serial.println(" °C");
    
      Serial.print("Pressure = ");
      Serial.print(pressure_received);
      Serial.println(" hPa");
    
      Serial.print("Approx. Altitude = ");
      Serial.print(altitude_received);
      Serial.println(" m");
    
      Serial.print("Humidity = ");
      Serial.print(humidity_received);
      Serial.println(" %");

      Serial.println();
  }


  if(((millis() - timer) >= SEND_PERIOD_MS) && (master_or_slave == SLAVE))     //(KOD TYLKO DLA SLAVE)                
  {
      next = true;
  }
  
}
