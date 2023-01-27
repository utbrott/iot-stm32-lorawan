/******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    01-June-2017
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power.h"
#include "bsp.h"
#include "timeServer.h"
#include <string.h>
#include <stdlib.h>

#include "stm32l0xx_hal.h"

#include "lora_driver.h"
#include <stdarg.h>
    
//#include "atcmd.h"
#include ATCMD_MODEM        /* preprocessing definition in hw_conf.h*/    



/* Private typedef -----------------------------------------------------------*/
/*!
 * LoRa modem State Machine states 
 */
typedef enum eDevicState
{
    DEVICE_INIT,
    DEVICE_READY,
    DEVICE_JOINED,
    DEVICE_SEND,
    DEVICE_SLEEP
} DeviceState_t;
/* Private define ------------------------------------------------------------*/

/* CAYENNE_LLP is myDevices Application server*/
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT 0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY 0x68
#define LPP_DATATYPE_TEMPERATURE 0x67
#define LPP_DATATYPE_BAROMETER 0x73



/* Private variables ---------------------------------------------------------*/
__IO uint8_t PushButtonOn = 0;
__IO uint8_t SensorTimerWakeup = 0;

/* Private variables ---------------------------------------------------------*/
static char DataBinaryBuff[64];

#ifdef USE_MDM32L07X01
static sSendDataBinary_t SendDataBinary={ DataBinaryBuff, 0 , 0};
#elif USE_I_NUCLEO_LRWAN1
static sSendDataBinary_t SendDataBinary={ DataBinaryBuff, 0 , 0 ,0};

#endif

static sensor_t Sensor;                            /* struct for data sensor*/ 
    
static TimerEvent_t NextSensorMeasureTimer; /*timer to handle next sensor measure*/

static DeviceState_t DeviceState = DEVICE_INIT ;

static TimerEvent_t DemoLedTimer;                  /*timer to handle Demo Led*/

/* Private function prototypes -----------------------------------------------*/

static void OnNextSensorMeasureTimerEvt( void );

static void Lora_Modem( void);

static void SensorMeasureData( void );

static void OnLedTimerEvent( void );       /*timer function to handle Demo Led*/

/* Private macro ------------- -----------------------------------------------*/


#define LORAWAN_APP_PORT           2;            /*LoRaWAN application port*/

#define LORAWAN_CONFIRMED_MSG      DISABLE       /*LoRaWAN confirmed messages*/

#define SENSORS_MEASURE_CYCLE      5000          /*Periode to do sensors measurement*/

#if 0
#define JOIN_MODE       ABP_JOIN_MODE            /*LoRaWan join methode*/ 
#else                                            
#define JOIN_MODE       OTAA_JOIN_MODE
#endif

  /******************************************************************************/
  /*                    To put USI modem in sleep mode                          */
  /*  From USI FW V2.6, modem sleep mode is only supported on ABP Join mode     */
  /*  From USI FW V3.0, modem sleep mode is supported for ABP and OTAA Join mode*/ 
  /******************************************************************************/

                                         
/*put in comment MODEM_IN_SLEEP_MODE defined in "hw_conf.h" */
/*if we do not want to have the USI MCU modem in sleep mode*/  

#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)
#define USI_FW_V26               26
#define USI_FW_V30               30
#define STRING_USI_FW_V26       "2.6"
#define STRING_USI_FW_V30       "3.0"
static sPowerCtrlSet_t   PowerCtrlSettings= {0,1,0};
uint8_t FWVersion;
#endif





/**
 * @brief  Entry point program
 * @param  None
 * @retval int
 */

int main(void)
{


  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  HW_GpioInit();

  /* Configure the debug mode*/
#if 0  
  DBG_Init( );
#endif  

#ifdef USE_I_NUCLEO_LRWAN1  
    BSP_LED_Modem_Init(LED_GREEN);   /*Led indicator on Modem slave device*/
#elif USE_MDM32L07X01    
    BSP_LED_Init(LED2);              /*Led indicator on Nucleo master board*/
#endif     

  /* Configure the hardware*/
  HW_Init( );
  
  /*Do a Modem reset in a hidden way in order to synchronize Host and Modem*/
//  Lora_Reset();
  
    
  /***************************************************************************/
  /*          disabling the echo mode which is by default active             */
  /*        from USI FW V2.6, echo mode will be disable by default.          */
  /*        Then this line could be removed                                  */
  /***************************************************************************/
#ifdef USE_I_NUCLEO_LRWAN1 
uint8_t   Mode = 0; 

   Modem_AT_Cmd(AT_EXCEPT, AT_ATE, &Mode );
#endif   
   
   
  /***************************************************************************/
  /*    To discriminate the FW version where Sleep mode works correctly      */
  /***************************************************************************/   
#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)
uint8_t   RetCode;
uint8_t PtrFWVersion[6];

   Lora_GetFWVersion(PtrFWVersion);   
   RetCode = strncmp((char*)PtrFWVersion, STRING_USI_FW_V26, sizeof(STRING_USI_FW_V26)-1);   
   if (RetCode == 0)
     FWVersion = USI_FW_V26;    /* FW version where OTAA bug prevents the sleep mode (MCU slave stop mode)*/
   else
     FWVersion = USI_FW_V30;    /* FW version where OTAA bug has been fixed*/
#endif                    
   
   
  DBG_PRINTF("before idle \n");
    
  /* Infinite loop */
  while (1)
  {
  
    /* run the LoRa Modem state machine*/
    Lora_Modem( );
    
    DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */
    if ( (DeviceState == DEVICE_SLEEP) && (HW_UART_Modem_IsNewCharReceived() == RESET))
    {
#ifndef LOW_POWER_DISABLE
       LowPower_Handler( );
#endif
    }
    ENABLE_IRQ();  

  }  
}
 

/******************************************************************************
  * @Brief lora Modem state machine
  * @param void
  * @retval None   
******************************************************************************/
void Lora_Modem( void)
{
RetCode_t LoraModuleRetCode; 
ATEerror_t LoraCmdRetCode;


  switch( DeviceState )
  {
    case DEVICE_INIT:
    {
       /* Check if the LoRa Modem is ready to work*/

       LoraModuleRetCode = Lora_Init();          
   
       if (LoraModuleRetCode == MODULE_READY)
       {  
          DeviceState = DEVICE_READY; 
          DBG_PRINTF("Lora module ready\n");
 
#if USE_MDM32L07X01           
          /*Set the modem Join mode following application set-up*/
          LoraCmdRetCode = Lora_SetJoinMode(JOIN_MODE);
#endif
          
#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)
          /*Set the low power control - Modem MCU will enter stop mode if get command at+sleep */
          if(((JOIN_MODE == ABP_JOIN_MODE) && (FWVersion == USI_FW_V26)) || (FWVersion == USI_FW_V30))
          LoraCmdRetCode = Lora_SetMCUPowerCtrl(&PowerCtrlSettings);
#endif          
            
          /*to adapt the data rate during transmission*/
          LoraCmdRetCode = Lora_SetAdaptiveDataRate(ADAPT_DATA_RATE_ENABLE); 
          
          /* Timer for sensor occurence measure*/
          TimerInit( &NextSensorMeasureTimer, OnNextSensorMeasureTimerEvt );
       }   
       else
       {  
          DBG_PRINTF("Lora module not ready\n");   /*we stay in Init state and redo Lora_Init*/
       }
    break;
    }
    case DEVICE_READY:
    {
        /* Do a join request to the LoRaWAN network - can be ABP or OTAA  Join mode*/
        /* Nota : Join Mode parameter relevant for USI modem - For Murata not releavant cf.User manual*/


#ifdef USE_I_NUCLEO_LRWAN1             /* Led on Modem slave device to indicate a Join request*/   
        BSP_LED_Modem_On(LED_GREEN);
#elif USE_MDM32L07X01                  /* Led on Nucleo Master device to indicate a Join request*/
        BSP_LED_On(LED2);
#endif  
        
        TimerInit( &DemoLedTimer,  OnLedTimerEvent);  
        TimerSetValue( &DemoLedTimer,  200);  
        TimerStart( &DemoLedTimer);          
        
        LoraCmdRetCode = Lora_Join(JOIN_MODE);    
        if (LoraCmdRetCode == AT_OK)
        {  
          DeviceState = DEVICE_JOINED;          
          DBG_PRINTF("Nwk Joined\n");         
        }  
        else
        {  
           DBG_PRINTF("Nwk Not Joined\n");   /* we stay in ready state and redo LoRa_Join*/
        } 
    break;
    } 
    case DEVICE_JOINED:      
    {
       DBG_PRINTF("Nwk Joined - waiting\n");        
      /*Schedule the first mesaure */
      TimerSetValue( &NextSensorMeasureTimer, SENSORS_MEASURE_CYCLE);      /* every #n ms*/
      TimerStart( &NextSensorMeasureTimer ); 
      
#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)       
      /*put the modem MCU in stop mode*/
      if(((JOIN_MODE == ABP_JOIN_MODE) && (FWVersion == USI_FW_V26)) || (FWVersion == USI_FW_V30))
      LoraCmdRetCode = Lora_SleepMode();      
#endif      
      DeviceState = DEVICE_SLEEP;  
    break;
    }    
    case DEVICE_SLEEP:
    {
      /* Wake up through RTC events*/
      break;
    }    
    case DEVICE_SEND:
    {

#if defined (USE_I_NUCLEO_LRWAN1)     /*only applicable for USI LoRA modem*/ 
        if(SensorTimerWakeup)  /*the wakeup comes from Timer*/
        {      
         SensorTimerWakeup = 0;
#endif         
         
#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)
        /*Dumy Cmd after wakeup to resynchronize host/modem*/
        if(((JOIN_MODE == ABP_JOIN_MODE) && (FWVersion == USI_FW_V26)) || (FWVersion == USI_FW_V30))      
        LoRa_DumyRequest();
#endif 
              
       /*Sensor reading on slave device*/
        SensorMeasureData();

#ifdef USE_I_NUCLEO_LRWAN1              /* Led on Modem slave device to indicate a send request*/
        BSP_LED_Modem_On(LED_GREEN);       
#elif USE_MDM32L07X01                   /* Led on Nucleo Master device to indicate a send request*/
        BSP_LED_On(LED2);        
#endif        
 
        TimerInit( &DemoLedTimer,  OnLedTimerEvent);  
        TimerSetValue( &DemoLedTimer,  200);  
        TimerStart( &DemoLedTimer);         

       /*Send data to Slave device  */        
        LoraCmdRetCode = Lora_SendDataBin(&SendDataBinary);
        if (LoraCmdRetCode == AT_OK)
           DBG_PRINTF("Data binary send on port = %d --> OK\n",SendDataBinary.Port);
        else
           DBG_PRINTF("Data binary Send on port = %d --> KO\n",SendDataBinary.Port);  
        
      /*Schedule the next measure */
        TimerSetValue( &NextSensorMeasureTimer,  SENSORS_MEASURE_CYCLE);
        TimerStart( &NextSensorMeasureTimer ); 
        
#if defined (USE_I_NUCLEO_LRWAN1) && defined (MODEM_IN_SLEEP_MODE)          
       /*put the modem MCU in stop mode*/
        if(((JOIN_MODE == ABP_JOIN_MODE) && (FWVersion == USI_FW_V26)) || (FWVersion == USI_FW_V30))        
        LoraCmdRetCode = Lora_SleepMode();        
#endif 
        
#if defined (USE_I_NUCLEO_LRWAN1)     /*only applicable for USI LoRA modem*/         
        } /*end of SensorTimerWakeup*/
        else  /*the wakeup comes from network Event downlink data*/
        {

uint8_t ReceivedDownlinkData[64];  
sReceivedDataBinary_t StructDownLinkData ={ ReceivedDownlinkData, 0 , 0}; 


        LoraCmdRetCode = Lora_AsyncDownLinkData(&StructDownLinkData);
        DBG_PRINTF("downlink data\n");
        /************************************************************************/
        /* following the use case,                                              */
        /*   - switch on LED                                                    */
        /*   - activator setting                                                */
        /*   - LoRa mdeom setting                                               */
        /*   - .................                                                */
        /* user has to manage the received donwlink data                        */
        /************************************************************************/
        } 
#endif        
        
        DeviceState = DEVICE_SLEEP;
    break;
    }     
    default:
    {
      DeviceState = DEVICE_INIT;
      break;
    }
  }   
}  

/******************************************************************************
 * @brief Function executed on LedTimer Timeout event
 * @param none
 * @return none
******************************************************************************/

static void OnLedTimerEvent( void )
{ 
#ifdef USE_I_NUCLEO_LRWAN1   
  BSP_LED_Modem_Off(LED_GREEN);
#elif  USE_MDM32L07X01
  BSP_LED_Off(LED2);
#endif  
}  


/******************************************************************************
 * @brief Function executed on NextSensorMeeasure Timeout event
 * @param none
 * @return none
******************************************************************************/
static void OnNextSensorMeasureTimerEvt( void )
{
    TimerStop( &NextSensorMeasureTimer );
    SensorTimerWakeup = 1;
    DeviceState = DEVICE_SEND;     
}    


/******************************************************************************
 * @brief SensorMeasureData
 * @param none
 * @return none
******************************************************************************/
static void SensorMeasureData( void )
{
uint8_t LedState = 0;                /*just for padding*/
uint16_t pressure = 0;
int16_t temperature = 0;
uint16_t humidity = 0;
uint32_t BatLevel=0;                  /*end device connected to external power source*/ 
uint8_t index = 0;
ATEerror_t LoraCmdRetCode;
  

    /*read pressure, Humidity and Temperature in order to be send on LoRaWAN*/
    BSP_sensor_Read(&Sensor);
    
#ifdef CAYENNE_LPP 
uint8_t cchannel=0;  

    temperature = ( int16_t )( Sensor.temperature * 10 );     /* in °C * 10 */
    pressure    = ( uint16_t )( Sensor.pressure * 100 / 10 );  /* in hPa / 10 */
    humidity    = ( uint16_t )( Sensor.humidity * 2 );        /* in %*2     */

    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_DIGITAL_INPUT; 
    SendDataBinary.Buffer[index++] = LedState;
    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_BAROMETER;
    SendDataBinary.Buffer[index++] = ( pressure >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] = pressure & 0xFF;
    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_TEMPERATURE; 
    SendDataBinary.Buffer[index++] = ( temperature >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] = temperature & 0xFF;
    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_HUMIDITY;
    SendDataBinary.Buffer[index++] = humidity & 0xFF;
    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_DIGITAL_INPUT;
    
    /*get battery level of the modem (slave)*/
    LoraCmdRetCode = Lora_GetBatLevel(&BatLevel);
      if (LoraCmdRetCode == AT_OK)
        DBG_PRINTF("Msg status = %d --> OK\n",BatLevel);
      else
        DBG_PRINTF("Msg status = %d --> KO\n",BatLevel);      

    SendDataBinary.Buffer[index++] = BatLevel*100/254;
    SendDataBinary.Buffer[index++] = cchannel++;
    SendDataBinary.Buffer[index++] = LPP_DATATYPE_DIGITAL_OUTPUT; 
    SendDataBinary.Buffer[index++] = LedState;

#else
    
int32_t latitude, longitude = 0;     /*just for padding*/
uint16_t altitudeGps = 0;            /*just for padding*/ 

    temperature = ( int16_t )( Sensor.temperature * 100 );     /* in °C * 100 */
    pressure    = ( uint16_t )( Sensor.pressure * 100 / 10 );  /* in hPa / 10 */
    humidity    = ( uint16_t )( Sensor.humidity * 10 );        /* in %*10     */

    latitude = Sensor.latitude;    /* not relevant*/
    longitude= Sensor.longitude;   /* not relevant*/
    
    /*fill up the send buffer*/
    SendDataBinary.Buffer[index++] = LedState;    
    SendDataBinary.Buffer[index++] = ( pressure >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] =  pressure & 0xFF;
    SendDataBinary.Buffer[index++] = ( temperature >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] =  temperature & 0xFF;
    SendDataBinary.Buffer[index++] = ( humidity >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] =  humidity & 0xFF;

    /*get battery level of the modem (slave)*/
    LoraCmdRetCode = Lora_GetBatLevel(&BatLevel);
      if (LoraCmdRetCode == AT_OK)
        DBG_PRINTF("Msg status = %d --> OK\n",BatLevel);
      else
        DBG_PRINTF("Msg status = %d --> KO\n",BatLevel);      
    SendDataBinary.Buffer[index++] = (uint8_t)BatLevel;
    
    /*remaining data just for padding*/    
    SendDataBinary.Buffer[index++] = ( latitude >> 16 ) & 0xFF;
    SendDataBinary.Buffer[index++] = ( latitude >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] = latitude & 0xFF;
    SendDataBinary.Buffer[index++] = ( longitude >> 16 ) & 0xFF;
    SendDataBinary.Buffer[index++] = ( longitude >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] = longitude & 0xFF;
    SendDataBinary.Buffer[index++] = ( altitudeGps >> 8 ) & 0xFF;
    SendDataBinary.Buffer[index++] = altitudeGps & 0xFF;    
    
#endif  /*CAYENNE_LPP*/  
    
    SendDataBinary.DataSize = index;
    SendDataBinary.Port = LORAWAN_APP_PORT;
    
#ifdef USE_I_NUCLEO_LRWAN1      
    SendDataBinary.Ack = !LORAWAN_CONFIRMED_MSG;
#endif  
} 

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
