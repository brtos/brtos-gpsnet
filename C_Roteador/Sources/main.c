/***********************************************************************************
@file   main.c
@brief  Main application file
@authors: Gustavo Weber Denardin
          Carlos Henrique Barriquello

Copyright (c) <2009-2013> <Universidade Federal de Santa Maria>

  * Software License Agreement
  *
  * The Software is owned by the authors, and is protected under 
  * applicable copyright laws. All rights are reserved.
  *
  * The above copyright notice shall be included in
  * all copies or substantial portions of the Software.
  *  
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  * THE SOFTWARE. 
**********************************************************************************/

/* MCU and OS includes */
#include "hardware.h"
#include "MCUinit.h"
#include "BRTOS.h"
#include "drivers.h"

/* Config. files */
#include "BRTOSConfig.h"
#include "AppConfig.h"
#include "NetConfig.h"
#include "BoardConfig.h" 

#include "tasks.h"        /* for tasks prototypes */
#include "smartmeter.h"
#include "gpsnet_api.h"   /* for GPSNET network functions */

#if (BOOTLOADER_ENABLE == 1)
  #include "MCF51_Bootloader.h"
  #include "Bootloader_wireless.h" 
#endif

#ifdef __cplusplus
 extern "C"
#endif


void main(void)
{
    
  /* MCU inicialization */
  MCU_init();
     
  #if (BOOTLOADER_ENABLE == 1)  
  /* Interrupt vector table reallocation to RAM
     for use with the bootloader */  
    ISR_vtable_reallocation(); 
  #endif  

  /**********************************/
  /* Inicialization of peripherals  */   
  /**********************************/

  /* Flash memory clock */
  #if (DEBUG == 1)  
    Flash_Clock_Init();
  #endif  
  
  /* Initialize I/O Ports */
  GPIO_Setup();

  /* Initialize A/D Converter and bandgap reference */
  ADC_Setup(HighSpeed, ShortSampleTime, 12);
    
  #if(DEBUG == 1) 
    if(ADC_Bandgap_Set() != BANDGAP_OK){ while(1){}}
  #endif  
  
  /* Erase flash memory for code update */  
  #if (BOOTLOADER_ENABLE == 1)       
    WBootloader_Flash_Init();
  #endif
  
  /* Initialize BRTOS */
  BRTOS_Init();      
  
  /* Start GPSNET network services */  
  #if(DEBUG == 1)    
  #if(NETWORK_ENABLE == 1)       
  
  GPSNET_Init();      /* Install 3 task: PHY, MAC and NWK */
  
  /* Install apps that use GPSNET */
  if(InstallTask(&GPSNET_App,"GPSNET Applications",GPSNET_App_StackSize,APP1_Priority) != OK)
  {
    while(1){}; // Installation error: please check error type. 
                // Usually BUSY_PRIORITY or NO_MEMORY
  }
  
  if(InstallTask(&GPSNET_SensorsApp,"App to send measurements",GPSNET_SensorsApp_StackSize,APP2_Priority) != OK)
  {
     while(1){};  // Installation error: please check error type. 
                  // Usually BUSY_PRIORITY or NO_MEMORY
  }
  
  #endif
  #endif
  
#if 1
  if(InstallTask(&EnergyMetering_Task,"Energy meter Task",EnergyMetering_StackSize,EnergyMetering_Task_Priority) != OK)
  {
    while(1){};  // Installation error: please check error type. 
                 // Usually BUSY_PRIORITY or NO_MEMORY
  }    
  
#endif

  if(InstallTask(&RelayControl_Task,"Relay control Task",RelayControl_StackSize,RelayControl_Task_Priority) != OK)
  {
    while(1){};  // Installation error: please check error type. 
                 // Usually BUSY_PRIORITY or NO_MEMORY
  } 
  
  
  if(InstallTask(&System_Time,"System Time",System_Time_StackSize,System_Time_Priority) != OK)
  {
    while(1){};  // Installation error: please check error type. 
                 // Usually BUSY_PRIORITY or NO_MEMORY
  }

  /* Start Task Scheduler */
  if(BRTOSStart() != OK)
  {
    while(1){};   // Idle task installation error.
                  // Please check error type: usually NO_MEMORY !
  }
  

  for(;;) {
    
  } /* loop forever */
  /* please make sure that you never leave main */
}


/* RELEASE CONFIGURATION */
/********************************************************************
"BRTOSConfig.h" 
#define DEBUG           1
#define LOW_POWER_MODE  0  

"BoardConfig.h" 
#define TEST_PIN         0
#define TX_POWER_LEVEL   RFTX_0dB

"AppConfig.h"
#define CALIB_CURRENT                     0
#define SMARTMETER_TEST_CALCULATIONS      0
#define LUX_MIN                           35
#define LUX_MAX                           (2*LUX_MIN)
#define RELAY_CHECK_CC                    0
#define RELAY_TURNON_VOLTAGE              1
#define REPORTING_PERIOD_MS               5000
#define REPORTING_JITTER_MS               100
#define BOOTLOADER_ENABLE                 1  

"NetConfig.h"
#define NETWORK_ENABLE                    1
#define MULTICHANNEL_SUPPORT              1
#define NUM_ALLOWED_CHANNELS              4
#define FORCE_NO_MULTICHANNEL_SUPPORT     0

"network.h"
#define RSSI_THRESHOLD          (INT8U)0
#define LOW_PARENT_THRESHOLD    (INT8U)1
#define RSSI_PARENT_THRESHOLD   (INT8U)2
#define NWK_TX_RETRIES          (INT8U)10
#define AUTO_ACK_CONTROL        1   

*********************************************************************/


