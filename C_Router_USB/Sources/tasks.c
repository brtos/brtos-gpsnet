/***********************************************************************************
@file   tasks.c
@brief  Tasks application file
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

// include declarations
#include "BRTOS.h"
#include "drivers.h" 
#include "app.h"
#include "tasks.h"
#include "comm2pc.h" 

#include "gpsnet_api.h"

#pragma warn_implicitconv off

/*************************************************/
/* Task: keep watchdog happy and system time     */
/*************************************************/
void System_Time(void)
{
   // task setup
   INT8U i = 0;
   INT8U cnt = 0;
   
   OSResetTime();
  
   // task main loop
   for (;;)
   {
      #if (WATCHDOG == 1)
        __RESET_WATCHDOG();
      #endif
      
      DelayTask(10);
      i++;
      
      VerifyNeighbourhoodLastIDTimeout();
      
      if (i >= 100)
      {
        OSUpdateUptime();
        i = 0;
      }
   }
}


/*************************************************/
/* Task: to communicate to the PC                */
/*************************************************/
/* USB comm is used only on real hardware */
#if (DEBUG == 1)
void Comm2PC_Task(void)
{
  CHAR8 c;  
  
  /* configure USB and init CDC driver */
  usb_cfg_init();
  cdc_init();    
  
  /* This loop will receive and process characters from the USB. */
  for(;;)
  {  
    while((*cdc_kbhit)())         
    {
      c=(CHAR8)(*cdc_getch)();
      Comm2PC_FSM(c);
    }
    cdc_process();
    OSSemPend(USB_Sem,0);    
  }  
}
#else
#include "uart.h"
/* UART comm is used in simulation only */
void Comm2PC_Task(void)
{
   INT8U c;
   
   // Init UART port with a max mutex priority
   UART_init(Serial_Mutex_Priority);
   
   if(Comm2PC_check_cmdorder() != OK){
      while(1){} /* command are out of order */
   }
   
   for(;;){
   
      (void)OSQueuePend(Serial, &c, 0);
      Comm2PC_FSM((CHAR8)c);
   }


}
#endif


/*************************************************/
/* Task: to toggle the LED on board              */
/*************************************************/
void HeartBeat_LED_Task(void)
{
   /* task setup */
  LED_HEARTBEAT_INIT();
   
   for (;;) 
   {
      LED_HEARTBEAT_TOGGLE();
      DelayTask(200);
   }
}

/*************************************************/
/* Task: to process received packets             */
/*************************************************/

// Sanity check
#ifndef SIGNAL_APP1
  #error  "Please define SIGNAL_APP1 for this app"
#endif

/* For bootloader - only if enabled */ 
#if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1) 
#define    SIGNAL_TIMEOUT   200  
#include "MCF51_Bootloader.h"  
#include "Bootloader_wireless.h"  
#else
#define    SIGNAL_TIMEOUT   0
#endif


void GPSNET_RxApp(void)
{

   INT8U  ret = 0; 
       
   // task setup   
   if ((INT8U)OSQueueCreate(&PacketsBuffer,RFBufferSize,&Pkt) != ALLOC_EVENT_OK)
   {
      while(1){};
   }; 
      
   // task main loop
   for (;;)
   {
     
      // Wait event from APP layer, with or without timeout             
      ret = OSSemPend(SIGNAL_APP1, SIGNAL_TIMEOUT);      
      
      #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)       
      if(ret != TIMEOUT){      
      #endif
       
       acquireRadio();
       
       switch(app_packet.APP_Profile) 
       {
        case GENERAL_PROFILE:
          Decode_General_Profile();
          break;
          
        case LIGHTING_PROFILE:
          Decode_Lighting_Profile();
          break;
        
        #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)  
        case BULK_DATA_PROFILE:
          WBootloader_Handler();
          break;
        #endif
        
        default:
          break;
       }        
       releaseRadio();
      
      #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)       
      }else{          
        WBootloader_Handler_Timeout();         
      }
      #endif 

   }
}

/*************************************************/
/* Task: to transmit packets                     */
/*************************************************/

void GPSNET_TxApp(void)
{
   // task setup        
   //UINT_DWORD   latitude;
   //UINT_DWORD   longitude;
   LATITUDE     lat;
   LONGITUDE    lon;
   INT16U       node_id = 0;   
   INT8U        i = 0;     
   
   while(NeighborTable == 0)
   {
      DelayTask(1000);
   }      
          
   // task main loop       
   for (;;)
   {
      
      /* Get node to snif */
      node_id = RequestNodeID();

      if (node_id != 0xFFFE) 
      {        
        for (i=0;i<NEIGHBOURHOOD_SIZE;i++)
        {
          if (node_id == gpsnet_neighbourhood[i].Addr_16b) 
          {
            lat.x = gpsnet_neighbourhood[i].NeighborGPS_Latitude.x;            
            lon.y = gpsnet_neighbourhood[i].NeighborGPS_Longitude.y;
            break;
          }
        }          
        /* Request data */
        // SniferReq(UP_ROUTE, &lat, &lon);      
      }
      
      DelayTask(3000);
   }

}

#include "sensors.h"

/************************************************************/
/* Task: takes temperature measurements                     */
/* and reports all measurements                             */
/* to the nearest base station at every REPORT_TIME seconds */
/************************************************************/
void GPSNET_SensorApp(void)   {
  

    // task setup
   INT8U        status = 0;
   INT8U        Temperature = 0; 
   SE_STRUCT    SmartEnergy;
   extern INT8U PIN_RELAY;

   //#define TEST_PARENT_DOWN
      
   #ifdef TEST_PARENT_DOWN
      LATITUDE   lat;
      LONGITUDE  lon;
      INT8U      seq = 0;
   #endif
   
   
   /* enable sensors for Temperature and Light */
   SensorTemp_Enable();      
  
   // task main loop
   for (;;)
   {
     
      NetClearSE(&SmartEnergy);
      SmartEnergy.params.Byte = P_VOLTAGE + P_CURRENT + P_POWER_P + P_POWER_Q + P_POWER_S + P_PF + P_ENERGY;                                
      SmartEnergy.v_rms = 2200;        
      status = NetMultiMeasureSE(&SmartEnergy);
      
      
      DelayTask(REPORTING_PERIOD_MS + RadioRand()*REPORTING_JITTER_MS);
      
      /* Temperature measurement and reporting */                     
      if((status = SensorTemp_Get(&Temperature)) == SENSOR_OK)
      {                  
          status = NetGeneralInfo(TEMPERATURE, (INT8U)Temperature, (INT16U)0);
      }
      
      DelayTask(1000);  
      
      //DelayTask(REPORTING_PERIOD_MS + RadioRand()*REPORTING_JITTER_MS);
      
      /* Lamp state reporting */
      if(PIN_RELAY==ON) status = ON; else status=OFF;
     
      status = NetLightingProfile(DOWN_ROUTE, SIMPLE_METERING, LAMP_STATE, status , (INT16U)0, (INT32U) 0);    
      
         
      
      #ifdef TEST_PARENT_DOWN
      
        DelayTask(REPORTING_PERIOD_MS + RadioRand()*REPORTING_JITTER_MS); 
        
        lat.x = BaseStations[NearBase].Par_GPS_Latitude.x;
        lon.y = BaseStations[NearBase].Par_GPS_Longitude.y;
             
      
        DelayTask(15000);         
        
        if (BetterDepth >= 1 && BetterDepth < 254)
        {         
          NetDebugPacket(DEBUG_PKT, DEBUG_COUNTER, (INT8U)0, (INT16U)seq, &lat, &lon);                 
          ++seq;
        }

      
      #endif     
                  
   }
}



