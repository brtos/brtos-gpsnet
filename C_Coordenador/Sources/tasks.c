/********************************************************************************
@file   tasks.c
@brief  Application tasks file
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
*********************************************************************************/

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
  
   for(;;){
   
      (void)OSQueuePend(Serial, &c, 0);
      Comm2PC_FSM((CHAR8)c);
   }


}
#endif

void HeartBeat_LED_Task(void)
{
   /* task setup */
   struct
   {
      UINT_DWORD  latitude;
      UINT_DWORD  longitude;
   } address,*ptraddr;
    
   LED_HEARTBEAT_INIT();
   LED_HEARTBEAT_TOGGLE();
  
#ifndef GPSNET_TEST_POSITION   
   ptraddr = (void *)Latitude;
  
   // Check if coordinator has a GPS address
   while((ptraddr->latitude.int32u == 0x00000000) && (ptraddr->longitude.int32u == 0x00000000)) 
   {
     // If not, try to get it
     RequestGPSAddress(&address.latitude.int32u, &address.longitude.int32u);
     
     if ((address.latitude.int32u != 0x00000000) && (address.longitude.int32u != 0x00000000)) 
     {        
      gps_latX = address.latitude.int32u;
      BaseStations[0].GPS_LatX  = address.latitude.int32u;          
      gps_longY = address.longitude.int32u;
      BaseStations[0].GPS_LongY = address.longitude.int32u;
      
      /* store GPS address in non-volatile memory */
#if (DEBUG == 0)      
      break;
#else
      // EEPROM_Store((INT32U)EEPROM_ADDRESS, &address.latitude.int32u,2);
#endif      
     }
     LED_HEARTBEAT_TOGGLE();
     DelayTask(600);
   } 
#else
    (void)address; (void)ptraddr;  
#endif     
   
  #if(DEBUG == 1)    
  #if(NETWORK_ENABLE == 1)     
  
  GPSNET_Init();     
  
  if(InstallTask(&GPSNET_Apps,"Applications",384,APP1_Priority) != OK)
  {
    while(1){};
  };   
  
  #if BOOTLOADER_ENABLE == 1
  
  if(InstallTask(&GPSNET_FOTA_Task,"Bootloader",1280,APP2_Priority) != OK)
  {
    while(1){};
  }  
  
  #endif  
  
  #endif   
  #endif   
   
   
   for (;;) 
   {
      LED_HEARTBEAT_TOGGLE();
      DelayTask(200);
   }
}

#ifndef SIGNAL_APP1
  #error  "Please define SIGNAL_APP1 for this RX app"
#endif 

// GPSNET Applications
void GPSNET_Apps(void)
{   
   // task setup   
   if ((INT8U)OSQueueCreate(&PacketsBuffer,(RFBufferSize * 2),&Pkt) != ALLOC_EVENT_OK)
   {
      while(1){};
   };

#if 0   
   if((INT8U)OSMutexCreate(&PktQueueMutex, PktQueue_Mutex_Priority) !=ALLOC_EVENT_OK){
      while(1){};
   }  
#endif   
        
   // task main loop
   for (;;)
   {
     // Wait event from APP layer, no timeout
     OSSemPend(SIGNAL_APP1,0);     
     
     acquireRadio();
     
     if (app_packet.APP_Profile == BULK_DATA_PROFILE) 
     {
      
        Data_receive();  /* used by Bootloader app */
     }else {
        
        SendPacketToQueue();
       
     }
     releaseRadio();
   }
}



