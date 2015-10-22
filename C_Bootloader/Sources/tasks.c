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
  
   for(;;){
   
      (void)OSQueuePend(Serial, &c, 0);
      Comm2PC_FSM((CHAR8)c);
   }


}
#endif




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




/* GPSNET Applications */
void GPSNET_Apps(void)
{

   
   // task setup   
   if ((INT8U)OSQueueCreate(&PacketsBuffer,RFBufferSize,&Pkt) != ALLOC_EVENT_OK)
   {
      while(1){};
   };   
  
   
   // task main loop
   for (;;)
   {
     // Wait event from APP layer, no timeout
     OSSemPend(SIGNAL_APP1,0);     
              
     acquireRadio();
     if (app_packet.APP_Profile == BULK_DATA_PROFILE) 
     {
        Data_receive();  /* used by Bootloader app */
     }else 
     {     
      if(app_packet.APP_Profile == GENERAL_PROFILE) 
      {
        Decode_General_Profile();      
      }
     }
     releaseRadio();
   }
}

/* Task for Sniffer and GPS address installation */
void GPSNET_AddressApp(void)
{
   // task setup        
   UINT_DWORD   latitude;
   UINT_DWORD   longitude;
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
        SniferReq(UP_ROUTE, &lat, &lon);      
      }
      
      // Search for neighbor without address
      for(i=0;i<NEIGHBOURHOOD_SIZE;i++) 
      {
        if (gpsnet_neighbourhood[i].NeighborGPS_LatX == 0xFFFFFFFF)
        {
            RequestGPSAddress(&latitude.int32u, &longitude.int32u);
            if ((latitude.int32u != 0x00000000) && (longitude.int32u != 0x00000000)) 
            {
              GPSAddressMessage(&latitude.int8u[0], &longitude.int8u[0]);
            }
            
            break;
        }
      }

      DelayTask(3000);
   }

}



