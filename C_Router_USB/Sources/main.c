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

#include "hardware.h"
#include "BRTOS.h"
#include "tasks.h"
#include "app.h"
#include "drivers.h"
#include "MCUinit.h"

#ifdef __cplusplus
 extern "C"
#endif

#if(BOOTLOADER_ENABLE == 1)
  extern void ISR_vtable_reallocation(void);
#endif

void main(void) 
{

  #if(BOOTLOADER_ENABLE == 1)
    ISR_vtable_reallocation();
  #endif  

  MCU_init(); /* MCU Initialization */   
  
  // Initialize BRTOS
  BRTOS_Init();  
  
  /* Peripheral inicialization */
#if (DEBUG == 1) 

  /* flash clock for reprogramming */ 
  Flash_Clock_Init();     
  
  /* Initialize A/D Converter and bandgap reference */
  ADC_Setup(HighSpeed, ShortSampleTime, 12);
  if(ADC_Bandgap_Set() != BANDGAP_OK){
    while(1){}
  }
 
#endif    
  
  
  #if(DEBUG == 1)    
  #if(NETWORK_ENABLE == 1)     
  
  GPSNET_Init();     
  
  if(InstallTask(&GPSNET_RxApp,"GPSNET RX task",320,APP3_Priority) != OK)
  {
    while(1){};
  }
  /*
  if(InstallTask(&GPSNET_TxApp,"GPSNET TX task",1280,APP2_Priority) != OK)
  {
    while(1){};
  }
  */
  
  if(InstallTask(&GPSNET_SensorApp,"GPSNET Sensor task",1280,APP1_Priority) != OK)
  {
    while(1){};
  }
    

  #endif   
  #endif
  
     
  if(InstallTask(&System_Time,"System Time",320,System_Time_Priority) != OK)
  {
    while(1){};
  };
  
  if(InstallTask(&Comm2PC_Task,"Comm2PC task",448,Comm2PC_Task_Priority) != OK)
  {
    while(1){};
  };    
  
  
  if(InstallTask(&HeartBeat_LED_Task,"HeartBeat LED task",320,HeartBeat_LED_Priority) != OK)
  {
    while(1){};
  };
    

  // Start Task Scheduler
  if(BRTOSStart() != OK)
  {
    while(1){};
  };  



  for(;;) {
    /* __RESET_WATCHDOG(); by default, COP is disabled with device init. When enabling, also reset the watchdog. */
  } /* loop forever */
  /* please make sure that you never leave main */
}
