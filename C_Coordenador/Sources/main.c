/********************************************************************************
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

#include "BRTOS.h"
#include "MCUinit.h"
#include "drivers.h"
#include "tasks.h"
#include "app.h"
#include "gpsnet_api.h"

#ifdef __cplusplus
 extern "C"
#endif

#if(BOOTLOADER_ENABLE == 1)
extern void ISR_vtable_reallocation(void);
#endif

/* main entry point */
void main(void) 
{
  #if(BOOTLOADER_ENABLE == 1)
    ISR_vtable_reallocation();
  #endif  

  MCU_init(); /* call Device Initialization */
  
#if (DEBUG == 1)  
  Flash_Clock_Init();
#endif  

  // Initialize BRTOS
  BRTOS_Init();   
       
  if(InstallTask(&System_Time,"System Time",320,System_Time_Priority) != OK)
  {
    while(1){};
  };
  
  if(InstallTask(&Comm2PC_Task,"Comm2PC task",1280,Comm2PC_Task_Priority) != OK)
  {
    while(1){};
  };     
  
  if(InstallTask(&HeartBeat_LED_Task,"HeartBeat LED task",320, HeartBeat_LED_Priority) != OK)
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
