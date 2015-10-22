#include "hardware.h"
#include "BRTOS.h"
#include "BRTOSConfig.h"
#include "AppConfig.h"
#include "NetConfig.h"   


#pragma warn_implicitconv off


#define MAIN_ENTRY_POINT       0x00000410
#define BOOTLOADER_ENTER_FLAG  0xFFFF0000

extern void __initialize_hardware(void);
 
void _Entry(void)
{

/* Check if bootloader flasher is enable */
#ifdef BOOTLOADER_ENABLE

#if (BOOTLOADER_ENABLE == 1 && DEVICE_TYPE == ROUTER)
#include "MCF51_Bootloader.h" 

unsigned char i;             
unsigned int status_prog;
    
    
/* check flag status (@STATUS_ADDR address) which indicate where to go:
 bootloader mode or application (main) mode */ 
    
status_prog = *(dword *) STATUS_ADDR;   

for(i=0;i<3;i++) {     
      __RESET_WATCHDOG();
  }
 
  // decide which mode to go: user code or bootloader 
  if(status_prog != BOOTLOADER_ENTER_FLAG) 
  {
     asm (JMP MAIN_ENTRY_POINT);      // jump to user entry       
  } 
  
  else 
  {
    SOPT1 = 0x42;         // Disable COP, STOP and WAITE
    asm {NOP};                             
    SOPT2 = 0x08;                               


  __initialize_hardware();  


    Bootloader_Main();
  
  }

#else

  asm (JMP MAIN_ENTRY_POINT);      // jump to user entry

#endif

#else

  asm (JMP MAIN_ENTRY_POINT);      // jump to user entry

#endif
 
}

  