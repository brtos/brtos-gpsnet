/**
@file   usr_entry.c
@brief  MCU code entry point
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
*/


#include "hardware.h"
#include "BRTOS.h"
#include "BRTOSConfig.h"
#include "AppConfig.h"


#pragma warn_implicitconv off


#define MAIN_ENTRY_POINT       0x000021D0
#define BOOTLOADER_ENTER_FLAG  0xFFFF0000

 
void _Entry(void)
{

/* Check if bootloader flasher is enable */
#if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE == 1) 
#include "MCF51_Bootloader.h"
            
  unsigned int status_prog;
    
    
/* check flag status (@STATUS_ADDR address) which indicate where to go:
 bootloader mode or application (main) mode */ 
    
  status_prog = *(dword *) STATUS_ADDR;   

    
  __RESET_WATCHDOG();

 
  // decide which mode to go: user code or bootloader 
  if(status_prog != BOOTLOADER_ENTER_FLAG) 
  {
     asm (JMP MAIN_ENTRY_POINT);      // jump to user entry       
  }  
  else 
  {
    SOPT1 = 0x42;         // Disable COP, STOP and WAITE
    asm
    {
      NOP
    };                             
    SOPT2 = 0x08;
    
    /* SPMSC1: LVDF=0,LVDACK=0,LVDIE=0,LVDRE=1,LVDSE=1,LVDE=1,BGBE=0 */
    SPMSC1 = 0x1C;                                      
    /* SPMSC2: LPR=0,LPRS=0,LPWUI=0,PPDF=0,PPDACK=0,PPDE=1,PPDC=0 */
    SPMSC2 = 0x02;                                      
    /* SPMSC3: LVDV=0,LVWV=0,LVWIE=0 */
    SPMSC3 &= (unsigned char)~0x38;                     
    /* Initialization of CPU registers */
    #if (NESTING_INT == 1)
    asm {
      /* VBR: ADDRESS=0 */
      clr.l d0
      movec d0,VBR
      /* CPUCR: ARD=0,IRD=0,IAE=0,IME=0,BWD=0,FSD=0 */
      move #0x82000000, d0      
      // move #0x10000000, d0 (com nesting)
      movec d0,CPUCR
    }
    #else
    asm {
      /* VBR: ADDRESS=0 */
      clr.l d0
      movec d0,VBR
      /* CPUCR: ARD=0,IRD=0,IAE=0,IME=1,BWD=1,FSD=0 */
      move #0x92000000, d0  
      // move #0x12000000, d0 (sem nesting)
      movec d0,CPUCR
    }  
    #endif       
      
    /*  System clock initialization */
    /* ICSC1: CLKS=0,RDIV=0,IREFS=1,IRCLKEN=0,IREFSTEN=0 */
    ICSC1 = 0x04;                        /* Initialization of the ICS control register 1 */
    /* ICSC2: BDIV=0,RANGE=0,HGO=0,LP=0,EREFS=0,ERCLKEN=0,EREFSTEN=0 */
    ICSC2 = 0x00;                        /* Initialization of the ICS control register 2 */
    while(!ICSSC_IREFST) {               /* Wait until the source of reference clock is internal clock */
    }
    /* ICSSC: DRST_DRS=2,DMX32=0 */
    ICSSC = (ICSSC & (unsigned char)~0x60) | (unsigned char)0x80; /* Initialization of the ICS status and control */
    while((ICSSC & 0xC0) != 0x80) {      /* Wait until the FLL switches to High range DCO mode */
    }

    Bootloader_Main();
  
  }

#else

  asm (JMP MAIN_ENTRY_POINT);      // jump to user entry

#endif                               
  
}