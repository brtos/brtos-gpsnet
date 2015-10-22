/*
** ###################################################################
**     This code is generated by the Device Initialization Tool.
**     It is overwritten during code generation.
**     USER MODIFICATION ARE PRESERVED ONLY INSIDE INTERRUPT SERVICE ROUTINES
**     OR EXPLICITLY MARKED SECTIONS
**
**     Project   : BRTOS
**     Processor : MCF51QE128CLK
**     Version   : Bean 01.011, Driver 01.05, CPU db: 3.00.052
**     Datasheet : MCF51QE128RM, Rev. 3, 9/2007
**     Date/Time : 18/09/2008, 17:41
**     Abstract  :
**         This module contains device initialization code 
**         for selected on-chip peripherals.
**     Contents  :
**         Function "MCU_init" initializes selected peripherals
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################
*/

/* MODULE MCUinit */

#include <MCF51QE128.h>                /* I/O map for MCF51QE128CLK */
#include <hidef.h> /* for EnableInterrupts macro */

#include "MCUinit.h"
#include "drivers.h"
#include "BRTOS.h"


#include "AppConfig.h"   

/* pragma to disable "possibly unassigned ISR handler" message generated by compiler on definition of ISR without vector number */
#pragma warn_absolute off
#pragma warn_implicitconv off

/* User declarations and definitions */
/*   Code, declarations and definitions here will be preserved during code generation */
/* End of user declarations and definitions */

/*
** ===================================================================
**     Method      :  __initialize_hardware (bean MCF51QE128_80)
**
**     Description :
**         Initialization code for CPU core and a clock source.
** ===================================================================
*/

void __initialize_hardware(void)
{
  /* ### MCF51QE128_80 "Cpu" init code ... */
  /*  PE initialization code after reset */
  asm {                                /* Set Interrupt level 7 */
    move.w SR,D0;
    ori.l  #0x0700,D0;
    move.w D0,SR;
  }  
  /* Common initialization of the write once registers */
  /* SOPT1: COPE=0,COPT=1,STOPE=1,WAITE=1,RSTOPE=0,BKGDPE=1,RSTPE=0 */
  #if (WATCHDOG == 1)
    #if (BDM_ENABLE == 1)    
      SOPT1 = 0xB2;
    #else
      SOPT1 = 0xB0;
    #endif
  #else
    #if (BDM_ENABLE == 1)    
      SOPT1 = 0x72;
    #else
      SOPT1 = 0x70;
    #endif                                      
  #endif  
  SOPT2 = 0x08;
  /* SPMSC1: LVDF=0,LVDACK=0,LVDIE=0,LVDRE=1,LVDSE=1,LVDE=1,BGBE=1 */
  SPMSC1 = 0x1D;
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
    move #0x82000000, d0  
    // move #0x10000000, d0 (com nesting) 
    /* CPUCR: ARD=1,IRD=0,IAE=0,IME=0,BWD=0,FSD=0 */
    movec d0,CPUCR
  }
  #else
  asm {
    /* VBR: ADDRESS=0 */
    clr.l d0
    movec d0,VBR
    /* CPUCR: ARD=1,IRD=0,IAE=0,IME=1,BWD=1,FSD=0 */
    move #0x92000000, d0  
    // move #0x12000000, d0 (sem nesting)
    movec d0,CPUCR
  }  
  #endif
   
  
  
  // Configura Clock interno como fonte de relógio
  // Barramento de 25Mhz
  
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
  
  
  
  /* INTC_WCR: ENB=1,MASK=0 */
  // Quando ENB = 1, permite voltar de um wait State
  // Ainda, configura a mascara de interrupção p/ zero
  // ou seja, qq interrupção pode retirar o processador do modo Wait
  INTC_WCR = 0x80;
}

/*
** ===================================================================
**     Method      :  MCU_init (bean MCF51QE128_80)
**
**     Description :
**         Device initialization code for selected peripherals.
** ===================================================================
*/
void MCU_init(void)
{
  /* Common initialization of the CPU registers */
  /* SCGC1: TPM3=1,TPM2=1,TPM1=1,ADC=1,IIC2=0,IIC1=0,SCI2=0,SCI1=0 */
  SCGC1 = 0xF0; 
                                     
  /* SCGC2: FLS=1,IRQ=0,KBI=0,ACMP=0,RTC=0,SPI2=1,SPI1=0 */
  SCGC2 = 0xC2;
  
  /* PTASE: PTASE7=0,PTASE6=0,PTASE4=0,PTASE3=0,PTASE2=0,PTASE1=0,PTASE0=0 */
  PTASE &= (unsigned char)~0xDF;                     
  /* PTBSE: PTBSE7=0,PTBSE6=0,PTBSE5=0,PTBSE4=0,PTBSE3=0,PTBSE2=0,PTBSE1=0,PTBSE0=0 */
  PTBSE = 0x00;                                      
  /* PTCSE: PTCSE7=0,PTCSE6=0,PTCSE5=0,PTCSE4=0,PTCSE3=0,PTCSE2=0,PTCSE1=0,PTCSE0=0 */
  PTCSE = 0x00;                                      
  /* PTDSE: PTDSE7=0,PTDSE6=0,PTDSE5=0,PTDSE4=0,PTDSE3=0,PTDSE2=0,PTDSE1=0,PTDSE0=0 */
  PTDSE = 0x00;                                      
  /* PTESE: PTESE7=0,PTESE6=0,PTESE5=0,PTESE4=0,PTESE3=0,PTESE2=0,PTESE1=0,PTESE0=0 */
  PTESE = 0x00;                                      
  /* PTFSE: PTFSE7=0,PTFSE6=0,PTFSE5=0,PTFSE4=0,PTFSE3=0,PTFSE2=0,PTFSE1=0,PTFSE0=0 */
  PTFSE = 0x00;                                      
  /* PTGSE: PTGSE7=0,PTGSE6=0,PTGSE5=0,PTGSE4=0,PTGSE3=0,PTGSE2=0,PTGSE1=0,PTGSE0=0 */
  PTGSE = 0x00;                                      
  /* PTHSE: PTHSE7=0,PTHSE6=0,PTHSE5=0,PTHSE4=0,PTHSE3=0,PTHSE2=0,PTHSE1=0,PTHSE0=0 */
  PTHSE = 0x00;                                      
  /* PTJSE: PTJSE7=0,PTJSE6=0,PTJSE5=0,PTJSE4=0,PTJSE3=0,PTJSE2=0,PTJSE1=0,PTJSE0=0 */
  PTJSE = 0x00;                                      
  /* PTADS: PTADS7=0,PTADS6=0,PTADS5=0,PTADS4=0,PTADS3=0,PTADS2=0,PTADS1=0,PTADS0=0 */
  PTADS = 0x00;                                      
  /* PTBDS: PTBDS7=0,PTBDS6=0,PTBDS5=0,PTBDS4=0,PTBDS3=0,PTBDS2=0,PTBDS1=0,PTBDS0=0 */
  PTBDS = 0x00;                                      
  /* PTCDS: PTCDS7=0,PTCDS6=0,PTCDS5=0,PTCDS4=0,PTCDS3=0,PTCDS2=0,PTCDS1=0,PTCDS0=0 */
  PTCDS = 0x00;                                      
  /* PTDDS: PTDDS7=0,PTDDS6=0,PTDDS5=0,PTDDS4=0,PTDDS3=0,PTDDS2=0,PTDDS1=0,PTDDS0=0 */
  PTDDS = 0x00;                                      
  /* PTEDS: PTEDS7=0,PTEDS6=0,PTEDS5=0,PTEDS4=0,PTEDS3=0,PTEDS2=0,PTEDS1=0,PTEDS0=0 */
  PTEDS = 0x00;                                      
  /* PTFDS: PTFDS7=0,PTFDS6=0,PTFDS5=0,PTFDS4=0,PTFDS3=0,PTFDS2=0,PTFDS1=0,PTFDS0=0 */
  PTFDS = 0x00;                                      
  /* PTGDS: PTGDS7=0,PTGDS6=0,PTGDS5=0,PTGDS4=0,PTGDS3=0,PTGDS2=0,PTGDS1=0,PTGDS0=0 */
  PTGDS = 0x00;                                      
  /* PTHDS: PTHDS7=0,PTHDS6=0,PTHDS5=0,PTHDS4=0,PTHDS3=0,PTHDS2=0,PTHDS1=0,PTHDS0=0 */
  PTHDS = 0x00;                                      
  /* PTJDS: PTJDS7=0,PTJDS6=0,PTJDS5=0,PTJDS4=0,PTJDS3=0,PTJDS2=0,PTJDS1=0,PTJDS0=0 */
  PTJDS = 0x00;
  // Habilita IRQ Externo sem pull-up interno e ativo para borda de subida
  //IRQSC = 0x56;

//  asm {                                /* Set Interrupt level 0 */
//    move.w SR,D0;
//    andi.l #0xF8FF,D0;
//    move.w D0,SR;
//  }
} /*MCU_init*/


/*
** ===================================================================
**     Interrupt handler : isr_default
**
**     Description :
**         User interrupt service routine. 
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void Erro_Address(void);
void Erro_accerr(void);


__interrupt void isr_default(void)
{
  /* Write your interrupt code here ... */
  UserEnterCritical();
  while(1)
  {
    //__RESET_WATCHDOG();     
  }
}
/* end of isr_default */


__interrupt void Erro_Address(void)
{
  /* Write your interrupt code here ... */
  UserEnterCritical();
  while(1)
  {
    //__RESET_WATCHDOG();    
  }
}
/* end of isr_default */


__interrupt void Erro_accerr(void)
{
  /* Write your interrupt code here ... */
  UserEnterCritical();
  while(1)
  {
     //__RESET_WATCHDOG(); 
  }
}
/* end of isr_default */

/*
** ===================================================================
**     Interrupt handler : isrVlvd
**
**     Description :
**         User interrupt service routine. 
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
__interrupt void isrVlvd(void)
{
  /* Write your interrupt code here ... */
  UserEnterCritical();
  while(1)
  {
    // __RESET_WATCHDOG();
 
  }
 
}
/* end of isrVlvd */

/* external ISR handlers */
extern asm void   _startup(void); 
extern void       SwitchContext(void);
extern void       Radio_Interrupt(void);
extern void       ADC_Tick(void);
extern void       Relay_TurnOn(void);
extern void       TickTimer(void);
extern void       isrVlvd(void);

void  (* const _UserEntry[])()@0x000021D0=
{
  (void (*const)(void))0x4E714EF9,           //asm NOP(0x4E71), asm JMP(0x4EF9)
  _startup,
};

/* Initialization of the CPU registers in FLASH */

#if (BOOTLOADER_ENABLE == 1)

  #include "MCF51_Bootloader.h"

  /* Bootloader code space 4kB*/
  // 0x00000000-0x00000FFF are protected          
  const byte NVPROT_INIT @0x0000040D  = 0xFB;
  
  /* old: Bootloader code space 8kB */
  // 0x00000000-0x00001FFF are protected 
  //const byte NVPROT_INIT @0x0000040D  = 0xF7;
  
typedef void (* pFun)(void);

void dummy_ISR(void);

__interrupt void dummy_ISR(void)
{
  
  UserEnterCritical();
  while(1)
  {
     //__RESET_WATCHDOG();
 
  }
} 

 
void  (* const RAM_vector[])()@0x00002000= {
    (pFun)&dummy_ISR,             // vector_0  INITSP
    (pFun)&dummy_ISR,             // vector_1  INITPC
    (pFun)&dummy_ISR,             // vector_2  Vaccerr
    (pFun)&dummy_ISR,             // vector_3  Vadderr
    (pFun)&dummy_ISR,             // vector_4  Viinstr
    (pFun)&dummy_ISR,             // vector_5  Vdivz
    (pFun)&dummy_ISR,             // vector_6  VReserved6
    (pFun)&dummy_ISR,             // vector_7  VReserved7
    (pFun)&dummy_ISR,             // vector_8  Vprviol
    (pFun)&dummy_ISR,             // vector_9  Vtrace
    (pFun)&dummy_ISR,             // vector_10 Vunilaop
    (pFun)&dummy_ISR,             // vector_11 Vunilfop
    (pFun)&dummy_ISR,             // vector_12 Vdbgi
    (pFun)&dummy_ISR,             // vector_13 VReserved13
    (pFun)&dummy_ISR,             // vector_14 Vferror
    (pFun)&dummy_ISR,             // vector_15 VReserved15
    (pFun)&dummy_ISR,             // vector_16 VReserved16
    (pFun)&dummy_ISR,             // vector_17 VReserved17
    (pFun)&dummy_ISR,             // vector_18 VReserved18
    (pFun)&dummy_ISR,             // vector_19 VReserved19
    (pFun)&dummy_ISR,             // vector_20 VReserved20
    (pFun)&dummy_ISR,             // vector_21 VReserved21
    (pFun)&dummy_ISR,             // vector_22 VReserved22
    (pFun)&dummy_ISR,             // vector_23 VReserved23
    (pFun)&dummy_ISR,             // vector_24 Vspuri
    (pFun)&dummy_ISR,             // vector_25 VReserved25
    (pFun)&dummy_ISR,             // vector_26 VReserved26
    (pFun)&dummy_ISR,             // vector_27 VReserved27 
    (pFun)&dummy_ISR,             // vector_28 VReserved28
    (pFun)&dummy_ISR,             // vector_29 VReserved29
    (pFun)&dummy_ISR,             // vector_30 VReserved30
    (pFun)&dummy_ISR,             // vector_31 VReserved31
    (pFun)&dummy_ISR,             // vector_32 Vtrap0
    (pFun)&dummy_ISR,             // vector_33 Vtrap1
    (pFun)&dummy_ISR,             // vector_34 Vtrap2 
    (pFun)&dummy_ISR,             // vector_35 Vtrap3
    (pFun)&dummy_ISR,             // vector_36 Vtrap4
    (pFun)&dummy_ISR,             // vector_37 Vtrap5
    (pFun)&dummy_ISR,             // vector_38 Vtrap6
    (pFun)&dummy_ISR,             // vector_39 Vtrap7
    (pFun)&dummy_ISR,             // vector_40 Vtrap8
    (pFun)&dummy_ISR,             // vector_41 Vtrap9 
    (pFun)&dummy_ISR,             // vector_42 Vtrap10
    (pFun)&dummy_ISR,             // vector_43 Vtrap11
    (pFun)&dummy_ISR,             // vector_44 Vtrap12
    (pFun)&dummy_ISR,             // vector_45 Vtrap13 
    (pFun)&SwitchContext,         // vector_46 Vtrap14
    (pFun)&dummy_ISR,             // vector_47 Vtrap15
    (pFun)&dummy_ISR,             // vector_48 VReserved48
    (pFun)&dummy_ISR,             // vector_49 VReserved49
    (pFun)&dummy_ISR,             // vector_50 VReserved50
    (pFun)&dummy_ISR,             // vector_51 VReserved51
    (pFun)&dummy_ISR,             // vector_52 VReserved52
    (pFun)&dummy_ISR,             // vector_53 VReserved53
    (pFun)&dummy_ISR,             // vector_54 VReserved54
    (pFun)&dummy_ISR,             // vector_55 VReserved55
    (pFun)&dummy_ISR,             // vector_56 VReserved56
    (pFun)&dummy_ISR,             // vector_57 VReserved57
    (pFun)&dummy_ISR,             // vector_58 VReserved58
    (pFun)&dummy_ISR,             // vector_59 VReserved59
    (pFun)&dummy_ISR,             // vector_60 VReserved60
    (pFun)&dummy_ISR,             // vector_61 VReserved61
    (pFun)&dummy_ISR,             // vector_62 VReserved62
    (pFun)&dummy_ISR,             // vector_63 VReserved63
    (pFun)&dummy_ISR,             // vector_64 Virq
    (pFun)&isrVlvd,               // vector_65 Vlvd
    (pFun)&dummy_ISR,             // vector_66 Vtpm1ch0
    #if(NETWORK_ENABLE == 1)
    (pFun)&Radio_Interrupt,       // vector_67 Vtpm1ch1
    #else
    (pFun)&dummy_ISR,             // vector_67 Vtpm1ch2
    #endif
    (pFun)&dummy_ISR,             // vector_68 Vtpm1ch2
    (pFun)&TickTimer,             // vector_69 Vtpm1ovf
    (pFun)&dummy_ISR,             // vector_70 Vtpm2ch0
    (pFun)&dummy_ISR,             // vector_71 Vtpm2ch1
    (pFun)&dummy_ISR,             // vector_72 Vtpm2ch2    
    (pFun)&ADC_Tick,               // vector_73 Vtpm2ovf
    (pFun)&dummy_ISR,             // vector_74 Vspi2    
    (pFun)&dummy_ISR,             // vector_75 Vspi1
    (pFun)&dummy_ISR,             // vector_76 Vsci1err
    (pFun)&dummy_ISR,             // vector_77 Vsci1rx
    (pFun)&dummy_ISR,             // vector_78 Vsci1tx
    (pFun)&dummy_ISR,             // vector_79 Viicx
    (pFun)&dummy_ISR,             // vector_80 Vkeyboard
    (pFun)&dummy_ISR,             // vector_81 Vadc
    (pFun)&dummy_ISR,             // vector_82 Vacmpx
    (pFun)&dummy_ISR,             // vector_83 Vsci2err
    (pFun)&dummy_ISR,             // vector_84 Vsci2rx
    (pFun)&dummy_ISR,             // vector_85 Vsci2tx
    (pFun)&dummy_ISR,             // vector_86 Vrtc
    (pFun)&dummy_ISR,             // vector_87 Vtpm3ch0
    (pFun)&dummy_ISR,             // vector_88 Vtpm3ch1
    (pFun)&dummy_ISR,             // vector_89 Vtpm3ch2
    (pFun)&dummy_ISR,             // vector_90 Vtpm3ch3
    (pFun)&dummy_ISR,             // vector_91 Vtpm3ch4
    (pFun)&dummy_ISR,             // vector_92 Vtpm3ch5
    (pFun)&Relay_TurnOn,          // vector_93 Vtpm3ovf
    (pFun)&dummy_ISR,             // vector_94 VReserved94
    (pFun)&dummy_ISR,             // vector_95 VReserved95
    (pFun)&dummy_ISR,             // vector_96 VL7swi 
    (pFun)&dummy_ISR,             // vector_97 VL6swi
    (pFun)&dummy_ISR,             // vector_98 VL5swi
    (pFun)&dummy_ISR,             // vector_99 VL4swi
    (pFun)&dummy_ISR,             // vector_100 VL3swi
    (pFun)&dummy_ISR,             // vector_101 VL2swi
    (pFun)&dummy_ISR,             // vector_102 VL1swi
}; 


void ISR_vtable_reallocation(void) 
{
  
// vector table reallocation into RAM
/**************************************/ 
INT32U *pdst,*psrc;
byte i; 

asm (move.l  #0x00800000,d0);
asm (movec   d0,vbr);

pdst=(INT32U*)0x00800000;
psrc=(INT32U*)&RAM_vector;

for (i=0;i<103;i++,pdst++,psrc++)
{
  *pdst=*psrc;
}

/**************************************/ 
} 

#endif



#if(BOOTLOADER_ENABLE == 0)


extern unsigned long far _SP_INIT[];

 //interrupt vector table 
#ifndef UNASSIGNED_ISR
  #define UNASSIGNED_ISR isr_default   // unassigned interrupt service routine 
#endif

 
                                                               //ddress    Lvl Pri 
void (*const vector_0)(void)   @INITSP      = (void(*const )(void))_SP_INIT; //x00000000 -   - //
void (*const vector_1)(void)   @INITPC      = _startup;         /*0x00000004 -   - */
void (*const vector_2)(void)   @Vaccerr     = Erro_accerr;      /*0x00000008 -   - */
void (*const vector_3)(void)   @Vadderr     = Erro_Address;     /*0x0000000C -   - */
void (*const vector_4)(void)   @Viinstr     = UNASSIGNED_ISR;   /*0x00000010 -   - */
void (*const vector_5)(void)   @VReserved5  = UNASSIGNED_ISR;   /*0x00000014 -   - */
void (*const vector_6)(void)   @VReserved6  = UNASSIGNED_ISR;   /*0x00000018 -   - */
void (*const vector_7)(void)   @VReserved7  = UNASSIGNED_ISR;   /*0x0000001C -   - */
void (*const vector_8)(void)   @Vprviol     = UNASSIGNED_ISR;   /*0x00000020 -   - */
void (*const vector_9)(void)   @Vtrace      = UNASSIGNED_ISR;   /*0x00000024 -   - */
void (*const vector_10)(void)  @Vunilaop    = UNASSIGNED_ISR;   /*0x00000028 -   - */
void (*const vector_11)(void)  @Vunilfop    = UNASSIGNED_ISR;   /*0x0000002C -   - */
void (*const vector_12)(void)  @Vdbgi       = UNASSIGNED_ISR;   /*0x00000030 -   - */
void (*const vector_13)(void)  @VReserved13 = UNASSIGNED_ISR;   /*0x00000034 -   - */
void (*const vector_14)(void)  @Vferror     = UNASSIGNED_ISR;   /*0x00000038 -   - */
void (*const vector_15)(void)  @VReserved15 = UNASSIGNED_ISR;   /*0x0000003C -   - */
void (*const vector_16)(void)  @VReserved16 = UNASSIGNED_ISR;   /*0x00000040 -   - */
void (*const vector_17)(void)  @VReserved17 = UNASSIGNED_ISR;   /*0x00000044 -   - */
void (*const vector_18)(void)  @VReserved18 = UNASSIGNED_ISR;   /*0x00000048 -   - */
void (*const vector_19)(void)  @VReserved19 = UNASSIGNED_ISR;   /*0x0000004C -   - */
void (*const vector_20)(void)  @VReserved20 = UNASSIGNED_ISR;   /*0x00000050 -   - */
void (*const vector_21)(void)  @VReserved21 = UNASSIGNED_ISR;   /*0x00000054 -   - */
void (*const vector_22)(void)  @VReserved22 = UNASSIGNED_ISR;   /*0x00000058 -   - */
void (*const vector_23)(void)  @VReserved23 = UNASSIGNED_ISR;   /*0x0000005C -   - */
void (*const vector_24)(void)  @Vspuri      = UNASSIGNED_ISR;   /*0x00000060 -   - */
void (*const vector_25)(void)  @VReserved25 = UNASSIGNED_ISR;   /*0x00000064 -   - */
void (*const vector_26)(void)  @VReserved26 = UNASSIGNED_ISR;   /*0x00000068 -   - */
void (*const vector_27)(void)  @VReserved27 = UNASSIGNED_ISR;   /*0x0000006C -   - */
void (*const vector_28)(void)  @VReserved28 = UNASSIGNED_ISR;   /*0x00000070 -   - */
void (*const vector_29)(void)  @VReserved29 = UNASSIGNED_ISR;   /*0x00000074 -   - */
void (*const vector_30)(void)  @VReserved30 = UNASSIGNED_ISR;   /*0x00000078 -   - */
void (*const vector_31)(void)  @VReserved31 = UNASSIGNED_ISR;   /*0x0000007C -   - */
void (*const vector_32)(void)  @Vtrap0      = UNASSIGNED_ISR;    /*0x00000080 -   - */
void (*const vector_33)(void)  @Vtrap1      = UNASSIGNED_ISR;   /*0x00000084 -   - */
void (*const vector_34)(void)  @Vtrap2      = UNASSIGNED_ISR;   /*0x00000088 -   - */
void (*const vector_35)(void)  @Vtrap3      = UNASSIGNED_ISR;   /*0x0000008C -   - */
void (*const vector_36)(void)  @Vtrap4      = UNASSIGNED_ISR;   /*0x00000090 -   - */
void (*const vector_37)(void)  @Vtrap5      = UNASSIGNED_ISR;   /*0x00000094 -   - */
void (*const vector_38)(void)  @Vtrap6      = UNASSIGNED_ISR;   /*0x00000098 -   - */
void (*const vector_39)(void)  @Vtrap7      = UNASSIGNED_ISR;   /*0x0000009C -   - */
void (*const vector_40)(void)  @Vtrap8      = UNASSIGNED_ISR;   /*0x000000A0 -   - */
void (*const vector_41)(void)  @Vtrap9      = UNASSIGNED_ISR;   /*0x000000A4 -   - */
void (*const vector_42)(void)  @Vtrap10     = UNASSIGNED_ISR;   /*0x000000A8 -   - */
void (*const vector_43)(void)  @Vtrap11     = UNASSIGNED_ISR;   /*0x000000AC -   - */
void (*const vector_44)(void)  @Vtrap12     = UNASSIGNED_ISR;   /*0x000000B0 -   - */
void (*const vector_45)(void)  @Vtrap13     = UNASSIGNED_ISR;   /*0x000000B4 -   - */
void (*const vector_46)(void)  @Vtrap14     = SwitchContext;    /*0x000000B8 -   - */
void (*const vector_47)(void)  @Vtrap15     = UNASSIGNED_ISR;   /*0x000000BC -   - */
void (*const vector_48)(void)  @VReserved48 = UNASSIGNED_ISR;   /*0x000000C0 -   - */
void (*const vector_49)(void)  @VReserved49 = UNASSIGNED_ISR;   /*0x000000C4 -   - */
void (*const vector_50)(void)  @VReserved50 = UNASSIGNED_ISR;   /*0x000000C8 -   - */
void (*const vector_51)(void)  @VReserved51 = UNASSIGNED_ISR;   /*0x000000CC -   - */
void (*const vector_52)(void)  @VReserved52 = UNASSIGNED_ISR;   /*0x000000D0 -   - */
void (*const vector_53)(void)  @VReserved53 = UNASSIGNED_ISR;   /*0x000000D4 -   - */
void (*const vector_54)(void)  @VReserved54 = UNASSIGNED_ISR;   /*0x000000D8 -   - */
void (*const vector_55)(void)  @VReserved55 = UNASSIGNED_ISR;   /*0x000000DC -   - */
void (*const vector_56)(void)  @VReserved56 = UNASSIGNED_ISR;   /*0x000000E0 -   - */
void (*const vector_57)(void)  @VReserved57 = UNASSIGNED_ISR;   /*0x000000E4 -   - */
void (*const vector_58)(void)  @VReserved58 = UNASSIGNED_ISR;   /*0x000000E8 -   - */
void (*const vector_59)(void)  @VReserved59 = UNASSIGNED_ISR;   /*0x000000EC -   - */
void (*const vector_60)(void)  @VReserved60 = UNASSIGNED_ISR;   /*0x000000F0 -   - */
void (*const vector_61)(void)  @Vunsinstr   = UNASSIGNED_ISR;   /*0x000000F4 -   - */
void (*const vector_62)(void)  @VReserved62 = UNASSIGNED_ISR;   /*0x000000F8 -   - */
void (*const vector_63)(void)  @VReserved63 = UNASSIGNED_ISR;   /*0x000000FC -   - */
void (*const vector_64)(void)  @Virq        = UNASSIGNED_ISR;   /*0x00000100 -   - */
void (*const vector_65)(void)  @Vlvd        = isrVlvd;          /*0x00000104 7   3 */
void (*const vector_66)(void)  @Vtpm1ch0    = UNASSIGNED_ISR;   /*0x00000108 -   - */
void (*const vector_67)(void)  @Vtpm1ch1    = Radio_Interrupt;  /*0x0000010C -   - */
void (*const vector_68)(void)  @Vtpm1ch2    = UNASSIGNED_ISR;   /*0x00000110 -   - */
void (*const vector_69)(void)  @Vtpm1ovf    = TickTimer;        /*0x00000114 6   1 */
void (*const vector_70)(void)  @Vtpm2ch0    = UNASSIGNED_ISR;   /*0x00000118 -   - */
void (*const vector_71)(void)  @Vtpm2ch1    = UNASSIGNED_ISR;   /*0x0000011C -   - */        
void (*const vector_72)(void)  @Vtpm2ch2    = UNASSIGNED_ISR;   /*0x00000120 -   - */
void (*const vector_73)(void)  @Vtpm2ovf    = ADC_Tick;          /*0x00000124 -   - */
void (*const vector_74)(void)  @Vspi2       = UNASSIGNED_ISR;   /*0x00000128 -   - */
void (*const vector_75)(void)  @Vspi1       = UNASSIGNED_ISR;   /*0x0000012C -   - */
void (*const vector_76)(void)  @Vsci1err    = UNASSIGNED_ISR;   /*0x00000130 -   - */
void (*const vector_77)(void)  @Vsci1rx     = UNASSIGNED_ISR;   /*0x00000134 -   - */
void (*const vector_78)(void)  @Vsci1tx     = UNASSIGNED_ISR;   /*0x00000138 -   - */
void (*const vector_79)(void)  @Viicx       = UNASSIGNED_ISR;   /*0x0000013C -   - */
void (*const vector_80)(void)  @Vkeyboard   = UNASSIGNED_ISR;   /*0x00000140 3   6 */
void (*const vector_81)(void)  @Vadc        = UNASSIGNED_ISR;   /*0x00000144 -   - */
void (*const vector_82)(void)  @Vacmpx      = UNASSIGNED_ISR;   /*0x00000148 -   - */
void (*const vector_83)(void)  @Vsci2err    = UNASSIGNED_ISR;   /*0x0000014C -   - */
void (*const vector_84)(void)  @Vsci2rx     = UNASSIGNED_ISR;   /*0x00000150 -   - */
void (*const vector_85)(void)  @Vsci2tx     = UNASSIGNED_ISR;   /*0x00000154 -   - */
void (*const vector_86)(void)  @Vrtc        = UNASSIGNED_ISR;   /*0x00000158 -   - */
void (*const vector_87)(void)  @Vtpm3ch0    = UNASSIGNED_ISR;    /*0x0000015C -   - */
void (*const vector_88)(void)  @Vtpm3ch1    = UNASSIGNED_ISR;   /*0x00000160 -   - */
void (*const vector_89)(void)  @Vtpm3ch2    = UNASSIGNED_ISR;   /*0x00000164 -   - */
void (*const vector_90)(void)  @Vtpm3ch3    = UNASSIGNED_ISR;   /*0x00000168 -   - */
void (*const vector_91)(void)  @Vtpm3ch4    = UNASSIGNED_ISR;   /*0x0000016C -   - */
void (*const vector_92)(void)  @Vtpm3ch5    = UNASSIGNED_ISR;   /*0x00000170 -   - */
void (*const vector_93)(void)  @Vtpm3ovf    = Relay_TurnOn;     /*0x00000174 -   - */
void (*const vector_94)(void)  @VReserved94 = UNASSIGNED_ISR;   /*0x00000178 -   - */
void (*const vector_95)(void)  @VReserved95 = UNASSIGNED_ISR;   /*0x0000017C -   - */
void (*const vector_96)(void)  @VL7swi      = UNASSIGNED_ISR;   /*0x00000180 -   - */
void (*const vector_97)(void)  @VL6swi      = UNASSIGNED_ISR;   /*0x00000184 -   - */
void (*const vector_98)(void)  @VL5swi      = UNASSIGNED_ISR;   /*0x00000188 -   - */
void (*const vector_99)(void)  @VL4swi      = UNASSIGNED_ISR;   /*0x0000018C -   - */
void (*const vector_100)(void) @VL3swi      = UNASSIGNED_ISR;   /*0x00000190 -   - */
void (*const vector_101)(void) @VL2swi      = UNASSIGNED_ISR;   /*0x00000194 -   - */
void (*const vector_102)(void) @VL1swi      = UNASSIGNED_ISR;   /*0x00000198 -   - */

#endif



/* END MCUinit */

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
