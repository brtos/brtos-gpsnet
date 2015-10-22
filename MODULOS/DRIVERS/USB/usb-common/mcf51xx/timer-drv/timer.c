/***************************************************************************
 *
 *            Copyright (c) 2007 by CMX Systems, Inc.
 *
 * This software is copyrighted by and is the sole property of
 * CMX.  All rights, title, ownership, or other interests
 * in the software remain the property of CMX.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of CMX.
 *
 * CMX reserves the right to modify this software without notice.
 *
 * CMX Systems, Inc.
 * 12276 San Jose Blvd. #511
 * Jacksonville, FL 32223
 * USA
 *
 * Tel:  (904) 880-1840
 * Fax:  (904) 880-1632
 * http: www.cmx.com
 * email: cmx@cmx.com
 *
 ***************************************************************************/
#include "timer.h"
#include "mcf51xx_reg.h"
#include "hcc_types.h"

/*
 * FUNCTION NAME: start_mS_timer:
 * DESCRIPTION:   to start a timer in milliseconds based on 1ms timer. 
 *                For fixed clock source, it is MCGFFCLK/2 as reference to TPM (see Figure 1-2. System Clock Distribution Diagram)
 *                For JM60/16, since the fixed clock is 1.5MHz derived from 12MHz external clock,
 *                TPM reference clcok is 1.5MHz/2 = 750KHz, and the TPM output clock is 5.859 KHz, after devided by 128. 
 *                With multiplication factor of 6 in MOD register and 
 *                it can achieve 1.024ms. The tolerance of the frequency is 6.15%.
 *                In addition, it can achieve 6.5s max.
 *                If MOD register value is M, then it will count M+1 prescaler clocks for free running counter.
 *                
 * INPUT PARAMETER:
 *    delay  -     the desired delay in millisconds
 * OUTPUT PARAMETER:
 * RETURN:
 */
void start_mS_timer(hcc_u16 delay)
{     
 TPM1MOD = (delay*6);
 TPM1C0V = (delay*6);
 TPM1CNT = 0;
// TPM1SC_TOIE  = 1;       /* enable the timer overflow interrupt */
// TPM1C0SC_CH0IE = 1;
 TPM1SC_CPWMS = 0;
 TPM1C0SC_MS0x = 1;
 TPM1C0SC_ELS0x = 1;    /* toggle output */ 
 TPM1SC_PS =   7;        /* prescalor 128 */
 TPM1SC_CLKSA = 0;
 TPM1SC_CLKSB = 1;       /* select Fixed clock as source clock */

}

#if 0
interrupt VectorNumber_Vtpm1ovf void  TPMOvrFlow() 
{
  
  /* Clear interrupt flag */
  TPM1SC_TOF = 0;  

}

interrupt  VectorNumber_Vtpm1ch0 void TPMCHnEvent()
{ 
    
   /* Clear interrupt flag */
   TPM1C0SC_CH0F = 0;
}
#endif

/*
 * FUNCTION NAME: check_mS_timer:
 * DESCRIPTION:   to check the timer if it is expired.
 * INPUT PARAMETER:
 *    none
 * OUTPUT PARAMETER:
 * RETURN:
 *    0		-	timer is not expired
 *    1		-	timer is expired
 */
hcc_u8 check_mS_timer(void)
{
  if (!TPM1SC_TOF) {
    return (0);
  }
  TPM1SC_TOF = 0;
  TPM1SC_CLKSx = 0;   /* disable the timer */
  return(1);
 
}

/****************************** END OF FILE **********************************/
