/****************************************************************************
 *
 *            Copyright (c) 2006-2007 by CMX Systems, Inc.
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
#include "uart-drv/uart.h"
#include "derivative.h"

/****************************************************************************
 ************************** Macro definitions *******************************
 ***************************************************************************/
#define UART_NUMBER   0u
#define	BUS_CLK       24000000ul	/* bus frequency in Hz */

#if UART_NUMBER==0
  #define SCIBD SCI1BD
  #define SCIC1 SCI1C1
  #define SCIC2 SCI1C2
  #define SCIS1 SCI1S1
  #define SCIS2 SCI1S2
  #define SCIC3 SCI1C3
  #define SCID  SCI1D
#elif UART_NUMBER==1
  #define SCIBD SCI2BD
  #define SCIC1 SCI2C1
  #define SCIC2 SCI2C2
  #define SCIS1 SCI2S1
  #define SCIS2 SCI2S2
  #define SCIC3 SCI2C3
  #define SCID  SCI2D  
#endif

/****************************************************************************
 ************************** Module variables ********************************
 ***************************************************************************/
/* none */

/****************************************************************************
 ************************** Function definitions ****************************
 ***************************************************************************/

/*****************************************************************************
 * Configure UART.
 ****************************************************************************/
void uart_init(hcc_u32 bps, hcc_u8 stp, hcc_u8 par, hcc_u8 ndata)
{
  hcc_u8 c1=0, c3=0;

  /* Disable receiver and transmitter */
  SCIC2=0;
  while(SCIC2 & (SCI1C2_RE_MASK | SCI1C2_TE_MASK))
    ;

  /* Select configuration. */
  switch(par)
  {
  case 'n': /* no parity */
    break;
  case 'o': /* odd parity */
    c1 |= SCI1C1_PE_MASK;
    break;  
  case 'e': /* even parity */
    c1 |= SCI1C1_PE_MASK | SCI1C1_PT;
    break;
  default:  
    /* unsupported format */
    CMX_ASSERT(0);  
    break;          
  }
  
  switch(ndata)
  {
  case 8:
    break;
  default:  
    /* unsupported format */
    CMX_ASSERT(0);  
    break;      
  }
  
  switch(stp)
  {
  case 1:      /* 1 stop bit */
    break;
  default:
    /* unsupported format. */
    CMX_ASSERT(0)    ;
    break;
  }

  /* Set line coding. */
  SCIC1=c1;
  /* Set Rx and Tx baud */
  (void)uart_set_bps(bps);
  /* Enable receiver and transmitter */ 
  SCIC2 = SCI1C2_TE_MASK | SCI1C2_RE_MASK;
}

/*****************************************************************************
 * Return current used baud rate.
 ****************************************************************************/
hcc_u32 uart_get_bps(void)
{
  return((BUS_CLK/16)/(SCIBD&((1<<13)-1)));
}

/*****************************************************************************
 * Seat baud rate.
 ****************************************************************************/
hcc_u32 uart_set_bps(hcc_u32 bps)
{

  /* Calculate baud settings */
  hcc_u16 d = (hcc_u16)((BUS_CLK/16)/bps);

  if (d> 8192)
  {
    d=8192;  
  }
  if (d <1)
  {
    d=1;
  }
  SCIBD=d;
  return(uart_get_bps());
}

/*****************************************************************************
 * Send one character.
 ****************************************************************************/
int uart_putch(char c)
{ 
  if (SCIS1 & SCI1S1_TDRE_MASK)
  {
    /* Send the character */
    SCID = c;
    return((int)c);
  }
  return((int)(c+1));
}

/*****************************************************************************
 * Wait till a character is received, and return it.
 ****************************************************************************/
int uart_getch(void)
{
  return ((int)SCID);
}

/*****************************************************************************
 * Return !=0 if there is at least one pending character in the receiver.
 ****************************************************************************/
int uart_kbhit(void)
{
  return((int)((SCIS1 & SCI1S1_RDRF_MASK) ? 1 : 0));  
}

#if 0
int hcc_u8 uart_input_ready(void) 
{
  return ((int)((SCIS1 & SCI1S1_RDRF_MASK) ? 1:0));
}
#endif
/****************************** END OF FILE **********************************/
