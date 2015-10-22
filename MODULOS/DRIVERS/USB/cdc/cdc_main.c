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
#include "../usb-drv/usb.h"
#include "target.h"
#include "../cdc_drv/usb_cdc.h"
#include "hcc_terminal.h"

/****************************************************************************
 ************************** Macro definitions *******************************
 ***************************************************************************/
/* none */

/****************************************************************************
 ************************** Function predefinitions. ************************
 ***************************************************************************/
/* none */

/****************************************************************************
 ************************** Global variables ********************************
 ***************************************************************************/
/* none */

/****************************************************************************
 ************************** Module variables ********************************
 ***************************************************************************/
/* none */

/****************************************************************************
 ************************** Function definitions ****************************
 ***************************************************************************/
 
static void cmd_led(char *param)
{
  static led_on=0;
  param++;  
  if (led_on)
  {
    led_on=0;
    PTED_PTED2=0;
  }
  else
  {
    led_on=1;
    PTED_PTED2=1;
  }

}

static const command_t led_cmd = {
  "led", cmd_led, "Toggles leds state."
};

int main()
{
  hw_init();
  
  usb_cfg_init();

  cdc_init();
  terminal_init(cdc_putch, cdc_getch, cdc_kbhit);
  (void)terminal_add_cmd((command_t*)&led_cmd);

  /* This loop will gateway charactert from the UART to the USB and back. */
  while(1)
  {
    terminal_process();
    cdc_process();
  }
  return(0);
} 
 
 
#if 0
int main()
{
  int cdc_in;
  int uart_in;
  static char uart_out[]=" This is UART side\n\rHit enter after USB term ready\n\r\n";
  static char usb_out[]=" This is USB side\n\r\n";
  unsigned char index;

  stack_init(0x88);  

  hw_init();

  Usb_Vbus_Off();

  cdc_in=uart_in=0xff+1;

  usb_cfg_init();

  /* 9600 bps, 1 sto bit, no parity, 8 data bits */
  /* Because the PE USB Terminal does not support higher baudrate. */
  uart_init(9600u, 1, 'n', 8);

  cdc_init();

  for (index=0; index<sizeof(uart_out); index++)
  {
    while((int)uart_out[index]!=uart_putch(uart_out[index]))
	  ;
  }

  while(usb_get_state() != USBST_CONFIGURED)
  {
    cdc_process();
  }

  uart_getch();
  for (index=0; index<sizeof(usb_out); index++)
  {
	while((int)usb_out[index]!=cdc_putch(usb_out[index]))
	  ;
  }

  /* This loop will gateway charactert from the UART to the USB and back. */
  while(1)
  {
    cdc_process();
    /* Set uart line coding to match USB CDC settings if neded. */
    if (cdc_line_coding_changed())
    {
       line_coding_t l;
       hcc_u8 parity[]="noe";
       cdc_get_line_coding(&l);
       uart_init(l.bps, (hcc_u8)(l.nstp+1), parity[l.parity], l.ndata);
    }
    
    /* if cdc2uart buffer is not empry */
    if (cdc_in <= 0xff)
    {
      /* Try to send */
      int uart_out=uart_putch((char)cdc_in);
      /* if successfull, set cdc2uart buffer to empty */
      if (uart_out==cdc_in)
      {
        cdc_in=0xff+1;
      }
    }
    
    /* if cdc2uart buffer is not busy and cdchas received somethign,
       fill cdc2uart buffer  */
    if (cdc_in > 0xff && cdc_kbhit())
    {
      cdc_in=cdc_getch();        
    }

    /* if uart2cdc buffer is empty */
    if (uart_in <= 0xff)
    {
      /* try to send on CDC */
      int cdc_out=cdc_putch((char)uart_in);
      /* if success, set uart2cdc buffer empty */
      if (cdc_out==uart_in)
      {
        uart_in=0xff+1;
      }
    }

    /* if uart2cdc buffer is empty and uart has received something,
       fill uart2cdc buffer */
    if (uart_in > 0xff && uart_kbhit())
    {
      uart_in=uart_getch();
    }
    {
      long stackSize = stack_size(0x88);
    }
  }
  return 0;
}

#endif
/****************************** END OF FILE **********************************/
