/***************************************************************************
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

#ifndef _UART_H_
#define _UART_H_

#include "hcc_types.h"

extern void uart_init(hcc_u32 bps, hcc_u8 stp, hcc_u8 par, hcc_u8 ndata);
extern hcc_u32 uart_get_bps(void);
extern hcc_u32 uart_set_bps(hcc_u32 bps);
extern int uart_putch(char c);
extern int uart_getch(void);
extern int uart_kbhit(void);

#endif
/****************************** END OF FILE **********************************/
