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
#define USB_CDC_TERMINAL 
 
#ifdef HID_MOUSE
#include "hid_usb_config.h"
#elif defined HID_KBD_DEMO
#include "kbd_usb_config.h"
#elif defined HID_JOYSTICK_DEMO
#include "joy_usb_config.h"
#elif defined USB_CDC_TERMINAL
#include "cdc_usb_config.h"
#elif defined USB_CDC_SERIAL
#include "cdc_usb_config.h"
#elif defined HID_DEMO
#include "hid_usb_config.h"
#else
#error "No USB configuration selected!"
#endif

/****************************** END OF FILE **********************************/
