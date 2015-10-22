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
#ifndef _CMX_TYPES_H_
#define _CMX_TYPES_H_
#include  <mcf51jm128.h>
#include "BRTOS_USB.h"

/* Type definitions */
typedef unsigned char hcc_u8;
typedef unsigned short hcc_u16;
typedef unsigned long int hcc_u32;

typedef volatile hcc_u8 hcc_reg8;
typedef volatile hcc_u16 hcc_reg16;
typedef volatile hcc_u32 hcc_reg32;

typedef hcc_u32 hcc_imask;

#ifdef NDEBUG
#define CMX_ASSERT(c) (void)0
#else
#define CMX_ASSERT(c)\
do {\
  if(!(c))\
  {\
    int a=1;\
    while(a)\
      ;\
  }\
}while(0)
#endif


#define WR_LE32(a, v) ((*(hcc_u32*)(a))= mcf5xxx_byterev(v))
#define WR_LE16(a, v) ((*(hcc_u16*)(a))=(hcc_u16)(mcf5xxx_byterev(v) >> 16))
#define WR_LE16(a, v) ((*(hcc_u16*)(a))=(hcc_u16)(mcf5xxx_byterev(v) >> 16))
#define RD_LE32(a)    (mcf5xxx_byterev(*(hcc_u32*)(a)))
#define RD_LE16(a)    ((hcc_u16)(mcf5xxx_byterev((hcc_u32)*(hcc_u16*)(a))>>16))

/* Read 16 bit big endian value from address. */
#define RD_BE16(a) (*(hcc_u16*)(a))
/* Write 16bit value in v to address a in big endian order. */
#define WR_BE16(a, v) (*(hcc_u16*)(a) = (hcc_u16)(v))
/* Read 32 bit little endian value from address. */
#define RD_BE32(a) (*(hcc_u32*)(a))
/* Write 32bit value in v to address a in big endian order. */
#define WR_BE32(a, v) (*(hcc_u32*)(a) = (hcc_u32)(v))


#endif /*_CMX_TYPES_H_*/

/****************************** END OF FILE **********************************/
