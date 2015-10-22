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
 /*
  * Author: William Jiang
  * Change History:
  *       version 1, Oct.31, 2007 --- initial version
  */
#ifndef _MCF51XX_REGS_H_
#define _MCF51XX_REGS_H_

#include "hcc_types.h"
#include <mcf51jm128.h>

#define LITTLE_ENDIAN_MODE

#define nop()       asm( nop)

/*
 * Memory map definitions from linker command files
 */
extern volatile byte  const _IPSBAR[];

/*
 * Memory Map Info
 */

#define BITX(x) (1U<<(x))
#define BIT0	BITX(0)
#define BIT1	BITX(1)
#define BIT2	BITX(2)
#define BIT3	BITX(3)
#define BIT4	BITX(4)
#define BIT5	BITX(5)
#define BIT6	BITX(6)
#define BIT7 	BITX(7)
#define BIT8	BITX(8)
#define BIT9	BITX(9)
#define BIT10   BITX(10)
#define BIT11	BITX(11)
#define BIT12   BITX(12)
#define BIT13	BITX(13)
#define BIT14   BITX(14)
#define BIT15	BITX(15)
#define BIT16   BITX(16)
#define BIT17	BITX(17)
#define BIT18   BITX(18)
#define BIT19	BITX(19)
#define BIT20   BITX(20)
#define BIT21	BITX(21)
#define BIT22   BITX(22)
#define BIT23	BITX(23)
#define BIT24   BITX(24)
#define BIT25	BITX(25)
#define BIT26   BITX(26)
#define BIT27	BITX(27)
#define BIT28   BITX(28)
#define BIT29	BITX(29)
#define BIT30   BITX(30)
#define BIT31	BITX(31)


/*********************************************************************
*
* Core register bit definitions
*
*********************************************************************/
/* Status Register */
#define MCF5XXX_SR_T        (0x8000)
#define MCF5XXX_SR_S        (0x2000)
#define MCF5XXX_SR_M        (0x1000)
#define MCF5XXX_SR_IPL      (0x0700)
#define MCF5XXX_SR_IPL_0    (0x0000)
#define MCF5XXX_SR_IPL_1    (0x0100)
#define MCF5XXX_SR_IPL_2    (0x0200)
#define MCF5XXX_SR_IPL_3    (0x0300)
#define MCF5XXX_SR_IPL_4    (0x0400)
#define MCF5XXX_SR_IPL_5    (0x0500)
#define MCF5XXX_SR_IPL_6    (0x0600)
#define MCF5XXX_SR_IPL_7    (0x0700)
#define MCF5XXX_SR_X        (0x0010)
#define MCF5XXX_SR_N        (0x0008)
#define MCF5XXX_SR_Z        (0x0004)
#define MCF5XXX_SR_V        (0x0002)
#define MCF5XXX_SR_C        (0x0001)


/*********************************************************************
*
* Universal Serial Bus (USB)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_USB_PER_ID                       (*(hcc_reg8 *)(&_IPSBAR[0x1A00]))
#define MCF_USB_IP_COMP                      (*(hcc_reg8 *)(&_IPSBAR[0x1A04]))
#define MCF_USB_REV                          (*(hcc_reg8 *)(&_IPSBAR[0x1A08]))
#define MCF_USB_ADD_INFO                     (*(hcc_reg8 *)(&_IPSBAR[0x1A0C]))
#define MCF_USB_OTG_INT_STAT                 (*(hcc_reg8 *)(&_IPSBAR[0x1A10]))
#define MCF_USB_OTG_INT_EN                   (*(hcc_reg8 *)(&_IPSBAR[0x1A14]))
#define MCF_USB_OTG_STATUS                   (*(hcc_reg8 *)(&_IPSBAR[0x1A18]))
#define MCF_USB_OTG_CTRL                     (*(hcc_reg8 *)(&_IPSBAR[0x1A1C]))
#define MCF_USB_INT_STAT                     (*(hcc_reg8 *)(&_IPSBAR[0x1A80]))
#define MCF_USB_INT_ENB                      (*(hcc_reg8 *)(&_IPSBAR[0x1A84]))
#define MCF_USB_ERR_STAT                     (*(hcc_reg8 *)(&_IPSBAR[0x1A88]))
#define MCF_USB_ERR_ENB                      (*(hcc_reg8 *)(&_IPSBAR[0x1A8C]))
#define MCF_USB_STAT                         (*(hcc_reg8 *)(&_IPSBAR[0x1A90]))
#define MCF_USB_CTL                          (*(hcc_reg8 *)(&_IPSBAR[0x1A94]))
#define MCF_USB_ADDR                         (*(hcc_reg8 *)(&_IPSBAR[0x1A98]))
#define MCF_USB_BDT_PAGE_01                  (*(hcc_reg8 *)(&_IPSBAR[0x1A9C]))
#define MCF_USB_FRM_NUML                     (*(hcc_reg8 *)(&_IPSBAR[0x1AA0]))
#define MCF_USB_FRM_NUMH                     (*(hcc_reg8 *)(&_IPSBAR[0x1AA4]))
#define MCF_USB_FRM_NUM       (MCF_USB_INT_STAT=MCF_USB_INT_STAT_SOF_TOK ,MCF_USB_FRM_NUML | (((hcc_u16)MCF_USB_FRM_NUMH)<<8))
#define MCF_USB_TOKEN                        (*(hcc_reg8 *)(&_IPSBAR[0x1AA8]))
#define MCF_USB_SOF_THLDL                    (*(hcc_reg8 *)(&_IPSBAR[0x1AAC]))
#define MCF_USB_BDT_PAGE_02                  (*(hcc_reg8 *)(&_IPSBAR[0x1AB0]))
#define MCF_USB_BDT_PAGE_03                  (*(hcc_reg8 *)(&_IPSBAR[0x1AB4]))
#define MCF_USB_ENDPT0                       (*(hcc_reg8 *)(&_IPSBAR[0x1AC0]))
#define MCF_USB_ENDPT1                       (*(hcc_reg8 *)(&_IPSBAR[0x1AC4]))
#define MCF_USB_ENDPT2                       (*(hcc_reg8 *)(&_IPSBAR[0x1AC8]))
#define MCF_USB_ENDPT3                       (*(hcc_reg8 *)(&_IPSBAR[0x1ACC]))
#define MCF_USB_ENDPT4                       (*(hcc_reg8 *)(&_IPSBAR[0x1AD0]))
#define MCF_USB_ENDPT5                       (*(hcc_reg8 *)(&_IPSBAR[0x1AD4]))
#define MCF_USB_ENDPT6                       (*(hcc_reg8 *)(&_IPSBAR[0x1AD8]))
#define MCF_USB_ENDPT7                       (*(hcc_reg8 *)(&_IPSBAR[0x1ADC]))
#define MCF_USB_ENDPT8                       (*(hcc_reg8 *)(&_IPSBAR[0x1AE0]))
#define MCF_USB_ENDPT9                       (*(hcc_reg8 *)(&_IPSBAR[0x1AE4]))
#define MCF_USB_ENDPT10                      (*(hcc_reg8 *)(&_IPSBAR[0x1AE8]))
#define MCF_USB_ENDPT11                      (*(hcc_reg8 *)(&_IPSBAR[0x1AEC]))
#define MCF_USB_ENDPT12                      (*(hcc_reg8 *)(&_IPSBAR[0x1AF0]))
#define MCF_USB_ENDPT13                      (*(hcc_reg8 *)(&_IPSBAR[0x1AF4]))
#define MCF_USB_ENDPT14                      (*(hcc_reg8 *)(&_IPSBAR[0x1AF8]))
#define MCF_USB_ENDPT15                      (*(hcc_reg8 *)(&_IPSBAR[0x1AFC]))
#define MCF_USB_USB_CTRL                     (*(hcc_reg8 *)(&_IPSBAR[0x1B00]))
#define MCF_USB_USB_OTG_OBSERVE              (*(hcc_reg8 *)(&_IPSBAR[0x1B04]))
#define MCF_USB_USB_OTG_CONTROL              (*(hcc_reg8 *)(&_IPSBAR[0x1B08]))

/* two new added registers for V1 */
#define MCF_USB_USBTRC0			                 (*(hcc_reg8 *)(&_IPSBAR[0x1B0C]))
#define MCF_USB_OTGPIN			                 (*(hcc_reg8 *)(&_IPSBAR[0x1B10]))

/* Bit definitions and macros for MCF_USB_USBTRC0 */
#define MCF_USB_USBTRC0_USB_RESUME_INT		(0x01)
#define MCF_USB_USBTRC0_USBVREN			(0x04)
#define MCF_USB_USBTRC0_USBRESMEN		(0x20)
#define MCF_USB_USBTRC0_USBPU			(0x40)
#define MCF_USB_USBTRC0_USBRESET		(0x80)

/* Bit definitions and macros for MCF_USB_OTGPIN */
#define MCF_USB_OTGPIN_SESSVLD			(0x01)
#define MCF_USB_OTGPIN_SESSEND			(0x02)
#define MCF_USB_OTGPIN_VBUSVLD			(0x04)
#define MCF_USB_OTGPIN_PULLUP			(0x08)
#define MCF_USB_OTGPIN_DPDOWN			(0x10)
#define MCF_USB_OTGPIN_DMDOWN			(0x20)
#define MCF_USB_OTGPIN_USBID			(0x40)


/* Bit definitions and macros for MCF_USB_PER_ID */
#define MCF_USB_PER_ID_ID(x)                 (((x)&0x3F)<<0)

/* Bit definitions and macros for MCF_USB_IP_COMP */
#define MCF_USB_IP_COMP_NID(x)               (((x)&0x3F)<<0)

/* Bit definitions and macros for MCF_USB_REV */
#define MCF_USB_REV_REV(x)                   (((x)&0xFF)<<0)

/* Bit definitions and macros for MCF_USB_ADD_INFO */
#define MCF_USB_ADD_INFO_HOST                (0x01)
#define MCF_USB_ADD_INFO_IRQ_NUM(x)          (((x)&0x1F)<<3)

/* Bit definitions and macros for MCF_USB_OTG_INT_STAT */
#define MCF_USB_OTG_INT_STAT_A_VBUS_VLD_CHG  (0x01)
#define MCF_USB_OTG_INT_STAT_B_SESS_END_CHG  (0x04)
#define MCF_USB_OTG_INT_STAT_SESS_VLD_CHG    (0x08)
#define MCF_USB_OTG_INT_STAT_LINE_STATE      (0x20)
#define MCF_USB_OTG_INT_STAT_1_MSEC          (0x40)
#define MCF_USB_OTG_INT_STAT_ID_CHG          (0x80)

/* Bit definitions and macros for MCF_USB_OTG_INT_EN */
#define MCF_USB_OTG_INT_EN_A_VBUS_VLD_EN     (0x01)
#define MCF_USB_OTG_INT_EN_B_SESS_END_EN     (0x04)
#define MCF_USB_OTG_INT_EN_SESS_VLD_EN       (0x08)
#define MCF_USB_OTG_INT_EN_LINE_STATE        (0x20)
#define MCF_USB_OTG_INT_EN_1_MSEC_EN         (0x40)
#define MCF_USB_OTG_INT_EN_ID_EN             (0x80)

/* Bit definitions and macros for MCF_USB_OTG_STATUS */
#define MCF_USB_OTG_STATUS_A_VBUS_VLD        (0x01)
#define MCF_USB_OTG_STATUS_B_SESS_END        (0x04)
#define MCF_USB_OTG_STATUS_SESS_VLD          (0x08)
#define MCF_USB_OTG_STATUS_LINE_STATE        (0x20)
#define MCF_USB_OTG_STATUS_1_MSEC            (0x40)
#define MCF_USB_OTG_STATUS_ID                (0x80)

/* Bit definitions and macros for MCF_USB_OTG_CTRL */
#define MCF_USB_OTG_CTRL_VBUS_DSCHG          (0x01)
#define MCF_USB_OTG_CTRL_VBUS_CHG            (0x02)
#define MCF_USB_OTG_CTRL_OTG_EN              (0x04)
#define MCF_USB_OTG_CTRL_VBUS_ON             (0x08)
#define MCF_USB_OTG_CTRL_DM_LOW              (0x10)
#define MCF_USB_OTG_CTRL_DP_LOW              (0x20)
#define MCF_USB_OTG_CTRL_DM_HIGH             (0x40)
#define MCF_USB_OTG_CTRL_DP_HIGH             (0x80)

/* Bit definitions and macros for MCF_USB_INT_STAT */
#define MCF_USB_INT_STAT_USB_RST             (0x01)
#define MCF_USB_INT_STAT_ERROR               (0x02)
#define MCF_USB_INT_STAT_SOF_TOK             (0x04)
#define MCF_USB_INT_STAT_TOK_DNE             (0x08)
#define MCF_USB_INT_STAT_SLEEP               (0x10)
#define MCF_USB_INT_STAT_RESUME              (0x20)
#define MCF_USB_INT_STAT_ATTACH              (0x40)
#define MCF_USB_INT_STAT_STALL               (0x80)

/* Bit definitions and macros for MCF_USB_INT_ENB */
#define MCF_USB_INT_ENB_USB_RST              (0x01)
#define MCF_USB_INT_ENB_ERROR                (0x02)
#define MCF_USB_INT_ENB_SOF_TOK              (0x04)
#define MCF_USB_INT_ENB_TOK_DNE              (0x08)
#define MCF_USB_INT_ENB_SLEEP                (0x10)
#define MCF_USB_INT_ENB_RESUME               (0x20)
#define MCF_USB_INT_ENB_ATTACH               (0x40)
#define MCF_USB_INT_ENB_STALL                (0x80)

/* Bit definitions and macros for MCF_USB_ERR_STAT */
#define MCF_USB_ERR_STAT_PID_ERR             (0x01)
#define MCF_USB_ERR_STAT_CRC5_EOF            (0x02)
#define MCF_USB_ERR_STAT_CRC16               (0x04)
#define MCF_USB_ERR_STAT_DFN8                (0x08)
#define MCF_USB_ERR_STAT_BTO_ERR             (0x10)
#define MCF_USB_ERR_STAT_DMA_ERR             (0x20)
#define MCF_USB_ERR_STAT_BTS_ERR             (0x80)

/* Bit definitions and macros for MCF_USB_ERR_ENB */
#define MCF_USB_ERR_ENB_PID_ERR              (0x01)
#define MCF_USB_ERR_ENB_CRC5_EOF             (0x02)
#define MCF_USB_ERR_ENB_CRC16                (0x04)
#define MCF_USB_ERR_ENB_DFN8                 (0x08)
#define MCF_USB_ERR_ENB_BTO_ERR              (0x10)
#define MCF_USB_ERR_ENB_DMA_ERR              (0x20)
#define MCF_USB_ERR_ENB_BTS_ERR              (0x80)

/* Bit definitions and macros for MCF_USB_STAT */
#define MCF_USB_STAT_ODD                     (0x04)
#define MCF_USB_STAT_TX                      (0x08)
#define MCF_USB_STAT_ENDP(x)                 (((x)&0x0F)<<4)

/* Bit definitions and macros for MCF_USB_CTL */
#define MCF_USB_CTL_USB_EN_SOF_EN            (0x01)
#define MCF_USB_CTL_ODD_RST                  (0x02)
#define MCF_USB_CTL_RESUME                   (0x04)
#define MCF_USB_CTL_HOST_MODE_EN             (0x08)
#define MCF_USB_CTL_RESET                    (0x10)
#define MCF_USB_CTL_TXDSUSPEND_TOKBUSY       (0x20)
#define MCF_USB_CTL_SE0                      (0x40)
#define MCF_USB_CTL_JSTATE                   (0x80)

/* Bit definitions and macros for MCF_USB_ADDR */
#define MCF_USB_ADDR_ADDR(x)                 (((x)&0x7F)<<0)
#define MCF_USB_ADDR_LS_EN                   (0x80)

/* Bit definitions and macros for MCF_USB_BDT_PAGE_01 */
#define MCF_USB_BDT_PAGE_01_BDT_BA9          (0x02)
#define MCF_USB_BDT_PAGE_01_BDT_BA10         (0x04)
#define MCF_USB_BDT_PAGE_01_BDT_BA11         (0x08)
#define MCF_USB_BDT_PAGE_01_BDT_BA12         (0x10)
#define MCF_USB_BDT_PAGE_01_BDT_BA13         (0x20)
#define MCF_USB_BDT_PAGE_01_BDT_BA14         (0x40)
#define MCF_USB_BDT_PAGE_01_BDT_BA15         (0x80)

/* Bit definitions and macros for MCF_USB_FRM_NUM */
#define MCF_USB_FRM_NUM_FRM(x)               (((x)&0xFFFF)<<0)

/* Bit definitions and macros for MCF_USB_FRM_NUML */
#define MCF_USB_FRM_NUML_FRM0                (0x01)
#define MCF_USB_FRM_NUML_FRM1                (0x02)
#define MCF_USB_FRM_NUML_FRM2                (0x04)
#define MCF_USB_FRM_NUML_FRM3                (0x08)
#define MCF_USB_FRM_NUML_FRM4                (0x10)
#define MCF_USB_FRM_NUML_FRM5                (0x20)
#define MCF_USB_FRM_NUML_FRM6                (0x40)
#define MCF_USB_FRM_NUML_FRM7                (0x80)

/* Bit definitions and macros for MCF_USB_FRM_NUMH */
#define MCF_USB_FRM_NUMH_FRM8                (0x01)
#define MCF_USB_FRM_NUMH_FRM9                (0x02)
#define MCF_USB_FRM_NUMH_FRM10               (0x04)
#define MCF_USB_FRM_NUMH_FRM11               (0x08)
#define MCF_USB_FRM_NUMH_FRM12               (0x10)
#define MCF_USB_FRM_NUMH_FRM13               (0x20)
#define MCF_USB_FRM_NUMH_FRM14               (0x40)
#define MCF_USB_FRM_NUMH_FRM15               (0x80)

/* Bit definitions and macros for MCF_USB_TOKEN */
#define MCF_USB_TOKEN_TOKEN_ENDPT(x)         (((x)&0x0F)<<0)
#define MCF_USB_TOKEN_TOKEN_PID(x)           (((x)&0x0F)<<4)

/* Bit definitions and macros for MCF_USB_SOF_THLDL */
#define MCF_USB_SOF_THLDL_CNT(x)             (((x)&0xFF)<<0)

/* Bit definitions and macros for MCF_USB_BDT_PAGE_02 */
#define MCF_USB_BDT_PAGE_02_BDT_BA16         (0x01)
#define MCF_USB_BDT_PAGE_02_BDT_BA17         (0x02)
#define MCF_USB_BDT_PAGE_02_BDT_BA18         (0x04)
#define MCF_USB_BDT_PAGE_02_BDT_BA19         (0x08)
#define MCF_USB_BDT_PAGE_02_BDT_BA20         (0x10)
#define MCF_USB_BDT_PAGE_02_BDT_BA21         (0x20)
#define MCF_USB_BDT_PAGE_02_BDT_BA22         (0x40)
#define MCF_USB_BDT_PAGE_02_BDT_BA23         (0x80)

/* Bit definitions and macros for MCF_USB_BDT_PAGE_03 */
#define MCF_USB_BDT_PAGE_03_BDT_BA24         (0x01)
#define MCF_USB_BDT_PAGE_03_BDT_BA25         (0x02)
#define MCF_USB_BDT_PAGE_03_BDT_BA26         (0x04)
#define MCF_USB_BDT_PAGE_03_BDT_BA27         (0x08)
#define MCF_USB_BDT_PAGE_03_BDT_BA28         (0x10)
#define MCF_USB_BDT_PAGE_03_BDT_BA29         (0x20)
#define MCF_USB_BDT_PAGE_03_BDT_BA30         (0x40)
#define MCF_USB_BDT_PAGE_03_BDT_BA31         (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT0 */
#define MCF_USB_ENDPT0_EP_HSHK               (0x01)
#define MCF_USB_ENDPT0_EP_STALL              (0x02)
#define MCF_USB_ENDPT0_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT0_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT0_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT0_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT0_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT1 */
#define MCF_USB_ENDPT1_EP_HSHK               (0x01)
#define MCF_USB_ENDPT1_EP_STALL              (0x02)
#define MCF_USB_ENDPT1_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT1_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT1_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT1_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT1_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT2 */
#define MCF_USB_ENDPT2_EP_HSHK               (0x01)
#define MCF_USB_ENDPT2_EP_STALL              (0x02)
#define MCF_USB_ENDPT2_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT2_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT2_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT2_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT2_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT3 */
#define MCF_USB_ENDPT3_EP_HSHK               (0x01)
#define MCF_USB_ENDPT3_EP_STALL              (0x02)
#define MCF_USB_ENDPT3_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT3_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT3_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT3_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT3_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT4 */
#define MCF_USB_ENDPT4_EP_HSHK               (0x01)
#define MCF_USB_ENDPT4_EP_STALL              (0x02)
#define MCF_USB_ENDPT4_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT4_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT4_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT4_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT4_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT5 */
#define MCF_USB_ENDPT5_EP_HSHK               (0x01)
#define MCF_USB_ENDPT5_EP_STALL              (0x02)
#define MCF_USB_ENDPT5_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT5_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT5_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT5_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT5_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT6 */
#define MCF_USB_ENDPT6_EP_HSHK               (0x01)
#define MCF_USB_ENDPT6_EP_STALL              (0x02)
#define MCF_USB_ENDPT6_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT6_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT6_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT6_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT6_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT7 */
#define MCF_USB_ENDPT7_EP_HSHK               (0x01)
#define MCF_USB_ENDPT7_EP_STALL              (0x02)
#define MCF_USB_ENDPT7_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT7_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT7_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT7_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT7_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT8 */
#define MCF_USB_ENDPT8_EP_HSHK               (0x01)
#define MCF_USB_ENDPT8_EP_STALL              (0x02)
#define MCF_USB_ENDPT8_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT8_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT8_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT8_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT8_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT9 */
#define MCF_USB_ENDPT9_EP_HSHK               (0x01)
#define MCF_USB_ENDPT9_EP_STALL              (0x02)
#define MCF_USB_ENDPT9_EP_EN_TX              (0x04)
#define MCF_USB_ENDPT9_EP_EN_RX              (0x08)
#define MCF_USB_ENDPT9_EP_CTL_DIS            (0x10)
#define MCF_USB_ENDPT9_RETRY_DIS             (0x40)
#define MCF_USB_ENDPT9_HOST_WO_HUB           (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT10 */
#define MCF_USB_ENDPT10_EP_HSHK              (0x01)
#define MCF_USB_ENDPT10_EP_STALL             (0x02)
#define MCF_USB_ENDPT10_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT10_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT10_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT10_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT10_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT11 */
#define MCF_USB_ENDPT11_EP_HSHK              (0x01)
#define MCF_USB_ENDPT11_EP_STALL             (0x02)
#define MCF_USB_ENDPT11_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT11_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT11_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT11_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT11_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT12 */
#define MCF_USB_ENDPT12_EP_HSHK              (0x01)
#define MCF_USB_ENDPT12_EP_STALL             (0x02)
#define MCF_USB_ENDPT12_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT12_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT12_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT12_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT12_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT13 */
#define MCF_USB_ENDPT13_EP_HSHK              (0x01)
#define MCF_USB_ENDPT13_EP_STALL             (0x02)
#define MCF_USB_ENDPT13_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT13_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT13_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT13_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT13_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT14 */
#define MCF_USB_ENDPT14_EP_HSHK              (0x01)
#define MCF_USB_ENDPT14_EP_STALL             (0x02)
#define MCF_USB_ENDPT14_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT14_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT14_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT14_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT14_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_ENDPT15 */
#define MCF_USB_ENDPT15_EP_HSHK              (0x01)
#define MCF_USB_ENDPT15_EP_STALL             (0x02)
#define MCF_USB_ENDPT15_EP_EN_TX             (0x04)
#define MCF_USB_ENDPT15_EP_EN_RX             (0x08)
#define MCF_USB_ENDPT15_EP_CTL_DIS           (0x10)
#define MCF_USB_ENDPT15_RETRY_DIS            (0x40)
#define MCF_USB_ENDPT15_HOST_WO_HUB          (0x80)

/* Bit definitions and macros for MCF_USB_USB_CTRL */
#define MCF_USB_USB_CTRL_SUSP                (1<<7)
#define MCF_USB_USB_CTRL_PDE                 (1<<6)
#define MCF_USB_USB_CTRL_CLKSRC_ALT          (0<<0)
//#define MCF_USB_USB_CTRL_CLKSRC_OSC          (1<<0)
#define MCF_USB_USB_CTRL_CLKSRC_SYS          (3<<0)


  
/*********************************************************************
*
* I2C Module (I2C)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_I2C_I2AR          (*(hcc_reg8 *)(&_IPSBAR[0x001838]))
#define MCF_I2C_I2FDR         (*(hcc_reg8 *)(&_IPSBAR[0x001839]))
#define MCF_I2C_I2CR          (*(hcc_reg8 *)(&_IPSBAR[0x00183A]))
#define MCF_I2C_I2SR          (*(hcc_reg8 *)(&_IPSBAR[0x00183B]))
#define MCF_I2C_I2DR          (*(hcc_reg8 *)(&_IPSBAR[0x00183C]))
#define MCF_I2C_I2CR2         (*(hcc_reg8 *)(&_IPSBAR[0x00183D]))

/* Bit definitions and macros for MCF_I2C_I2AR */
#define MCF_I2C_I2AR_ADR(x)   (((x)&0x7F)<<1)

/* Bit definitions and macros for MCF_I2C_I2FDR */
#define MCF_I2C_I2FDR_IC(x)   (((x)&0x3F)<<0)

/* Bit definitions and macros for MCF_I2C_I2CR */
#define MCF_I2C_I2CR_RSTA     (0x04)
#define MCF_I2C_I2CR_TXAK     (0x08)
#define MCF_I2C_I2CR_MTX      (0x10)
#define MCF_I2C_I2CR_MSTA     (0x20)
#define MCF_I2C_I2CR_IIEN     (0x40)
#define MCF_I2C_I2CR_IEN      (0x80)

/* Bit definitions and macros for MCF_I2C_I2SR */
#define MCF_I2C_I2SR_RXAK     (0x01)
#define MCF_I2C_I2SR_IIF      (0x02)
#define MCF_I2C_I2SR_SRW      (0x04)
#define MCF_I2C_I2SR_IAL      (0x10)
#define MCF_I2C_I2SR_IBB      (0x20)
#define MCF_I2C_I2SR_IAAS     (0x40)
#define MCF_I2C_I2SR_ICF      (0x80)

/* Bit definitions and macros for MCF_I2C_I2DR */
#define MCF_I2C_I2DR_DATA(x)  (((x)&0xFF)<<0)


/*---------- William ------------------------------------------*/
/* SCI1 */
#define MCF_SCI1BDH_REG		(*(hcc_reg8*)(&_IPSBAR[0x38]))
#define MCF_SCI1BDL_REG		(*(hcc_reg8*)(&_IPSBAR[0x39]))
#define MCF_SCI1C1_REG		(*(hcc_reg8*)(&_IPSBAR[0x3A]))
#define MCF_SCI1C2_REG		(*(hcc_reg8*)(&_IPSBAR[0x3B]))
#define MCF_SCI1S1_REG		(*(hcc_reg8*)(&_IPSBAR[0x3C]))
#define MCF_SCI1S2_REG		(*(hcc_reg8*)(&_IPSBAR[0x3D]))
#define MCF_SCI1C3_REG		(*(hcc_reg8*)(&_IPSBAR[0x3E]))
#define MCF_SCI1D_REG		  (*(hcc_reg8*)(&_IPSBAR[0x3F]))
#define MCF_SCI1BD_REG    (*(hcc_reg16*)(&_IPSBAR[0x38]))


/* SCI2 */
#define MCF_SCI2BDH_REG		(*(hcc_reg8*)(&_IPSBAR[0x40]))
#define MCF_SCI2BDL_REG		(*(hcc_reg8*)(&_IPSBAR[0x41]))
#define MCF_SCI2C1_REG		(*(hcc_reg8*)(&_IPSBAR[0x42]))
#define MCF_SCI2C2_REG		(*(hcc_reg8*)(&_IPSBAR[0x43]))
#define MCF_SCI2S1_REG		(*(hcc_reg8*)(&_IPSBAR[0x44]))
#define MCF_SCI2S2_REG		(*(hcc_reg8*)(&_IPSBAR[0x45]))
#define MCF_SCI2C3_REG		(*(hcc_reg8*)(&_IPSBAR[0x46]))
#define MCF_SCI2D_REG		  (*(hcc_reg8*)(&_IPSBAR[0x47]))
#define MCF_SCI2BD_REG    (*(hcc_reg16*)(&_IPSBAR[0x40]))


#define MCF_SCIBDH_REG(portno) (*(hcc_reg8*)(&_IPSBAR[0x38+(portno*8)]))
#define MCF_SCIBDL_REG(portno) (*(hcc_reg8*)(&_IPSBAR[0x39+(portno*8)]))
#define MCF_SCIC1_REG(portno)  (*(hcc_reg8*)(&_IPSBAR[0x3A+(portno*8)]))
#define MCF_SCIC2_REG(portno)  (*(hcc_reg8*)(&_IPSBAR[0x3B+(portno*8)]))
#define MCF_SCIS1_REG(portno)	 (*(hcc_reg8*)(&_IPSBAR[0x3C+(portno*8)]))
#define MCF_SCIS2_REG(portno)	 (*(hcc_reg8*)(&_IPSBAR[0x3D+(portno*8)]))	
#define MCF_SCIC3_REG(portno)  (*(hcc_reg8*)(&_IPSBAR[0x3E+(portno*8)]))
#define MCF_SCID_REG(portno)	 (*(hcc_reg8*)(&_IPSBAR[0x3F+(portno*8)]))	
#define MCF_SCIBD_REG(portno)  (*(hcc_reg16*)(&_IPSBAR[0x38+(portno*8)]))

#if  (SCI_NO==0)
#define MCF_SCIBDH              MCF_SCI1BDH_REG
#define MCF_SCIBDL              MCF_SCI1BDL_REG
#define MCF_SCIC1               MCF_SCI1C1_REG
#define MCF_SCIC2               MCF_SCI1C2_REG
#define MCF_SCIS1               MCF_SCI1S1_REG
#define MCF_SCIS2               MCF_SCI1S2_REG
#define MCF_SCID                MCF_SCI1D_REG
#define MCF_SCIBD               MCF_SCI1BD_REG
#else
#define MCF_SCIBDH              MCF_SCI2BDH_REG
#define MCF_SCIBDL              MCF_SCI2BDL_REG
#define MCF_SCIC1               MCF_SCI2C1_REG
#define MCF_SCIC2               MCF_SCI2C2_REG
#define MCF_SCIS1               MCF_SCI2S1_REG
#define MCF_SCIS2               MCF_SCI2S2_REG
#define MCF_SCID                MCF_SCI2D_REG
#define MCF_SCIBD               MCF_SCI2BD_REG
#endif

/* Bit definition for SCIBDH register */
#define MCF_SCIBDH_REG_LBKDIE_BIT   (1<<7)
#define MCF_SCIBDH_REG_RXEDGIE_BIT  (1<<6)

/* Bit definition for SCIC1 register */
#define MCF_SCIC1_REG_LOOPS_BIT     (1<<7)
#define MCF_SCIC1_REG_SCISWAI_BIT   (1<<6)
#define MCF_SCIC1_REG_RSRC_BIT      (1<<5)
#define MCF_SCIC1_REG_M_BIT         (1<<4)
#define MCF_SCIC1_REG_WAKE_BIT      (1<<3)
#define MCF_SCIC1_REG_ILT_BIT       (1<<2)
#define MCF_SCIC1_REG_PE_BIT        (1<<1)
#define MCF_SCIC1_REG_PT_BIT        (1<<0)

/* Bit definition for SCIC2 register */
#define MCF_SCIC2_REG_TIE_BIT       (1<<7)
#define MCF_SCIC2_REG_TCIE_BIT      (1<<6)
#define MCF_SCIC2_REG_RIE_BIT       (1<<5)
#define MCF_SCIC2_REG_ILIE_BIT      (1<<4)
#define MCF_SCIC2_REG_TE_BIT        (1<<3)
#define MCF_SCIC2_REG_RE_BIT        (1<<2)
#define MCF_SCIC2_REG_RWU_BIT       (1<<1)
#define MCF_SCIC2_REG_SBK_BIT       (1<<0)

/* Bit definition for SCIC3 register */
#define MCF_SCIC3_REG_R8_BIT        (1<<7)
#define MCF_SCIC3_REG_T8_BIT        (1<<6)
#define MCF_SCIC3_REG_TXDIR_BIT     (1<<5)
#define MCF_SCIC3_REG_TXINV_BIT     (1<<4)
#define MCF_SCIC3_REG_ORIE_BIT      (1<<3)
#define MCF_SCIC3_REG_NEIE_BIT      (1<<2)
#define MCF_SCIC3_REG_FEIE_BIT      (1<<1)
#define MCF_SCIC3_REG_PEIE_BIT      (1<<0)


/* Bit definition for SCIS1 register */
#define MCF_SCIS1_REG_TDRE_BIT      (1<<7)
#define MCF_SCIS1_REG_TC_BIT        (1<<6)
#define MCF_SCIS1_REG_RDRF_BIT      (1<<5)
#define MCF_SCIS1_REG_IDLE_BIT      (1<<4)
#define MCF_SCIS1_REG_OR_BIT        (1<<3)
#define MCF_SCIS1_REG_NF_BIT        (1<<2)
#define MCF_SCIS1_REG_FE_BIT        (1<<1)
#define MCF_SCIS1_REG_PE_BIT        (1<<0)


/* Bit definition for SCIS2 register */
#define MCF_SCIS2_REG_LBKDIF_BIT    (1<<7)
#define MCF_SCIS2_REG_RXEDGIF_BIT   (1<<6)
#define MCF_SCIS2_REG_RXINV_BIT     (1<<4)
#define MCF_SCIS2_REG_RWUID_BIT     (1<<3)
#define MCF_SCIS2_REG_BRK13_BIT     (1<<2)
#define MCF_SCIS2_REG_LBKDE_BIT     (1<<1)
#define MCF_SCIS2_REG_RAF_BIT       (1<<0)


/* MCG */
#define MCF_MCGC1_REG     (*(hcc_reg8*)(&_IPSBAR[0x48]))
#define MCF_MCGC2_REG     (*(hcc_reg8*)(&_IPSBAR[0x49]))
#define MCF_MCGTRM_REG     (*(hcc_reg8*)(&_IPSBAR[0x4A]))
#define MCF_MCGSC_REG     (*(hcc_reg8*)(&_IPSBAR[0x4B]))
#define MCF_MCGC3_REG     (*(hcc_reg8*)(&_IPSBAR[0x4C]))
#define MCF_MCGT_REG     (*(hcc_reg8*)(&_IPSBAR[0x4D]))

/* Bit defintions for MCGC1 register */
#define MCF_MCGC1_REG_CLKS_BIT    (3<<6)
#define MCF_MCGC1_REG_RDIV_BIT    (7<<3)
#define MCF_MCGC1_REG_IREFS_BIT   (1<<2)
#define MCF_MCGC1_REG_IRCLKEN_BIT (1<<1)
#define MCF_MCGC1_REG_IREFSTEN_BIT    (1)


/* Bit defintions for MCGC2 register */
#define MCF_MCGC2_REG_BDIV_BIT    (2<<6)
#define MCF_MCGC2_REG_RANGE_BIT   (1<<5)
#define MCF_MCGC2_REG_HGO_BIT     (1<<4)
#define MCF_MCGC2_REG_LP_BIT      (1<<3)
#define MCF_MCGC2_REG_EREFS_BIT   (1<<2)
#define MCF_MCGC2_REG_ERCLKEN_BIT (1<<1)
#define MCF_MCGC2_REG_EREFSTEN_BIT    (1)


/* Bit defintions for MCGC3 register */
#define MCF_MCGC3_REG_LOLIE_BIT   (1<<7)
#define MCF_MCGC3_REG_PLLS_BIT    (1<<6)
#define MCF_MCGC3_REG_CME_BIT     (1<<5)
#define MCF_MCGC3_REG_DIV32_BIT   (1<<4)
#define MCF_MCGC3_REG_VDIV_BIT    (1)


/* Bit defintions for MCGSC register */
#define MCF_MCGSC_REG_LOLS_BIT    (1<<7)
#define MCF_MCGSC_REG_LOCK_BIT    (1<<6)
#define MCF_MCGSC_REG_PLLST_BIT   (1<<5)
#define MCF_MCGSC_REG_IREFST_BIT  (1<<4)
#define MCF_MCGSC_REG_CLKST_BIT   (3<<2)
#define MCF_MCGSC_REG_OSCINIT_BIT (1<<1)
#define MCF_MCGSC_REG_FTRIM_BIT   (1)


/* Real Time Clock */
#define MCF_RTCSC_REG		  (*(hcc_reg8*)(&_IPSBAR[0x6C]))
#define MCF_RTCCNT_REG		(*(hcc_reg8*)(&_IPSBAR[0x6D]))
#define MCF_RTCMOD_REG		(*(hcc_reg8*)(&_IPSBAR[0x6E]))

/* Bit definitions for RTC */
#define MCF_RTCSC_REG_RTIE_BIT  (1<<4)
#define MCF_RTCSC_REG_RTIF_BIT  (1<<7)

/* Bit values for RTC */
#define MCF_RTC_SOURCE_LPO   0
#define MCF_RTC_SOURCE_EXT   1
#define MCF_RTC_SOURCE_INT   2


/* Timer/PWM */
#define MCF_TPM1SC_REG        (*(hcc_reg8*)(&_IPSBAR[0x20]))
#define MCF_TPM1CNTH_REG      (*(hcc_reg8*)(&_IPSBAR[0x21]))
#define MCF_TPM1CNTL_REG      (*(hcc_reg8*)(&_IPSBAR[0x22]))
#define MCF_TPM1MODH_REG      (*(hcc_reg8*)(&_IPSBAR[0x23]))
#define MCF_TPM1MODL_REG      (*(hcc_reg8*)(&_IPSBAR[0x24]))
#define MCF_TPM1C0SC_REG      (*(hcc_reg8*)(&_IPSBAR[0x25]))
#define MCF_TPM1C0VH_REG      (*(hcc_reg8*)(&_IPSBAR[0x26]))
#define MCF_TPM1C0VL_REG      (*(hcc_reg8*)(&_IPSBAR[0x27]))
#define MCF_TPM1C1SC_REG      (*(hcc_reg8*)(&_IPSBAR[0x28]))
#define MCF_TPM1C1VH_REG      (*(hcc_reg8*)(&_IPSBAR[0x29]))
#define MCF_TPM1C1VL_REG      (*(hcc_reg8*)(&_IPSBAR[0x2A]))
#define MCF_TPM1C2SC_REG      (*(hcc_reg8*)(&_IPSBAR[0x2B]))
#define MCF_TPM1C2VH_REG      (*(hcc_reg8*)(&_IPSBAR[0x2C]))
#define MCF_TPM1C2VL_REG      (*(hcc_reg8*)(&_IPSBAR[0x2D]))
#define MCF_TPM1C3SC_REG      (*(hcc_reg8*)(&_IPSBAR[0x2E]))
#define MCF_TPM1C3VH_REG      (*(hcc_reg8*)(&_IPSBAR[0x2F]))
#define MCF_TPM1C3VL_REG      (*(hcc_reg8*)(&_IPSBAR[0x30]))

/* Timer/PWM word type registers */
#define MCF_TPM1CNT_REG      (*(hcc_reg16*)(&_IPSBAR[0x21]))  /* this may be problem because it is not aligned to word boundary.*/
#define MCF_TPM1MOD_REG      (*(hcc_reg16*)(&_IPSBAR[0x23]))  /* same boundary issue as above */
#define MCF_TPM1C0V_REG      (*(hcc_reg16*)(&_IPSBAR[0x26]))
#define MCF_TPM1C1V_REG      (*(hcc_reg16*)(&_IPSBAR[0x29]))
#define MCF_TPM1C2V_REG      (*(hcc_reg16*)(&_IPSBAR[0x2C]))
#define MCF_TPM1C3V_REG      (*(hcc_reg16*)(&_IPSBAR[0x2F]))

#define MCF_TPMxSC_REG        MCF_TPM1SC_REG
#define MCF_TPMxCNT_REG       MCF_TPM1CNT_REG
#define MCF_TPMxMOD_REG       MCF_TPM1MOD_REG
#define MCF_TPMxMODH_REG      MCF_TPM1MODH_REG
#define MCF_TPMxMODL_REG      MCF_TPM1MODL_REG
#define MCF_TPMxCnSC_REG      MCF_TPM1C0SC_REG
#define MCF_TPMxCnV_REG       MCF_TPM1C0V_REG


/* Bit/Bit mask definitions for TPMxSC register */
#define MCF_TPMSC_REG_TOF_BIT   (1<<7)   
#define MCF_TPMSC_REG_TOIE_BIT  (1<<6)
#define MCF_TPMSC_REG_CPWMS_BIT (1<<5)
#define MCF_TPMSC_REG_CLKS_BIT  (3<<3)
#define MCF_TPMSC_REG_CLKS_BIT_POS  (3)
#define MCF_TPMSC_REG_PS_BIT    (7)

/* TPM Prescaler */
#define MCF_TPM_CLK_PRESCALOR_1   0
#define MCF_TPM_CLK_PRESCALOR_2   1
#define MCF_TPM_CLK_PRESCALOR_4   2
#define MCF_TPM_CLK_PRESCALOR_8   3
#define MCF_TPM_CLK_PRESCALOR_16  4
#define MCF_TPM_CLK_PRESCALOR_32  5
#define MCF_TPM_CLK_PRESCALOR_64  6
#define MCF_TPM_CLK_PRESCALOR_128 7


/* Bit/Bit mask definitions for TPMxCnSC register */
#define MCF_TPMCnSC_REG_CHnF_BIT   (1<<7)   
#define MCF_TPMCnSC_REG_CHnIE_BIT  (1<<6)   
#define MCF_TPMCnSC_REG_MSnBA_BIT_POS 4 
#define MCF_TPMCnSC_REG_MSnBA_BIT  (3<<MCF_TPMCnSC_REG_MSnBA_BIT_POS)
#define MCF_TPMCnSC_REG_ELSn_BIT_POS  2  
#define MCF_TPMCnSC_REG_ELSn_BIT   (2<<MCF_TPMCnSC_REG_ELSn_BIT_POS)   

/* Mode, edge and level selection */
#define MCF_TPM_MODE_INPUT_CAPTURE  0    /* CPWMS = 0 in TPMSC register */ 
#define MCF_TPM_MODE_OUTPUT_COMPARE (1 << MCF_TPMCnSC_REG_MSnBA_BIT_POS)    /* CPWMS = 0 in TPMSC register */
#define MCF_TPM_MODE_PWM_EDGE       (2 << MCF_TPMCnSC_REG_MSnBA_BIT_POS)    /* CPWMS = 0 in TPMSC register */
#define MCF_TPM_MODE_PWM_CENTER     MCF_TPMSC_REG_CPWMS_BIT    /* CPWMS = 1 in TPMSC register */

#define MCF_TPM_EDGE_RISING         (1 << MCF_TPMCnSC_REG_ELSn_BIT_POS)
#define MCF_TPM_EDGE_FALLING        (2 << MCF_TPMCnSC_REG_ELSn_BIT_POS)
#define MCF_TPM_EDGE_BOTH           (3 << MCF_TPMCnSC_REG_ELSn_BIT_POS)

#define MCF_TPM_OUTPUT_SW           0
#define MCF_TPM_OUTPUT_TOGGLE       (1 << MCF_TPMCnSC_REG_ELSn_BIT_POS)
#define MCF_TPM_OUTPUT_CLEAR        (2 << MCF_TPMCnSC_REG_ELSn_BIT_POS)
#define MCF_TPM_OUTPUT_SET          (3 << MCF_TPMCnSC_REG_ELSn_BIT_POS)

#define MCF_TPM_PWM_OUTPUT_HIGH       MCF_TPM_OUTPUT_CLEAR
#define MCF_TPM_PWM_OUTPUT_LOW        MCF_TPM_OUTPUT_SET

/* Input capture macros */
#define MCF_TPM_MODE_INPUT_CAPTURE_RISING_EDGE  ( MCF_TPM_MODE_INPUT_CAPTURE |  MCF_TPM_EDGE_RISING)
#define MCF_TPM_MODE_INPUT_CAPTURE_FALLING_EDGE  ( MCF_TPM_MODE_INPUT_CAPTURE |  MCF_TPM_EDGE_FALLING)
#define MCF_TPM_MODE_INPUT_CAPTURE_BOTH_EDGE  ( MCF_TPM_MODE_INPUT_CAPTURE |  MCF_TPM_EDGE_BOTH)

/* Output compare macros */
#define MCF_TPM_MODE_OUTPUT_COMPARE_SW        (MCF_TPM_MODE_OUTPUT_COMPARE |  MCF_TPM_OUTPUT_SW)
#define MCF_TPM_MODE_OUTPUT_COMPARE_TOGGLE    (MCF_TPM_MODE_OUTPUT_COMPARE |  MCF_TPM_OUTPUT_TOGGLE)
#define MCF_TPM_MODE_OUTPUT_COMPARE_CLEAR     (MCF_TPM_MODE_OUTPUT_COMPARE |  MCF_TPM_OUTPUT_CLEAR)
#define MCF_TPM_MODE_OUTPUT_COMPARE_SET       (MCF_TPM_MODE_OUTPUT_COMPARE |  MCF_TPM_OUTPUT_SET)

/* Clock source selection */
#define MCF_TPM_CLK_SRC_NO          0
#define MCF_TPM_CLK_SRC_BUS_CLK     (1<<MCF_TPMSC_REG_CLKS_BIT_POS)
#define MCF_TPM_CLK_SRC_FIX_CLK     (2<<MCF_TPMSC_REG_CLKS_BIT_POS)
#define MCF_TPM_CLK_SRC_EXT_CLK     (3<<MCF_TPMSC_REG_CLKS_BIT_POS)

/* Port D register definitions */
#define  MCF_PTDD_REG         (*(hcc_reg8*)(&_IPSBAR[0x6]))
#define  MCF_PTDDD_REG        (*(hcc_reg8*)(&_IPSBAR[0x7]))
#define   MCF_PTDPE_REG        (*(hcc_reg8*)(&_IPSBAR[0x184C]))
#define   MCF_PTDSE_REG        (*(hcc_reg8*)(&_IPSBAR[0x184D]))
#define   MCF_PTDDS_REG        (*(hcc_reg8*)(&_IPSBAR[0x184E]))
#define   MCF_PTDIFE_REG       (*(hcc_reg8*)(&_IPSBAR[0x184F]))


/* System Integration Module (Refer to Chapter 5 of Soc Guide) */
#define MCF_SIMOPT1_REG       (*(hcc_reg8*)(&_IPSBAR[0x1802]))
#define MCF_SIMOPT2_REG       (*(hcc_reg8*)(&_IPSBAR[0x1803]))

#define MCF_SIMOPT1_REG_COPT_BIT  (3<<6)


/* Flash */
#define MCF_FCDIV_REG       (*(hcc_reg8*)(&_IPSBAR[0x1820]))
#define MCF_FOPT_REG        (*(hcc_reg8*)(&_IPSBAR[0x1821]))
#define MCF_FRSV0_REG       (*(hcc_reg8*)(&_IPSBAR[0x1822]))
#define MCF_FCNFG_REG       (*(hcc_reg8*)(&_IPSBAR[0x1823]))
#define MCF_FPROT_REG       (*(hcc_reg8*)(&_IPSBAR[0x1824]))
#define MCF_FSTAT_REG       (*(hcc_reg8*)(&_IPSBAR[0x1825]))
#define MCF_FCMD_REG        (*(hcc_reg8*)(&_IPSBAR[0x1826]))
#define MCF_FRSV1_REG       (*(hcc_reg8*)(&_IPSBAR[0x1827]))
#define MCF_FADDRHI_REG     (*(hcc_reg8*)(&_IPSBAR[0x1828]))
#define MCF_FADDRLO_REG     (*(hcc_reg8*)(&_IPSBAR[0x1829]))
#define MCF_FRSV2_REG       (*(hcc_reg8*)(&_IPSBAR[0x182A]))
#define MCF_FRSV3_REG       (*(hcc_reg8*)(&_IPSBAR[0x182B]))
#define MCF_FDATAHI1_REG    (*(hcc_reg8*)(&_IPSBAR[0x182C]))
#define MCF_FDATALO1_REG       (*(hcc_reg8*)(&_IPSBAR[0x182D]))
#define MCF_FDATAHI0_REG       (*(hcc_reg8*)(&_IPSBAR[0x182E]))
#define MCF_FDATALO0_REG       (*(hcc_reg8*)(&_IPSBAR[0x182F]))




/* Interrupt No. */
#define VectorNumber_VRTC     VectorNumber_Vrtc //29
#define VectorNumber_TPM1Ovf    VectorNumber_Vtpm1ovf   
#define VectorNumber_TPM1CH0F    VectorNumber_Vtpm1ch0

#define VectorNumber_TICK1ms  VectorNumber_TPM1CH0F //VectorNumber_TPM1

#endif
/****************************** END OF FILE **********************************/
