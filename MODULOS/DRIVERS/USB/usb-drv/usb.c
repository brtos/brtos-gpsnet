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
#pragma warn_implicitconv off
 
#include "hcc_types.h"
#include "mcf51xx_reg.h"
#include "usb.h"
#include "BRTOS_USB.h"
#include "BRTOS.h"

BRTOS_Sem   *USB_Sem;


/*****************************************************************************
 * Local types.
 *****************************************************************************/
typedef struct {
  volatile hcc_u32 tlength;
  volatile hcc_u32 maxlength;
  void * volatile address;
  volatile usb_callback_t data_func;
  hcc_u16 psize;
  hcc_u32 data0_tx;
  hcc_u32 data0_rx;
  volatile hcc_u8 state;
  volatile hcc_u8 flags;
  volatile hcc_u8 error;
  hcc_u8 next_rx;
  hcc_u8 next_tx;
} ep_info_t;

/*****************************************************************************
 * Macro definitions.
 *****************************************************************************/
#define DIR_TX             BIT7

#define SOF_PID_VALUE		0xd
/* Note: BIT3 shows if the data packet is data0 or data1. */
#define DATA_PID_VALUE      0x3
#define DATA0_PID_VALUE     0x3
#define DATA1_PID_VALUE     0xb

#define MCF_USB_ENDPT(ep)  (((hcc_u8*)&MCF_USB_ENDPT0)[ep<<2])
#define MCF_USB_ENDPT_EP_HSHK      BIT0
#define MCF_USB_ENDPT_EP_STALL     BIT1
#define MCF_USB_ENDPT_EP_TX_EN     BIT2
#define MCF_USB_ENDPT_EP_RX_EN     BIT3
#define MCF_USB_ENDPT_EP_CTL_DIS   BIT4
#define MCF_USB_ENDPT_RETRY_DIS    BIT6
#define MCF_USB_ENDPT_HOST_WO_HUB  BIT7

#define MIN(a,b)     ((a) < (b) ? (a) : (b))

/* Control endpoint state machine state values. */
#define EPST_IDLE             0x0
#define EPST_DATA_TX          0x1
#define EPST_DATA_TX_LAST     0x2
#define EPST_DATA_RX          0x3
#define EPST_STATUS_TX        0x4
#define EPST_STATUS_RX        0x5
#define EPST_TX_STOP          0x6
#define EPST_ABORT_TX         0x7
#define EPST_DATA_TX_WAIT_DB  0x8
#define EPST_DATA_TX_EMPTY_DB 0x9

/* Standard USB feature selector values. */
#define FEAT_ENDPOINT_HALT        0u
#define FEAT_DEVICE_REMOTE_WAKEUP 1u

/* Endpoint flag bits. */
#define EPFL_ERROR    (hcc_u8)BIT0  /* There was an error during the ongoing
                               transfer. */
#define EPFL_ZPACKET  (hcc_u8)BIT1  /* After the last data packet an additional zero
                               length packet needs to be transmitted to close
                               the transfer. */

/* This macro shall evaluate to a uint32 pointer to the start address of the
   buffer descriptor table (BDT). The BDT has 32 bytes for each endpoint.
   The BDT shall be alignet to 512 byte boundary! */
extern hcc_u32 _BDT_BASE[];
#define BDT_BASE              ((hcc_u32*)(_BDT_BASE))
#define BDT_CTL_RX(ep, b)     (BDT_BASE[((ep)<<3)+((b)<<1)+0])
#define BDT_ADR_RX(ep, b)     (BDT_BASE[((ep)<<3)+((b)<<1)+1])
#define BDT_CTL_TX(ep, b)     (BDT_BASE[((ep)<<3)+((b)<<1)+4])
#define BDT_ADR_TX(ep, b)     (BDT_BASE[((ep)<<3)+((b)<<1)+5])

#define BDT_CTL_STALL BIT2
#define BDT_CTL_DTS   BIT3
#define BDT_CTL_DATA  BIT6
#define BDT_CTL_OWN   BIT7

/*****************************************************************************
 * Global variables.
 *****************************************************************************/
/* N/A */

/*****************************************************************************
 * Module variables.
 *****************************************************************************/
static volatile hcc_u8 usb_current_config;
static volatile hcc_u8 usb_state;
static volatile hcc_u8 new_address;
static ep_info_t ep_info[16];

/*****************************************************************************
 * Function predefinitions.
 *****************************************************************************/
static void enter_default_state(void);
static void disable_ep_rx(hcc_u8);
static void disable_ep_tx(hcc_u8);
static void ready_ep_rx(hcc_u8, hcc_u8);
static void ready_ep_tx(hcc_u8, hcc_u8);
static hcc_u8 select_tx_buf(hcc_u8 ep);
static void send_zero_packet(hcc_u8 ep);
static void _usb_send(hcc_u8 ep);
static void _usb_receive(hcc_u8 ep);
static callback_state_t cb_set_address(void);
static callback_state_t usb_stm_ctrl0(void);
static hcc_u8 usb_setup_ep(hcc_u8 addr, hcc_u8 type, hcc_u8 ep, hcc_u16 psize);

extern void enable_usb_pull_up();
/*****************************************************************************
 * Name:
 *   usb_get_rx_pptr
 * In:
 *   ep: number of endpoint.
 * Out:
 *   Pointer to the packet buffer containing data of the last received packet.
 *
 * Description:
 *   Returns te number of bytes that are left of the transfer. If
 *   usb_ep_is_busy returns false, then the transfer was aborted either
 *   by the host or by the application.
 *****************************************************************************/
hcc_u8* usb_get_rx_pptr(hcc_u8 ep)
{
  hcc_u8 cur_buf=(hcc_u8)(MCF_USB_STAT & MCF_USB_STAT_ODD ? 1 : 0);
  return((hcc_u8 *)(RD_LE32(&BDT_ADR_RX(ep, cur_buf))));
}

/*****************************************************************************
 * Name:
 *   select_tx_buf
 * In:
 *   ep: number of endpoint.
 * Out:
 *   The index of the next tx buffer.
 *
 * Description:
 *   Returns te number of bytes that are left of the transfer. If
 *   usb_ep_is_busy returns false, then the transfer was aborted either
 *   by the host or by the application.
 *****************************************************************************/
static hcc_u8 select_tx_buf(hcc_u8 ep)
{
/* SHALL ONLY BE CALLED WITH USB INTERRUPTS DISABLED. */
  hcc_u8 buf;
  hcc_u32 ctl;
  /* Find out which buffer shall be used. */
  ctl=(RD_LE32(&BDT_CTL_TX(ep, 0)) & BDT_CTL_OWN)
      | (RD_LE32(&BDT_CTL_TX(ep, 1)) & BDT_CTL_OWN) << 1;

  switch(ctl)
  {
  case 0:  /* No buffer is used by the USB module. Fill the one
           we think is the next. */
    buf=ep_info[ep].next_tx;
    ep_info[ep].next_tx ^= 0x1;
    break;
  case BDT_CTL_OWN: /* Buffer 0 is used by the USB. */
    buf=1;
    ep_info[ep].next_tx=0;
    break;
  case BDT_CTL_OWN<<1: /* Buffer 1 is used by the USB. */
    buf=0;
    ep_info[ep].next_tx=1;
    break;
  default: /* Both buffers are used by the usb. This is
              an error. */
    CMX_ASSERT(0);
  }
  return(buf);
}

/*****************************************************************************
 * Name:
 *   usb_get_done
 * In:
 *   ep: number of endpoint.
 * Out:
 *   The number of bytes the endpoint tansferred.
 *
 * Description:
 *   Returns te number of bytes that were transferred.
 *****************************************************************************/
hcc_u32 usb_get_done(hcc_u8 ep)
{
  return(ep_info[ep].maxlength-ep_info[ep].tlength);
}

/*****************************************************************************
 * Name:
 *    usb_ep_is_busy
 * In:
 *   ep: number of endpoint.
 * Out:
 *   nonzero if endpoint is buys (a transfer is ongoing).
 *
 * Description:
 *
 *****************************************************************************/
hcc_u8 usb_ep_is_busy(hcc_u8 ep)
{
  return(ep_info[ep].state != EPST_IDLE ? (hcc_u8)1 : (hcc_u8)0);
}

/*****************************************************************************
 * Name:
 *    usb_get_state
 * In:
 *   N/A
 * Out:
 *   Current USB state. See USBST_xxx in usb.h
 *
 * Description:
 *
 *****************************************************************************/
hcc_u8 usb_get_state(void)
{
  return(usb_state);
}

/*****************************************************************************
 * Name:
 *    usb_ep_error
 * In:
 *   ep: number fo endpoint
 * Out:
 *   Endpoint specific error code. (See USBEPERR_xx macro definitions in usb.h)
 *
 * Description:
 *
 *****************************************************************************/
hcc_u8 usb_ep_error(hcc_u8 ep)
{
  hcc_u8 tmp=ep_info[ep].error;
  ep_info[ep].error=USBEPERR_NONE;
  return(tmp);
}

/*****************************************************************************
 * Name:
 *    send_zero_packet
 * In:
 *    N/A
 * Out:
 *    N/A
 *
 * Description:
 *    Will send a zero length data packet.
 *
 * Assumptions:
 *    ep is the index of a TX endpoint.
 *****************************************************************************/
static void send_zero_packet(hcc_u8 ep)
{
  hcc_u8 buf;

  buf=select_tx_buf(ep);
  /* Write packet length. */
  WR_LE32(&BDT_CTL_TX(ep, buf), 0);

  /* Make buffer ready for transmission. */
  ready_ep_tx(ep, buf);
}

/*****************************************************************************
 * Name:
 *    usb_init
 * In:
 *   None
 * Out:
 *   0  - if all ok
 *   !0 - if failed
 *
 * Description:
 *   Initialises the usb driver. Will set the interrupt level.
 *   Note: clock source is the system clock (48MHz).
 *****************************************************************************/
hcc_u8 usb_init(void)
{
  /* Reset USB module first. */
  USBTRC0_USBRESET = 1;
  while (USBTRC0_USBRESET ){};

  /* exit suspend and select USB clock source
   * If you program CLK_SRC bits as 2¡¯b11 in the USB_CTRL register then MCGPLLSCK is connected to the USB 48 Mhz clock. 
   * If you program CLK_SRC bits to 2¡¯b00 then the usb_alt_clk input pin to the chip is used as the USB 48 Mhz clock . This is the PTG0 pad. (Pin number 32 for 80LQFP) 
   * Plz ignore programmation 2¡¯b01,2¡¯b10 for CLK_SRC bit. They are reserved. 
   */
  MCF_USB_USB_CTRL = MCF_USB_USB_CTRL_CLKSRC_SYS;

  /* Disable all USB interrupts. */
  MCF_USB_INT_ENB = 0x0;
  /* Disable all OTG interupts. */
  MCF_USB_OTG_INT_EN = 0x0;
  /* Disable all USB error interrupts. */
  MCF_USB_ERR_ENB = 0x0;

  /* Put USB to default state. */
  enter_default_state();

  /* Clear any pending OTG interupts. */
  MCF_USB_OTG_INT_STAT = 0xff;
  /* Clear any pending USB interrupts.*/
  MCF_USB_INT_STAT = 0xff;
  /* Clear any pending USB error interrupt. */
  MCF_USB_ERR_STAT = 0xff;

  /* Enable USB interrupts. */
  /*MCF_USB_INT_ENB = MCF_USB_INT_ENB_SLEEP | MCF_USB_INT_ENB_TOK_DNE
                    | MCF_USB_INT_ENB_ERROR | MCF_USB_INT_ENB_USB_RST
                    | MCF_USB_INT_ENB_STALL;*/
  MCF_USB_INT_ENB = MCF_USB_INT_ENB_TOK_DNE | MCF_USB_INT_ENB_STALL
                    | MCF_USB_INT_ENB_ERROR | MCF_USB_INT_ENB_USB_RST;

  /* Set BDT address. */
  MCF_USB_BDT_PAGE_01 = (hcc_u8)(((hcc_u32)BDT_BASE) >> 8);
  MCF_USB_BDT_PAGE_02 = (hcc_u8)(((hcc_u32)BDT_BASE) >> 16);
  MCF_USB_BDT_PAGE_03 = (hcc_u8)(((hcc_u32)BDT_BASE) >> 24);
  
   
    /* Enable USB PHY before enabling USB module and wait from interrupt.
   */
  //USBTRC0 =  USBTRC0_USBVREN_MASK;
  
  enable_usb_pull_up();
 
   /* Enable USB functionality*/
  MCF_USB_CTL = MCF_USB_CTL_USB_EN_SOF_EN;

  return(0);
}


/*****************************************************************************
 * Name:
 *    usb_stop
 * In:
 *   n/a
 * Out:
 *   n/a
 *
 * Description:
 *   Stops USB driver.
 *****************************************************************************/
void usb_stop(void)
{
  /* Disable all USB interrupts. */
  MCF_USB_INT_ENB = 0x0;
  /* Disable all OTG interupts. */
  MCF_USB_OTG_INT_EN = 0x0;
  /* Disable all USB error interrupts. */
  MCF_USB_ERR_ENB = 0x0;

  /* Clear any pending OTG interupts. */
  MCF_USB_OTG_INT_STAT = 0xff;
  /* Clear any pending USB interrupts.*/
  MCF_USB_INT_STAT = 0xff;
  /* Clear any pending USB error interrupt. */
  MCF_USB_ERR_STAT = 0xff;

  MCF_USB_USB_CTRL=0;
  /* Disable USB module. */
  MCF_USB_CTL = 0;

  enter_default_state();
  usb_setup_ep(0, EP_TYPE_DISABLE, 0, 0);
  usb_state=USBST_DISABLED;
}

/*****************************************************************************
 * Name:
 *    usb_setup_ep
 * In:
 *    addr  - endpoint address
 *    type  - endpoint type (control, bulk, interrupt, iso). This is the value
 *            of the endpoint type filed of the endpoint descriptor.
 *    ep    - number of endpoint
 *    psize - maximum packet size allowed for this endpoint.
 *    db    - nonzer of endpoint shall be double buffered. Note: only iso and
 *            bulk endpoints can be double buffered (this is hardware
 *            specific).
 * Out:
 *    0  - all ok
 *    !0 - initialisation failed
 *
 * Description:
 *    Configures the spcified endpoint.
 *****************************************************************************/
static hcc_u8 usb_setup_ep(hcc_u8 addr, hcc_u8 type, hcc_u8 ep, hcc_u16 psize)
{
  hcc_u8 endpt;
   /* Disable endpoint. */
  disable_ep_tx(ep);
  disable_ep_rx(ep);

  if (ep_info[ep].state!=EPST_IDLE)
  {
    ep_info[ep].error=USBEPERR_HOST_ABORT;
  }
  else
  {
    ep_info[ep].error=USBEPERR_NONE;
  }

  ep_info[ep].state=EPST_IDLE;
  ep_info[ep].flags=0;
  ep_info[ep].data_func=(void*)0;
  ep_info[ep].address=0;
  ep_info[ep].tlength=0;
  ep_info[ep].maxlength=0;
  ep_info[ep].psize=psize;
  ep_info[ep].data0_tx=0;
  ep_info[ep].data0_rx=0;
  if (type == EP_TYPE_DISABLE)
  {
    return(0);
  }

  endpt =0;

  /* if RX side of the endpoint needs to be configured */
  if (!(addr & DIR_TX) || type == EP_TYPE_CONTROL)
  {
    endpt |=MCF_USB_ENDPT_EP_RX_EN;

    /* Set BDT_CTL to default value. Set packet size, buffer is owned by the CPU,
       enable data toggle synchronisation, expect data0 packet. Note: data0/1
       will be set when the buffer is given to the USB. */
	WR_LE32(&BDT_CTL_RX(ep, 0), (hcc_u32)((psize << 16) | BDT_CTL_DTS));
    /* Set BDT_CTL to default value. */
    WR_LE32(&BDT_CTL_RX(ep, 1), (hcc_u32)((psize << 16) | BDT_CTL_DTS));

    /* Set RX buffer address. */
    WR_LE32(&BDT_ADR_RX(ep, 0), (hcc_u32)get_ep_rx_buffer(ep, 0));
    WR_LE32(&BDT_ADR_RX(ep, 1), (hcc_u32)get_ep_rx_buffer(ep, 1));
  }
  if ((addr & DIR_TX) || type == EP_TYPE_CONTROL)
  {
    endpt |= MCF_USB_ENDPT_EP_TX_EN;
    /* Set BDT_CTL to default value. Set buffer owned by the CPU. Value of other
       fileds is not important. Those will get a correct value when the buffer is
       made ready for transmission. */
    WR_LE32(&BDT_CTL_TX(ep, 0), 0);
    /* Set BDT_CTL to default value. */
	WR_LE32(&BDT_CTL_TX(ep, 1), 0);
    /* Set TX buffer address. */
  }

  if (type != EP_TYPE_ISO)
  { /* Non ISO endpoints need handhaking. */
    endpt |= MCF_USB_ENDPT_EP_HSHK;
  }
  else
  { /* ISO endpoints do not need handshaking, and retry. */
    endpt |= MCF_USB_ENDPT_RETRY_DIS;
  }

  /* If a non control endpoint uses both directions (e.g. two endpoint is
    implemented in the same hardware slot), then CTL_DIS shall be set. */
  if (type != EP_TYPE_CONTROL)
  {
    endpt |= MCF_USB_ENDPT_EP_CTL_DIS;
  }
  else
  { /* Reception on control endpoints shall be enabled by default. */
    /* Set maximum packet size. */
    WR_LE32(&BDT_CTL_RX(ep, ep_info[ep].next_rx), (hcc_u32)ep_info[ep].psize<<16u);
    ready_ep_rx(ep, ep_info[ep].next_rx);
  }

  MCF_USB_ENDPT(ep)=endpt;

  return(0);
}

/*****************************************************************************
 * Name:
 *    set_config
 * In:
 *    cfg_ndx - index of the configuration to be activated. The value shall
 *              shall equal to one defined in a configuration descriptor.
 * Out:
 *    N/A
 *
 * Description:
 *    Configures the USB module according to the specifyed configuration.
 * Assumptions:
 *    the spefified configuration exists.
 *    the first interface descriptor is for the default alternate setting.
 *    interfaces must be numbered from 0 increasing continously (0,1,2,3...)
 *    configurations must be numbered from 0 increasing continously
 *****************************************************************************/
static void set_config(hcc_u8 cfg_ndx)
{
  hcc_u8 cfg_ep=0;

  usb_current_config=cfg_ndx;

  /* All endpoint shall use buffer 0 now. */
  if (cfg_ndx != 0)
  {
    /* For all interfaces in this configuration. */
    hcc_u8 ifc=0;
    while(is_ifc_ndx(cfg_ndx, ifc, 0))
    { /* Emdpoint descriptor index in configuration descriptor. */
      hcc_u8 ifc_ep=0;
      while(is_ep_ndx(cfg_ndx, ifc, 0, ifc_ep))
      {
        const hcc_u8 *epd=get_ep_descriptor(cfg_ndx, ifc, 0, ifc_ep);
        /* Endpoint index and address is the same. */
        usb_setup_ep(epd[2], epd[3], (hcc_u8)(epd[2] & 0x7fu), RD_LE16(&epd[4]));
        ifc_ep++;
        cfg_ep++;
      }
      ifc++;
    }
    usb_state = USBST_CONFIGURED;
  }
  else
  {
    usb_state=USBST_ADDRESSED;
    /* No endpoints to configure. The loop below will disable all except 0. */
  }

  cfg_ep++;

  while(cfg_ep < 16)
  {
    usb_setup_ep(0, EP_TYPE_DISABLE, cfg_ep++, 0);
  }
}

/*****************************************************************************
 * Name:
 *    enter_default_state
 * In:
 *    N/A
 * Out:
 *    N/A
 *
 * Description:
 *    Configure USB driver to reflect the default state. In this state only
 *    standard requests on the default pipe are answered, all other endpoints
 *    are disabled.
 *****************************************************************************/
static void enter_default_state(void)
{
  int ep;
  /* In this state the USB module responds to the default address.
     Only EP0 is configured. */

  MCF_USB_CTL |= MCF_USB_CTL_ODD_RST;

  for(ep=0; ep<sizeof(ep_info)/sizeof(ep_info[0]); ep++)
  {
    ep_info[ep].next_rx=0;
    ep_info[ep].next_tx=0;
  }
  MCF_USB_CTL &= ~MCF_USB_CTL_ODD_RST;

  /* Configure ep 0. */
  usb_setup_ep(0, EP_TYPE_CONTROL, 0, EP0_PACKET_SIZE);
  /* Disable all other endpoints. */
  set_config(0);

  /* Set address to default address. */
  MCF_USB_ADDR = 0;

  usb_state=USBST_DEFAULT;
  new_address=0;
}


/*****************************************************************************
 * Name:
 *    usb_stop_ep_tx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Will stall a tx endpoint (endpoint will not transmit any more packages,
 *    all IN request from the host will be denyed with error handsake).
 *****************************************************************************/
void usb_stop_ep_tx(hcc_u8 ep)
{
  OS_SR_SAVE_VAR
  /* This cal needs to be protected againt USB interrupts, to
     make BDT_CTL assecc atomic. */
  _irq_disable();

  /* To enforce stall handshake, we stop both buffers. */
  WR_LE32(&BDT_CTL_TX(ep, 0), BDT_CTL_OWN | BDT_CTL_STALL);
  WR_LE32(&BDT_CTL_TX(ep, 1), BDT_CTL_OWN | BDT_CTL_STALL);

  _irq_restore();
}

/*****************************************************************************
 * Name:
 *    usb_stop_ep_rx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Will stall a rx endpoint (endpoint will not treceive any more packages,
 *    all OUT request from the host will be denyed with error handsake).
 *****************************************************************************/
void usb_stop_ep_rx(hcc_u8 ep)
{
  OS_SR_SAVE_VAR
  /* This cal needs to be protected againt USB interrupts, to
     make BDT_CTL assecc atomic. */
  _irq_disable();

  /* To enforce stall handshake, we stop both buffers. */
  WR_LE32(&BDT_CTL_RX(ep, 0), BDT_CTL_OWN | BDT_CTL_STALL);
  WR_LE32(&BDT_CTL_RX(ep, 1), BDT_CTL_OWN | BDT_CTL_STALL);

  _irq_restore();
}

/*****************************************************************************
 * Name:
 *    ready_ep_tx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Make tx endpoint ready for transmission.
 *****************************************************************************/
static void ready_ep_tx(hcc_u8 ep, hcc_u8 buf)
{ /* Simply give buffer to USB. */
  hcc_u32 ctl;

  /* Give buffer to USB, set correct data0/1 flag, configure if data toggle
     synchronisation shall be used or not. */
  ctl=RD_LE32(&BDT_CTL_TX(ep, buf)) & ~0xff;
  ctl |= BDT_CTL_OWN | ep_info[ep].data0_tx | BDT_CTL_DTS;
  WR_LE32(&BDT_CTL_TX(ep, buf), ctl);
}

/*****************************************************************************
 * Name:
 *    ready_ep_rx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Make rx endpoint ready for reception.
 *****************************************************************************/
static void ready_ep_rx(hcc_u8 ep, hcc_u8 buf)
{ /* Simply give buffer to USB. */
  hcc_u32 ctl;

  /* Give buffer to USB, set correct data0/1 flag, configure if data toggle
     synchronisation shall be used or not. */
  ctl = (ep_info[ep].psize << 16) | BDT_CTL_OWN | ep_info[ep].data0_rx | BDT_CTL_DTS;

  ep_info[ep].data0_rx ^= BDT_CTL_DATA;

  WR_LE32(&BDT_CTL_RX(ep, buf), ctl);
}

/*****************************************************************************
 * Name:
 *    disable_ep_tx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Disable TX endpoint. Endpoint behaves as it would not exist (it will not
 *    affect the USB and will not generate any events).
 *****************************************************************************/
static void disable_ep_tx(hcc_u8 ep)
{
  MCF_USB_ENDPT(ep) &= ~MCF_USB_ENDPT_EP_TX_EN;
  while(MCF_USB_ENDPT(ep) & MCF_USB_ENDPT_EP_TX_EN)
    ;
  WR_LE32(&BDT_CTL_TX(ep, 0), 0);
  WR_LE32(&BDT_CTL_TX(ep, 1), 0);
}


/*****************************************************************************
 * Name:
 *    disable_ep_rx
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    Disable RX endpoint. Endpoint behaves as it would not exist (it will not
 *    affect the USB and will not generate any events).
 *****************************************************************************/
static void disable_ep_rx(hcc_u8 ep)
{
  MCF_USB_ENDPT(ep) &= ~MCF_USB_ENDPT_EP_RX_EN;
}

/*****************************************************************************
 * Name:
 *    _usb_send
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    This fucntion inmplements the basic state machine for transmit (IN)
 *    endpoints. It will
 *        - call user callback functions if neccessary,
 *        - set endpoint specific error codes
 *        - split data to be sent to packet sized pieces
 *    Note: it is called from the interrupt handler routine and from "user
 *          space" too. The function needs to be reentrant!
 *****************************************************************************/
static void _usb_send(hcc_u8 ep)
{
  hcc_u32 l;
  hcc_u8 buf;
  hcc_u32 length;

  switch (ep_info[ep].state)
  {
  case EPST_DATA_TX:
    l=MIN(ep_info[ep].tlength, ep_info[ep].psize);
    CMX_ASSERT(l == ep_info[ep].psize || l==ep_info[ep].tlength);      

    /* Select next ep buffer. */
    buf=select_tx_buf(ep);
    /* Set-up UBS module to directly send from applications buffer. */
    WR_LE32(&BDT_ADR_TX(ep, buf), (hcc_u32)ep_info[ep].address);

    /* Set tx packet length. */
    length=MIN(ep_info[ep].psize, ep_info[ep].tlength);
    WR_LE32(&BDT_CTL_TX(ep, buf), (hcc_u32)(length<<16u));

    /* Make buffer ready for transmission. */
    ready_ep_tx(ep, buf);
    /* Invert the data toggle bit. */
    ep_info[ep].data0_tx ^= BDT_CTL_DATA;

    /* Calculate transfer status. */
    ep_info[ep].tlength -= l;
    ep_info[ep].address = (hcc_u8*)ep_info[ep].address + l;

    /* Is the just created packet the last one? */
    if (ep_info[ep].tlength == 0)
    { /* Do we need to send a zero length packet to terminate the transmission? */
      if ((l == ep_info[ep].psize) && (ep_info[ep].flags & EPFL_ZPACKET))
      {
        ep_info[ep].state=EPST_TX_STOP;
      }
      else
      {
        ep_info[ep].state=EPST_DATA_TX_LAST;
      }
      /* Warning: when getting there, an additional interrupt is needed to advance the
         state succesfully. */
      break;
    }
    break;
  case EPST_DATA_TX_WAIT_DB:
    /* Let second buffer get empty while waiting for a new buffer. */
    ep_info[ep].state=EPST_DATA_TX_EMPTY_DB;
    break;
  case EPST_TX_STOP:
    /* Send a sort packet. */
    ep_info[ep].state=EPST_DATA_TX_LAST;
    send_zero_packet(ep);
    break;
  case EPST_DATA_TX_LAST:
    /* In this state we may have one or two packets in the packet buffers.
       We shall wait till both packets are sent, and both buffers are empty. */

    /* Check if both buffers are empty. */
    if ((RD_LE32(&BDT_CTL_TX(ep, 0)) & BDT_CTL_OWN) == 0
       && (RD_LE32(&BDT_CTL_TX(ep, 1)) & BDT_CTL_OWN) == 0)
    {
      /* Enter IDLE state. */
      ep_info[ep].state=EPST_IDLE;
    }
    break;
  default:
    CMX_ASSERT(0);

  }
}

/*****************************************************************************
 * Name:
 *    usb_send
 * In:
 *    ep          - endpoint number
 *    f           - pointer to user callback function. A callback will bemade
 *                  if:  - the buffer is empty and more data needs to be sent
 *                       - all transmission is finished
 *                       - in case of an error
 *    data        - pointer to data buffer
 *    buffer_size - size of data buffer
 *    tr_length   - number of bytes to be transferred.
 *    req_length  - the number of bytes the host wants to receive.
 *
 *    Note: since all packes transmission on USB are started by the host, it
 *          needs to know how many bytes shall be transferred during a
 *          transfer. Because of this the host will always tell the device
 *          how many bytes it can receive (req_length). On the other hand, the
 *          the device may have less data ready (tr_length).
 * Out:
 *    N/A
 * Description:
 *    Using this function an TX (IN) transfer can be enabled.
 *****************************************************************************/
void usb_send(hcc_u8 ep, usb_callback_t f, hcc_u8* data, hcc_u32 tr_length, hcc_u32 req_length)
{
  OS_SR_SAVE_VAR
  ep_info[ep].tlength=tr_length <= req_length ? tr_length : req_length;
  ep_info[ep].maxlength=ep_info[ep].tlength;
  ep_info[ep].data_func=f;
  ep_info[ep].address=data;
  ep_info[ep].flags=req_length > tr_length ?   (hcc_u8)EPFL_ZPACKET : (hcc_u8)0;
  ep_info[ep].error = USBEPERR_NONE;
  ep_info[ep].state=EPST_DATA_TX;

  _irq_disable();
  _usb_send(ep);
  _irq_restore();
}

/*****************************************************************************
 * Name:
 *    _usb_receive
 * In:
 *    ep - endpoint number
 * Out:
 *    N/A
 * Description:
 *    This fucntion inmplements the basic state machine for receive (OUT)
 *    endpoints. It will
 *        - call user callback functions if neccessary,
 *        - set endpoint specific error codes
 *        - reassemble packets into the specified buffer
 *    Note: it is called from the interrupt handler routine and from "user
 *          space" too. The function needs to be reentrant!
 *****************************************************************************/
void _usb_receive(hcc_u8 ep)
{
  hcc_u8 loops=0;
  hcc_u8* buf;
  int x;

  /* Copy data to USER space, or call callback if no buffer is available. */
  do
  {
    hcc_u8 this_buf=(hcc_u8)((MCF_USB_STAT & MCF_USB_STAT_ODD) ? 1 : 0);
    hcc_u16 plength=(hcc_u16)(RD_LE32(&BDT_CTL_RX(ep, this_buf)) >> 16);

    /* Check if amount of received data is ok. */
    if (ep_info[ep].tlength < plength)
    { /* Host sent too many data! This is a protocol error. */
      usb_stop_ep_rx(ep);
      ep_info[ep].error = USBEPERR_TO_MANY_DATA;
      return;
    }

    /* Copy data to the user buffer. */

    /* Copy packet data to destination address. */
    /* Get next RX buffer address. */
    buf=(hcc_u8*)RD_LE32(&BDT_ADR_RX(ep, (MCF_USB_STAT & MCF_USB_STAT_ODD) ? 1 : 0));

    for(x=0; x < plength; x++)
    {
      ((hcc_u8*)(ep_info[ep].address))[x]=buf[x];
    }

    ep_info[ep].tlength -= plength;
    ep_info[ep].address = ((hcc_u8 *)ep_info[ep].address) + plength;

    /* Was this the last data packet? */
    if ((ep_info[ep].tlength == 0) || (ep_info[ep].psize != plength))
    {
      /* Uuuupps: at this point there sall be no RX buffer enabled. But
         if the hosts terminates the transfer early (by sending a short
         packet) then one buffer may remain enabled. So we may have an
         unexpected interrupt and an unexpected packet.
         We try to disable the buffer, but this is not safe. There is a
         race. If the host already sent additional data to this endpoint,
         then the first transfer will be dropped as a spurious packet.
         Note: control endpoints are not affected because befor enabling RX
         these don't need to know how much data will be transfered (setup
         transfer will tell this).
         Note1: we don't know if "aborting" the reception was succesfull or
         not. */
      WR_LE32(&BDT_CTL_RX(ep, ep_info[ep].next_rx ? 0 : 1), 0);
      ep_info[ep].state=EPST_IDLE;
      /* Control endpoints will execute the callback after the status stage. */
      /* Check if control endpoint. (Note: both ENDPT registers will have
         the same content, so we check reg0. )*/
      if (! MCF_USB_ENDPT(ep) & MCF_USB_ENDPT_EP_CTL_DIS)
      {
        /* Tell application transfer ended. */
        if (ep_info[ep].data_func != (void *) 0)
        {
          /* We don't care about the return valus since the transfer is already
             finished and we can do nothing in case of an error. */
          (void)(*ep_info[ep].data_func)();
        }
        /* Disable callbacks. */
        ep_info[ep].data_func = (void *)0;
      }
    }
    else
    {
	  /* Make buffer ready for reception if two buffers can be filled by the
	     USB module. */
      if (ep_info[ep].tlength > ep_info[ep].psize)
      {
        ready_ep_rx(ep, this_buf);
      }
    }
  }while(loops);
}

/*****************************************************************************
 * Name:
 *    usb_receive
 * In:
 *    ep          - endpoint number
 *    f           - pointer to user callback function. A callback will bemade
 *                  if:  - the buffer is empty and more data needs to be sent
 *                       - all transmission is finished
 *                       - in case of an error
 *    data        - pointer to data buffer
 *    buffer_size - size of data buffer
 *    tr_length   - number of bytes to be transferred. (This shal be the same
 *                  amount that the host wants to send).
 *
 * Out:
 *    N/A
 * Description:
 *    Using this function an RX (OUT) transfer can be enabled.
 *****************************************************************************/
void usb_receive(hcc_u8 ep, usb_callback_t f, hcc_u8* data, hcc_u32 tr_length)
{
  ep_info[ep].tlength=tr_length;
  ep_info[ep].maxlength=ep_info[ep].tlength;
  ep_info[ep].data_func=f;
  ep_info[ep].address=data;
  ep_info[ep].state=EPST_DATA_RX;
  ep_info[ep].error=USBEPERR_NONE;
  ep_info[ep].flags=0;

  /* Enable next rx buffer. */
  ready_ep_rx(ep, ep_info[ep].next_rx);

  if (ep_info[ep].tlength > ep_info[ep].psize)
  {
    ready_ep_rx(ep, (hcc_u8)(ep_info[ep].next_rx ^ 0x1));
  }
}

/*****************************************************************************
 * Name:
 *    usb_abort_ep
 * In:
 *    ep          - endpoint number
 *
 * Out:
 *    N/A
 * Description:
 *    A call to this function will stop the ongoing transfer on the selected
 *    endpoint (early stop). Early stop is only possible if there is some
 *    penging date left in the buffer. If not, then transfer willstop anyway.
 *****************************************************************************/
void usb_abort_ep(hcc_u8 ep)
{
  OS_SR_SAVE_VAR
  /* Get endpoint into a fixed state. */
  _irq_disable();
  /* If the endpoint is busy. */
  switch(ep_info[ep].state)
  {
  case EPST_IDLE:
    /* If endpoint is idle, we can not stop it. */
    break;
    /* In these cases is is too late for an early stop since the transfer
       is already going to end. */
  case EPST_DATA_TX_LAST:
  case EPST_STATUS_TX:
  case EPST_STATUS_RX:
    break;

  case EPST_DATA_RX:
    ep_info[ep].error=USBEPERR_USER_ABORT;
    ep_info[ep].flags |= EPFL_ERROR;

    /* Disable following calbacks in case of an error. */
    ep_info[ep].data_func=(void *) 0;
    /* Stall endpoint. */
    usb_stop_ep_rx(ep);
    break;
  default:
    /* UPS! invalid or unknown state... */
    CMX_ASSERT(0);
  }
  _irq_restore();
}

/*****************************************************************************
 * Name:
 *    cb_set_address
 * In:
 *    N/A
 *
 * Out:
 *    N/A
 *
 * Description:
 *    This callback is used by the state machine that handles the standard
 *    requests on the default pipe to set the device address after the
 *    status stage of the "set address" request.
 *****************************************************************************/
static callback_state_t cb_set_address()
{
  if (new_address != 0)
  {
    usb_state=USBST_ADDRESSED;
  }
  else
  {
    enter_default_state();
  }
  MCF_USB_ADDR = new_address;
  new_address = 0;
  return(clbst_ok);
}

/*****************************************************************************
 * Name:
 *    usb_stm_ctrl0
 * In:
 *    N/A
 * Out:
 *    status of callback execution
 *
 * Description:
 *    Will decode and handle setup packets on the default endpoint.
 * Assumptions:
 *    Is only called if a setup packet is received.
 *****************************************************************************/
static callback_state_t usb_stm_ctrl0()
{
  hcc_u8 *pdata=(hcc_u8 *)usb_get_rx_pptr(0);

  /* The return value shall reflect the direction of the transfer. */
  callback_state_t r=(STP_REQU_TYPE(pdata) & 1u<<7) ? clbst_in: clbst_out;

  switch (STP_REQU_TYPE(pdata) & 0x7f)
  {
  case 0: /* Standard request for the device. */
    /* Determine what request this is. */
    switch (STP_REQUEST(pdata))
    {
    case USBRQ_SET_ADDRESS:
      new_address=(hcc_u8)(STP_VALUE(pdata) & ((1u << 7)-1u));
      ep_info[0].data_func=cb_set_address;
      break;
    case USBRQ_GET_DESCRIPTOR:
      switch(STP_VALUE(pdata) & 0xff00u)
      {
      case STDD_DEVICE << 8:
        {
          hcc_u8 *p=(hcc_u8*)get_device_descriptor();
          usb_send(0, (void *) 0, p, p[0], STP_LENGTH(pdata));
        }
        break;
      case STDD_CONFIG << 8:
        /* Do we have a CFG descriptor with the requested index? */
        {
          hcc_u8 cfg=(hcc_u8)(STP_VALUE(pdata) & 0xffu);
          /* For index 0 we return the first config descriptor. */
          if (cfg == 0)
          {
            cfg++;
          }
          if (is_cfgd_index(cfg))
          {
            const hcc_u8 *cd;
            cd=get_cfg_descriptor(cfg);
            usb_send(0, (void *) 0, (void*)cd, RD_LE16(&cd[2])
                     , STP_LENGTH(pdata));
            break;
          }
        }
        /* No such descriptor, report error. */
        r=clbst_error;
        break;

      case STDD_STRING << 8:
        /* See if te required descriptor exists. */
        if (is_str_index((hcc_u8)(STP_VALUE(pdata) & 0xffu)))
        {
          usb_send(0, (void *) 0, (void *)get_str_descriptor((hcc_u8)(STP_VALUE(pdata) & 0xffu))
                   , *(hcc_u8*)get_str_descriptor((hcc_u8)(STP_VALUE(pdata) & 0xffu))
                   , STP_LENGTH(pdata));
          break;
        }
        /* No such string descriptor. */
        r=clbst_error;
        break;

      default:
        /* Call user callback if avaiable. */
        goto call_usercb;
      }
      break;
    case USBRQ_GET_CONFIGURATION:
      usb_send(0, (void *) 0, (void *)&usb_current_config
                 , 1
                 , STP_LENGTH(pdata));
      break;
    case USBRQ_SET_CONFIGURATION:
      if (STP_VALUE(pdata) == 0)
      {
        set_config(0);
        break;
      }
      else if (is_cfgd_index(STP_VALUE(pdata)))
      {

        set_config((hcc_u8)STP_VALUE(pdata));
        break;
      }
      r=clbst_error;
      break;
    case USBRQ_SYNCH_FRAME:
    case USBRQ_SET_FEATURE:
    case USBRQ_CLEAR_FEATURE:
    case USBRQ_GET_STATUS:
    case USBRQ_SET_DESCRIPTOR:
    default: /* Unknown or not implemented request. */
      /* Call user callback if avaiable. */
      goto call_usercb;
    }
    break;
  case 1: /* Standard request for an interface. */
      /* Call user callback. */
      goto call_usercb;
  case 2: /* Standard request for an endpoint. */
    switch(STP_REQUEST(pdata))
    {
    case USBRQ_CLEAR_FEATURE:
      switch(STP_VALUE(pdata))
      {
      case FEAT_ENDPOINT_HALT:
        {
          /* Find the endpoint with the address specified int pdata->index.
          assumption: endpoints are allocated after each other. */
          hcc_u8 adr=(hcc_u8)(STP_INDEX(pdata) & 0x7f);

          if (STP_INDEX(pdata) & 0x80)
          {
            hcc_u32 bdt;
            ep_info[adr].data0_tx=0;
            bdt=RD_LE32(&BDT_CTL_TX(adr, 0));
            bdt &= ~BDT_CTL_STALL;
          	WR_LE32(&BDT_CTL_TX(adr, 0), bdt);
            bdt=RD_LE32(&BDT_CTL_TX(adr, 1));
            bdt &= ~BDT_CTL_STALL;
          	WR_LE32(&BDT_CTL_TX(adr, 1), bdt);
          }
          else
          {
            hcc_u32 bdt;
            ep_info[adr].data0_rx=0;
            bdt=RD_LE32(&BDT_CTL_RX(adr, 0));
            bdt &= ~BDT_CTL_STALL;
          	WR_LE32(&BDT_CTL_RX(adr, 0), bdt);
            bdt=RD_LE32(&BDT_CTL_RX(adr, 1));
            bdt &= ~BDT_CTL_STALL;
          	WR_LE32(&BDT_CTL_RX(adr, 1), bdt);
          }
        }
        break;
      default:
        goto call_usercb;
      }
      break;
    default:
      /* Call user callback if avaiable. */
      goto call_usercb;
    }
    break;
  default:
  call_usercb:
    r=usb_ep0_callback();
  }

  return(r);
}

/*****************************************************************************
 * Name:
 *    usb_it_handler
 * In:
 *    N/A
 * Out:
 *    N/A
 *
 * Description:
 *    Low priority interrupt handler.
 *
 * Assumptions:
 *
 *****************************************************************************/

void USB_Handler(void)
{
  INT8U istr = 0;
  
  istr = INT_STAT;
  
  if(istr & ~MCF_USB_INT_STAT_TOK_DNE)
  {
    if (istr & MCF_USB_INT_STAT_USB_RST)
    {
      /* Clear IT flag. */
      MCF_USB_INT_STAT = MCF_USB_INT_STAT_USB_RST;
      /* Enter default state. */
      enter_default_state();
      usb_reset_event();
      goto it_exit;
    }

    if (istr & MCF_USB_INT_STAT_ERROR)
    {
      MCF_USB_INT_STAT = MCF_USB_INT_STAT_ERROR;
      usb_bus_error_event();
    }
    if (istr & MCF_USB_INT_STAT_RESUME)
    {
      MCF_USB_INT_ENB &= ~MCF_USB_INT_ENB_RESUME;
      MCF_USB_INT_STAT = MCF_USB_INT_STAT_SLEEP;
      
      //MCF_USB_INT_ENB |= MCF_USB_INT_ENB_SLEEP;
      usb_wakeup_event();
    }
    if (istr & MCF_USB_INT_STAT_SLEEP)
    {
      MCF_USB_INT_ENB &= ~MCF_USB_INT_ENB_SLEEP;
      MCF_USB_INT_STAT = MCF_USB_INT_STAT_RESUME;
      //MCF_USB_INT_ENB |= MCF_USB_INT_ENB_RESUME;
      usb_suspend_event();
    }

    /* Stall was sent. */
    if (istr & MCF_USB_INT_STAT_STALL)
    {
      hcc_u8 ep=(hcc_u8)(MCF_USB_STAT >> 4);
      MCF_USB_INT_STAT = MCF_USB_INT_STAT_STALL;
      /* Remove stall for control endpoints. ??*/
      if ((MCF_USB_ENDPT(ep) & MCF_USB_ENDPT_EP_CTL_DIS) == 0)
      {
        WR_LE32(&BDT_CTL_RX(ep, 0), 0);
        WR_LE32(&BDT_CTL_RX(ep, 1), 0);
        WR_LE32(&BDT_CTL_TX(ep, 0), 0);
        WR_LE32(&BDT_CTL_TX(ep, 1), 0);
        ready_ep_rx(ep, ep_info[ep].next_rx);
      }
      OSSemPost(USB_Sem);
    }
  }

  if (istr & MCF_USB_INT_STAT_TOK_DNE)
  {
    OSSemPost(USB_Sem);
    hcc_u8 ep=(hcc_u8)(MCF_USB_STAT >> 4);
    int is_rx;
    int is_stp;
    hcc_u8 this_buf=(hcc_u8)(MCF_USB_STAT & MCF_USB_STAT_ODD ? 1 : 0);

    if (MCF_USB_STAT & MCF_USB_STAT_TX)
    {
      is_rx=0;
      is_stp=0;

      WR_LE32(&BDT_CTL_TX(ep, this_buf),
               RD_LE32(&BDT_CTL_TX(ep, this_buf)) & ~(BDT_CTL_STALL|BDT_CTL_OWN));
      ep_info[ep].next_tx = (hcc_u8)(this_buf ? 0u : 1u);
    }
    else
    {/* This is an RX packet.*/
      is_rx=1;
      ep_info[ep].next_rx=(hcc_u8)(this_buf ^ 0x1u);
      is_stp=(hcc_u8)(RD_LE32(&BDT_CTL_RX(ep, this_buf)) & 0x3f) == (SOF_PID_VALUE<<2) ? 1 : 0;

      WR_LE32(&BDT_CTL_RX(ep, this_buf),
               RD_LE32(&BDT_CTL_RX(ep, this_buf)) & ~(BDT_CTL_STALL|BDT_CTL_OWN));
    }

    /* is this a control endpoint? */
    if ((MCF_USB_ENDPT(ep) & MCF_USB_ENDPT_EP_CTL_DIS) == 0)
    {/* then use a special state machine. */
      switch(ep_info[ep].state)
      {
      default:
        CMX_ASSERT(0);
        break;
      case EPST_IDLE:
      idle:
        if (is_stp)
        {
          callback_state_t r;
          /* Reset the endpoint state. */

          /* Since TOKBUSY is still set, this is a safe point to disable
             packet buffers (NAK in and out direction). */
          WR_LE32(&BDT_CTL_TX(ep, 0), 0);
          WR_LE32(&BDT_CTL_TX(ep, 1), 0);
          WR_LE32(&BDT_CTL_RX(ep, 0), 0);
          WR_LE32(&BDT_CTL_RX(ep, 1), 0);


          /* Dnager: if a setup frame will be received not in idle
             state, then the TOKBUSY will not be cleared,
             and the whole USB will not answer any more.*/

          /* This flag shall be cleared after each setup packet. Note:
             it seems it shall be cleared before trying to use endpoint
             buffers. */
          MCF_USB_CTL &= ~MCF_USB_CTL_TXDSUSPEND_TOKBUSY;

             /* remove any callback */
          ep_info[ep].data_func = (void*)0;
             /* clear flags*/
          ep_info[ep].flags=0;
             /* clear user error indicator */
          ep_info[ep].error=USBEPERR_NONE;
          /* After a setup packet the next packet is always a data1. */
          ep_info[ep].data0_tx = BDT_CTL_DATA;
          ep_info[ep].data0_rx = BDT_CTL_DATA;
          /* Remove stall condition. */

          if (ep == 0)
          {
            r=usb_stm_ctrl0();
          }
          else
          {
            /* Only one control channel is supported. */
            r=clbst_error;
          }
          switch(r)
          {
            case clbst_error:
              usb_stop_ep_tx(ep);
              usb_stop_ep_rx(ep);
              ep_info[ep].state = EPST_IDLE;
              ep_info[ep].error = USBEPERR_USER_ABORT;
              break;
            case clbst_not_ready:
              /* Nothing to do. */
              break;
            case clbst_in:
              ready_ep_rx(ep, ep_info[ep].next_rx);
              break;
            case clbst_out:
              if (ep_info[ep].state == EPST_IDLE)
              {
                goto send_status;
              }
              break;
            default:
              break;
          }
        }
        else
        {/* This is not a setup packet, and we encountered a protocol error.
            Possible reasons:
               -spurious package on the bus (not our fault)
               -we ended the transfer before the host.
                  - host and device transfer length was not the same
                  - driver error miscounted packages
          */
          /* Stall endpoint to make error visible to the host. */
          ep_info[ep].error = USBEPERR_TO_MANY_DATA;

          usb_stop_ep_rx(ep);
          usb_stop_ep_tx(ep);
        }
        break;
      case EPST_DATA_TX:
      case EPST_DATA_TX_LAST:
      case EPST_TX_STOP:

        /* If there is an RX interrupt pending , stop transmission. */
        if (is_rx)
        {
          /* Inform application about transfer end. */
          ep_info[ep].state=EPST_IDLE;
          if (ep_info[ep].data_func != (void *)0)
          {
            (void)(*ep_info[ep].data_func)();
          }
          /* Go to EPST_IDLE and handle the packet as it would be the start
             of a new transfer.  */
          goto idle;
        }
        _usb_send(ep);

        /* If transfer is ended, then we shall receive a handshake from the host. */
        if (ep_info[ep].state == EPST_IDLE)
        {
          ep_info[ep].state=EPST_STATUS_TX;
        }

        break;
      case EPST_STATUS_TX:
        /* We may have some TX interrupt events on the USB module event stack. These
           events are already handled, but wi may get dome dummy interrupts.
           So simply dropp such events to keep the state machine synchron to the USB. */
        if (!is_rx)
        {
          break;
        }
        /* In the IN transfer status stage we received a 0 byte long DATA 1 packet. */
        ep_info[ep].state=EPST_IDLE;
        /* Tell application transfer ended. */
        if (ep_info[ep].data_func != (void *) 0)
        {
          /* We don't care about the return value at this point, since we already
             sent the status, and the transfer is already ended. */
          (void)(*ep_info[ep].data_func)();
          /* Disable callbacks. */
          ep_info[ep].data_func = (void *)0;
        }
        ready_ep_rx(ep, ep_info[ep].next_rx);
        break;
      case EPST_DATA_RX:
        _usb_receive(ep);
        /* Was this the last packet to receive? */
        send_status:
        if(ep_info[ep].state == EPST_IDLE)
        {
          /* If there was an error, stall the status stage. */
          if (ep_info[ep].flags & EPFL_ERROR)
          {
            usb_stop_ep_tx(ep);
          }
          else
          {
            /* Handshake is allways a data 1 packet. */
            ep_info[ep].data0_tx = BDT_CTL_DATA;
            /* If all ok send handshake. */
            ep_info[ep].state=EPST_STATUS_RX;
            /* Send a 0 byte long data1 packet as response.*/
            send_zero_packet(ep);
          }
        }
        break;
      case EPST_STATUS_RX:
        /* In the OUT transfer status stage we sent a 0 byte long DATA 0 packet. */
        ep_info[ep].state=EPST_IDLE;
        /* Tell application transfer ended. */
        if (ep_info[ep].data_func != (void *) 0)
        {
          /* We don't care about the return value at this point, since we already
             sent the status, and the transfer is already ended. */
          (void)(*ep_info[ep].data_func)();
          /* Disable callbacks. */
          ep_info[ep].data_func = (void *)0;
        }
        ready_ep_rx(ep, ep_info[ep].next_rx);
        break;
      }
    }
    else
    {
      /* Is this an OUT (rx) endpoint? */
      if(is_rx)
      {
        /* Handle reception. */
        _usb_receive(ep);
      }
      else
      { /* This is an IN endpoint (TX) */
        _usb_send(ep);
        if (ep_info[ep].state==EPST_IDLE)
        {
          /* Inform user application. */
          if (ep_info[ep].data_func != (void *)0)
          {
            (*ep_info[ep].data_func)();
            /* Disable futher callbacks. */
            ep_info[ep].data_func = (void *)0;
          }
        }
      }
    }
	  /* Clear interrupt flag. */
    MCF_USB_INT_STAT=MCF_USB_INT_STAT_TOK_DNE;
  }
  it_exit:
}
  





void USB_ISR(void);

#if (NESTING_INT == 1)
#pragma TRAP_PROC
void USB_ISR(void)
#else
interrupt void USB_ISR(void)
#endif
{
  // ************************
  // Entrada de interrupção
  // ************************
  OS_INT_ENTER();   
  
  #if (NESTING_INT == 1)
  OS_ENABLE_NESTING();
  #endif    
  
  USB_Handler();
  
  // ************************
  // Interrupt Exit
  // ************************
  OS_INT_EXIT();  
  // ************************    
}

/****************************** END OF FILE **********************************/


