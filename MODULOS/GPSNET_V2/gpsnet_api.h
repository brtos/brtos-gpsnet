/**********************************************************************************
@file   gpsnet_api.h
@brief  public interface to the GPSNET network
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
*********************************************************************************/

#ifndef GPSNET_API_H
#define GPSNET_API_H   

/* This file is a public interface to the GPSNET network */
/* Any application should reference only this file in order to use the GPSNET API */

#include "NetConfig.h"
#include "mac.h"
#include "network.h"
#include "gpsnet_app.h"
#include "MRF24J40.h"

#define GPSNET_VERSION    "Network Ver. 1.0.7"

/* Message code returns */ 
#define SEND_OK              (INT8U)0x00
#define SEND_ERROR           (INT8U)0x01

/* Functions used to send data in the network */
INT8U NetSimpledata(INT8U direction, INT8U destino, INT8U *p_data);
INT8U NetSimpledataXY(INT8U direction, LATITUDE *lat, LONGITUDE *lon, INT8U *p_data);
void  NetSimpleMeasure(INT8U direction, INT8U MeasureType, INT16U Value16, INT32U Value32);

/* Functions used to receive and decode data from the network */
void  Decode_General_Profile(void);
void  Decode_Lighting_Profile(void);
void  Decode_SmartEnergy_Profile(void);
INT8U Decode_Data_Profile(void);

INT8U* GetGPSNET_Statistics(INT8U* tamanho);

/* External functions */
extern void GPSNET_Init(void);
extern void GPSNET_APP(void);

/* GPSNET tasks */
extern void GPSNET_RF_Event(void);
extern void GPSNET_MAC(void);
extern void GPSNET_NWK(void);

/* External Variables */
extern        OS_QUEUE     RFBuffer;
extern        BRTOS_Queue  *RF;
extern        BRTOS_Sem    *RF_Event;
extern        BRTOS_Sem    *RF_RX_Event;
extern        BRTOS_Sem    *RF_TX_Event;
extern        BRTOS_Sem    *MAC_Event;

// Apps signals

#ifdef SIGNAL_APP1
extern BRTOS_Sem    *(SIGNAL_APP1);
#endif
#ifdef SIGNAL_APP2
extern BRTOS_Sem    *(SIGNAL_APP2);
#endif
#ifdef SIGNAL_APP3
extern BRTOS_Sem    *(SIGNAL_APP3);
#endif
#ifdef SIGNAL_APP4
extern BRTOS_Sem    *(SIGNAL_APP4);
#endif
#ifdef SIGNAL_APP255
extern BRTOS_Sem    *(SIGNAL_APP255);
#endif 

/* payload vector - max. bytes = MAX_APP_PAYLOAD_SIZE */
extern  volatile INT8U     NWKPayload[MAX_APP_PAYLOAD_SIZE];

void IncGPSNET_NodeStat_apptxed(void);

#endif
