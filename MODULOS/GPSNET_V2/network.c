/**********************************************************************************
@file   network.c
@brief  GPSNET NWK-layer functions
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

#include "hardware.h"
#include "BRTOS.h"
#include "drivers.h"
#include "MRF24J40.h"
 
#include "gpsr.h"
#include "gpsnet_api.h"

#if (PROCESSOR == ARM_Cortex_M0)
#include "FLASH.h"
#endif

#if(DEVICE_TYPE == ROUTER)
#include "app.h" 
#define CFG_PARENT_THRESHOLD      Config_PARENT_THRESHOLD
#define CFG_PARENT_THRESHOLD_MIN  Config_PARENT_THRESHOLD_MIN
#else
#define CFG_PARENT_THRESHOLD      0
#define CFG_PARENT_THRESHOLD_MIN  0
#endif


// Endereço de rede
#if PROCESSOR == COLDFIRE_V1
const unsigned char Latitude[4]  @LAT_MEM_ADDRESS = {LAT_GPS_DEG,LAT_GPS_MIN,LAT_GPS_SEC,LAT_GPS_DECSEC};
const unsigned char Longitude[4] @LON_MEM_ADDRESS = {LONG_GPS_DEG,LONG_GPS_MIN,LONG_GPS_SEC,LONG_GPS_DECSEC};
#endif

#if PROCESSOR == ARM_Cortex_M0
unsigned char *Latitude  = (unsigned char *)LAT_MEM_ADDRESS;
unsigned char *Longitude = (unsigned char *)LON_MEM_ADDRESS;
#endif

 
// Tabela de nós vizinhos
volatile GPSNET_NEIGHBOURHOOD            gpsnet_neighbourhood[NEIGHBOURHOOD_SIZE];

// Tabela de estações base
volatile GPSNET_BASESTATION              BaseStations[MAX_BASE_STATION];

// 
volatile GPSNET_SYMMETRIC_NEIGHBOURHOOD  gpsnet_neighbor_ping;
volatile NEIGHBOR_TABLE_T                NeighborTable = 0;

#if (USE_REACTIVE_UP_ROUTE == 1)
volatile GPSNET_ROUTING_UP_TABLE         gpsnet_routing_up_table[ROUTING_UP_TABLE_SIZE];
#endif

// Posição GPS do nó
volatile LATITUDE                   gps_latitude;
volatile LONGITUDE                  gps_longitude;

// Posição GPS de um destino
volatile LATITUDE                   dst_gps_latitude;
volatile LONGITUDE                  dst_gps_longitude;

// Base stations
#if  (DEVICE_TYPE != PAN_COORDINATOR)
  #define BASECNT_INIT_VALUE   0
#else
  #define BASECNT_INIT_VALUE   1
#endif 

volatile INT8U                 BaseCnt = BASECNT_INIT_VALUE;
volatile INT8U                 NearBase = 0;
volatile INT8U                 BetterDepth = 0xFF; 

// Payload das mensagens roteadas
volatile INT8U                 NWKPayload[MAX_APP_PAYLOAD_SIZE];

#if (NWK_MUTEX_TYPE == BRTOS_MUTEX)
/* Mutex for Radio IEEE 802.15.4 */
BRTOS_Mutex                  *Radio_IEEE802;
#endif

//volatile INT16U              AssociateTimeout     = 0;
//volatile INT16U              NeighborCnt          = 0;
//volatile INT32U              NeighbourhoodCnt     = 0;
static   INT16U              DepthWatchdog        = 0;
volatile INT16U              NeighborPingTimeV    = 10000;
volatile INT16U              RadioWatchdog        = 5000;
volatile INT8U               NeighborPingTimeCnt  = 1;

volatile NWK_TASKS_PENDING   nwk_tasks_pending;  

#if(defined SUPPORT_APP_SEQNUMBER && SUPPORT_APP_SEQNUMBER == 1)
volatile INT8U App_SeqNumber = 1;
#endif


//Função para adquirir direito exclusivo ao radio
void acquireRadio(void)
{  
  #if (NWK_MUTEX_TYPE == BRTOS_MUTEX)
    OSMutexAcquire(Radio_IEEE802);
  #else
    // Bloqueia GPSNET_TASKS tarefas a partir da tarefa GPSNET_TASKS_INIT
    BlockMultipleTask(GPSNET_TASKS_INIT,GPSNET_TASKS);
  #endif
}

//Função para liberar o radio
void releaseRadio(void)
{
  #if (NWK_MUTEX_TYPE == BRTOS_MUTEX)
    OSMutexRelease(Radio_IEEE802);
  #else  
    // Desbloqueia GPSNET_TASKS tarefas a partir da tarefa GPSNET_TASKS_INIT
    UnBlockMultipleTask(GPSNET_TASKS_INIT,GPSNET_TASKS);
  #endif
}


/* 
   Cria um mutex informando que o recurso está disponível
   após a inicialização da prioridade máxima a acessar o recurso = priority
*/
void init_radio_resource(INT8U priority)
{  
  
  #if (NWK_MUTEX_TYPE == BRTOS_MUTEX)  
  if (OSMutexCreate(&Radio_IEEE802,priority) != ALLOC_EVENT_OK)
  {
    while(1){};
  };
  #else
   (void) priority;
  #endif
}

// Monta pacote de comando Neighbor Ping
// HeaderSize = 9 bytes 
// MAX_BASE_STATION = 4
// Payload Max. = 2 + 9*4 + 8 + 16*3  = 94 bytes
// Sobram 22 bytes dos 125 disponíveis
// 4 bits por canal dos vizinhos * 16 = 8 bytes com suporte multicanal
// 1 byte para canal e contagem de vizinhos 
void NeighborPing(void)
{
  INT8U i           = 0;
  INT8U j           = 0;
  INT8U address     = 0;
  INT8U HeaderSize  = 0;
  INT8U PayloadSize = 0;
  INT8U basis       = 0;
                            
  // Inicia montagem do pacote Data
  
  // Indicação de Beacon no Frame Control
  PHYSetLongRAMAddr(2, 0x41);
  HeaderSize++;
  
  // Indicação de Dest e Source Address de 16b, no Frame Control    
  PHYSetLongRAMAddr(3, 0x88);
  HeaderSize++;  
  
  // Sequence Number
  PHYSetLongRAMAddr(4, SequenceNumber);
  if (++SequenceNumber == 0) SequenceNumber = 1;
  HeaderSize++;
  
  // PanId do coordenador que gera o data packet
  PHYSetLongRAMAddr(5, (INT8U)(macPANId & 0xFF));
  PHYSetLongRAMAddr(6, (INT8U)(macPANId >> 8));
  
  // Endereço de destino broadcast do data packet
  PHYSetLongRAMAddr(7, (INT8U)0xFF);
  PHYSetLongRAMAddr(8, (INT8U)0xFF);
  
  // Endereço fonte do data packet
  PHYSetLongRAMAddr(9, (INT8U)(macAddr & 0xFF));
  PHYSetLongRAMAddr(10, (INT8U)(macAddr >> 8));
  HeaderSize += 6;  
  
  
  // Enviar dados do pacote de vizinhança padrão da rede
  
  // Tipo de pacote de dados
  PHYSetLongRAMAddr(11, DATA_PING);
  PayloadSize++;
  
  // Base stations info
  UserEnterCritical();    
    basis = BaseCnt;
  UserExitCritical(); 
 
#if (defined MULTICHANNEL_SUPPORT &&  MULTICHANNEL_SUPPORT == 1)
  // first nibble is RX channel and last nibble is basis count 
  PHYSetLongRAMAddr(12, (INT8U)((GetChannel() & 0xF0) | (basis & 0x0F)));
#else
  #if (defined FORCE_NO_MULTICHANNEL_SUPPORT && FORCE_NO_MULTICHANNEL_SUPPORT == 1)
  PHYSetLongRAMAddr(12, (INT8U)(basis));
  #else
  PHYSetLongRAMAddr(12, (INT8U)((basis & 0x0F) | 0xF0));
  #endif
#endif  

  PayloadSize++;
  address = 12;
  
  for(i=0;i<basis;i++)
  {
    PHYSetLongRAMAddr(++address, BaseStations[i].BaseDepth);
    
    for (j=0;j<4;j++)
    {
      PHYSetLongRAMAddr(++address, BaseStations[i].GPS_Lat[j]);
    }
    
    for (j=0;j<4;j++)
    {
      PHYSetLongRAMAddr(++address, BaseStations[i].GPS_Long[j]);
    }    
    
    PayloadSize += 9;
  }
  
  // Envia posição GPS do nó 
  PHYSetLongRAMAddr(++address, gps_lat.GPS_Degrees);
  PHYSetLongRAMAddr(++address, gps_lat.GPS_Minutes);
  PHYSetLongRAMAddr(++address, gps_lat.GPS_Seconds);
  PHYSetLongRAMAddr(++address, gps_lat.GPS_DecSeconds);
  
  PHYSetLongRAMAddr(++address, gps_long.GPS_Degrees);
  PHYSetLongRAMAddr(++address, gps_long.GPS_Minutes);
  PHYSetLongRAMAddr(++address, gps_long.GPS_Seconds);
  PHYSetLongRAMAddr(++address, gps_long.GPS_DecSeconds);

  PayloadSize += 8;
  
  // Coloca os vizinhos no pacote
  for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
  {
    if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE)
    {
      PHYSetLongRAMAddr((INT16U)(++address), (INT8U)(gpsnet_neighbourhood[i].Addr_16b >> 8));
      PHYSetLongRAMAddr((INT16U)(++address), (INT8U)(gpsnet_neighbourhood[i].Addr_16b & 0xFF));
      PHYSetLongRAMAddr((INT16U)(++address), (INT8U)(gpsnet_neighbourhood[i].NeighborRSSI & 0xFF));
      PayloadSize += 3;
    }
  }     

  // Informação do tamanho do MAC header em bytes
  // No modo não seguro é ignorado
  PHYSetLongRAMAddr(0x000,HeaderSize);
  // Informação do tamanho em bytes do MAC header + Payload
  PHYSetLongRAMAddr(0x001,(INT8U)(HeaderSize+PayloadSize));

  //transmit packet without ACK requested
  mac_tasks_pending.bits.PacketPendingAck = 1;
  PHYSetShortRAMAddr(WRITE_TXNMTRIG,0b00000001);
}



// Monta pacote para enviar endereço GPS
void GPSAddressMessage(INT8U *latitude, INT8U *longitude)
{
  INT8U address     = 0;
  INT8U HeaderSize  = 0;
  INT8U PayloadSize = 0;
                            
  // Inicia montagem do pacote Data
  
  // Indicação de Beacon no Frame Control
  PHYSetLongRAMAddr(2, 0x41);
  HeaderSize++;
  
  // Indicação de Dest e Source Address de 16b, no Frame Control    
  PHYSetLongRAMAddr(3, 0x88);
  HeaderSize++;
 
  
  // Sequence Number
  PHYSetLongRAMAddr(4, SequenceNumber);
  if (++SequenceNumber == 0) SequenceNumber = 1;
  HeaderSize++;
  
  // PanId do coordenador que gera o data packet
  PHYSetLongRAMAddr(5, (INT8U)(macPANId & 0xFF));
  PHYSetLongRAMAddr(6, (INT8U)(macPANId >> 8));
  
  // Endereço de destino broadcast do data packet
  PHYSetLongRAMAddr(7, (INT8U)0xFF);
  PHYSetLongRAMAddr(8, (INT8U)0xFF);
  
  // Endereço fonte do data packet
  PHYSetLongRAMAddr(9, (INT8U)(macAddr & 0xFF));
  PHYSetLongRAMAddr(10, (INT8U)(macAddr >> 8));
  HeaderSize += 6;    
  
  // Tipo de pacote de dados
  PHYSetLongRAMAddr(11, ADDRESS_PACKET);
  PayloadSize++;
  address = 11; 
  
  // Envia posição GPS do nó
  PHYSetLongRAMAddr(++address, *latitude++);
  PHYSetLongRAMAddr(++address, *latitude++);
  PHYSetLongRAMAddr(++address, *latitude++);
  PHYSetLongRAMAddr(++address, *latitude);
  
  PHYSetLongRAMAddr(++address, *longitude++);
  PHYSetLongRAMAddr(++address, *longitude++);
  PHYSetLongRAMAddr(++address, *longitude++);
  PHYSetLongRAMAddr(++address, *longitude);
  PayloadSize += 8;

  // Informação do tamanho do MAC header em bytes
  // No modo não seguro é ignorado
  PHYSetLongRAMAddr(0x000,HeaderSize);
  // Informação do tamanho em bytes do MAC header + Payload
  PHYSetLongRAMAddr(0x001,(INT8U)(HeaderSize+PayloadSize));

  //transmit packet without ACK requested
  mac_tasks_pending.bits.PacketPendingAck = 1;
  PHYSetShortRAMAddr(WRITE_TXNMTRIG,0b00000001);
}


// Monta pacote para enviar endereço GPS
void GPSAddressMessageToMAC(INT8U *latitude, INT8U *longitude, INT16U mac16)
{
  INT8U address     = 0;
  INT8U HeaderSize  = 0;
  INT8U PayloadSize = 0;
  
  #if (defined MULTICHANNEL_SUPPORT) && (MULTICHANNEL_SUPPORT==1) 
    
    INT8U rx_channel;
    INT8U tx_channel;
    
    rx_channel = GetChannel();   /* Save current RX channel */
    
    if(mac16 != 0xFFFF){         
        tx_channel = 0;        
        goto loop;      /* tx only once */
        
    }else{
    
      /* Broacast in all channels */
     for(tx_channel = 0;tx_channel < NUM_ALLOWED_CHANNELS;tx_channel++){
     
   loop:
                      
        SetChannel(GetAllowedChannels(tx_channel));   /* Set TX channel */                        
            
   #endif
   
  
        address = 0; HeaderSize = 0;PayloadSize = 0;
                                  
        // Inicia montagem do pacote Data
        
        // Indicação de Beacon no Frame Control
        PHYSetLongRAMAddr(2, 0x41);
        HeaderSize++;
        
        // Indicação de Dest e Source Address de 16b, no Frame Control    
        PHYSetLongRAMAddr(3, 0x88);
        HeaderSize++;
       
        
        // Sequence Number
        PHYSetLongRAMAddr(4, SequenceNumber);
        if (++SequenceNumber == 0) SequenceNumber = 1;
        HeaderSize++;
        
        // PanId do coordenador que gera o data packet
        PHYSetLongRAMAddr(5, (INT8U)(macPANId & 0xFF));
        PHYSetLongRAMAddr(6, (INT8U)(macPANId >> 8));
        
        // Endereço de destino do data packet  (MAC)
        PHYSetLongRAMAddr(7, (INT8U)(mac16 & 0xFF));
        PHYSetLongRAMAddr(8, (INT8U)(mac16 >> 8));
        
        // Endereço fonte do data packet
        PHYSetLongRAMAddr(9, (INT8U)(macAddr & 0xFF));
        PHYSetLongRAMAddr(10, (INT8U)(macAddr >> 8));
        HeaderSize += 6;    
        
        // Tipo de pacote de dados
        PHYSetLongRAMAddr(11, ADDRESS_PACKET);
        PayloadSize++;
        address = 11; 
        
        // Envia posição GPS do nó
        PHYSetLongRAMAddr(++address, *latitude++);
        PHYSetLongRAMAddr(++address, *latitude++);
        PHYSetLongRAMAddr(++address, *latitude++);
        PHYSetLongRAMAddr(++address, *latitude);
        
        PHYSetLongRAMAddr(++address, *longitude++);
        PHYSetLongRAMAddr(++address, *longitude++);
        PHYSetLongRAMAddr(++address, *longitude++);
        PHYSetLongRAMAddr(++address, *longitude);
        PayloadSize += 8;

        // Informação do tamanho do MAC header em bytes
        // No modo não seguro é ignorado
        PHYSetLongRAMAddr(0x000,HeaderSize);
        // Informação do tamanho em bytes do MAC header + Payload
        PHYSetLongRAMAddr(0x001,(INT8U)(HeaderSize+PayloadSize));

        //transmit packet without ACK requested
        mac_tasks_pending.bits.PacketPendingAck = 1;
        PHYSetShortRAMAddr(WRITE_TXNMTRIG,0b00000001);
  
  
  #if (defined MULTICHANNEL_SUPPORT) && (MULTICHANNEL_SUPPORT==1)
          if (mac16 != 0xFFFF) break;    /* tx only once */
    
       }   /** end of loop */
    }   /* end if */
            
    SetChannel(rx_channel); /* back to saved RX channel */
  #endif 
}


// Verifica se o endereço será reprogramado
void VerifyNewAddress(void)
{
  INT8U  index = 0;
  INT8U  i = 0;
  INT32U OldAddress = 0; 
  
#if DEVICE_TYPE != PAN_COORDINATOR 
  INT8U  latitude[4];
  INT8U  longitude[4];  
#endif    
  
  
#if DEVICE_TYPE != PAN_COORDINATOR
#if (PROCESSOR == COLDFIRE_V1)
  OldAddress = (INT32U)(*(INT32U*)Latitude);
#endif

#if (PROCESSOR == ARM_Cortex_M0)
  OldAddress = (INT32U)(*(INT32U*)LAT_MEM_ADDRESS);
#endif
  
  if (OldAddress == 0xFFFFFFFF)
  {
    for (i=0;i<4;i++)
    {
      latitude[i] = mac_packet.MAC_Payload[++index];
    }
    
    for (i=0;i<4;i++)
    {
      longitude[i] = mac_packet.MAC_Payload[++index];
    }
    
    // Grava na flash endereco de rede    
#if FLASH_SUPPORTED == 1
#if (PROCESSOR == COLDFIRE_V1)
    UserEnterCritical();
    Flash_Prog((INT32U)Latitude, (INT32U)latitude, 1);
    Flash_Prog((INT32U)Longitude, (INT32U)longitude, 1);
    
    // Grava na flash endereco mac
    OldAddress = (INT32U)(macAddr & 0xFFFF);
    Flash_Prog((INT32U)&macAddress, (INT32U)&OldAddress, 1);
    
    // Grava na flash mac pan id
    OldAddress = (INT32U)(macPANId & 0xFFFF);
    Flash_Prog((INT32U)&macPANIdentificator, (INT32U)&OldAddress, 1);    
    UserExitCritical();
#endif

#if (PROCESSOR == ARM_Cortex_M0)
    WriteToFlash(latitude, LAT_MEM_ADDRESS, 4);
    WriteToFlash(longitude, LON_MEM_ADDRESS, 4);

    // Grava na flash endereco mac
    OldAddress = (INT32U)(macAddr & 0xFFFF);
    WriteToFlash((INT8U*)&OldAddress, MAC16_MEM_ADDRESS, 4);

    // Grava na flash mac pan id
    OldAddress = (INT32U)(macPANId & 0xFFFF);
    WriteToFlash((INT8U*)&OldAddress, PANID_MEM_ADDRESS, 4);
#endif
    
    // Atualiza posição GPS  
    gps_lat.GPS_Degrees     = Latitude[0];
    gps_lat.GPS_Minutes     = Latitude[1];
    gps_lat.GPS_Seconds     = Latitude[2];
    gps_lat.GPS_DecSeconds  = Latitude[3];
   
    gps_long.GPS_Degrees    = Longitude[0];
    gps_long.GPS_Minutes    = Longitude[1];
    gps_long.GPS_Seconds    = Longitude[2];
    gps_long.GPS_DecSeconds = Longitude[3];
#else
    // Atualiza posição GPS
    gps_lat.GPS_Degrees     = latitude[0];
    gps_lat.GPS_Minutes     = latitude[1];
    gps_lat.GPS_Seconds     = latitude[2];
    gps_lat.GPS_DecSeconds  = latitude[3];

    gps_long.GPS_Degrees    = longitude[0];
    gps_long.GPS_Minutes    = longitude[1];
    gps_long.GPS_Seconds    = longitude[2];
    gps_long.GPS_DecSeconds = longitude[3];
#endif
       
  }
#endif
}



INT8U VerifyPacketReplicated(void)
{
    INT8U i = 0;
    
    for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
    {
        if (gpsnet_neighbourhood[i].Addr_16b == mac_packet.SrcAddr_16b)
        {
          if (gpsnet_neighbourhood[i].NeighborLastID == mac_packet.Sequence_Number)
          {
            return TRUE;
          }else
          {
            gpsnet_neighbourhood[i].NeighborLastID = mac_packet.Sequence_Number;
            gpsnet_neighbourhood[i].IDTimeout      = LAST_ID_SYSTEM_TIMER_TIMEOUT;
            return OK;
          }
        }
    }
    return OK;
}



void VerifyNeighbourhoodLastIDTimeout(void)
{
    INT8U i = 0;
    
    for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
    {
        if (gpsnet_neighbourhood[i].IDTimeout)
        {
          gpsnet_neighbourhood[i].IDTimeout--;
          
          if (gpsnet_neighbourhood[i].IDTimeout == 0)
          {
              gpsnet_neighbourhood[i].NeighborLastID = 0;
          }
        }
    }
}

void VerifyNeighbourhood(void)
{
      INT8U i = 0;
      INT8U j = 0;
      INT8U k = 0;       
      
      // Verifica se existem vizinhos inativos
      for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
      {        
        // Se o nó está inativo por um determinado tempo
        if ((NeighborTable & (0x01 << i)) == 0)
        {
          // Retira da tabela de vizinhos
          if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE)
          {
            // Update Base Station Tables
            #if (DEVICE_TYPE != INSTALLER)
            for (j=0;j<BaseCnt;j++)
            {
              // Se este vizinho era responsável pela rota a base, marcar como rota perdida
              if (BaseStations[j].NeighborID == gpsnet_neighbourhood[i].Addr_16b)
              {
                BaseStations[j].BaseDepth  = ROUTE_TO_BASESTATION_LOST;
                BaseStations[j].NeighborID = 0xFFFF;
                BaseStations[j].ParentRSSI = 0;
                
                // Acelera o ping para propagar esta informação
                NeighborPingTimeCnt = 1;
                
                // Inicia contador de estabilização de profundidade
                ClearDepthWatchdog();
              }
            }
            
            // Update near base station info
            BetterDepth = 0xFF;
            for (j=0;j<BaseCnt;j++)
            {
              if (BaseStations[j].BaseDepth < BetterDepth)
              {
                NearBase = j;
                BetterDepth = BaseStations[j].BaseDepth;
              }
            }
            #endif
            
            // Delete Neighbor information
            gpsnet_neighbourhood[i].Addr_16b      = 0xFFFE;
            gpsnet_neighbourhood[i].NeighborRSSI  = 0;
            gpsnet_neighbourhood[i].NeighborGPS_LatX  = 0;
            gpsnet_neighbourhood[i].NeighborGPS_LongY = 0;
          }
        }
#if (DEVICE_TYPE == ROUTER)          
        else{
    
            // Verifica se o vizinho é simétrico
            if(gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE){
              k++;
            }
            
        }
#endif        
      }
      
#if (DEVICE_TYPE == ROUTER)  
      if(k==0) {  // Nenhum vizinho é simétrico
        SetTxPower(TX_POWER_LEVEL); // Volta potência inicial 
      }
#endif       
      NeighborTable = 0;
}



void IncDepthWatchdog(void) 
{
    DepthWatchdog++;
}

INT16U GetDepthWatchdog(void) 
{
  return DepthWatchdog;
}

void ClearDepthWatchdog(void) 
{
  DepthWatchdog = 0;
}


void UpdateBaseStation(INT8U neighbor)
{
    INT8U  i     = 0;
    INT8U  j     = 0;
    INT8U  index = 0;
    INT8U  cnt   = 0;
    
    for (i=0;i<gpsnet_neighbor_ping.NeighborBSCnt;i++)
    {
       // Search for base station in BaseStations table
       for (j=0;j<BaseCnt;j++)
       {
          cnt = 0;
          if (BaseStations[j].BaseDepth != 0xFF)
          {
              if (BaseStations[j].GPS_LatX == gpsnet_neighbor_ping.NeighborBS[i].GPS_LatX) 
              {
                cnt++;
              }
            
              if (BaseStations[j].GPS_LongY == gpsnet_neighbor_ping.NeighborBS[i].GPS_LongY) 
              {
                cnt++;
              }
          }
          if (cnt == 2)
          {
            break;
          }
       }
                     
       // If do not exist, create new base station
       if (cnt != 2)
       {
          // New Base station
          if (BaseCnt < MAX_BASE_STATION)
          {            
            
            //if ((gpsnet_neighbor_ping.NeighborBS[i].GPS_LatX > 0) && (gpsnet_neighbor_ping.NeighborBS[i].GPS_LatX < 65535) && (gpsnet_neighbor_ping.NeighborBS[i].GPS_LongY > 0) && (gpsnet_neighbor_ping.NeighborBS[i].GPS_LongY < 65535)) 
            if ((gpsnet_neighbor_ping.NeighborBS[i].GPS_LatX > 0) && (gpsnet_neighbor_ping.NeighborBS[i].GPS_LongY > 0))
            {
              // Ocupa posição na tabela de base station
              UserEnterCritical();
              index = BaseCnt;
              BaseCnt++;
              UserExitCritical();
              
              BaseStations[index].GPS_LatX  = gpsnet_neighbor_ping.NeighborBS[i].GPS_LatX;
              BaseStations[index].GPS_LongY = gpsnet_neighbor_ping.NeighborBS[i].GPS_LongY;
              BaseStations[index].BaseDepth = ROUTE_TO_BASESTATION_LOST;
              cnt = 2;
              j = index;
            }else
            {
              cnt = 0;
            }
          }
       }
       
       // If the base station is in the base station list
       if (cnt == 2)
       {
          // Se o vizinho é simetrico, verifica profundidade e RSSI
          // para reordenar dinamicamente a profundidade do nó referente a uma determinada base
          gpsnet_neighbourhood[neighbor].BaseDepth[j] = gpsnet_neighbor_ping.NeighborBS[i].BaseDepth;            
          
          #if (DEVICE_TYPE != INSTALLER)
          if (gpsnet_neighbourhood[neighbor].NeighborStatus.bits.Symmetric == TRUE) 
          {
            // Se o depth irá diminuir através deste vizinho, tenta selecioná-lo como parent
            if (gpsnet_neighbourhood[neighbor].BaseDepth[j] < (BaseStations[j].BaseDepth - 1))
            {
              // Para ser o pai de uma conexão o nó deve ter uma conexão com RSSI médio acima de RSSI_PARENT_THRESHOLD
              if (gpsnet_neighbourhood[neighbor].NeighborRSSI > (RSSI_PARENT_THRESHOLD + CFG_PARENT_THRESHOLD))
              {
                BaseStations[j].BaseDepth = (INT8U)(gpsnet_neighbourhood[neighbor].BaseDepth[j] + 1);
                BaseStations[j].NeighborID = gpsnet_neighbourhood[neighbor].Addr_16b;
                BaseStations[j].ParentRSSI = gpsnet_neighbourhood[neighbor].NeighborRSSI;
                BaseStations[j].Par_GPS_Latitude.x = gpsnet_neighbourhood[neighbor].NeighborGPS_Latitude.x;
                BaseStations[j].Par_GPS_Longitude.y = gpsnet_neighbourhood[neighbor].NeighborGPS_Longitude.y;
                
                
                // Acelera o ping para propagar esta informação
                NeighborPingTimeCnt = 1;                
              }else 
              {                  
                  if (BaseStations[j].BaseDepth == ROUTE_TO_BASESTATION_LOST) 
                  {
                    if (GetDepthWatchdog() > DEPTH_TIMEOUT) 
                    {
                      if (gpsnet_neighbourhood[neighbor].NeighborRSSI >= (LOW_PARENT_THRESHOLD + CFG_PARENT_THRESHOLD_MIN)) 
                      {
                        BaseStations[j].BaseDepth = (INT8U)(gpsnet_neighbourhood[neighbor].BaseDepth[j] + 1);
                        BaseStations[j].NeighborID = gpsnet_neighbourhood[neighbor].Addr_16b;
                        BaseStations[j].ParentRSSI = gpsnet_neighbourhood[neighbor].NeighborRSSI;
                        BaseStations[j].Par_GPS_Latitude.x = gpsnet_neighbourhood[neighbor].NeighborGPS_Latitude.x;
                        BaseStations[j].Par_GPS_Longitude.y = gpsnet_neighbourhood[neighbor].NeighborGPS_Longitude.y;
                                
                        // Acelera o ping para propagar esta informação
                        NeighborPingTimeCnt = 1;                    
                      }
                    }
                  }
              }
            } else 
            {              
              // Senão, se achar um vizinho com mesmo depth do parent, seleciona o que tem maior RSSI
              if (gpsnet_neighbourhood[neighbor].BaseDepth[j] == (BaseStations[j].BaseDepth - 1)) 
              {
                 if (gpsnet_neighbourhood[neighbor].NeighborRSSI > BaseStations[j].ParentRSSI) 
                 {
                    //BaseStations[j].BaseDepth = (INT8U)(gps_neighbourhood[neighbor].BaseDepth[j] + 1);
                    BaseStations[j].NeighborID = gpsnet_neighbourhood[neighbor].Addr_16b;
                    BaseStations[j].ParentRSSI = gpsnet_neighbourhood[neighbor].NeighborRSSI; 
                    BaseStations[j].Par_GPS_Latitude.x = gpsnet_neighbourhood[neighbor].NeighborGPS_Latitude.x;
                    BaseStations[j].Par_GPS_Longitude.y = gpsnet_neighbourhood[neighbor].NeighborGPS_Longitude.y;
                                        
                 }
              }
            }
          }
          #endif
       }
    } 
    
    #if (DEVICE_TYPE != INSTALLER)
    UserEnterCritical();
    index = BaseCnt;                        
    UserExitCritical();    
    
    // Search for loss of base station route (due to no symmetric route)
    for (j=0;j<index;j++)
    {
      if (BaseStations[j].NeighborID == gpsnet_neighbourhood[neighbor].Addr_16b)
      {
          if (gpsnet_neighbourhood[neighbor].BaseDepth[j] == ROUTE_TO_BASESTATION_LOST) 
          {
            BaseStations[j].BaseDepth  = ROUTE_TO_BASESTATION_LOST;
            BaseStations[j].NeighborID = 0xFFFF;
            BaseStations[j].ParentRSSI = 0;
            
            // Acelera o ping para propagar esta informação
            NeighborPingTimeCnt = 1;            
            
            // Inicia contador de estabilização de profundidade
            ClearDepthWatchdog();
          }
          else 
          {
            if (gpsnet_neighbourhood[neighbor].NeighborStatus.bits.Symmetric == FALSE) 
            {
              BaseStations[j].ParentRSSI = 0;
            }
          }
      }
    }    
    
    // Find near base station
    BetterDepth = 0xFF;
    for (j=0;j<index;j++)
    {
      if (BaseStations[j].BaseDepth < BetterDepth)
      {
        NearBase = j;
        BetterDepth = BaseStations[j].BaseDepth;
      }
    }    
    #endif
}



 
void HandleNewNeighborPing(void)
{
      INT8U i = 0;
      INT8U j = 0;
      INT8U k = 0;
#if (defined CHECK_DUPLICATE_MAC) && (CHECK_DUPLICATE_MAC == 1)
      INT8U foundme = 0;
#endif
      
      i = NEIGHBOURHOOD_SIZE;
      
      // Verifica se o vizinho já está na tabela
      for(j=0;j<NEIGHBOURHOOD_SIZE;j++)
      {
        if (gpsnet_neighbourhood[j].Addr_16b == gpsnet_neighbor_ping.Addr_16b)
        {
          i = j;
          break;
        }
      }
      
      
      // Se não está na tabela
      if (i == NEIGHBOURHOOD_SIZE) 
      {
          // Procura posição vazia
          for(j=0;j<NEIGHBOURHOOD_SIZE;j++)
          {
            if (gpsnet_neighbourhood[j].Addr_16b == 0xFFFE)
            { 
              i = j;
              k = 1;
              break;
            }
          }      
        
      }
      
      
      // Só grava vizinho se houver posição livre
      if (i<NEIGHBOURHOOD_SIZE)
      {
          gpsnet_neighbourhood[i].Addr_16b      = gpsnet_neighbor_ping.Addr_16b;
          
          // Verifica se o vizinho é novo para calcular média de RSSI
          if (k)
          {
            gpsnet_neighbourhood[i].NeighborRSSI = gpsnet_neighbor_ping.NeighborRSSI;
          }else
          {
            gpsnet_neighbourhood[i].NeighborRSSI = (INT8U)(((gpsnet_neighbourhood[i].NeighborRSSI * 7) + gpsnet_neighbor_ping.NeighborRSSI)>> 3);
          }
          
          gpsnet_neighbourhood[i].NeighborLQI  = gpsnet_neighbor_ping.NeighborLQI;      
          gpsnet_neighbourhood[i].NeighborGPS_LatX  = gpsnet_neighbor_ping.NeighborGPS_LatX;
          gpsnet_neighbourhood[i].NeighborGPS_LongY = gpsnet_neighbor_ping.NeighborGPS_LongY;          
          gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = FALSE;          
          gpsnet_neighbourhood[i].NeighborStatus.bits.RxChannel = (INT8U)(gpsnet_neighbor_ping.NeighborRxChannel >> 4);
          
          // Varre os endereços de vizinhos do ping
          for(j=0;j<gpsnet_neighbor_ping.NeighborsNumber;j++)
          {
            // Se o nó encontra seu endereço nesta lista
            if (gpsnet_neighbor_ping.Neighbors[j] == macAddr)
            {
#if (defined CHECK_DUPLICATE_MAC) && (CHECK_DUPLICATE_MAC == 1)              
              if(++foundme == 2){ // MAC duplicado ?
                 macAddr = (INT16U)(macAddr + RadioRand()); //pequena mudança no MAC address
                 foundme = 0; 
              }
#endif              
              // Verifica se o RSSI de seu sinal e do vizinho estão acima do threshold minimo
              if ((gpsnet_neighbor_ping.NeighborsRSSI[j] >= RSSI_THRESHOLD) && (gpsnet_neighbourhood[i].NeighborRSSI >= RSSI_THRESHOLD))
              {              
                // Se sim, o nó é simetrico
                gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;
              }
                          
            #if (defined GPSR_MW && GPSR_MW==1)

            }else{
                // Verifica quais dos outros vizinhos também são
                // seus vizinhos (isto é usado para implementar a técnica
                // MW (mutual witness) para planarização 
                 for (k=0;k<NEIGHBOURHOOD_SIZE;k++){
                     if(k==i) {continue;} // pula o vizinho que enviou o ping 
                                          
                     // verifica se j-esimo nó do ping também é um vizinho
                     if(gpsnet_neighbourhood[k].Addr_16b == gpsnet_neighbor_ping.Neighbors[j] && gpsnet_neighbourhood[k].NeighborStatus.bits.Symmetric == TRUE && gpsnet_neighbor_ping.NeighborsRSSI[j] >= RSSI_THRESHOLD){
                     // deve ser marcado como vizinho mutuo do nó i 
                        gpsnet_neighbourhood[i].NeighborTblMW = (NEIGHBOR_TABLE_T)(gpsnet_neighbourhood[i].NeighborTblMW | (NEIGHBOR_TABLE_T)(0x01 << k));                       
                     }else{
                     // apaga o bit respectivo
                        gpsnet_neighbourhood[i].NeighborTblMW = (NEIGHBOR_TABLE_T)(gpsnet_neighbourhood[i].NeighborTblMW &~ (NEIGHBOR_TABLE_T)(0x01 << k));                       
                     }
                 } 
            }
            #else
              // não precisa continuar a varredura se encontrou o id na tabela
              // break; // removido para poder verificar MAC address duplicado e MW
            }
            #endif
          }          
          
          UpdateBaseStation(i);
          
          // Informa atividade do nó
          NeighborTable = (NEIGHBOR_TABLE_T)(NeighborTable | (NEIGHBOR_TABLE_T)(0x01 << i));          
      }
      
      // Apaga contagem de estações base do último ping recebido
      gpsnet_neighbor_ping.NeighborBSCnt = 0;
}




// Monta pacote de comando NWK
// o endereço de rede do destino (posição gps) deve ser
// previamente informado nas variáveis:
// dst_gps_lat[4] e dst_gps_long[4]
// o payload do pacote será copiado do vetor NWKPayload
void NWK_Command(INT16U Address, INT8U r_parameter, INT8U payload_size, INT8U packet_life, LATITUDE *better_lat, LONGITUDE *better_long)
{
  INT8U i = 0;
  INT8U FrameIndex = 0;
  INT8U HeaderSize = 0;
  INT8U PayloadSize = 0;
  INT8U tmp = 0;
                            
  // Inicia montagem do pacote NWK Command
  // Inicia montagem do pacote Data p/ roteamento
  
  // Indicação de Beacon no Frame Control
  PHYSetLongRAMAddr(2, 0x61);
  HeaderSize++;
  
  // Indicação de Dest e Source Address de 16b, no Frame Control
  PHYSetLongRAMAddr(3, 0x88);
  HeaderSize++;
 
  
  // Sequence Number
  UserEnterCritical();
  tmp = SequenceNumber;
  UserExitCritical();
  
  PHYSetLongRAMAddr(4, tmp);  
  
  HeaderSize++;
  
  // PanId do coordenador que gera o data packet
  PHYSetLongRAMAddr(5, (INT8U)(macPANId & 0xFF));
  PHYSetLongRAMAddr(6, (INT8U)(macPANId >> 8));
  
  // Endereço de destino do data packet
  PHYSetLongRAMAddr(7, (INT8U)(Address & 0xFF));
  PHYSetLongRAMAddr(8, (INT8U)(Address >> 8));
  
  // Não precisa PANId da fonte pq
  // o pacote é intrapan
      
  // Endereço fonte do data packet
  PHYSetLongRAMAddr(9, (INT8U)(macAddr & 0xFF));
  PHYSetLongRAMAddr(10, (INT8U)(macAddr >> 8));
  HeaderSize += 6;
  FrameIndex = 11;
  
  // Tipo de pacote de dados
  PHYSetLongRAMAddr(FrameIndex, ROUTE_PACKET);
  FrameIndex++;
  PayloadSize++;
  
  // Adiciona os parametros de roteamento
  // Sentido de transmissão
  // Verificação se é o destino do pacote
  // outros
  PHYSetLongRAMAddr(FrameIndex, r_parameter);
  FrameIndex++;
  PayloadSize++;  
                        
  // Adiciona endereço de rede (localização GPS)  
  if (packet_life == 0)
  {
    // Endereço GPS do destino
    for(i=0;i<4;i++)
    {
      UserEnterCritical();
      tmp = dst_gps_lat[i];
      UserExitCritical();
      PHYSetLongRAMAddr(FrameIndex, tmp);
      FrameIndex++;
    }
    
    for(i=0;i<4;i++)
    {
      UserEnterCritical();
      tmp = dst_gps_long[i];
      UserExitCritical();    
      PHYSetLongRAMAddr(FrameIndex, tmp);
      FrameIndex++;
    }

    // Endereço GPS da fonte
    UserEnterCritical();
    tmp = gps_lat.GPS_Degrees;
    UserExitCritical();
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_lat.GPS_Minutes;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_lat.GPS_Seconds;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_lat.GPS_DecSeconds;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;


    UserEnterCritical();
    tmp = gps_long.GPS_Degrees;
    UserExitCritical();
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_long.GPS_Minutes;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_long.GPS_Seconds;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;
    
    UserEnterCritical();
    tmp = gps_long.GPS_DecSeconds;
    UserExitCritical();    
    PHYSetLongRAMAddr(FrameIndex, tmp);
    FrameIndex++;     
       
  }else
  {
    // Copia os endereços de rede de destino e fonte (posição GPS)
        
    // Copia os endereços de rede (posição GPS do nó de destino)
    for(i=0;i<4;i++)
    {
      tmp = nwk_packet.NWK_Dst_Lat[i];
      PHYSetLongRAMAddr(FrameIndex, (INT8U)(tmp));
      FrameIndex++;
    }
    
    for(i=0;i<4;i++)
    {
      tmp = nwk_packet.NWK_Dst_Long[i];
      PHYSetLongRAMAddr(FrameIndex, (INT8U)(tmp));
      FrameIndex++;
    }
    
    // Copia os endereços de rede (posição GPS do nó fonte)
    for(i=0;i<4;i++)
    {
      tmp = nwk_packet.NWK_Src_Lat[i];
      PHYSetLongRAMAddr(FrameIndex, (INT8U)(tmp));
      FrameIndex++;
    }
    
    for(i=0;i<4;i++)
    {
      tmp = nwk_packet.NWK_Src_Long[i];
      PHYSetLongRAMAddr(FrameIndex, (INT8U)(tmp));
      FrameIndex++;
    }    
  }
  
  // Endereço GPS da melhor posição em relação ao destino até este momento
  if (better_lat != NULL) 
  {
    for(i=0;i<4;i++)
    {
      PHYSetLongRAMAddr(FrameIndex, better_lat->bytes[i]);
      FrameIndex++;
    }
    
    for(i=0;i<4;i++)
    {
      PHYSetLongRAMAddr(FrameIndex, better_long->bytes[i]);
      FrameIndex++;
    }    
  }else 
  {
    for(i=0;i<8;i++)
    {
      PHYSetLongRAMAddr(FrameIndex, 0xFF);
      FrameIndex++;
    }      
  }
  
  /////////////////////////////////////////////////////////////////////////////////////////////////
  /* 
  Algoritmo de roteamento utilizado. Acredito que este dado é desnecessário, 
  pois esta informação está nos parâmetros de roteamento
  */
  /////////////////////////////////////////////////////////////////////////////////////////////////
  #if(defined SUPPORT_APP_SEQNUMBER && SUPPORT_APP_SEQNUMBER == 1)
    if(packet_life == 0){
      PHYSetLongRAMAddr(FrameIndex, App_SeqNumber);
      if(++App_SeqNumber == 0) App_SeqNumber = 1;
    }else{
      PHYSetLongRAMAddr(FrameIndex,nwk_packet.NWK_APP_SeqNumber);
    }     
  #else
    PHYSetLongRAMAddr(FrameIndex, 0);
  #endif
  FrameIndex++;  
  /////////////////////////////////////////////////////////////////////////////////////////////////
  
  PayloadSize += 25;
  
  // Adiciona o tempo de vida do pacote
  // ou seja, numero de saltos
  PHYSetLongRAMAddr(FrameIndex, packet_life);
  FrameIndex++;
  PayloadSize++;
  
  if (packet_life == 0)
  {
    for(i=0;i<payload_size;i++)
    {
      PHYSetLongRAMAddr(FrameIndex, NWKPayload[i]);
      FrameIndex++;
      PayloadSize++;
    }  
    
      /* keep stats */
      IncGPSNET_NodeStat_apptxed(); 
        
  }else
  {
    // Copia o payload do pacote anterior
    for(i=0;i<payload_size;i++)
    {
      // Copia somente o payload da mensagem
      // Cabeçalho MAC + Rede           = 11 bytes
      // Endereços de Rede + algoritmo  = 25 bytes
      // Tempo de vida do pacote        =  1 byte
      // Total                          = 37 bytes
      //UserEnterCritical();
      tmp = nwk_packet.NWK_Payload[i];
      //NWKPayload[i] = tmp; // backup do payload do pacote 
      //UserExitCritical();
      PHYSetLongRAMAddr(FrameIndex, (INT8U)(tmp));
      FrameIndex++;
      PayloadSize++;
    }    
  }

  // Informação do tamanho do MAC header em bytes
  // No modo não seguro é ignorado
  PHYSetLongRAMAddr(0x000,HeaderSize);
  
  // Informação do tamanho em bytes do MAC header + Payload
  PHYSetLongRAMAddr(0x001,(INT8U)(HeaderSize+PayloadSize));

  // transmit packet with ACK requested
  // Para solicitar ACK, bit2 = 1
  mac_tasks_pending.bits.PacketPendingAck = 1;
  PHYSetShortRAMAddr(WRITE_TXNMTRIG,0b00000101);
}


INT8U HandleRoutePacket(void)
{
  INT8U i = 0;
  INT8U j = 0;
  INT8U match_count = 0;
  INT8U semaphore_return = 0;
  INT8U attempts = 0;
  INT8U state = 0;  
  INT8U nwk_state ; /* Variável utilizada na máquina de estados de roteamento */
  
  // Verifica se o destino existe na tabela de vizinhos   
  // Inicia máquina de estados para decodificar pacote e realizar roteamento
  nwk_state = start_route;
  
  while(nwk_state != end_route)
  {  
    switch(nwk_state)
    {
      case start_route:
        // Verifica o tempo de vida do pacote
        // Se maior que nwkMaxDepth saltos, descarta o pacote
        if ((INT8U)(nwk_packet.NWK_Packet_Life+1) > nwkMaxDepth)
        {
          nwk_state = end_route;
          state = PACKET_LIFE_ERROR;          
        }else
        {

          #if (USE_REACTIVE_UP_ROUTE == 1)
            // ********************************************************************************
            // Guarda a informação de rota do nó que passou por este roteador
            // Verifica se ja existe este endereço na tabela
            for(i=0;i<ROUTING_UP_TABLE_SIZE;i++)
            {
              match_count = 0;

              for(j=0;j<4;j++)
              {
                if (nwk_packet.NWK_Src_Lat[j] == gps_routing_up_table[i].NeighborGPS_Lat[j])
                  match_count++;
              }

              for(j=0;j<4;j++)
              {
                if (nwk_packet.NWK_Src_Long[j] == gps_routing_up_table[i].NeighborGPS_Long[j])
                  match_count++;
              }
                         
              
              if (match_count == 8) 
              {
                // Para o laço "for" se encontrar o endereço na lista de vizinhos
                // E faz com que a maquina de estados repasse o pacote
                // ao seu destino
                break;
              }
            }
            
            // Se não está na tabela
            if (match_count != 8)
            {
                // Procura posição vazia
                for(i=0;i<ROUTING_UP_TABLE_SIZE;i++)
                {
                  if (gps_routing_up_table[i].Addr_16b == 0xFFFE)
                  { 
                    match_count = 8;
                    break;
                  }
                }      
              
            }          
            
            // Se existe posição na tabela ou estiver atualizando a posição
            if (match_count == 8)
            {
              gps_routing_up_table[i].Addr_16b = mac_packet.SrcAddr_16b;
              if (nwk_packet.NWK_Packet_Life == 0)
              {
                gps_routing_up_table[i].Destination = TRUE;
              }else
              {
                gps_routing_up_table[i].Destination = FALSE;
              }
              for(j=0;j<4;j++)
              {
                gps_routing_up_table[i].NeighborGPS_Lat[j] = nwk_packet.NWK_Src_Lat[j];
              }

              for(j=0;j<4;j++)
              {
                gps_routing_up_table[i].NeighborGPS_Long[j] = nwk_packet.NWK_Src_Long[j];
              }
            }
          #endif
          
          // ********************************************************************************          
          
          // Informa atividade do nó
          for(i=0;i<NEIGHBOURHOOD_SIZE;i++) 
          {            
            if (mac_packet.SrcAddr_16b == gpsnet_neighbourhood[i].Addr_16b) 
            {
                NeighborTable = (NEIGHBOR_TABLE_T)(NeighborTable | (NEIGHBOR_TABLE_T)(0x01 << i));
                gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;
                break;
            }
          }
          
          // Verifica se o nó é o destino do pacote
          if(nwk_packet.NWK_Packet_Type == BROADCAST_PACKET || nwk_packet.NWK_Parameter&NWK_BROADCAST){
            nwk_state = broadcast;
          }else{
            if ((nwk_packet.NWK_Parameter&NWK_DEST) == NWK_DEST)
            {
              nwk_state = call_app_layer;
            } else
            {
              nwk_state = neighbor_table_search;
            }
          }
        }
                
        break;
      case broadcast:
          // repassa pacote
          UpBroadcastRoute((INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD));
          // depois passa uma cópia para a camada de aplicação
          nwk_state = call_app_layer;
      break;
      
      case neighbor_table_search:
        // Verifica se o endereço de destino pertence a um vizinho
        for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
        {
          match_count = 0;

          if (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE) 
          {          
            for(j=0;j<4;j++)
            {
              if (nwk_packet.NWK_Dst_Lat[j] == gpsnet_neighbourhood[i].NeighborGPS_Lat[j])
                match_count++;
            }

            for(j=0;j<4;j++)
            {
              if (nwk_packet.NWK_Dst_Long[j] == gpsnet_neighbourhood[i].NeighborGPS_Long[j])
                match_count++;
            }          
          }
          
          if (match_count == 8) 
          {
            // Parar o laço "for" se encontrar o endereço na lista de vizinhos
            // E fazer com que a maquina de estados repasse o pacote ao seu destino
            break;
          }
        }
        
        if (match_count == 8)
        {
          // Encontrou o destino na lista de vizinhos
          match_count = i;
          attempts = 0;
          nwk_state = send_dest_packet;
        }else
        {
          // Continua o processo de roteamento
          if ((nwk_packet.NWK_Parameter&NWK_DIRECTION) == NOT_DEST_UP)
          {
            nwk_state = route_up;
          }else
          {
            nwk_state = route_down;
          }
        }
        break;

      case route_up:
        // Realiza o roteamento no sentido contrário ao dos coordenadores
        // Roteamento por GPSR
        #if (USE_REACTIVE_UP_ROUTE == 1)
        state = ReactiveUpRoute(IN_PROGRESS_ROUTE,0);
        #else
        state = UpRoute(IN_PROGRESS_ROUTE,0);
        #endif
        nwk_state = end_route;
        break;

      case route_down:
        // Realiza o roteamento no sentido do pan coordinator
        // Roteamento por Node Depth
        #if (DEVICE_TYPE != PAN_COORDINATOR)        
          state = DownRoute(IN_PROGRESS_ROUTE,0);
        #endif
        
        nwk_state = end_route;
        break;
                
      case send_dest_packet:
        // Envia o pacote para o seu destino
        attempts = 0;
        while (attempts < NWK_TX_RETRIES)
        {
          if (attempts < (NWK_TX_RETRIES-1))
          {
            
            NWK_SET_CHANNEL(match_count);
            
            NWK_Command(gpsnet_neighbourhood[match_count].Addr_16b, NWK_DEST, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1),NULL,NULL);
            semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
            
            NWK_RESET_CHANNEL();
            
            if (semaphore_return == OK)
            {
              if (macACK == TRUE) 
              {
                gpsnet_neighbourhood[match_count].NeighborStatus.bits.Symmetric = TRUE;
                
                nwk_state = end_route;
                state = OK;
                // Sai do laço while
                break;
              }else 
              {
                attempts++;
                // Espera tempo de bursting error
                DelayTask((INT16U)(RadioRand()*30));
              }
            } else
            {              
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);    
            }
          }else
          {
            nwk_state = end_route;
            state = ROUTE_ATTEMPTS_ERROR;
            break;
          }
        }
        
        // Increments Packet Sequence ID
        // Used to identify replicated packets
        UserEnterCritical();
        if (++SequenceNumber == 0) SequenceNumber = 1;
        UserExitCritical();        
        
        break;
        
      case call_app_layer:
        // Acorda a tarefa de aplicação e termina o processo de roteamento
        GPSNET_APP();
        nwk_state = end_route;
        state = OK;
        break;

      default:
        // Sai sem fazer nada
        // Erro de formato do pacote
        nwk_state = end_route;
        state = ROUTE_FRAME_ERROR;
        break;                    
    }
  }
  
  return state;
  
}


// Realiza o roteamento no sentido do PAN Coordinator "NearBase"
INT8U DownRoute(INT8U RouteInit, INT8U NWKPayloadSize)
{

  INT8U   i = 0;
  INT8U   selected_node = 0;
  INT8U   attempts = 0;
  INT8U   semaphore_return = 0;
  INT8U   MinorDepth = 255;
  INT8U   MaxRSSI = 0;
  INT16U  BlackList = 0;
  
  
  if (NWKPayloadSize > MAX_APP_PAYLOAD_SIZE){   
       return PAYLOAD_OVERFLOW;
  }
  
  if (BetterDepth > ROUTE_TO_BASESTATION_LOST){
      return NO_ROUTE_AVAILABLE;
  }
  
  
  // Encontra a menor profundidade na tabela de vizinhos
  TryAnotherNodeDown:
   
    MinorDepth = 255;
    MaxRSSI = 0;
    selected_node = 0;
    
    // Varrendo a tabela de vizinhos em busca do nó simétrico de menor profundidade
    for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
    {
      // Sempre que a profundidade do nó for menor que a atualmente selecionada,
      // escolhe o nó como próximo valor de menor profundidade
      // <= devido a um dos nós de mesma profundidade estar na black list
      if ((gpsnet_neighbourhood[i].BaseDepth[NearBase] <= MinorDepth) && (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE))
      {
        // O nó escolhido não deve ter profundidade superior a do nó que está roteando
        if (gpsnet_neighbourhood[i].BaseDepth[NearBase] <= BetterDepth)
        {
          // Verifica se o nó selecionado não está na Black List
          // Se não está na black list, é o próximo nó de menor profundidade
          if ((BlackList & (1 << i)) == 0) 
          {  
            if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE)
            {
              // Verifica se a profundidade diminuiu
              // para procurar pelo melhor RSSI nesta profundidade
              if (gpsnet_neighbourhood[i].BaseDepth[NearBase] < MinorDepth)
              {
                MaxRSSI = 0;
              }              
              
              if (gpsnet_neighbourhood[i].NeighborRSSI >= MaxRSSI)
              {
                selected_node = i;
                MinorDepth = gpsnet_neighbourhood[i].BaseDepth[NearBase];
                MaxRSSI = gpsnet_neighbourhood[i].NeighborRSSI;
              }
            }
          }
        }
      }
    }
    
    // Se não selecionou um nó simétrico para tentativa de roteamento
    if (MinorDepth == 255)
    {
      MaxRSSI = 0;
      selected_node = 0;      
      // Se ainda existe conexão com uma estação base
      if (BetterDepth < ROUTE_TO_BASESTATION_LOST) 
      {
        // Varrendo a tabela de vizinhos em busca do nó de menor profundidade
        for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
        {
          // Sempre que a profundidade do nó for menor que a atualmente selecionada,
          // escolhe o nó como próximo valor de menor profundidade
          // <= devido a um dos nós de mesma profundidade estar na black list
          if (gpsnet_neighbourhood[i].BaseDepth[NearBase] <= MinorDepth)
          {
            // O nó escolhido não deve ter profundidade superior a do nó que está roteando
            if (gpsnet_neighbourhood[i].BaseDepth[NearBase] <= BetterDepth)
            {
              // Verifica se o nó selecionado não está na Black List
              // Se não está na black list, é o próximo nó de menor profundidade
              if ((BlackList & (1 << i)) == 0) 
              {  
                if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE)
                {
                  // Verifica se a profundidade diminuiu
                  // para procurar pelo melhor RSSI nesta profundidade
                  if (gpsnet_neighbourhood[i].BaseDepth[NearBase] < MinorDepth)
                  {
                    MaxRSSI = 0;
                  }              
                  
                  if (gpsnet_neighbourhood[i].NeighborRSSI >= MaxRSSI)
                  {
                    selected_node = i;
                    MinorDepth = gpsnet_neighbourhood[i].BaseDepth[NearBase];
                    MaxRSSI = gpsnet_neighbourhood[i].NeighborRSSI;
                  }
                }
              }
            }
          }
        }      
      }
    }
    
    // Se selecionou um nó para tentativa de roteamento
    if (MinorDepth != 255)
    {      
      attempts = 0;      
      // Tenta rotear o pacote 3 vezes
      while (attempts < NWK_TX_RETRIES)
      {
        if (attempts < (NWK_TX_RETRIES-1))
        {
          NWK_SET_CHANNEL(selected_node);
          
          // Envia pacote a ser roteado
          if (RouteInit != START_ROUTE)
          {
            if (MinorDepth == 0)              
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, DEST_DOWN, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1),NULL,NULL);
            else
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, NOT_DEST_DOWN, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1),NULL,NULL);                        
          }else
          {
            if (MinorDepth == 0)
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, DEST_DOWN, NWKPayloadSize,0,NULL,NULL);
            else
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, NOT_DEST_DOWN, NWKPayloadSize,0,NULL,NULL);
          }
          // Espera confirmação de recepção
          semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
          
          NWK_RESET_CHANNEL();
          
          // Se foi recebido, ok
          if (semaphore_return == OK)
          {
            if (macACK == TRUE) 
            {
              i = OK;
              
              // Informa atividade do nó
              NeighborTable = (NEIGHBOR_TABLE_T)(NeighborTable | (NEIGHBOR_TABLE_T)(0x01 << selected_node));
              gpsnet_neighbourhood[selected_node].NeighborStatus.bits.Symmetric = TRUE;
              
              // Sai do laço while
              break;
            }else 
            {
              attempts++;
              // Espera tempo de bursting error
              DelayTask((INT16U)(RadioRand()*30));
            }
          } else
          {
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
          }
        }else
        {
          // Se estorou o número de tentativas, desiste de rotear por este nó
          i = ROUTE_NODE_ERROR;
          BlackList = (INT16U)(BlackList | (1 << selected_node));
          goto TryAnotherNodeDown;
        }
      }            
    }else
    {
      i = ROUTE_ATTEMPTS_ERROR;
    }

    // Increments Packet Sequence ID
    // Used to identify replicated packets
    UserEnterCritical();
    if (++SequenceNumber == 0) SequenceNumber = 1;
    UserExitCritical();
    
    return i;
}

// Envia mensagens para todos os nós com maior profundidade a 1 salto de distância
INT8U UpSimpleRoute(INT8U NWKPayloadSize)
{
  INT8U i = 0;
  INT8U attempts = 0;
  INT8U semaphore_return = 0;
  
    
  // Varrendo a tabela de vizinhos em busca de nós com maior profundidade
  for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
  {
  
    if(gpsnet_neighbourhood[i].BaseDepth[NearBase] == (BaseStations[NearBase].BaseDepth + 1))
    {
      
      attempts = 0;
      
      // Tenta rotear o pacote NWK_TX_RETRIES vezes
      while (attempts <= NWK_TX_RETRIES)
      {
          NWK_SET_CHANNEL(i);
          
          // Envia pacote a ser roteado
          NWK_Command(gpsnet_neighbourhood[i].Addr_16b, DEST_UP, NWKPayloadSize,0,NULL,NULL);
          // Espera confirmação de recepção
          semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
          
          NWK_RESET_CHANNEL();
          
          // Se foi recebido, ok
          if (semaphore_return == OK)
          {
            if (macACK == TRUE) 
            {
              i = OK;
              gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;
              // Sai do laço while
              break;
            }else 
            {
              attempts++;
              // Espera tempo de bursting error
              DelayTask((INT16U)(RadioRand()*30));              
            }
          } else
          {
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
          }
      }
      // Increments Packet Sequence ID
      // Used to identify replicated packets
      UserEnterCritical();
      if (++SequenceNumber == 0) SequenceNumber = 1;
      UserExitCritical();
    }
    
    return i;
  }

  return i;

}


// Envia mensagens para todos os nós com maior profundidade a 1 salto de distância
INT8U UpBroadcastRoute(INT8U NWKPayloadSize)
{
  INT8U i = 0;
  INT8U attempts = 0;
  INT8U semaphore_return = 0;
  INT8U ret = ROUTE_ATTEMPTS_ERROR;
  
    
  // Varrendo a tabela de vizinhos em busca de nós com maior profundidade
  for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
  {
  
    if(gpsnet_neighbourhood[i].BaseDepth[NearBase] == (BaseStations[NearBase].BaseDepth + 1))
    {
      
      attempts = 0;
      
      // Tenta rotear o pacote NWK_TX_RETRIES vezes
      while (attempts <= NWK_TX_RETRIES)
      {
          NWK_SET_CHANNEL(i);
          
          // Envia pacote a ser roteado
          NWK_Command(gpsnet_neighbourhood[i].Addr_16b, NWK_BROADCAST, NWKPayloadSize,0,NULL,NULL);
          
          // Espera confirmação de recepção
          semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
          
          NWK_RESET_CHANNEL();
          
          // Se foi recebido, ok
          if (semaphore_return == OK)
          {
            if (macACK == TRUE) 
            {
              ret = OK; 
              gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;              
              break;  // Sai do laço while
            }else 
            {
              attempts++;
              // Espera tempo de bursting error
              DelayTask((INT16U)(RadioRand()*30));              
            }
          } else
          {
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
          }
      }
      // Increments Packet Sequence ID
      // Used to identify replicated packets
      UserEnterCritical();
      if (++SequenceNumber == 0) SequenceNumber = 1;
      UserExitCritical();
    }
    
    return ret;
  }

  return ret;
}

#if (USE_REACTIVE_UP_ROUTE == 1)
  // Realiza o roteamento no sentido reverso ao PAN Coordinator
  INT8U ReactiveUpRoute(INT8U RouteInit, INT8U NWKPayloadSize)
  {
    INT8U i = 0;
    INT8U j = 0;
    INT8U attempts = 0;
    INT8U match_count = 0;
    INT8U semaphore_return = 0;


    if (RouteInit == IN_PROGRESS_ROUTE) 
    {
      for(i=0;i<ROUTING_UP_TABLE_SIZE;i++)
      {
        match_count = 0;

        for(j=0;j<4;j++)
        {
          if (nwk_packet.NWK_Dst_Lat[j] == gps_routing_up_table[i].NeighborGPS_Lat[j])
            match_count++;
        }

        for(j=0;j<4;j++)
        {
          if (nwk_packet.NWK_Dst_Long[j] == gps_routing_up_table[i].NeighborGPS_Long[j])
            match_count++;
        }
                   
        
        if (match_count == 8) 
        {
          // Para o laço "for" se encontrar o endereço na lista de vizinhos
          // E faz com que a maquina de estados repasse o pacote
          // ao seu destino
          break;
        }
      }  
    }else
    {
      for(i=0;i<ROUTING_UP_TABLE_SIZE;i++)
      {
        match_count = 0;

        for(j=0;j<4;j++)
        {
          if (dst_gps_lat[j] == gps_routing_up_table[i].NeighborGPS_Lat[j])
            match_count++;
        }

        for(j=0;j<4;j++)
        {
          if (dst_gps_long[j] == gps_routing_up_table[i].NeighborGPS_Long[j])
            match_count++;
        }
                   
        
        if (match_count == 8) 
        {
          // Para o laço "for" se encontrar o endereço na lista de vizinhos
          // E faz com que a maquina de estados repasse o pacote
          // ao seu destino
          break;
        }
      }
    }
      
    
    if (match_count == 8) 
    {
    
        attempts = 0;
        
        // Tenta rotear o pacote NWK_TX_RETRIES vezes
        while (attempts < NWK_TX_RETRIES)
        {
          if (attempts < (NWK_TX_RETRIES-1))
          {
            
            NWK_SET_CHANNEL(i);
            
            // Envia pacote a ser roteado
            
            // Analisa se é o destino do pacote
            // Envia pacote a ser roteado
            if (RouteInit == IN_PROGRESS_ROUTE)
            {
              if (gps_routing_up_table[i].Destination == TRUE)
                NWK_Command(gps_routing_up_table[i].Addr_16b, DEST_UP, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1), NULL, NULL);
              else
                NWK_Command(gps_routing_up_table[i].Addr_16b, NOT_DEST_UP, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1), NULL, NULL);                        
            }else
            {
              if (gps_routing_up_table[i].Destination == TRUE)
                NWK_Command(gps_routing_up_table[i].Addr_16b, DEST_UP, NWKPayloadSize,0, NULL, NULL);
              else
                NWK_Command(gps_routing_up_table[i].Addr_16b, NOT_DEST_UP, NWKPayloadSize,0, NULL, NULL);
            }
            // Espera confirmação de recepção
            semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
            
            NWK_RESET_CHANNEL();
            
            // Se foi recebido, ok
            if (semaphore_return == OK)
            {
              if (macACK == TRUE) 
              {
                i = OK;
                // Sai do laço while
                break;
              }else 
              {
                attempts++;
                // Espera tempo de bursting error
                DelayTask((INT16U)(RadioRand()*30));                              
              }
            } else
            {
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
            }
          }else
          {
            // Se estourou o número de tentativas, desiste de rotear por este nó
            j = ROUTE_NODE_ERROR;
            break;
          }
        }
        
        // Increments Packet Sequence ID
        // Used to identify replicated packets
        UserEnterCritical();
        if (++SequenceNumber == 0) SequenceNumber = 1;
        UserExitCritical();      
    }else
    {
      j = NO_ROUTE_AVAILABLE;
    }
      
    return j;
  }
#endif

INT8U OneHopRoute(INT8U NWKPayloadSize){

    INT8U       i = 0;
    INT8U       attempts = 0;  
    INT8U       semaphore_return = 0;
    INT8U       selected_node = 0;
    INT8U       match_count = 0;       
    
    LATITUDE     dest_lat;
    LONGITUDE    dest_long;
    
    
    dest_lat.x    = dst_gps_latitude.x;
    dest_long.y   = dst_gps_longitude.y;         
        
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ///// Procura por destino na tabela de vizinhos         ////
    ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////// 
      for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
      {
        match_count = 0;

        if (dest_lat.x == gpsnet_neighbourhood[i].NeighborGPS_Latitude.x)
          match_count++;

        if (dest_long.y == gpsnet_neighbourhood[i].NeighborGPS_Longitude.y)
          match_count++;
                   
        
        if (match_count == 2) 
        {
          // Para o laço "for" se encontrar o endereço na lista de vizinhos
          // E faz com que a maquina de estados repasse o pacote
          // ao seu destino
          if (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE)
          {
            selected_node = i;
            break;
          }
        }
      }        
    
      if (match_count != 2)   return NO_ROUTE_AVAILABLE;
    
      // Tenta transmitir o pacote NWK_TX_RETRIES vezes
      while (attempts < NWK_TX_RETRIES)
      {
        if (attempts < (NWK_TX_RETRIES-1))
        {
          
          NWK_SET_CHANNEL(selected_node);
          
          // Envia pacote  
          NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, DEST_UP, NWKPayloadSize,0, NULL, NULL);

          // Espera confirmação de recepção
          semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
          
          NWK_RESET_CHANNEL();
          
          // Se foi recebido, ok
          if (semaphore_return == OK)
          {
            if (macACK == TRUE) 
            {
              i = OK;
              
              // Informa atividade do nó
              NeighborTable = (NEIGHBOR_TABLE_T)(NeighborTable | (NEIGHBOR_TABLE_T)(0x01 << selected_node));                
              gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;
              // Sai do laço while
              break;
            }else 
            {
              attempts++;
              // Espera tempo de bursting error
              DelayTask((INT16U)(RadioRand()*30));                
            }
          } else
          {
            // Radio provavelmente travou, efetuar o reset
            attempts++;

            //  Disable receiving packets off air
            PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
            
            MRF24J40Reset();

            //  Enable receiving packets off air
            PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
          }
        }else
        {
          i = NO_ROUTE_AVAILABLE;
          break;
        }
      }                

      
    // Increments Packet Sequence ID
    // Used to identify replicated packets
    UserEnterCritical();
    if (++SequenceNumber == 0) SequenceNumber = 1;
    UserExitCritical();      
    
    return i;
  
}

// Realiza o roteamento no sentido inverso ao PAN Coordinator
INT8U UpRoute(INT8U RouteInit, INT8U NWKPayloadSize)
{
    INT8U       i = 0;
    INT8U       j = 0;
    INT8U       selected_node = 0;
    INT8U       attempts = 0;
    INT8U       match_count = 0;
    INT8U       semaphore_return = 0;
    INT8U       destination = FALSE;
    INT8U       parameter = NOT_DEST_UP;
    INT8U       valid_edge = 0;    
    INT16U      BlackList = 0;
    INT32U      my_distance = 0;
    INT32U      neighbor_distance = 0;
    INT32U      nearest_distance = 0;
 
    LATITUDE    dest_lat;
    LONGITUDE   dest_long;
    
    LATITUDE    better_lat;
    LONGITUDE   better_long;
    
         
#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)   
    INT32U      dci = 0;
    INT32U      dcj = 0;
    INT32U      dij = 0;

#if (defined TAXICAB_IMPLEMENTATION) && (TAXICAB_IMPLEMENTATION == 1)     
    INT32U      choosen_angle = (8<<20);
    INT32U      last_angle = 0;
#else
    double      choosen_angle = 6.3;
    double      last_angle = 0;
#endif       
    
    #if (DEVICE_TYPE != INSTALLER)    
      LATITUDE  ant_node_lat;    
      LONGITUDE ant_node_long;
    #endif
#endif    

    
    if (NWKPayloadSize > MAX_APP_PAYLOAD_SIZE){         
         return PAYLOAD_OVERFLOW;   // Limite de carga atingido
    }
  
  
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    ///// Copia as informações de nó de destino e de melhor  ////
    ///// localização para o destino até o momento do pacote ////
    ///// a ser roteado.                                     ////
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////            
    if (RouteInit == IN_PROGRESS_ROUTE) 
    {
      // Retira dados do destino do pacote a ser roteado
      for(i=0;i<4;i++)
      {
        dest_lat.bytes[i]   = nwk_packet.NWK_Dst_Lat[i];
        dest_long.bytes[i] = nwk_packet.NWK_Dst_Long[i];
        
#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)         
        // Verifica se está roteando por GPSR
        if ((nwk_packet.NWK_Parameter&NWK_GPSR) == NWK_GPSR) 
        {
          better_lat.bytes[i]   = nwk_packet.NWK_Better_Lat[i];
          better_long.bytes[i] = nwk_packet.NWK_Better_Long[i];
        }
#endif        
      }
    }else 
    {
        // Retira dados do destino da variável global associada a informação
        // no caso de inicio de roteamento
        dest_lat.x    = dst_gps_latitude.x;
        dest_long.y   = dst_gps_longitude.y;
        
#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)        
        better_lat.x  = 0;
        better_long.y = 0;
#endif          
    }
    
    
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ///// Procura por destino na tabela de vizinhos         ////
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////        
    for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
    {
      match_count = 0;

      if (dest_lat.x == gpsnet_neighbourhood[i].NeighborGPS_Latitude.x)
        match_count++;

      if (dest_long.y == gpsnet_neighbourhood[i].NeighborGPS_Longitude.y)
        match_count++;
                 
      
      if (match_count == 2) 
      {
        // Termina o laço "for" se encontrar o endereço na lista de vizinhos
        // E faz com que a maquina de estados repasse o pacote
        // ao seu destino
        if (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE)
        {
          destination = TRUE;
          selected_node = i;
          parameter = DEST_UP;
          break;
        }
      }
    }
      
    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////
    ///// Final da procura por destino na tabela de vizinhos          ////
    ///// destination = TRUE significa destino na tabela de vizinhos  ////
    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////    
    
    
    #if (DEVICE_TYPE != INSTALLER)
    if (destination == FALSE)
    {        
        ///////////////////////////////////////////
        ///////////////////////////////////////////
        ///// Inicio do roteamento geográfico  ////
        ///////////////////////////////////////////
        ///////////////////////////////////////////    
    
        // Calcula distancia do nó atual para o destino
        my_distance = ((dest_lat.x - gps_latitude.x) * (dest_lat.x - gps_latitude.x)) + ((dest_long.y - gps_longitude.y) * (dest_long.y - gps_longitude.y));        
        
#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)         
        if (RouteInit == IN_PROGRESS_ROUTE) 
        {          
          // Verifica se está roteando por GPSR
          if ((nwk_packet.NWK_Parameter&NWK_GPSR) == NWK_GPSR) 
          {
            // Se estiver em GPSR, verifica se a posição atual é melhor que a better position marcado no pacote
            // Caso seja, volta o roteamento para geográfico, se não for, continua GPSR
            
            // Calcula distancia da better position para o destino
            nearest_distance = ((dest_lat.x - better_lat.x) * (dest_lat.x - better_lat.x)) + ((dest_long.y - better_long.y) * (dest_long.y - better_long.y));
            
            // Verifica se a posição atual é mais próxima que a better position
            if (my_distance >= nearest_distance)
            {
              // se não for, continua com GPSR
              goto ContinueGPSRRouting;
            }
          }
        }
#endif                           
              
      TryAnotherNodeUP:
        match_count = 0;
        nearest_distance = my_distance;
        // Varrendo a tabela de vizinhos em busca do próximo salto
        for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
        {
          
          // Desconsidera os nós sem posição
          if (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x == (INT32U)(-1)) {
            continue;
          }
          
          // Escolhe somente vizinhos simetricos
          if (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE)
          {                      
            // Desconsidera o nó instalador
            if (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x != INSTALLER_LAT) 
            {
              // Verifica se o nó selecionado não está na Black List
              if ((BlackList & (1 << i)) == 0) 
              {                          
                // Calcula distancia do vizinho para o destino
                neighbor_distance = ((dest_lat.x - gpsnet_neighbourhood[i].NeighborGPS_Latitude.x) * (dest_lat.x - gpsnet_neighbourhood[i].NeighborGPS_Latitude.x)) + ((dest_long.y - gpsnet_neighbourhood[i].NeighborGPS_Longitude.y) * (dest_long.y - gpsnet_neighbourhood[i].NeighborGPS_Longitude.y));
                
                // Verifica se o vizinho esta mais proximo 
                if (neighbor_distance < nearest_distance)
                {
                  nearest_distance = neighbor_distance;
                  selected_node = i;
                  match_count = 2;
                  parameter = NOT_DEST_UP;
                }
              }
            }
          }
        }

        ////////////////////////////////////////////
        ////////////////////////////////////////////
        ///// Final do roteamento geográfico    ////
        ///// se match_count = 2, vizinho para  ////
        ///// continuar o roteamento encontrado ////
        ////////////////////////////////////////////
        //////////////////////////////////////////// 
           
#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)         
        if (match_count != 2) 
        {
                            
          
          ///////////////////////////////////////////
          ///////////////////////////////////////////
          ///// Inicio do roteamento GPSR        ////
          ///////////////////////////////////////////
          ///////////////////////////////////////////
        ContinueGPSRRouting:  
          match_count = 0;
          // Verifica se entrou agora no GPSR ou se já existia uma better position
          // se entrou agora utiliza o posição atual como better position
          // senão, a better position é a mesma contida no pacote
          if (RouteInit == IN_PROGRESS_ROUTE) 
          {
              // Verifica se não estava roteando por GPSR
              if ((nwk_packet.NWK_Parameter&NWK_GPSR) != NWK_GPSR) 
              {          
                // Se estava roteando por geográfico e entrou em GPSR agora
                // marca a posição atual como better position
                // caso contrário, a variável better position já irá conter o valor correto
                // copiado do pacote no inicio da rotina de roteamento
                better_lat.x = gps_latitude.x;
                better_long.y = gps_longitude.y;
#if 0                
                ant_node_lat.x  = 0;
                ant_node_long.y = 0;
#endif                 
                
              }else{
              
#if 0               
                // Copia a posição do nó anterior (nó pelo qual o pacote chegou) para o cálculo do angulo
                for(j=0;j<NEIGHBOURHOOD_SIZE;j++)
                {
                  if (gpsnet_neighbourhood[j].Addr_16b == mac_packet.SrcAddr_16b)
                  {
                    ant_node_lat.x  = gpsnet_neighbourhood[j].NeighborGPS_Latitude.x;
                    ant_node_long.y = gpsnet_neighbourhood[j].NeighborGPS_Longitude.y;
                    break;
                  }
                }
#endif                    
                
              }
          }else 
          {
            // Entrou em GPSR no início do roteamento
            better_lat.x = gps_latitude.x;
            better_long.y = gps_longitude.y;
#if 0            
            ant_node_lat.x  = 0;
            ant_node_long.y = 0;
#endif            
          }
          
          // Varrendo a tabela de vizinhos para planarizar o grafo
          for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
          {
            // Escolhe somente vizinhos simetricos
            if (gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric == TRUE)
            {          
              // Desconsidera os nós sem posição
              if (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x == (INT32U)(-1)) {
                continue;
              }
              
              // Desconsidera o nó instalador
              if (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x != INSTALLER_LAT) 
              {        				
        				// planarização
        				my_distance = ((gpsnet_neighbourhood[i].NeighborGPS_Latitude.x - gps_latitude.x) * (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x - gps_latitude.x)) + ((gpsnet_neighbourhood[i].NeighborGPS_Longitude.y - gps_longitude.y) * (gpsnet_neighbourhood[i].NeighborGPS_Longitude.y - gps_longitude.y));
        				//my_distance = SquareRoot(my_distance);
        				//dci = my_distance * my_distance;
        				dci = my_distance;
        				
        				valid_edge = 1;  
        				
        				for(j=0;j<NEIGHBOURHOOD_SIZE;j++)
        				{ 
                  // Desconsidera o próprio nó
                  if(i==j)
                  { continue;
                  }
                  // Desconsidera o nó instalador
                  if (gpsnet_neighbourhood[j].NeighborGPS_Latitude.x == INSTALLER_LAT) 
                  { continue;
                  }
                  // Desconsidera os nós sem posição
                  if (gpsnet_neighbourhood[j].NeighborGPS_Latitude.x == (INT32U)(-1)) 
                  { continue;
                  }
                  
                  // Escolhe somente vizinhos simetricos
                  if(gpsnet_neighbourhood[j].NeighborStatus.bits.Symmetric == FALSE) 
                  { continue;
                  }                  
                  
/*        				
        					if (gpsnet_neighbourhood[i].Addr_16b != gpsnet_neighbourhood[j].Addr_16b)
        					{
*/        					
                  my_distance = ((gps_latitude.x - gpsnet_neighbourhood[j].NeighborGPS_Latitude.x) * (gps_latitude.x - gpsnet_neighbourhood[j].NeighborGPS_Latitude.x)) + ((gps_longitude.y - gpsnet_neighbourhood[j].NeighborGPS_Longitude.y) * (gps_longitude.y - gpsnet_neighbourhood[j].NeighborGPS_Longitude.y));
                  //my_distance = SquareRoot(my_distance);
                  //dcj = my_distance * my_distance;
                  dcj = my_distance;
                  
                  my_distance = ((gpsnet_neighbourhood[i].NeighborGPS_Latitude.x - gpsnet_neighbourhood[j].NeighborGPS_Latitude.x) * (gpsnet_neighbourhood[i].NeighborGPS_Latitude.x - gpsnet_neighbourhood[j].NeighborGPS_Latitude.x)) + ((gpsnet_neighbourhood[i].NeighborGPS_Longitude.y - gpsnet_neighbourhood[j].NeighborGPS_Longitude.y) * (gpsnet_neighbourhood[i].NeighborGPS_Longitude.y - gpsnet_neighbourhood[j].NeighborGPS_Longitude.y));
                  //my_distance = SquareRoot(my_distance);
      				    //dij = my_distance * my_distance;    						
      				    dij = my_distance;
      						
      						if (dci > (dcj + dij))
      						{
      							
      							#if (defined GPSR_MW) && (GPSR_MW==1)
      						  // verifica se j e i são vizinhos mútuos antes de remover este enlace
                    if((gpsnet_neighbourhood[i].NeighborTblMW & (0x01<<j) == 0x01) && 
                    (gpsnet_neighbourhood[j].NeighborTblMW & (0x01<<i) == 0x01))         							
      							{
      							#endif
      							  
      						    valid_edge = 0;
      							  break;
      							    
      							#if (defined GPSR_MW) && (GPSR_MW==1)    
      							}  
      							#endif        							
      							
      						}
/*        						
        					}
*/
        					
        				}
        				
                if (valid_edge == 1)
                {

/* pode ser retirado do laço, pois so precisa ser feito uma unica vez */                	
#if 1                	
                	if (RouteInit == IN_PROGRESS_ROUTE) 
                	{
                    // Copia a posição do nó anterior (nó pelo qual o pacote chegou) para o cálculo do angulo
                    for(j=0;j<NEIGHBOURHOOD_SIZE;j++)
                    {
                      if (gpsnet_neighbourhood[j].Addr_16b == mac_packet.SrcAddr_16b)
                      {
                        ant_node_lat.x  = gpsnet_neighbourhood[j].NeighborGPS_Latitude.x;
                        ant_node_long.y = gpsnet_neighbourhood[j].NeighborGPS_Longitude.y;
                        break;
                      }
                    }        					
                	}else 
                	{
                	  ant_node_lat.x  = 0;
                	  ant_node_long.y = 0;
                	}
#endif                	
                	
                	// calculo do angulo GPSR
#if (defined TAXICAB_IMPLEMENTATION) && (TAXICAB_IMPLEMENTATION == 1)             	
                  last_angle = diferenca_ang_taxicab(gps_latitude, gps_longitude, gpsnet_neighbourhood[i].NeighborGPS_Latitude, gpsnet_neighbourhood[i].NeighborGPS_Longitude, &ant_node_lat.x, &ant_node_long.y, &dest_lat.x, &dest_long.y);
#else
                  last_angle = diferenca_angulo(gps_latitude, gps_longitude, gpsnet_neighbourhood[i].NeighborGPS_Latitude, gpsnet_neighbourhood[i].NeighborGPS_Longitude, &ant_node_lat.x, &ant_node_long.y, &dest_lat.x, &dest_long.y);
#endif            
                	if (last_angle < choosen_angle)
                	{
                		selected_node = i;
                		choosen_angle = last_angle;
                		match_count = 2;
                		parameter = NOT_DEST_UP | NWK_GPSR;
                	}	    				
                }   				
              }
            }
          }                          
          ////////////////////////////////////////////
          ////////////////////////////////////////////
          ///// Final do roteamento GPSR          ////
          ///// se match_count = 2, vizinho para  ////
          ///// continuar o roteamento encontrado ////
          ////////////////////////////////////////////
          ////////////////////////////////////////////          
        }
#endif
        
    }
    #endif
           
    // Tenta transmitir
    if (match_count == 2) 
    {    
        attempts = 0;
        
        // Tenta rotear o pacote 3 vezes
        while (attempts < NWK_TX_RETRIES_UP)
        {
          if (attempts < (NWK_TX_RETRIES_UP-1))
          {
            
            NWK_SET_CHANNEL(selected_node);
            
            // Envia pacote a ser roteado            
            // Analisa se é o destino do pacote
            // Envia pacote a ser roteado
            if (RouteInit == IN_PROGRESS_ROUTE)
            {
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, parameter, (INT8U)(mac_packet.Payload_Size - NWK_OVERHEAD),(INT8U)(nwk_packet.NWK_Packet_Life+1), &better_lat, &better_long);
            }else
            {
              NWK_Command(gpsnet_neighbourhood[selected_node].Addr_16b, parameter, NWKPayloadSize,0, &better_lat, &better_long);
            }
            // Espera confirmação de recepção
            semaphore_return = OSSemPend(RF_TX_Event,(INT16U)(TX_TIMEOUT+RadioRand()));
            
            NWK_RESET_CHANNEL();
            
            // Se foi recebido, ok
            if (semaphore_return == OK)
            {
              if (macACK == TRUE) 
              {
                i = OK;
                
                // Informa atividade do nó
                NeighborTable = (NEIGHBOR_TABLE_T)(NeighborTable | (NEIGHBOR_TABLE_T)(0x01 << selected_node));                
                gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric = TRUE;
                // Sai do laço while
                break;
              }else 
              {
                attempts++;
                // Espera tempo de bursting error
                DelayTask((INT16U)(RadioRand()*30));                
              }
            } else
            {
              // Radio provavelmente travou, efetuar o reset
              attempts++;

              //  Disable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x04);              
              
              MRF24J40Reset();

              //  Enable receiving packets off air
              PHYSetShortRAMAddr(WRITE_BBREG1,0x00);
            }
          }else
          {
            #if (DEVICE_TYPE != INSTALLER)
            
              // Se estorou o número de tentativas, desiste de rotear por este nó
              if (destination == TRUE) 
              {
                // nao permite mais a entrega direta para o destino (será retirado pela black list)
                // irá tentar por um nó mais próximo
                destination = FALSE;
              }              
              
              BlackList = (INT16U)(BlackList | (1 << selected_node));
              
              // Tenta outro nó se estiver em roteamento geográfico
              // se estiver em GPSR, desiste !!!
              if ((parameter&NWK_GPSR) == NWK_GPSR) 
              {
                i = NO_ROUTE_AVAILABLE;
                break;
              }
              goto TryAnotherNodeUP;
            
            #else
              i = NO_ROUTE_AVAILABLE;
              break;
            #endif
          }
        }                
        
    }else
    {
      i = NO_ROUTE_AVAILABLE;
    }
      
    // Incrementa Packet Sequence ID
    // Usado para identificar pacotes replicados
    UserEnterCritical();
      if (++SequenceNumber == 0) SequenceNumber = 1;
    UserExitCritical();      
    
    return i;
}








