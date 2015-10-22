/**********************************************************************************
@file   network.h
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

#ifndef GPSNET_NWK_H
#define GPSNET_NWK_H

#include "NetConfig.h"

// GPS address (node position)
#if (DEVICE_TYPE == PAN_COORDINATOR)
  #define LAT_GPS_DEG      0
  #define LAT_GPS_MIN      0
  #define LAT_GPS_SEC      0
  #define LAT_GPS_DECSEC   1

  #define LONG_GPS_DEG     0            
  #define LONG_GPS_MIN     0
  #define LONG_GPS_SEC     0
  #define LONG_GPS_DECSEC  1
#elif (DEVICE_TYPE == INSTALLER)
  #define LAT_GPS_DEG      0xAA
  #define LAT_GPS_MIN      0xBB
  #define LAT_GPS_SEC      0xCC
  #define LAT_GPS_DECSEC   0xDD               

  #define LONG_GPS_DEG     0xDD            
  #define LONG_GPS_MIN     0xCC
  #define LONG_GPS_SEC     0xBB
  #define LONG_GPS_DECSEC  0xAA
#else

#ifndef GPSNET_TEST_POSITION
  #define LAT_GPS_DEG      0xFF
  #define LAT_GPS_MIN      0xFF
  #define LAT_GPS_SEC      0xFF
  #define LAT_GPS_DECSEC   0xFF               

  #define LONG_GPS_DEG     0xFF            
  #define LONG_GPS_MIN     0xFF
  #define LONG_GPS_SEC     0xFF
  #define LONG_GPS_DECSEC  0xFF
#else
  #define LAT_GPS_DEG      TEST_LAT_GPS_DEG
  #define LAT_GPS_MIN      TEST_LAT_GPS_MIN
  #define LAT_GPS_SEC      TEST_LAT_GPS_SEC
  #define LAT_GPS_DECSEC   TEST_LAT_GPS_DECSEC               

  #define LONG_GPS_DEG     TEST_LONG_GPS_DEG            
  #define LONG_GPS_MIN     TEST_LONG_GPS_MIN
  #define LONG_GPS_SEC     TEST_LONG_GPS_SEC
  #define LONG_GPS_DECSEC  TEST_LONG_GPS_DECSEC
#endif

#endif

// Installer position
#define INSTALLER_LAT             0xAABBCCDD
#define INSTALLER_LNG             0xDDCCBBAA

// Number of base stations (max)
#define MAX_BASE_STATION     6

// Base station defines
#define ROUTE_TO_BASESTATION_LOST (INT8U)0xFE
#define NO_ROUTE_TO_BASESTATION   (INT8U)0xFF

// This timeout is based on the system timer 
// and is used to control the packets last ID timeout
#define LAST_ID_SYSTEM_TIMER_TIMEOUT  (INT8U)8

// Data packet types
#define DATA_PING            (INT8U)0x01
#define BROADCAST_PACKET     (INT8U)0x02
#define ROUTE_PACKET         (INT8U)0x03
#define ADDRESS_PACKET       (INT8U)0x04

// Routing Errors
#define PACKET_LIFE_ERROR    (INT8U)0x05
#define ROUTE_ATTEMPTS_ERROR (INT8U)0x06
#define ROUTE_FRAME_ERROR    (INT8U)0x07
#define ROUTE_NODE_ERROR     (INT8U)0x08
#define NO_ROUTE_AVAILABLE   (INT8U)0x09
#define PAYLOAD_OVERFLOW     (INT8U)0x10


// Routing algorithms
#define NWK_DEST             (INT8U)0b0001
#define NWK_DIRECTION        (INT8U)0b0010
#define NWK_GPSR             (INT8U)0b0100
//#define NWK_CDF              (INT8U)0b1000
#define NWK_BROADCAST        (INT8U)0b1000

#define GPSR_ENABLE          1  

#if (defined GPSR_ENABLE && GPSR_ENABLE ==1)
  #define GPSR_MW                1
  #define TAXICAB_IMPLEMENTATION 1
#endif

#define DEST_UP              (INT8U)0b0001
#define DEST_DOWN            (INT8U)0b0011
#define NOT_DEST_UP          (INT8U)0b0000
#define NOT_DEST_DOWN        (INT8U)0b0010

                                               
// Routes
#define DOWN_ROUTE           (INT8U)0x10      
#define UP_ROUTE             (INT8U)0x20

#define START_ROUTE          (INT8U)0x00
#define IN_PROGRESS_ROUTE    (INT8U)0x01

// Maximum size of the neighbourhood table
#define NEIGHBOURHOOD_SIZE      (INT8U)16
typedef INT16U                  NEIGHBOR_TABLE_T;
#define ROUTING_UP_TABLE_SIZE   (INT8U)8

/* Link reliability parameters */

//#define RSSI_THRESHOLD          (INT8U)4
//#define LOW_PARENT_THRESHOLD    (INT8U)8
//#define RSSI_PARENT_THRESHOLD   (INT8U)20

#define RSSI_THRESHOLD          (INT8U)0
#define LOW_PARENT_THRESHOLD    (INT8U)10
#define RSSI_PARENT_THRESHOLD   (INT8U)20

/* Nwk Tx retries */
#define NWK_TX_RETRIES          (INT8U)4

/* Nwk Tx retries */
#define NWK_TX_RETRIES_UP       (INT8U)(NWK_TX_RETRIES+6)

// Set RX buffer control
#define AUTO_ACK_CONTROL        0

// Depth Watchdog Timeout in msec
#define DEPTH_TIMEOUT           (INT16U)20000

/* Overhead of NWK layer in bytes */
#define NWK_OVERHEAD            (INT8U)28

/* Support for app packet sequence numbering */
// #define SUPPORT_APP_SEQNUMBER   1

typedef union _NWK_TASKS_PENDING
{
    INT8U Val;
    struct _nwk_bits
    {
        INT8U DataPingPending             :1;
        INT8U VerifyNeighbourhoodTable    :1;
        INT8U NewNeighborPing             :1;
        INT8U RadioReset                  :1;
        INT8U RoutePending                :1;
        INT8U NewAddressArrived           :1;
        INT8U BroadcastPending            :1;
    } bits;
} NWK_TASKS_PENDING;


typedef union _dword
{
  unsigned char   int8u[4];
  unsigned short  int16u[2];
  unsigned long   int32u;  
} UINT_DWORD; 
 


typedef struct _GPS_LAT
{
    INT8U   GPS_Degrees;
    INT8U   GPS_Minutes;
    INT8U   GPS_Seconds;
    INT8U   GPS_DecSeconds;
} GPS_LAT;


typedef struct _GPS_LONG
{
    INT8U   GPS_Degrees;
    INT8U   GPS_Minutes;
    INT8U   GPS_Seconds;
    INT8U   GPS_DecSeconds;
} GPS_LONG;


typedef union _LATITUDE
{
  GPS_LAT   gps_lat_u;
  INT8U     bytes[4];
  INT32S    x;
} LATITUDE;


typedef union _LONGITUDE
{
  GPS_LONG  gps_long_u;
  INT8U     bytes[4];
  INT32S    y;
} LONGITUDE;

typedef union _POSITION{
   INT8U bytes[8];   
   struct{   
      LATITUDE lat;
      LONGITUDE lng;
   }pos;   
}POSITION;



#define NeighborGPS_Lat   NeighborGPS_Latitude.bytes
#define NeighborGPS_Long  NeighborGPS_Longitude.bytes

#define NeighborGPS_LatX  NeighborGPS_Latitude.x
#define NeighborGPS_LongY NeighborGPS_Longitude.y

#define GPS_Lat           GPS_Latitude.bytes
#define GPS_Long          GPS_Longitude.bytes

#define GPS_LatX          GPS_Latitude.x
#define GPS_LongY         GPS_Longitude.y

#define gps_lat           gps_latitude.gps_lat_u  
#define gps_long          gps_longitude.gps_long_u

#define gps_latX          gps_latitude.x
#define gps_longY         gps_longitude.y

#define dst_gps_lat       dst_gps_latitude.bytes  
#define dst_gps_long      dst_gps_longitude.bytes

#define dst_gps_latX      dst_gps_latitude.x
#define dst_gps_longY     dst_gps_longitude.y

typedef union _NEIGHBOR_STATUS
{
    INT8U Val;
    struct 
    {
        INT8U Symmetric             :1;
        INT8U RxAllowed             :1;        
        INT8U TxPending             :1;
        INT8U Active                :1;
        INT8U RxChannel             :4;
    } bits;
} NB_STATUS;

typedef struct _GPSNET_NEIGHBOURHOOD
{
    INT16U        Addr_16b;                   // 16 bit address from neighbor
    //INT8U         IndexRSSI;                // Index of the Neighbor signal quality vector
    INT8U         NeighborRSSI;               // Average of the Neighbor signal quality     
    NB_STATUS     NeighborStatus;             // Neighbor status
    INT8U         NeighborLQI;                // Neighbor link quality indicator
    INT8U         NeighborLastID;             // Last message ID used for this neighbor
    INT8U         IDTimeout;                  // Timeout to delete info of the Last message
    LATITUDE      NeighborGPS_Latitude;       // Neighbor GPS position
    LONGITUDE     NeighborGPS_Longitude;
    INT8U         BaseDepth[MAX_BASE_STATION];
#if (defined GPSR_MW) && (GPSR_MW==1)    
    NEIGHBOR_TABLE_T NeighborTblMW;            // Mutual witness table for GSPR
#endif    
    //INT8U         Vector_RSSI[8];
} GPSNET_NEIGHBOURHOOD;

typedef struct _GPSNET_BASESTATION
{
    INT8U         BaseDepth;                  // Depth to the base station
    INT8U         ParentRSSI;                 // RSSI of the current parent neighbor
    INT16U        NeighborID;                 // ID of the neighbor that generates this route
    LATITUDE      GPS_Latitude;               // Base station GPS position
    LONGITUDE     GPS_Longitude;
    LATITUDE      Par_GPS_Latitude;           // Position of Parent to this Base station
    LONGITUDE     Par_GPS_Longitude;       
} GPSNET_BASESTATION;


typedef struct _GPSNET_SYMMETRIC_NEIGHBOURHOOD
{
    INT16U          Addr_16b;                           // 16 bit address from neighbor
    INT8U           NeighborRSSI;                       // Neighbor signal quality    
    INT8U           NeighborBSCnt;                      // Numero de base stations do vizinho 
    GPSNET_BASESTATION NeighborBS[MAX_BASE_STATION];    // Tabela de base stations do vizinho            
    LATITUDE        NeighborGPS_Latitude;               // Neighbor GPS position
    LONGITUDE       NeighborGPS_Longitude;
    INT16U          Neighbors[NEIGHBOURHOOD_SIZE];      // Vizinhos do nó que enviou o ping
    INT8U           NeighborsRSSI[NEIGHBOURHOOD_SIZE];  // Numero de vizinhos no nó que enviou o ping
    INT8U           NeighborsNumber;                    // Numero de vizinhos no nó que enviou o ping
    INT8U           NeighborLQI;
    INT8U           NeighborRxChannel;                  // Rx channel
} GPSNET_SYMMETRIC_NEIGHBOURHOOD; 


typedef struct _GPSNET_ROUTING_UP_TABLE
{
    INT16U        Addr_16b;                   // 16 bit address from neighbor
    INT8U         Destination;
    LATITUDE      NeighborGPS_Latitude;       // Neighbor GPS position
    LONGITUDE     NeighborGPS_Longitude;
} GPSNET_ROUTING_UP_TABLE;



/* Function Prototypes */

void NeighborPing(void);
void HandleNewNeighborPing(void);
INT8U HandleRoutePacket(void);
void VerifyNeighbourhood(void);
void VerifyNeighbourhoodLastIDTimeout(void);
INT8U VerifyPacketReplicated(void);
void UpdateBaseStation(INT8U neighbor);
void NWK_Command(INT16U Address, INT8U r_parameter, INT8U payload_size, INT8U packet_life, LATITUDE *better_lat, LONGITUDE *better_long);

void VerifyNewAddress(void);
void GPSAddressMessage(INT8U *latitude, INT8U *longitude);
void GPSAddressMessageToMAC(INT8U *latitude, INT8U *longitude, INT16U mac16);

void IncDepthWatchdog(void);
INT16U GetDepthWatchdog(void);
void ClearDepthWatchdog(void);

void acquireRadio(void);
void releaseRadio(void);
void init_radio_resource(INT8U priority);

INT8U DownRoute(INT8U RouteInit, INT8U AppPayloadSize);
INT8U UpRoute(INT8U RouteInit, INT8U AppPayloadSize);
INT8U UpSimpleRoute(INT8U AppPayloadSize);
INT8U UpBroadcastRoute(INT8U NWKPayloadSize);
INT8U OneHopRoute(INT8U NWKPayloadSize);

/* GPSNET API */
#if 0
INT8U GPSNET_TX_toBase(INT8U *ptr_data, INT8U nbr_bytes);
INT8U GPSNET_TX_toAll(INT8U *ptr_data, INT8U nbr_bytes);
INT8U GPSNET_TX_toChildren(INT8U *ptr_data, INT8U nbr_bytes);
INT8U GPSNET_TX_toAddress(INT8U *ptr_data, INT8U nbr_bytes, POSITION* address);
#endif


#if (USE_REACTIVE_UP_ROUTE == 1)
INT8U ReactiveUpRoute(INT8U RouteInit, INT8U AppPayloadSize);
#endif




// Routing states
enum nwk_packet_state
{
    start_route,              
    neighbor_table_search,
    broadcast,
    route_up,
    route_down,
    send_dest_packet,
    call_app_layer,
    end_packet_life,
    end_route
};



#if (USE_REACTIVE_UP_ROUTE == 1)
extern  volatile GPS_ROUTING_UP_TABLE        gps_routing_up_table[ROUTING_UP_TABLE_SIZE];
#endif

/* Moved to NetConfig.h file */
/* A copy is left here for reference only */
#ifdef REMOVED
  #if (DEVICE_TYPE == PAN_COORDINATOR)
    extern  const unsigned char Latitude[4]  @0x0001FC00;
    extern  const unsigned char Longitude[4] @0x0001FC04;
  #else
    extern  const unsigned char Latitude[4] @0x000021C0;
    extern  const unsigned char Longitude[4] @0x000021C4;
  #endif
#endif

#if PROCESSOR == COLDFIRE_V1
#if ! (defined LAT_MEM_ADDRESS || defined LON_MEM_ADDRESS)
#error "Please define 'LAT_MEM_ADDRESS' and 'LON_MEM_ADDRESS' in NetConfig.h file"
#else
extern  const unsigned char Latitude[4]  @LAT_MEM_ADDRESS;
extern  const unsigned char Longitude[4] @LON_MEM_ADDRESS;
#endif
#else
//#define Latitude         	((INT8U *) LAT_MEM_ADDRESS)
//#define Longitude					((INT8U *) LON_MEM_ADDRESS)
//extern const unsigned char Latitude[4];
//extern const unsigned char Longitude[4];
extern unsigned char *Latitude;
extern unsigned char *Longitude;
#endif

extern  volatile GPSNET_NEIGHBOURHOOD             gpsnet_neighbourhood[NEIGHBOURHOOD_SIZE];
extern  volatile GPSNET_SYMMETRIC_NEIGHBOURHOOD   gpsnet_neighbor_ping;
extern  volatile GPSNET_BASESTATION               BaseStations[MAX_BASE_STATION];
extern  volatile NEIGHBOR_TABLE_T                 NeighborTable;

extern  volatile INT8U               BaseCnt;
extern  volatile INT8U               NearBase;
extern  volatile INT8U               BetterDepth;

extern  volatile LATITUDE            gps_latitude;
extern  volatile LONGITUDE           gps_longitude;
extern  volatile LATITUDE            dst_gps_latitude;
extern  volatile LONGITUDE           dst_gps_longitude;

//extern  volatile INT16U              AssociateTimeout;
//extern  volatile INT16U              NeighborCnt;
//extern  volatile INT32U              NeighbourhoodCnt;

extern  volatile INT16U              RadioWatchdog;
extern  volatile INT16U              NeighborPingTimeV;
extern  volatile INT8U               NeighborPingTimeCnt;
extern  volatile NWK_TASKS_PENDING   nwk_tasks_pending;


/* multichannel support **/
#if (defined MULTICHANNEL_SUPPORT &&  MULTICHANNEL_SUPPORT == 1)
  #define NWK_SET_CHANNEL(x)    SetChannel((INT8U)(gpsnet_neighbourhood[x].NeighborStatus.bits.RxChannel << 4));
  #define NWK_RESET_CHANNEL()   SetChannel(GetChannel());
#else
  #define NWK_SET_CHANNEL(x)
  #define NWK_RESET_CHANNEL()
#endif

#endif
