/*********************************************************************************************************
*                                               BRTOS
*                                Brazilian Real-Time Operating System
*                            Acronymous of Basic Real-Time Operating System
*
*                              
*                                  Open Source RTOS under MIT License
*
*
*
*                                             Network defines
*
*                                     
*   Author: Gustavo Weber Denardin
*   Revision: 1.0
*   Date:     20/03/2009
*
*********************************************************************************************************/

#ifndef GPSNET_CONFIG_H
#define GPSNET_CONFIG_H

#include "BRTOS.h"

/// Define network support
#define NETWORK_ENABLE                      1

// Tipos de nós na rede
#define   PAN_COORDINATOR                   0
#define   ROUTER                            1
#define   INSTALLER                         2

// Escolhe o tipo de nó na rede
#define   DEVICE_TYPE                       PAN_COORDINATOR

// Alinhamento de memória dos pacotes para processadores de 8 bits
#define CPU_32_BITS                         1
#define CPU_16_BITS                         0
#define CPU_8_BITS                          0

// Reactive up route - 1 = on, 0 = off
#define USE_REACTIVE_UP_ROUTE               0

// Prioridade das tarefas de rede
// nao mudar!
#define RF_EventHandlerPriority     (INT8U)31
#define GPSNET_Mutex_Priority       (INT8U)28 
#define MAC_HandlerPriority         (INT8U)23
#define NWK_HandlerPriority         (INT8U)24
#define APP1_Priority               (INT8U)26
#define APP2_Priority               (INT8U)25
#define APP3_Priority               (INT8U)27

// APPs signals 
#define SIGNAL_APP1                 App1_event

// GPSNET Tasks Stacks
#if ((DEVICE_TYPE == PAN_COORDINATOR) || (DEVICE_TYPE == INSTALLER))
#define GPSNET_RF_Event_StackSize    (384)
#define GPSNET_MAC_StackSize         (384)
#define GPSNET_NWK_StackSize         (1280)
#else
#define GPSNET_RF_Event_StackSize    (288)
#define GPSNET_MAC_StackSize         (288)
#define GPSNET_NWK_StackSize         (1088)
#endif

// Init of the GPSNetwork tasks (used in the mutex)
#define GPSNET_TASKS_INIT 6

// Number os GPSNetwork tasks (used in the mutex)
#define GPSNET_TASKS      4

#define BRTOS_MUTEX       0
#define EMULATED_MUTEX    1
#define NWK_MUTEX_TYPE    BRTOS_MUTEX


/// RF Buffer Size
#define RFBufferSize      (INT16U)768


/// Memory locations for network address and configurations
#if (DEVICE_TYPE == PAN_COORDINATOR)
  #define LAT_MEM_ADDRESS    0x0001FC00
  #define LON_MEM_ADDRESS    0x0001FC04
  #define MAC16_MEM_ADDRESS
  #define MAC64_MEM_ADDRESS  0x00001FF0
  #define PANID_MEM_ADDRESS
  #define PANID_INIT_VALUE   0x4742
  #define MAC16_INIT_VALUE   0x0000
  #define ROUTC_INIT_VALUE   0x01    
#else 
  #define LAT_MEM_ADDRESS    0x000021C0
  #define LON_MEM_ADDRESS    0x000021C4
  #define MAC16_MEM_ADDRESS  0x000021C8
  #define MAC64_MEM_ADDRESS  0x00001FF0
  #define PANID_MEM_ADDRESS  0x000021CC
  #define PANID_INIT_VALUE   0xFFFF
  #define MAC16_INIT_VALUE   0xFFFF
  #define ROUTC_INIT_VALUE   0x00    
#endif


//IEEE EUI - globally unique number
#define EUI_7 0x00
#define EUI_6 0x02
#define EUI_5 0x02
#define EUI_4 0x03
#define EUI_3 0x04
#define EUI_2 0x05
#define EUI_1 0x06
#define EUI_0 0x07


#define GPSNET_PANID_TEST   0

#if (defined GPSNET_PANID_TEST && GPSNET_PANID_TEST == 1)
#if (defined PANID_INIT_VALUE)
   #undef  PANID_INIT_VALUE
   #define PANID_INIT_VALUE      0x4843
#endif
#endif

/// Configuration for tests only
//#define GPSNET_TEST_POSITION

#define MULTICHANNEL_SUPPORT 1
#define NUM_ALLOWED_CHANNELS 4

#define FORCE_NO_MULTICHANNEL_SUPPORT 0

#endif


