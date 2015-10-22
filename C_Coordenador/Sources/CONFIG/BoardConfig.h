#include "hardware.h"

/* Configurations of hardware drivers */

/************************/
/* EEPROM Config     ****/
#define FLASH_SUPPORTED       1
#define EEPROM_ADDRESS        0x0001FC00
/********************/


/************************/
/* LED HeartBeat        */
/*******************************/
/* Led heartbeat               */

#ifndef OLD_BOARD

#define LED_HEARTBEAT_INIT()     PTGDS_PTGDS1 = 1; \
                                 PTGDD_PTGDD1 = 1; \
                                 PTGD_PTGD1  = 1;
#define LED_HEARTBEAT_TOGGLE()   PTGD_PTGD1 = ~PTGD_PTGD1; 

#else /* old board config */


#define LED_HEARTBEAT_INIT()      PTCDS = 0x03; \
                                  PTCDD = 0x03; \
                                  PTCD  = 0x01; \
                                  
#define LED_HEARTBEAT_TOGGLE()    PTCD_PTCD1 = ~PTCD_PTCD1; 
                                
#endif

/************************/
/* UART number          */

#define   UART    1 
/************************/

/*******************************************/
/* Radio driver configuration */

// Power levels
#define RFTX_0dB    0x00
#define RFTX_m10dB  0x40
#define RFTX_m20dB  0x80
#define RFTX_m30dB  0xC0
#define RFTX_m36dB  0xF8

#ifndef OLD_BOARD

// Transceiver Configuration
#define RF_INT_TIMER     0
#define RF_INT_KEYB      1
#define RF_INT_TYPE      RF_INT_KEYB
#define RFIF             KBI1SC_KBACK = 1     // Flag da interrupção externa
#define RFIE             KBI1PE_KBIPE4        // habilita / desabilita interrupção externa
#define RFFLAG           KBI1SC_KBF  
#define RFINT_ENABLE     KBI1PE_KBIPE4 = 1 
#define RFINT_DISABLE    KBI1PE_KBIPE4 = 0        
#define NEEDS_INIT_TIMER 0
#define RF_TPMSC         TPM1SC
#define RF_MOD           TPM1MOD


#else
// Transceiver Configuration

#define RF_INT_TIMER     1
#define RF_INT_KEYB      0
#define RF_INT_TYPE      RF_INT_TIMER

#if RF_INT_TYPE == RF_INT_KEYB
#define RFIF             KBI1SC_KBACK = 1     // Flag da interrupção externa
#define RFIE             KBI1PE_KBIPE4        // habilita / desabilita interrupção externa
#define RFFLAG           KBI1SC_KBF           
#define RFINT_ENABLE     KBI1PE_KBIPE4 = 1 
#define RFINT_DISABLE    KBI1PE_KBIPE4 = 0    
#else
#define RFIF             TPM1C3SC_CH3F = 0   // Flag da interrupção externa
#define RFIE             TPM1C3SC_CH3IE      // habilita / desabilita interrupção externa
#define RFFLAG           TPM1C3SC 
#define RFINT_ENABLE     TPM1C1SC_CH1IE = 1 
#define RFINT_DISABLE    TPM1C1SC_CH1IE = 0    
#endif

#define NEEDS_INIT_TIMER 1
#define RF_TPMSC         TPM1SC
#define RF_MOD           TPM1MOD

#endif

#define TX_POWER_LEVEL   RFTX_m36dB
#define TX_TIMEOUT       50

// Defines the radio pins
#define PHY_CS           PTBD_PTBD3       ///< CS pin  
#define PHY_CS_TRIS      PTBDD_PTBDD3     ///< CS direction pin
#define PHY_RESETn       PTDD_PTDD1       ///< RESET pin
#define PHY_RESETn_TRIS  PTDDD_PTDDD1     ///< RESET direction pin
#define PHY_WAKE         PTDD_PTDD0       ///< WAKE pin
#define PHY_WAKE_TRIS    PTDDD_PTDDD0     ///< WAKE direction pin

#ifndef OLD_BOARD

// Defines activity LED pin
#define ACTIVITY_LED     PTGD_PTGD0       ///< RF activity LED  
#define ACTIVITY_LED_DD  PTGDD_PTGDD0     ///< activity LED direction
#define ACTIVITY_LED_AS_IO								    ///< Defines LED pin as IO
#define ACTIVITY_LED_DS			PTGDS_PTGDS0 = 1					    	///< Defines LED pin drive strength to high
#define ACTIVITY_LED_LOW   		PTGD_PTGD0 = 0     						///< LED pin = 0
#define ACTIVITY_LED_HIGH  		PTGD_PTGD0 = 1     						///< LED pin = 1
#define ACTIVITY_LED_TOGGLE  		ACTIVITY_LED = ~ACTIVITY_LED ///< Toggle LED pin
#define ACTIVITY_LED_DIR_IN  		PTGDD_PTGDD0 = 0 						///< LED direction pin = in
#define ACTIVITY_LED_DIR_OUT  	PTGDD_PTGDD0 = 1						///< LED direction pin = out

#else

// Defines activity LED pin

#define ACTIVITY_LED     PTCD_PTCD0           ///< RF activity LED  
#define ACTIVITY_LED_DD  PTCDD_PTCDD0         ///< activity LED direction
#define ACTIVITY_LED_AS_IO								    ///< Defines LED pin as IO
#define ACTIVITY_LED_DS			PTCDS_PTCDS0 = 1					    	///< Defines LED pin drive strength to high
#define ACTIVITY_LED_LOW   		PTCD_PTCD0 = 0     						///< LED pin = 0
#define ACTIVITY_LED_HIGH  		PTCD_PTCD0 = 1     						///< LED pin = 1
#define ACTIVITY_LED_TOGGLE  		ACTIVITY_LED = ~ACTIVITY_LED ///< Toggle LED pin
#define ACTIVITY_LED_DIR_IN  		PTCDD_PTCDD0 = 0 						///< LED direction pin = in
#define ACTIVITY_LED_DIR_OUT  	PTCDD_PTCDD0 = 1						///< LED direction pin = out

#endif


// Defines the tick timer used to compute stocastic address
#define TIMER_ADDR       TPM2CNT          ///< Tick Timer register used in stocastic address generation

// Defines SPI port number
#define SPINB            2

#if (SPINB == 1)
#define SPIS             SPI1S
#define SPID             SPI1D
#define SPIC1            SPI1C1
#define SPIC2            SPI1C2
#define SPIBR            SPI1BR
#define SPIC1_SPE        SPI1C1_SPE
#define SPIS_SPTEF       SPI1S_SPTEF
#define SPIS_SPRF        SPI1S_SPRF
#endif                   

#if (SPINB == 2)
#define SPIS             SPI2S
#define SPID             SPI2D
#define SPIC1            SPI2C1
#define SPIC2            SPI2C2
#define SPIBR            SPI2BR
#define SPIC1_SPE        SPI2C1_SPE
#define SPIS_SPTEF       SPI2S_SPTEF
#define SPIS_SPRF        SPI2S_SPRF
#endif


#define PA_LEVEL            0x00  // -0.00 dBm
#define FREQUENCY_BAND      2400
#define CHANNEL_ERROR       (INT8U)255
#define RSSI_BUFFER_SIZE    50     //1-100 only

