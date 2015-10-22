#ifndef _BOOTLOADER_SERIAL_H
#define _BOOTLOADER_SERIAL_H    

#include "AppConfig.h"

typedef enum 
{
    PROG,
    COMPRIMENTO,
    DADOS,
    RECEIVE_CRC,
    FIM,     
}BootloaderFSM_states;

typedef enum 
{
    
    ENTRY_BOOTLOADER = 1,
    DATA_DONE = 3,
    DATA_OK = 4,    
    DATA_ERROR = 5,
    CRC_OK = 8,
    CRC_ERROR = 9,
    EXIT_BOOTLOADER = 10,
} BootloaderFSM_status;


/* Bootloader vars */
 
#if (BOOTLOADER_ENABLE == 1)
extern  BRTOS_Sem     *Bootloader_Event;
extern  BRTOS_Mbox    *Bootloader_end;
extern  BRTOS_Sem     *Confirma;


//extern  INT8U         mensagem; 
extern  INT8U         comprimento;
extern  INT8U         comp;
extern  INT8U         length;
extern  INT8U         checksum;
extern  INT32U        checksum_add;
extern  INT32U        data_checksum[VECTOR32_SIZE];

extern  INT32U        endereco;
extern  INT8U         confirma_rede;
//extern  BootloaderFSM_states        estado_bootloader; 
//extern  INT8U         mensagem;

extern  INT16U        CRC16;
extern  INT16U        CRC_CODIGO_16;
extern  INT16U        CRC_CODIGO_162;
extern  INT16U        crc_codigo2;
extern  INT16U        crc_codigo;
extern  INT16U        crc_received2;
extern  INT16U        crc_received;

extern  BOOTLOADER_DATA_T structured_data;  

BootloaderFSM_status Bootloader_FSM(INT8U pedido);

void GPSNET_FOTA_Task(void);    
void Data_receive(void);

unsigned short int polydiv (byte data_CRC, unsigned short int accum);
unsigned short int gera_crc(unsigned short int cont, byte *frame);

/* end */
#endif 

#endif



