/************* TASKS PRIORIRY ASSIGNMENTS **************/

#define Comm2PC_Task_Priority       (INT8U)16
#define HeartBeat_LED_Priority      (INT8U)15
#define System_Time_Priority        (INT8U)30
#define Serial_Mutex_Priority       (INT8U)17

/****************************************************************/

/* Configuration of apps included */

/*** BOOTLOADER APP *******************/
#define BOOTLOADER_ENABLE  1 

//define the length of the data vector (if S19 file length is 64 bytes then the line will have 70 bytes) 
#define VECTOR8_SIZE  70 // It is necessary to change the max S19 record lentgh in Edit->Standard Settings-> Max S-Record legth                                       
#define VECTOR32_SIZE 18 
/////////////////////////////////////// 

// define the adrress wich the new code will be saved (COD_INICIO) and the flag adrress (STATUS_ADDR)
// usar 0xF800 -> codigo de 0xD800   --> 921 linhas -> 54kB
//  

#define STATUS_ADDR        0x11000
#define CRC_ADDR           0x11004
#define CODE_START         0x11010
#define FLASH_BLOCKS_1k    59

/******************************************/
/*** COMM2PC APP CONFIG *******************/
/* Supported commands */
/* Must be listed in alphabetical order */
/*  ------ NAME ------- CODE --- */
#define COMMAND_TABLE(ENTRY) \
    ENTRY(ERR,  0,"Command not found") \
    ENTRY(M16, '16',"MAC address 16bits") \
    ENTRY(M64, '64', "MAC address 64bits") \
    ENTRY(CFG_PARAM, 'CF', "Config Params") \
    ENTRY(CHAN, 'CH',"Channel get/set") \
    ENTRY(CPU,  'CP',"Cpu usage") \
    ENTRY(ERASE_FLASH, 'EF', "Flash erase") \
    ENTRY(FOTA_ADR, 'FA', "Send firmware over the air to address") \
    ENTRY(FOTA, 'FO', "Send firmware over the air") \
    ENTRY(GAS, 'GA',"Set GPS address") \
    ENTRY(HELP, 'HP', "This Help") \
    ENTRY(LED,  'LE', "On/off LED") \
    ENTRY(LSNIF, 'LS', "Return sniffer values") \
    ENTRY(NGB,  'NB',"Get neighbors") \
    ENTRY(NNB,  'NN',"Num. of neighbors") \
    ENTRY(C_OFF, 'OF',"Command OFF") \
    ENTRY(C_ON, 'ON',"Command ON") \
    ENTRY(POWL, 'PL', "TX power level adjust")\
    ENTRY(RPK,  'RP',"Get packets") \
    ENTRY(SEND, 'SD',"Send Data") \
    ENTRY(STORE_FIRMWARE, 'SF', "Flash load") \
    ENTRY(SNIF, 'SN',"Sniffer") \
    ENTRY(SETPOS, 'SP',"Send New Position") \
    ENTRY(UPT,  'UP',"Uptime") \
    ENTRY(VER,  'VE',"OS and NET versions") \
    ENTRY(WHO,  'WH',"Who I am - 0(p),1(r),2(i)")
         

#define HELP_COMMAND_ENABLED     1
#define HELP_DESCRIPTION         1
    
/******************************************/  