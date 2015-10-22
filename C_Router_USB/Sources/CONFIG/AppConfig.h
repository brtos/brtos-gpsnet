/************* TASKS PRIORIRY ASSIGNMENTS **************/

#define Comm2PC_Task_Priority       (INT8U)16
#define HeartBeat_LED_Priority      (INT8U)15
#define System_Time_Priority        (INT8U)30
#define Serial_Mutex_Priority       (INT8U)17

/****************************************************************/

/* Configuration of apps included */

/*** BOOTLOADER APP CONFIG *******************/
#define BOOTLOADER_ENABLE   1 
//#define SIGNAL_APP255       App255_event

//define the length of the data vector (if S19 file length is 64 bytes then the line will have 70 bytes) 
#define VECTOR8_SIZE  70 // It is necessary to change the max S19 record lentgh in Edit->Standard Settings-> Max S-Record legth                                       
#define VECTOR32_SIZE 18 
/////////////////////////////////////// 

//define the adrress wich the new code will be saved (CODE_START) and the flag adrress (STATUS_ADDR)

#define FLASH_BLOCKS_1k    59
#define CODE_START          (INT32U)0x11010                                                                      
#define STATUS_ADDR         (INT32U)0x11000 
#define CRC_ADDR            (INT32U)0x11004
#define CODE_START_ADDRESS  (INT32U)0x02000

#define Bootloader_Task_StackSize (352)

/******************************************/
/*** COMM2PC APP CONFIG *******************/
/* Supported commands */
/* Must be listed in alphabetical order */
/*  ------ NAME ------- CODE --- */
#define COMMAND_TABLE(ENTRY) \
    ENTRY(ERR,  0,"Command not found") \
    ENTRY(M16, '16',"MAC address 16bits") \
    ENTRY(M64, '64', "MAC address 64bits") \
    ENTRY(CPU,  'CP',"Cpu usage") \
    ENTRY(GAS, 'GA',"Set GPS address") \
    ENTRY(HELP, 'HP', "This Help") \
    ENTRY(LED,  'LE', "On/off LED") \
    ENTRY(LSNIF, 'LS', "Return sniffer values") \
    ENTRY(NGB,  'NB',"Get neighbors") \
    ENTRY(NNB,  'NN',"Num. of neighbors") \
    ENTRY(C_OFF, 'OF',"Command OFF") \
    ENTRY(C_ON, 'ON',"Command ON") \
    ENTRY(RPK,  'RP',"Get packets") \
    ENTRY(SEND, 'SD',"Send Data") \
    ENTRY(SNIF, 'SN',"Sniffer") \
    ENTRY(UPT,  'UP',"Uptime")\
    ENTRY(VER,  'VE',"OS and NET versions") \
    ENTRY(WHO,  'WH',"Who I am - 0(p),1(r),2(i)")
    

#define HELP_COMMAND_ENABLED     1
#define HELP_DESCRIPTION         1
    
/******************************************/ 

/*****************************************************************/
/** GPSNET SENSOR APP CONFIG                                    */
#define   REPORTING_PERIOD_MS       5000
#define   REPORTING_JITTER_MS       100

   