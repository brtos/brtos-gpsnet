#include <hidef.h> /* for EnableInterrupts macro */
#include "derivative.h" /* include peripheral declarations */

typedef byte uint8_t;
typedef word uint16_t;

typedef  struct {uint8_t command;}command_cmd_t;
typedef  void (*p_func_t)(void);


#if  0 
/* List of supported commands */
static const uint16_t ListOfCommands[] = {
  HELP, SND_CMD, CPU, VER, LED, NGB, NNB, UPT, RPK, GAD, SNIF, C_ON, C_OFF, 
};

/*  ------ NAME ------- FUNCTION --- CODE --- */
#define COMMAND_TABLE(ENTRY)               \
    ENTRY(cpu,    cpu,   0x00  ) \
    ENTRY(COMMAND1,    command1,     0x01) \
    ENTRY(COMMAND2,    command2,     0x02) \
    ENTRY(COMMAND3,    command3,     0x03)
    
#define EXPAND_AS_COMMAND_CODE_ENUM(a,b,c) a##_CMD = c,
        
#define EXPAND_AS_STRUCT(a,b,c) uint8_t b;

#define EXPAND_AS_JUMPTABLE(a,b,c) process_##b,

#define EXPAND_AS_PROTOTYPES(a,b,c) void process_##b(void); 


enum{
    COMMAND_TABLE(EXPAND_AS_COMMAND_CODE_ENUM)
};
        
typedef  struct{
    COMMAND_TABLE(EXPAND_AS_STRUCT)
} size_struct_t;

#define NUMBER_OF_COMMANDS sizeof(size_struct_t)

COMMAND_TABLE(EXPAND_AS_PROTOTYPES);

p_func_t jump_table[NUMBER_OF_COMMANDS] = {
    COMMAND_TABLE(EXPAND_AS_JUMPTABLE)
};


#define INIT_X1     process_reserved,
#define INIT_X2     INIT_X1   INIT_X1
#define INIT_X4     INIT_X2   INIT_X2
#define INIT_X8     INIT_X4   INIT_X4
#define INIT_X16   INIT_X8   INIT_X8
#define INIT_X32   INIT_X16  INIT_X16
#define INIT_X64   INIT_X32  INIT_X32
#define INIT_X128  INIT_X64  INIT_X64
#define INIT_X256  INIT_X128 INIT_X128
  
#define EXPAND_JUMP_TABLE(a,b,c) [c] = process_##b,

static const p_func_t command_jump_table[256] = {
   /* initialize all pointer to the  reserved function */
   INIT_X256, 
   /* overwrite pointers to valid  functions */
   COMMAND_TABLE(EXPAND_AS_JUMPTABLE)
};

#endif

typedef union _word
{
  uint8_t   int8u[2];
  uint16_t  int16u; 
} _UWORD;

typedef struct {
   uint16_t cmd;
   p_func_t pf;
}cmd_t;


/// commands
#if 0
#define HELP            {'H','P'}
#define SEND            {'S','D'}
#define CPU             {'C','P'}
#define VER             {'V','E'} 
#define LED             {'L','E'}
#endif

#if 0
enum cmd_codes {
  HELP = 'HP',
  SEND = 'SD',
  CPU  = 'CP',
  VER  = 'VE',  
  LED  = 'LE',  
};
#endif


/*  ------ NAME ------- FUNCTION --- CODE --- */
#define COMMAND_TABLE(ENTRY) \
    ENTRY(ERR,  0, NULL) \
    ENTRY(CPU,  'CP', NULL) \
    ENTRY(HELP, 'HP', NULL) \
    ENTRY(LED,  'LE', NULL) \
    ENTRY(SEND, 'SD', NULL) \
    ENTRY(VER,  'VE', NULL)     
    
#define EXPAND_AS_COMMAND_CODE_ENUM(a,b,c) a = b,  
#define EXPAND_AS_STRUCT(a,b,c) uint8_t a;   
#define EXPAND_AS_JUMPTABLE(a,b,c) {a, &process_cmd_##c},
#define EXPAND_AS_PROTOTYPES(a,b,c) void process_cmd_##a(void);
// #define EXPAND_AS_HANDLERS(a,b,c)   void process_cmd_##a(void){do{dummy_cmd();}while(0);}                              

enum{
    COMMAND_TABLE(EXPAND_AS_COMMAND_CODE_ENUM)
};

typedef  struct{
    COMMAND_TABLE(EXPAND_AS_STRUCT)
} size_struct_t;

#define NUMBER_OF_COMMANDS sizeof(size_struct_t)

COMMAND_TABLE(EXPAND_AS_PROTOTYPES)  

void process_cmd_NULL(void){


}

// COMMAND_TABLE(EXPAND_AS_HANDLERS)

const cmd_t cmds[NUMBER_OF_COMMANDS] = {
    COMMAND_TABLE(EXPAND_AS_JUMPTABLE)
};

uint8_t search_cmd (uint16_t c);

uint8_t search_cmd (uint16_t c) {
  uint8_t inf = 0;  // limite inferior
  uint8_t sup = NUMBER_OF_COMMANDS-1; // limite superior
  uint8_t meio;
  while(inf <= sup){
    meio = (uint8_t)((inf + sup) >> 1);
    if (cmds[meio].cmd < c) {
      inf = (uint8_t)(meio + 1);
    }else{
      if (cmds[meio].cmd > c){
        sup = (uint8_t)(meio-1);
      }else{
        return meio; // comando a ser executado
      }
    }
  } 
  return 0; // comando inexistente, executa o comando padrão

}

uint16_t cmd = LED;
uint8_t i=0;

void main(void) {
  i=search_cmd(cmd);  
  cmds[i].pf();
  
  
  for(;;) {     
  } 
}
