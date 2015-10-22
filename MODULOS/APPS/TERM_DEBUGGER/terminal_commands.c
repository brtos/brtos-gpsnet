#include "terminal_commands.h"
#include "debugger.h"


// TOP Command (similar to the linux command)
void cmd_top(char *param)
{
  (void)*param;
  #if (COMPUTES_CPU_LOAD == 1)
    Transmite_CPU_Load();
  #endif
  Transmite_Uptime();
  Transmite_RAM_Ocupada();
  Transmite_Task_Stacks();
}

const command_t top_cmd = {
  "top", cmd_top, "BRTOS TOP"
};


// Reason of Reset Command
void cmd_rst(char *param)
{
  (void)*param;
  Reason_of_Reset();
}

const command_t rst_cmd = {
  "rst", cmd_rst, "CPU Reason of the Reset"
};


// BRTOS version Command
void cmd_ver(char *param)
{
  (void)*param;
  Serial_Envia_Frase((CHAR8*)version);
  Serial_Envia_Frase("\n\r");
}

const command_t ver_cmd = {
  "ver", cmd_ver, "BRTOS Version"
};

// Reason of Reset Command
void cmd_snf(char *param)
{
  (void)*param;
  //Reason_of_Reset();
}

const command_t snf_cmd = {
  "snf", cmd_snf, "Snifer Node"
};

