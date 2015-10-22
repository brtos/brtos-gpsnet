#ifndef _DEBUGGER_H
#define _DEBUGGER_H

#include "BRTOS.h"
#include "BRTOSConfig.h"
#include "AppConfig.h"

#define MAX_DEPTH_TRACE_DF 200
#define MAX_TRACE_LINE_DF  10

#if !(defined  MAX_DEPTH_TRACE)
#define MAX_DEPTH_TRACE  MAX_DEPTH_TRACE_DF 
#endif
#if !(defined  MAX_TRACE_LINE)
#define MAX_TRACE_LINE   MAX_TRACE_LINE_DF
#endif



#define DELAYTASK 0
#define SEMPEND   1
#define QUEUEPEND 2
#define MUTEXPEND 3 
#define SEMPOST   4
#define QUEUEPOST 5
#define MUTEXPOST 6
#define OS_IDLE   7

#define ISR_ONLY   1
#define ISR_TICK   2

#define OS_TICK_SHOW 1
#define OS_IDLE_SHOW 0
#define OS_TRACE_BY_TASK 1                             

void Transmite_Uptime(void);
void Transmite_RAM_Ocupada(void);
void Transmite_Task_Stacks(void);
#if (COMPUTES_CPU_LOAD == 1)
void Transmite_CPU_Load(void);
#endif
void Reason_of_Reset(void);
#if (OS_TRACE == 1) 
void Update_OSTrace (INT8U task, INT8U syscall);
void Send_OSTrace(void);  
#endif

#endif

