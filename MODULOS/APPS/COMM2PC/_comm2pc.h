/***********************************************************************************
@file   comm2pc.h
@brief  Task for PC communication
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
**********************************************************************************/

#include "BRTOS.h"  









/*********************************************************************************
@file   comm2pc.h
@brief  Functions to communicate with a PC
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

#ifndef _COMM2PC_H
#define _COMM2PC_H

#include "BRTOS.h"
#include "AppConfig.h"

// errors   
#define COMMAND_OK       0
#define COMMAND_UNKNOWN  1 
#define COMMAND_INVALID  2

typedef union _word
{
  unsigned char   int8u[2];
  unsigned short  int16u; 
} UCHAR_WORD;


// possible states                    
enum Comm2PC_FSM_States 
{
  BEGIN,        // start ($) of a message
  COMMAND,      // command
  CMD_ARG,      // args
  END           // end of a message
};
  

// possible states                      
enum Comm2PC_FSM_States 
{
    BEGIN,        // message start
    COMMAND,      // command
    GPS_ADDRESS,  // dest. address 
    CMD_ARGS,     // command to send
    END           // end of message
};
  

#define MSG_START '$'
#define MSG_END   '#'

/// commands
#define HELP            ('H' << 8) | 'P'
#define SND_CMD         ('S' << 8) | 'C'
#define CPU             ('C' << 8) | 'P'
#define VER             ('V' << 8) | 'E' 
#define LED             ('L' << 8) | 'E'
#define NGB             ('N' << 8) | 'B'
#define NNB             ('N' << 8) | 'N'
#define UPT             ('U' << 8) | 'P'
#define RPK             ('R' << 8) | 'P'
#define GAD             ('G' << 8) | 'A'
#define SNIF            ('S' << 8) | 'N'
#define C_ON            ('O' << 8) | 'N'
#define C_OFF           ('O' << 8) | 'F'

#define ERASE_FLASH     ('E' << 8) | 'F'
#define STORE_FIRMWARE  ('S' << 8) | 'F'
#define FOTA            ('F' << 8) | 'O'
#define LSNIF           ('L' << 8) | 'S'
#define M64             ('6' << 8) | '4'
#define M16             ('1' << 8) | '6'


INT8U Comm2PC_FSM (INT8U message);
void  SendPacketToQueue(void);
void  RequestGPSAddress(INT32U *x, INT32U *y);
INT16U RequestNodeID(void); 

/* Supported commands */
void CMD_ERROR(void);
void CMD_CPU (void);
void CMD_VER (void);
void CMD_UPT (void);
void CMD_LED (void);
void CMD_REQ_PACKETS(void);
void CMD_NEIGHBORS (void);
void CMD_NUMBER_OF_NEIGHBORS (void);
void CMD_GPS_ADDRESS (void);
void CMD_SNIF (void);
void CMD_ON(void);
void CMD_OFF(void);
void CMD_HELP(void);
void CMD_M64(void);
void CMD_M16(void);


// Packet queue structure
extern OS_QUEUE      PacketsBuffer;
extern BRTOS_Queue  *Pkt;
extern BRTOS_Mutex  *PktQueueMutex;

#endif

    

    