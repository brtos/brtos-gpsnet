/***********************************************************************************
@file   smartmeter.h
@brief  functions for energy consumption, voltage, current, etc.
@authors: Carlos Henrique Barriquello

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

#ifndef _SMARTMETER_H
#define _SMARTMETER_H

/* MCU and OS includes */
#include "hardware.h"
#include "BRTOS.h"
#include "drivers.h"

typedef union{
  unsigned char Byte;
  struct{
    unsigned char Voltage          :1;
    unsigned char Current          :1;
    unsigned char Power_Factor     :1;
    unsigned char Apparent_Power   :1;
    unsigned char Reactive_Power   :1;
    unsigned char Active_Power     :1;
    unsigned char Energy           :1;
    unsigned char THD              :1;
  }Bits;
}param_union;


#define P_VOLTAGE        (INT8U)1
#define P_CURRENT        (INT8U)2
#define P_PF             (INT8U)4
#define P_POWER_S        (INT8U)8
#define P_POWER_Q        (INT8U)16
#define P_POWER_P        (INT8U)32
#define P_ENERGY         (INT8U)64
#define P_THD            (INT8U)128


#define SMARTMETER_OK    (INT8U)0
#define SMARTMETER_ON    (INT8U)1
#define SMARTMETER_OFF   (INT8U)2


typedef struct _SE_STRUCTURE
{
  
  param_union          params;   
  INT16U               v_rms;
  INT16U               i_rms;
  INT16U               power_factor;
  INT32U               power_S;
  INT32U               power_Q;
  INT32S               power_P;  
  INT32U               energy_meter;
} SE_STRUCT;

INT8U Smartmeter_GetValues(SE_STRUCT* value);
INT8U Smartmeter_GetEnergy(INT32U* value);
INT8U Smartmeter_GetStoredEnergy(void);
INT8U Smartmeter_Start(void);
INT8U Smartmeter_Stop(void);

/* Smart Meter Task */
void EnergyMetering_Task(void);


#endif