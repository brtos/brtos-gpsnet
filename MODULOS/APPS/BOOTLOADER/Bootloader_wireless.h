/***********************************************************************************
@file   tasks.c
@brief  Tasks application file
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

#ifndef _BOOTLOADER_WIRELESS_H_
#define _BOOTLOADER_WIRELESS_H_
#endif  

#include "AppConfig.h"

enum BOOTLOADER_DEST
{
  BOOT_ROUTER,
  BOOT_INSTALLER,  
  BOOT_COORDINATOR,  
};  

extern INT16U    crc_codigo;
//extern INT16U    crc_received;
//extern INT16U    crc_received2;
extern INT16U    crc_codigo2;
extern INT16U    CRC16;

typedef union _BOOTLOADER_DATA
{
  INT8U   bootloader_data_byte[VECTOR8_SIZE+2];
  INT32U  bootloader_datal_long[VECTOR32_SIZE];
} BOOTLOADER_DATA_T;

#define  bootloader_data structured_data.bootloader_data_byte  
#define  bootloader_datal structured_data.bootloader_datal_long 


#define BOOTLOADER_OK                 (INT8U)0     ///< OK - no error
#define BOOTLOADER_CHECKSUM_ERROR     (INT8U)1     ///< Error - wrong checksum
#define BOOTLOADER_CRC_ERROR          (INT8U)2     ///< Error - wrong crc
#define BOOTLOADER_FLASH_ERROR        (INT8U)3     ///< Error - flash error  
#define BOOTLOADER_DATA_OK            (INT8U)4
#define BOOTLOADER_DATA_ERROR         (INT8U)5
#define BOOTLOADER_NOT_S19            (INT8U)6
#define BOOTLOADER_NOT_DEVICE_TYPE    (INT8U)7
#define BOOTLOADER_CRC_OK             (INT8U)8


void WBootloader_Handler_Timeout(void);
void WBootloader_Handler(void);
void WBootloader_Wireless_Flash_EraseAll(INT32U start_address);
void WBootloader_Flash_Init(void);
INT16U WBootloader_crc(INT16U cont, INT8U *frame);
INT16U WBootloader_polydiv (INT8U data_CRC, INT16U accum);





