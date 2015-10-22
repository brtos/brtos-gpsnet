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

#ifndef _MCF51_BOOTLOADER_H_
#define _MCF51_BOOTLOADER_H_

#include "BRTOS.h"


#define  OS_Wait_BW      asm(STOP #0x2000); 


void ISR_vtable_reallocation(void);


void Bootloader_Main(void); 
void Bootloader_Flash_Clock_Init(void);
INT32U Bootloader_Flash_Read(INT32U data_addr);
INT8U Bootloader_Flash_Cmd(INT8U Cmd);
INT8U Bootloader_Flash_Erase(INT32U addr);
INT8U Bootloader_Flash_Prog(INT32U flash_addr, INT32U data_addr, INT8U number);    
INT8U Bootloader_Flash_Verify(INT32U dst_addr, INT32U data_addr, INT8U length);
void Bootloader_Flash_EraseAll(INT32U start_address);



#endif                    