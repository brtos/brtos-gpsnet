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
 /************************************************************************
 To use bootloader:

--> import files: usr_entry.c , MCF51_Bootloader.h , MFC51_Bootloader.c
--> BOOTLOADER_ENABLE 1
--> Edit->Standard Settings-> Entry Point: __Entry
 
 *************************************************************************/
  
#include "BRTOS.h"
#include "Bootloader_wireless.h"
#include "MCF51_Bootloader.h"

#include "BRTOSConfig.h"
#include "AppConfig.h"

#include "flash.h"

#pragma warn_implicitconv off 

//-------------------------- CRC code ---------------------------------------;
static INT16U Bootloader_polydiv (INT8U data_CRC, INT16U accum)
{
    INT16U gerador = 0xa001;
    INT16U test = 0;
    INT8U  i = 0;
    
    accum = accum ^ data_CRC;
    
    for (i = 0; i <= 7; i++)
    {
        test = accum & 1;
        accum = accum >> 1;
        
        if (test == 1)
        {
            accum = accum ^ gerador;
        }
    }
    return(accum);
}


static INT16U Bootloader_CRC(INT16U cont, INT8U *frame)
{
  INT8U i;
  
  for(i = 0; i < cont; i++)
  {
    CRC16 = Bootloader_polydiv(*frame, CRC16);
    frame++;
  }
  
  return (CRC16);
}
 
   

/********************************************************************
*********************************************************************
*     Flash subroutines
*********************************************************************
********************************************************************/


INT8U Bootloader_Flash_Cmd(INT8U Cmd)
{  
    
    FCMD = Cmd;
  
    FSTAT = 0x80;     // launch command
    
    if(FSTAT&0x30)
    {
      return 0xFF;
    }
    
    while(!FSTAT_FCCF)
    {
    }
    
    
  return 0x00;
  
}

// sector erase (1k bytes)
INT8U Bootloader_Flash_Erase(INT32U addr)
{
     byte i;
 #if (DEBUG == 1)
    
    FnCmdInRam = (void*)((INT32U)&CmdInRam); 
    CmdInRam = *(CmdInRam_t *)(Bootloader_Flash_Cmd);
    FSTAT = 0x30;  
    
    *(INT32U *)addr = 0x55; 
    i = FnCmdInRam(0x40);
    __RESET_WATCHDOG();   
    return(i);    
    
#else
    
    for(i=0;i<255;i++)
    {
      *(INT32U *)addr = 0xFFFFFFFF;
      addr +=4;
    }
    
    *(INT32U *)addr = 0xFFFFFFFF;
    addr +=4;

    return 0x00;    

#endif       

}


// INT32U (32-bit) programming
INT8U Bootloader_Flash_Prog(INT32U flash_addr, INT32U data_addr, INT8U number)
{
   
   INT8U  i;
   INT8U  result;
   
#if (DEBUG == 1)  
   
   FnCmdInRam = (void*)((INT32U)&CmdInRam); 
   CmdInRam = *(CmdInRam_t *)(Bootloader_Flash_Cmd);
   FSTAT = 0x30;
   
#endif 
    
   for(i=0;i<number;i++)
   {
      *(INT32U *)flash_addr = *(INT32U *)data_addr;
      
#if (DEBUG == 1)    
      result= FnCmdInRam(0x20);
      asm
      {
        NOP
        NOP
        NOP
        NOP
      }
      if(result == 0xFF)
      {
        __RESET_WATCHDOG(); 
        return 0xFF;
      }
#endif      
      flash_addr +=4;
      data_addr +=4;
   }
#if (DEBUG == 1)
   __RESET_WATCHDOG();  
   return result;
#else
   return 0x00;
#endif
   
}    

INT8U Bootloader_Flash_Verify(INT32U dst_addr, INT32U data_addr, INT8U length) 
{
  do
	 {
		if(*(INT32U *)dst_addr != *(INT32U *)data_addr) // compare two values
		{
		   return 0xFF;
		}	
		
		dst_addr +=4;
		data_addr +=4; 
	}	 
	while(--length);		
    
  return 0x00;
}

/* return a 32-bit value from flash at "data_addr" */
INT32U Bootloader_Flash_Read(INT32U data_addr) 
{
    return *(INT32U *)data_addr;
}


/**************************************************************************************************/

void Bootloader_Flash_Clock_Init(void)
{
  
  FSTAT = 0x30;
  FCDIV = 0x50; // flash clock must be set up in the range (150-200 kHz)
}


void Bootloader_Flash_EraseAll(INT32U start_address){
       
     INT32U  address = start_address;
     INT32U  i = 0;     
       
     for(i=0;i<FLASH_BLOCKS_1k;i++)          //apaga novamente todas as páginas de memória
     {
      
       UserEnterCritical();
          Bootloader_Flash_Erase(address); // erase function
       UserExitCritical();
       address+=0x400; 
     }
}


/* Bootloader code starts here */
/**
\brief Bootloader code entry point

Disable ISR and set stack pointer
Init processor and flash module clocks
Parse S19 flash backup code
Write to user code region
Set programmed ready flag
  
*/


   
void Bootloader_Main(void) 
{
     
   asm (move.w  #0x2700,sr);    // disable interrupt
   asm (move.l  #0x801E00,a7);  // initialize stack pointer
   
   
   INT32U    valor_lido;   
   INT8U     vetor[VECTOR8_SIZE];
   INT32U    source_data[VECTOR32_SIZE];   

   INT32U    address=CODE_START_ADDRESS;
   INT32U    status_address=STATUS_ADDR;
     
   INT16U    crc=0xFFFF;
   INT16U    crc_teste=0;
   
   INT8U     i=0;
   INT8U     type=0;   
   INT8U     numero=0;
   INT8U     size=0;

   INT32U    data = 0;
   INT32U    addr=0;
   INT32U    status;
   INT8U     z=0;
   INT32U    flash_retries = 0;
  
   
#if (DEBUG == 1) 
   Bootloader_Flash_Clock_Init();      // initialization of flash clock frequency
#endif

  grava_de_novo:
     
     Bootloader_Flash_EraseAll(CODE_START_ADDRESS);  //endereco inicia pelo CODE_START_ADDRESS
     
     address = CODE_START;   //passa para adress o endereco de onde esta o novo programa a ser gravado (0x11010)
    
     CRC16 = 0xFFFF;
          
     /* loop para gravar o código */  
     while(1) 
     {  
     
        for(z=0;z<VECTOR32_SIZE;z++)    //loop para fazer a leitura de uma linha que esta armazenada no endereco passado anteriormente
        {
           source_data[z]=Bootloader_Flash_Read(address);
           address+=4;      
        }
             
          
        type = (INT8U)((source_data[0] >> 8) & 0xFF);  //armazena o tipo 
        size = (INT8U)(source_data[0] & 0xFF);         //armazena o tamanho da linha
        addr = source_data[1];                         //armazena o endereco da linha
        numero = (INT8U)((size-5)>>2);

        if(type==7)                                    //quando o tipo for 7 chegou na ultima linha 
        {
        
          source_data[0] = Bootloader_Flash_Read(CRC_ADDR);
          crc_teste = (INT16U)(source_data[0] & 0xFFFF);
          
          if(crc == crc_teste) 
          { 
              /* retorna o valor da flag para zero. 
              Assim, quando o programa voltar do reset 
              entra no main e roda o programa novo  */
                           
              status=0;  
              Bootloader_Flash_Prog(status_address,(INT32U)&status,1);
              
              /* força reset por COP */                     
              OS_Wait_BW; 
                           
              for(;;)
              {
                asm (nop);
              }
          }
          else 
          {
              crc=0xFFFF;
              CRC16=0xFFFF;
              type=0;
              flash_retries++;
              if (flash_retries > 3) 
              {
                while(1) 
                {
                 /* loop eterno ! */
                 __RESET_WATCHDOG();
                }
              }
             goto grava_de_novo; 
          }
        }
        
        //grava no endereco addr (obtido da linha lida) apenas os DADOS contidos na linha          
        if(addr >= CODE_START_ADDRESS)
        {
          Bootloader_Flash_Prog(addr,(INT32U)&source_data[2],(INT8U)(numero)); 
        }
         
        z=0;
        for(i=0;i<numero;i++)                // este loop faz a leitura de toda a linha que foi gravada anteriormente
        {
          valor_lido = Bootloader_Flash_Read(addr);     // a leitura retorna uma palavra de 32bits
          vetor[z++] = (INT8U)(valor_lido>>24) & 0xFF;
          vetor[z++] = (INT8U)(valor_lido>>16) & 0xFF;
          vetor[z++] = (INT8U)(valor_lido>>8)  & 0xFF;
          vetor[z++] = (INT8U)(valor_lido)     & 0xFF;
          addr+=4;
        }
        
        crc = Bootloader_CRC((numero<<2), &vetor[0]);      
     }
     
 }
 
 
  
  


                          





