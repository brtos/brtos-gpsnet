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

#include "hardware.h"
#include "BRTOS.h"   
#include "drivers.h"

#include "AppConfig.h"
#include "NetConfig.h"

#include "app.h"
#include "gpsnet_api.h"

#ifndef BOOTLOADER_ENABLE
   #error "Please, enable or disable bootloader app in AppConfig.h file"
#endif

#if (BOOTLOADER_ENABLE == 1)
  #include "MCF51_Bootloader.h"  
  #include "Bootloader_wireless.h"  
#endif 

#pragma warn_implicitconv off

#if (BOOTLOADER_ENABLE == 1)
  INT32U  endereco = CODE_START;
  BOOTLOADER_DATA_T structured_data;  
  INT16U    CRC16=0xFFFF;
  INT16U    crc_codigo2=0xFFFF;
  INT16U    crc_codigo=0xFFFF;  
#endif  

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

#if (BOOTLOADER_ENABLE == 1)

/* bootloader vars */
static INT8U  resp[4];
static INT8U  error = 0; 
    
    
void WBootloader_Handler_Timeout(void){
    
  LATITUDE   lat;
  LONGITUDE  lon;
  INT8U k=0;
  INT8U  dir;
                      
    
  if (error > 3) 
  {
    error = 0;
    
    /* erase flash memory */         
    WBootloader_Wireless_Flash_EraseAll(CODE_START);
    
    UserEnterCritical();
      crc_codigo = 0xffff;
      crc_codigo2 = 0xffff;
      CRC16 = 0xffff;
      endereco = CODE_START;
    UserExitCritical();
    
    acquireRadio();
      resp[0] = BOOTLOADER_DATA_ERROR;
  
      /* copy source address */
      for(k=0;k<4;k++)
      {
        lat.bytes[k] = nwk_packet.NWK_Src_Lat[k];
      }
      
      for(k=0;k<4;k++)
      {
        lon.bytes[k] = nwk_packet.NWK_Src_Long[k];
      }        
      
      dst_gps_latX  = lat.x;
      dst_gps_longY = lon.y; 
      
      dir = UP_ROUTE;
      
      for(k=0;k<MAX_BASE_STATION;k++){
        
        if(BaseStations[k].GPS_Latitude.x == lat.x && BaseStations[k].GPS_Longitude.y == lon.y){
          dir = DOWN_ROUTE;
          break;
        }
      }
      
      (void)NetSimpledata(dir, (INT8U)BOOT_INSTALLER,&resp[0]);
      
    releaseRadio();
   
   }
  
}

void WBootloader_Handler(void){       
      
   INT8U  retorno = 0;
   INT32U crc_casting = 0;
   INT32U status = 0;
   INT8U  k=0;
   INT8U  dir;
   
    LATITUDE   lat;
    LONGITUDE  lon;
    
    // Reset timeout counter    
    error = 0;

    retorno = Decode_Data_Profile();
    if((retorno == BOOTLOADER_OK) || (retorno == BOOTLOADER_CRC_OK))
    {
         resp[0]=BOOTLOADER_DATA_OK;
         resp[1]=bootloader_data[5];
         resp[2]=bootloader_data[6];
         resp[3]=bootloader_data[7];  
    } 
    else
    {
        resp[0] = retorno;
        
        /* erase flash memory */
        WBootloader_Wireless_Flash_EraseAll(CODE_START);
        
        UserEnterCritical();                   
          crc_codigo = 0xffff;
          crc_codigo2 = 0xffff;
          CRC16 = 0xffff;
          endereco = CODE_START;
        UserExitCritical(); 
    }


    if(resp[0])
    {      
      /* copy source address */
      for(k=0;k<4;k++)
      {
        lat.bytes[k] = nwk_packet.NWK_Src_Lat[k];
      }
      
      for(k=0;k<4;k++)
      {
        lon.bytes[k] = nwk_packet.NWK_Src_Long[k];
      }        
      
      dst_gps_latX  = lat.x;
      dst_gps_longY = lon.y; 
      
      dir = UP_ROUTE;
      
      for(k=0;k<MAX_BASE_STATION;k++){
        
        if(BaseStations[k].GPS_Latitude.x == lat.x && BaseStations[k].GPS_Longitude.y == lon.y){
          dir = DOWN_ROUTE;
          break;
        }
      }
      
      if(NetSimpledata(dir, (INT8U)BOOT_INSTALLER,&resp[0]) != OK)
      {
        error++;
      }else
      {
        if (retorno == BOOTLOADER_CRC_OK)
        {
        
            /* 
            passa para a variavel status um valor que sera a
            armazenado na posicao STATUS_ADDR. 
            Ao iniciar o programa este valor sera lido 
            para decidir se o programa vai para o main ou 
            para o  bootloader
            */                  
            status=0xFFFF0000;  
                        
            UserEnterCritical();             
              
              #if (PROCESSOR == COLDFIRE_V1)
              /* erase also position, mac and pan id */
                  Flash_Erase(LAT_MEM_ADDRESS);
              #endif
            
            Flash_Prog(STATUS_ADDR,(dword)&status,1);
              crc_casting = (INT32U)(crc_codigo2 & 0xFFFF);
            Flash_Prog(CRC_ADDR,(dword)&crc_casting,1);
            
            /* força reinicialização (por watchdog) */
            for(;;)
            {
                asm(nop);
            } 
        }
      }
      
      resp[0]=0;
      resp[1]=0;
      resp[2]=0;
      resp[3]=0;
    }  
}

INT8U NetSimpledata(INT8U direction, INT8U destino, INT8U *p_data)
{
  INT8U i = 0;
  INT8U j = 0;
    
  NWKPayload[j++] = APP_01;
  NWKPayload[j++] = BULK_DATA_PROFILE;
  NWKPayload[j++] = FILE_S19;  
  NWKPayload[j++] = destino;
    


  NWKPayload[j++] = *p_data;
  p_data++;
  NWKPayload[j++] = *p_data;
  p_data++;
  NWKPayload[j++] = *p_data;
  p_data++;
  NWKPayload[j++] = *p_data;


    
  if (direction == DOWN_ROUTE)
  {
     i = DownRoute(START_ROUTE,(INT8U)(8));
  }else 
  {
     i = UpRoute(START_ROUTE,(INT8U)(8));
  }
    
  return i;
  
}



//////////////////////////////////
// funcao para receber arquivo S19
static INT32U data_checksum[VECTOR32_SIZE];

INT8U Decode_Data_Profile(void)  
{
  INT16U checksum_add=0;    
  INT8U  checksum=0;
  INT8U  comp=0;
  INT8U  length=0;    
  INT8U  i=0;
  INT8U  k = 0; 
  INT8U  tipo = 0;
  INT8U  retorno = BOOTLOADER_OK;   
  
  INT16U    crc_received;
  INT16U    crc_received2;   

  switch(app_packet.APP_Command)
  {
      case FILE_S19:
          switch(app_packet.APP_Command_Attribute) 
          {
            case BOOT_ROUTER:
              for(i=0;i<VECTOR8_SIZE;i++) 
              {
                bootloader_data[i]=app_packet.APP_Payload[i];
              }
                     
              // grava na flash
              k = 0;
                                                            
          tenta_de_novo:
                            
              
              // Copia o endereco dos dados da linha recebida
              // a leitura retorna uma palavra de 32bits
              data_checksum[0]= Flash_Read(endereco-((VECTOR32_SIZE*4)-4));  
                              
              // Verifica se a linha recebida já foi enviada por outro pacote
              if(data_checksum[0] == bootloader_datal[1]) 
              {
                  // Se a linha já havia sido recebida, simplesmente retorna ok
                  retorno = BOOTLOADER_OK;                    
                  break; 
              }
              
              //entra em estado critico para poder fazer a gravacao 
              UserEnterCritical();                 
              
              __RESET_WATCHDOG();     // reseta o watchdog 
              
              Flash_Prog(endereco,(INT32U)bootloader_data,VECTOR32_SIZE); //armazena todo o vetor na posicao indicada pelo endereco
              
              UserExitCritical();                      //sai do estado critico
                                        
              
              for(i=0;i<VECTOR32_SIZE;i++)                    //este loop faz a leitura de toda a linha que foi gravada anteriormente
              {
                data_checksum[i]=Flash_Read(endereco);  //a leitura retorna uma palavra de 32bits
                endereco+=4;
              }
                                  
              /*
              A leitura da memoria retorna uma palavra de 32 bits. Portanto temos:
              
              data_checksum[0]=SS|tipo|comprimento
              data_checksum[1]=endereco
              data_checksum[2]=dados
              .
              .
              .
              data_checksum[8]=dados
              data_checksum[9]=checksum|333333
              */ 
              
              /* armazena o comprimento da linha em bytes */
              comp = (INT8U)(data_checksum[0] & 0xFF);             
              
              if (comp == 0xFF)
              {
                k++;
                if (k<3)
                {
                    endereco -=VECTOR8_SIZE+2;  
                    
                    #if TESTE == 1
                      UserEnterCritical();
                      PINO_RELE = 1;
                      while(1){                       
                        __RESET_WATCHDOG();     // reseta o watchdog    
                      }
                    #endif
                                 
                    goto tenta_de_novo;
                }
                else
                {
                    retorno = BOOTLOADER_FLASH_ERROR;
                    break;
                }
              }
                                                
              i=1;
              length=0;

              while(length < (comp-1))   //loop para somar todos os bytes de cada linha, exceto o checksum, tipo e S
              {
                  checksum_add+=(data_checksum[i] & 0xFF)+(data_checksum[i]>>8 & 0xFF)+((data_checksum[i]>>16) & 0xFF)+((data_checksum[i]>>24) & 0xFF);  //faz o somatorio dos bytes de cada vetor    
                  length+=4;
                  i++;                     
              }
                
              checksum=(INT8U)((data_checksum[i])>>24 & 0xFF); //armazena o checksum da linha
              
              checksum_add+=comp;              // soma o comprimento ao checksum_add 
              
              tipo = (INT8U)((data_checksum[0]>>8) & 0xFF);
                                                     
              // checksum calculado
              comp=(INT8U)(0xFF - (INT8U)(checksum_add & 0xFF)); 
             
              if(checksum!=comp) 
              {
                  retorno = BOOTLOADER_CHECKSUM_ERROR;
              } 
              else 
              {
                CRC16 = crc_codigo;
                crc_codigo = WBootloader_crc((INT16U)(data_checksum[0] & 0xFF), &bootloader_data[4]);
                                    
                if(tipo != 7) 
                {
                  CRC16 = crc_codigo2;
                  crc_codigo2 =  WBootloader_crc((INT16U)((data_checksum[0] & 0xFF)-5), &bootloader_data[8]);
                  retorno = BOOTLOADER_OK;
                  break;
                }
                else 
                {
                   tipo = 0;
                   crc_received = bootloader_data[15];
                   crc_received =(INT16U)((crc_received<<8) + bootloader_data[16]);
                   
                   crc_received2 = bootloader_data[17];
                   crc_received2 = (INT16U)((crc_received2<<8) + bootloader_data[18]);
                                    
                   if((crc_codigo == crc_received) && (crc_codigo2 == crc_received2)) 
                   {                                               
                      retorno = BOOTLOADER_CRC_OK;
                   }
                   else 
                   {
                      retorno = BOOTLOADER_CRC_ERROR;
                   }
                }
              }
                         
              break;
            default:
              retorno = BOOTLOADER_NOT_DEVICE_TYPE;
              break; 
          }

        break;
      default:
        retorno = BOOTLOADER_NOT_S19;
        break;
      
    }
  
    return(retorno);
  }
  


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////  
    
void WBootloader_Wireless_Flash_EraseAll(INT32U start_address){
       
     INT32U  address = start_address;
     INT32U  i = 0;
     
     OS_SR_SAVE_VAR;
       
     for(i=0;i<FLASH_BLOCKS_1k;i++)          //apaga novamente todas as páginas de memória
     {
      
       OSEnterCritical();
        Flash_Erase(address); // erase function
       OSExitCritical();
       address+=0x400; 
     }

}

void WBootloader_Flash_Init(void){

  #if (BOOTLOADER_ENABLE == 1) 
      /* erase flash memory for code update */ 
    WBootloader_Wireless_Flash_EraseAll(STATUS_ADDR);
  #endif
  
}

//--------------------------polydiv---------------------------------------;
//CRC16_REV
//------------------------------------------------------------------------;
// Usado com a função gera_crc 
// entrada: dados da função gera_crc
// Saída: accum: CRC no campo crc16_teste
static INT16U WBootloader_polydiv (INT8U data_CRC, INT16U accum)
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


INT16U WBootloader_crc(INT16U cont, INT8U *frame)
{
  INT8U i;

  for(i = 0; i < cont; i++)
  {
    CRC16 = WBootloader_polydiv(*frame, CRC16);
    frame++;
  }
  
  return (CRC16);
}    
   

 
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#endif




