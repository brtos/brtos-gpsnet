#include "hardware.h"
#include "BRTOS.h"   
#include "drivers.h"

#include "comm2pc.h"
#include "gpsnet_api.h"
#include "app.h"

#ifndef BOOTLOADER_ENABLE
   #error "Please, enable or disable bootloader app in AppConfig.h file"
#endif

#if (BOOTLOADER_ENABLE == 1)
  #define    SIGNAL_TIMEOUT   200 
  #include "MCF51_Bootloader.h"  
  #include "Bootloader_wireless.h"  
  #include "Bootloader_serial.h" 
#endif 

#pragma warn_implicitconv off

/* Bootloader vars */
 
#if (BOOTLOADER_ENABLE == 1)
      INT32U        endereco = CODE_START;
      BootloaderFSM_states        estado_bootloader = PROG;
      
      INT16U        CRC_CODIGO_16;
      INT16U        CRC_CODIGO_162;

      INT8U         comprimento;
      INT8U         comp;
      INT8U         length;
      INT8U         checksum;
      INT32U        checksum_add;
      INT32U        data_checksum[VECTOR32_SIZE];

      INT8U         confirma_rede=0;
      INT8U         Bootloader_ForceStop = 0;

      INT16U        crc_codigo2=0xffff;
      INT16U        CRC16=0xffff;
      INT16U        crc_codigo=0xffff;
      INT16U        crc_received2=0;
      INT16U        crc_received=0;

static INT16U       boot_node_id = 0;  /* Id do nó que receberá o firmware */
static UINT_DWORD   GPS_X_tmp = {0,0,0,0};  /* Endereço do nó que receberá o firmware */
static UINT_DWORD   GPS_Y_tmp = {0,0,0,0}; 

      BOOTLOADER_DATA_T structured_data;

/* end */
#endif  

/*********************************** 
  Commandos para Bootloader App 
  depende de: Comm2Pc App
************************************/

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

#if (BOOTLOADER_ENABLE == 1)

// o código que será enviado pelo FOTA
CMD_FUNC(FOTA) 
{
    INT8U i = 0;
    INT8U *recebe_pont;
    INT8U recebe = 0;  
    
    static INT8U index = 0;  
    if(index == 0){
      boot_node_id = 0;
      boot_node_id = message;
      index++;      
      return CMD_ARG;
    }
    else
    {
      boot_node_id = (INT16U)((boot_node_id << 8) | message);
      index = 0;           
    }   
    
    // Find target node address in neighbourhood table
    for(i=0;i<NEIGHBOURHOOD_SIZE;i++)
    {
      if (gpsnet_neighbourhood[i].Addr_16b == boot_node_id) 
      {
        UserEnterCritical();
          GPS_X_tmp.int32u  = gpsnet_neighbourhood[i].NeighborGPS_Latitude.x;
          GPS_Y_tmp.int32u  = gpsnet_neighbourhood[i].NeighborGPS_Longitude.y;
        UserExitCritical(); 
        break;       
      }
    } 
   
    if (i < NEIGHBOURHOOD_SIZE){
      
    //CMD_FUNC_UNUSED_ARG();    
      endereco=CODE_START;
      
      Bootloader_ForceStop = 0;
      
      OSSemPost(Bootloader_Event);
      
      if(OSMboxPend(Bootloader_end,(void **)&recebe_pont,TICK_COUNT_OVERFLOW-1) == TIMEOUT){
         
         Bootloader_ForceStop = 1;
         
         OSMboxPend(Bootloader_end,(void **)&recebe_pont,TICK_COUNT_OVERFLOW-1);      
      }
      // Copia o valor passado pela mensagem
      recebe = (INT8U)recebe_pont;
    }
    
    CMD_BEGIN(FOTA);    
    
    PUTCHAR(recebe);
    
    CMD_END();  
}  

// o código que será enviado pelo FOTA para o endereço desejado
CMD_FUNC(FOTA_ADR) 
{
    INT8U i = 0;
    INT8U *recebe_pont;
    INT8U recebe;  
    
    static INT8U index = 0;   
    
        /* recebe argumentos */
    if(index == 0){
      GPS_X_tmp.int32u = 0;
      GPS_Y_tmp.int32u = 0;
    }
    
    if (index < 4)
    {
      GPS_X_tmp.int8u[index] = message;
      index++;
      return CMD_ARG;      
    }else
    {
      if(index < 8) {        
        GPS_Y_tmp.int8u[(index-4)] = message;
        index++; 
        return CMD_ARG;  
      }else{
        if (index >= 8) 
        {         
           index = 0;                      
           goto  do_it;
        }
      } 
    }
    
    do_it: 
    
    //CMD_FUNC_UNUSED_ARG();
    
    endereco=CODE_START;

      Bootloader_ForceStop = 0;
      
      OSSemPost(Bootloader_Event);
      
      i = 0;
      
      while(OSMboxPend(Bootloader_end,(void **)&recebe_pont,TICK_COUNT_OVERFLOW-1) == TIMEOUT){
         
            i++; 
                         
            if(i>3){
              
              Bootloader_ForceStop = 1;
                 
              OSMboxPend(Bootloader_end,(void **)&recebe_pont,TICK_COUNT_OVERFLOW-1);
            
              break;
            }  
      }       
      
      // Copia o valor passado pela mensagem
      recebe = (INT8U)recebe_pont;

    
    CMD_BEGIN(FOTA_ADR);    
    
    PUTCHAR(recebe);
    
    CMD_END();  
}

////////////////////////////////////////////////////////////////////////////
CMD_FUNC(STORE_FIRMWARE) 
{
    BootloaderFSM_status status = DATA_DONE;
       
    status = Bootloader_FSM(message);
    
    switch (status){
      case DATA_DONE:      
          return CMD_ARG; 
      case CRC_OK:
      case DATA_OK:
        break;  
      case ENTRY_BOOTLOADER:
      case EXIT_BOOTLOADER:
      case CRC_ERROR:
      case DATA_ERROR:
      default:      
        status = DATA_ERROR;      
       break;  
    }
                
    CMD_BEGIN(STORE_FIRMWARE);
    
    // PUTCHAR((INT8U)estado_bootloader);
    PUTCHAR((INT8U)status);
    
    CMD_END();  
} 
////////////////////////////////////////////////////////////////////////////

// Apaga bloco de flash onde é armazenado o código que será enviado pelo FOTA
CMD_FUNC(ERASE_FLASH) 
{
    INT32U endereco=CODE_START;
    INT32U i = 0;
    
    CMD_FUNC_UNUSED_ARG();
      
    CMD_BEGIN(ERASE_FLASH);
    
    for(i=0;i<FLASH_BLOCKS_1k;i++) 
    {
       UserEnterCritical();
       Flash_Erase(endereco); // erase function
       UserExitCritical();
       endereco+=0x400; 
    }
    crc_codigo2=0xffff;
    crc_codigo=0xffff;
    
    PUTCHAR('o');
    CMD_END();  
}    

////////////////////////////////////////////////////////////////////////////

/** Task for firmware over the air (FOTA) programming */
BRTOS_Sem    *Confirma;
BRTOS_Sem    *Bootloader_Event;
BRTOS_Mbox   *Bootloader_end;

void GPSNET_FOTA_Task(void)
{
   // task setup
   LATITUDE     latitude;
   LONGITUDE    longitude;   
   INT32U       i = 0;
   INT32U       j = 0;      
   INT32U       endereco_bw;
   INT8U        type = 0;
   INT8U        confirma=0;  
   INT8U        error=0;
   INT8U        nerror=0;   
   INT8U        confirma_rede_error = 0;
   INT8U        *boot_end_status;
   static       INT16U linhas; 
   static       INT16U linhas2;

   
   if (OSSemCreate(0,&Confirma) != ALLOC_EVENT_OK)
   {
     while(1){};
   }; 

   if (OSSemCreate(0,&Bootloader_Event) != ALLOC_EVENT_OK)
   {
     while(1){};
   };      
   
   if (OSMboxCreate(&Bootloader_end,NULL) != ALLOC_EVENT_OK)
   {
      while(1){};      
   }   
   
   
   // task main loop
   for (;;)
   {
      // Wait event from APP layer, no timeout
      OSSemPend(Bootloader_Event,0);
      
      // Set target node address

      UserEnterCritical();
        latitude.x  = GPS_X_tmp.int32u;
        longitude.y = GPS_Y_tmp.int32u;
      UserExitCritical(); 
      
      
      linhas = 0;
      linhas2 = 0;
      endereco_bw = CODE_START;
    
      for(;;) 
      {
           /* loop to read a line of code  */
           for(i=0;i<VECTOR32_SIZE;i++)  
           {
              // return 32-bit words
              data_checksum[i]=Flash_Read(endereco_bw); 
              bootloader_datal[i] = data_checksum[i]; 
              endereco_bw+=4;
           }
            
           linhas++;          
           j=0;                   
          
           type = (INT8U)(bootloader_data[2]);  // get line type
           
           if(type==7) 
           {
              bootloader_data[15] = (INT8U)((CRC_CODIGO_16>>8) & 0xFF);
              bootloader_data[16] = (INT8U)((CRC_CODIGO_16) & 0xFF);
              bootloader_data[17] = (INT8U)((CRC_CODIGO_162>>8) & 0xFF);
              bootloader_data[18] = (INT8U)((CRC_CODIGO_162) & 0xFF);                
              endereco_bw = CODE_START;
           }
           
           error = 0;
           nerror = 0;
           confirma_rede_error = 0;
                    
       envia_de_novo:      
           UserEnterCritical();
             confirma_rede = 0;
             //mensagem = 0;
           UserExitCritical();
             
           // send a line of code         
           i = NetSimpledataXY(UP_ROUTE, &latitude, &longitude, &bootloader_data[0]); 
             
           if(i==OK) 
           {
                 error = 0;
                 // wait for send to complete - SIGNAL_TIMEOUT ms                
                 i = OSSemPend (Confirma, SIGNAL_TIMEOUT); 
                                    
                 if(confirma_rede == DATA_OK) 
                 {
                     confirma_rede_error = 0;
                     linhas2++;
                     if(type==7) 
                       {
                         type=0;                       
                         endereco_bw = CODE_START;
                         //UserEnterCritical();
                          //mensagem = confirma_rede;
                         //UserExitCritical();
                         boot_end_status = (INT8U*)'o';
                         (void)OSMboxPost(Bootloader_end,(void *)boot_end_status);
                         break;
                       }
                 }
                 else
                 {
                     if (confirma_rede == 0)
                     {
                        /* timeout of semaphore pend */
                        confirma_rede_error++;                        
                        // DelayTask(100);
                          
                        if(confirma_rede_error>6) 
                        {
                            confirma_rede_error = 0;
                            
                            //UserEnterCritical();
                            //  mensagem = confirma_rede;
                            //UserExitCritical();                            
                            
                            if(Bootloader_ForceStop == 1){ // desiste 
                              Bootloader_ForceStop = 0;
                              endereco_bw = CODE_START; 
                              boot_end_status = (INT8U*)'f';
                              (void)OSMboxPost(Bootloader_end,(void *)boot_end_status);
                              break;
                            }
                        } 
                        
                        goto envia_de_novo;
                     }
                     endereco_bw = CODE_START;                      
                     //UserEnterCritical();
                     // mensagem = confirma_rede;
                     //UserExitCritical();
                     boot_end_status = (INT8U*)'g';
                     (void)OSMboxPost(Bootloader_end,(void *)boot_end_status);                     
                     break; 
                 } 
           }
           else 
           {
              error++;
              DelayTask(SIGNAL_TIMEOUT);  
              if(error==3) 
              {
                  DelayTask(RadioWatchdogTimeout + SIGNAL_TIMEOUT);   /* wait for a radio reset */
              }
              if(error>6) 
              {
                  error = 0;
                  //UserEnterCritical();
                  //  mensagem = confirma_rede;
                  //UserExitCritical();
                  endereco_bw = CODE_START;
                  boot_end_status = (INT8U*)'f';
                  (void)OSMboxPost(Bootloader_end,(void *)boot_end_status);                  
                  break; 
              } 
              else
              {
                   goto envia_de_novo;  
              }
           }
      }
   }
}

////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////
// Funcao para receber S19 pela rede /////////////////////////////////////////////////////////
void Data_receive(void) 
{
    INT8U i=0;
    INT8U address[3];

      
      switch(app_packet.APP_Command)
        {
          case FILE_S19:
              switch(app_packet.APP_Command_Attribute) 
              {
                case BOOT_COORDINATOR:
                case BOOT_INSTALLER:
                    
                  UserEnterCritical(); 
                    confirma_rede = app_packet.APP_Payload[0]; //armazena a confirmação recebida do roteador
                  UserExitCritical();
                  
                  i++;

                  
                  UserEnterCritical(); 
                  address[0]=app_packet.APP_Payload[i];   //armazena o endereço da linha que foi recebida pelo roteador
                  i++;
                  address[1]=app_packet.APP_Payload[i];
                  i++;
                  address[2]=app_packet.APP_Payload[i];                  
                  UserExitCritical();
                  
                  if (address[0] == bootloader_data[5])      //compara se a linha enviada foi a mesma recebida pelo roteador
                  {
                    if (address[1] == bootloader_data[6])
                    {
                      if (address[2] == bootloader_data[7])
                      {
                         OSSemPost(Confirma); //se a linha enviada for a mesma recebida, então da um post
                      }                            
                    }                        
                  }                  
                  
                    
                  break;
                default:
                  break;
              }
              
            break;
              default:            
              break;
          
        }
}



////////////////////////////////////////////////////////////////////////////
// Função para enviar as linhas
////////////////////////////////////////////////////////////////////////////


INT8U NetSimpledataXY(INT8U direction, LATITUDE *lat, LONGITUDE *lon, INT8U *p_data)
{                                                                          
  INT8U i = 0;
  INT8U j = 0;
   
  acquireRadio();
  
  // copia o endereço de destino
  UserEnterCritical();
    dst_gps_latX = lat->x;
    dst_gps_longY = lon->y;
  UserExitCritical();  

  // monta vetor NWKPayload para transmitir a linha S19 pela rede 
   
  NWKPayload[j++] = APP_01;                               
  NWKPayload[j++] = BULK_DATA_PROFILE;
  NWKPayload[j++] = FILE_S19;  
  NWKPayload[j++] = BOOT_ROUTER;

  //armazena a linha S19 no vetor    
  for(i=0;i<VECTOR8_SIZE;i++)                             
  {
     NWKPayload[j++] = *p_data;
     p_data++;  
  }

  if (direction == DOWN_ROUTE)
  {      
  }    
  else                                                     
  {
     //a transmissão é para um roteador da rede
     //envia vetor de dados
     i = UpRoute(START_ROUTE,(INT8U)(VECTOR8_SIZE + 4));  
  }
  
  releaseRadio();
  return i;
}


BootloaderFSM_status Bootloader_FSM(INT8U pedido)
{
   static INT8U p = 0;
   INT8U k = 0;
   BootloaderFSM_status status = ENTRY_BOOTLOADER;
   
   switch(estado_bootloader)
    {
           case RECEIVE_CRC: //recebe o CRC calculado pelo C#           
             bootloader_data[p] = pedido;
             p++;
             
             if(p>=4) 
             {
                 crc_received = bootloader_data[0];        //armazena o CRC
                 crc_received = (INT16U)((crc_received<<8) + bootloader_data[1]);
                 
                 crc_received2 = bootloader_data[2];
                 crc_received2 = (INT16U)((crc_received2<<8) + bootloader_data[3]);
                 
               if((crc_received == crc_codigo) && (crc_codigo2 == crc_received2)) //verifica se o CRC recebido é igual ao calculado
               {
                 UserEnterCritical();
                 CRC_CODIGO_16 = crc_codigo;
                 CRC_CODIGO_162 = crc_codigo2;                 
                 UserExitCritical();
                 endereco = CODE_START;          //retorna o endereco para seu valor inicial
                 status = CRC_OK;
                 estado_bootloader = PROG;
               } 
               else //se os CRCs não forem iguais apaga toda a memória flash e cancela a transmissao do código
               {                  
                  endereco = CODE_START;
                  for(p=0;p<FLASH_BLOCKS_1k;p++) 
                  {
                     UserEnterCritical();
                     Flash_Erase(endereco); // erase function
                     UserExitCritical();
                     endereco+=0x400; 
                  } 
                  endereco = CODE_START;
                  status = CRC_ERROR;
                  estado_bootloader = PROG;
               }
                 
                 crc_codigo2=0xffff;
                 crc_codigo=0xffff;
             }else{
                status = DATA_DONE;    
             }
                
             break;              

          case FIM:          
            bootloader_data[p] = pedido;            //armazena a ultima linha do arquivo S19
            p++;  
            status = DATA_DONE;
            comprimento--;
            if(comprimento==0)          //quando o comprimento da linha for zero grava na flash
            {                
              /// grava na flash
        
              UserEnterCritical();
                __RESET_WATCHDOG();
                Flash_Prog(endereco,(dword)bootloader_data,VECTOR32_SIZE);
              UserExitCritical();
              
              for(p=0;p<VECTOR32_SIZE;p++)                    //este loop faz a leitura de toda a linha que foi gravada anteriormente
              {
                data_checksum[p]=Flash_Read(endereco);  //a leitura retorna uma palavra de 32bits
                endereco+=4;
              }
                
              comp = (INT8U)(data_checksum[0] & 0xFF);//armazena o comprimento da linha em bytes. 
               
              CRC16 = crc_codigo;
              crc_codigo = gera_crc((INT16U)(data_checksum[0] & 0xFF), &bootloader_data[4]);
              p=0;         
              estado_bootloader = RECEIVE_CRC;
              status = DATA_DONE; 
            }  
            break;
                      
            
          case DADOS:    //neste estado serão recebidos todos os dados da linha
            
            bootloader_data[p]=pedido;     //armazena proximo pedido no vetor
            p++;                //incrementa o indice
            status = DATA_DONE;
            comprimento--;      //decrementa o comprimento da linha 

            if(comprimento==0)   //quando o comprimento for zero a linha acabou e ja pode ser gravada na flash
            {
                //grava na flash
                k=0;
                tenta_de_novo:
                  UserEnterCritical();        //entra em estado critico para poder fazer a gravacao 
                  __RESET_WATCHDOG();     //reseta o watchdog 
                  Flash_Prog(endereco,(dword)bootloader_data,VECTOR32_SIZE); //armazena todo o vetor na posicao indicada pelo endereco
                  UserExitCritical();                      //sai do estado critico

                  
                  for(p=0;p<VECTOR32_SIZE;p++)                    //este loop faz a leitura de toda a linha que foi gravada anteriormente
                  {
                    data_checksum[p]=Flash_Read(endereco);  //a leitura retorna uma palavra de 32bits
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
                     
                    comp = (INT8U)(data_checksum[0] & 0xFF);//armazena o comprimento da linha em bytes. 
                    
                    if (comp == 0xFF)  // caso a leitura retorne 0xFF, então ocorreu erro durante a gravação
                    {
                        k++;
                        if (k<3)
                        {
                          endereco -=40; //volta para o endereço inicial
                          goto tenta_de_novo;   //grava novamente
                        }else                   //se ja foram feitas 3 tentativas sem sucesso, desiste da gravação da linha
                        {
                          status = DATA_ERROR;
                          break;
                        }
                    }
                      
                    p=1;
                    length=0;
                    checksum_add = 0;
                    /*início do cálculo do checksum para a linha recebida*/
                    while(length < (comp-1))   //loop para somar todos os bytes de cada linha, exceto o checksum, tipo e S
                    {
                      checksum_add+=(data_checksum[p] & 0xFF)+(data_checksum[p]>>8 & 0xFF)+((data_checksum[p]>>16) & 0xFF)+((data_checksum[p]>>24) & 0xFF);  //faz o somatorio dos bytes de cada vetor    
                      length+=4;
                      p++;
                    }
                     checksum =(INT8U)((data_checksum[p])>>24 & 0xFF); //armazena o checksum da linha
                     checksum_add+=comp;                               // soma o comprimento ao checksum_add 
                     length=0;
                     // checksum calculado
                     comp=(INT8U)(0xFF - (INT8U)(checksum_add & 0xFF)); 
                     
                     if(checksum==comp)     //se o checksum calculado for igual ao checksum da linha envia uma confirmacao
                     {
                        CRC16 = crc_codigo;
                        crc_codigo = gera_crc((INT16U)(data_checksum[0] & 0xFF), &bootloader_data[4]); //calcula o CRC para a linha inteira
                        CRC16 = crc_codigo2;
                        crc_codigo2 =  gera_crc((INT16U)((data_checksum[0] & 0xFF)-5), &bootloader_data[8]); //calcula o CRC apenas para os dados contidos na linha
                        status = DATA_OK; //volta para o estado LINHA para receber a próxima linha do arquivo S19
                        estado_bootloader = PROG;
                     } else                         //se o checksum nao for o mesmo informa o C# do erro e cancela a transmissão 
                     {
                     
                        endereco=CODE_START; 
                        for(p=0;p<FLASH_BLOCKS_1k;p++)          //apaga novamente todas as paginas de memoria
                        {
                             UserEnterCritical();
                             Flash_Erase(endereco); // erase function
                             UserExitCritical();
                             endereco+=0x400; 
                        }
                        endereco=CODE_START;      //volta para o endereco inicial para reiniciar a gravacao
                        crc_codigo2=0xffff;
                        crc_codigo=0xffff;                            
                        status = EXIT_BOOTLOADER;
                        estado_bootloader = PROG;
                     }          
            }
            break;    
          
          case COMPRIMENTO:            
             comprimento = pedido;                              
             bootloader_data[3]=comprimento;     //armazena no vetor o comprimento da linha 
             p=4;
             if(bootloader_data[2] == 3)        //verifica o tipo, se for 3 muda o estado para DADOS e inicia o recebimento do restante da linha
             {
               estado_bootloader = DADOS;
             }else
             {
               estado_bootloader = FIM;          //se o tipo for diferente de 3 muda o estado para FIM pois chegou ao fim do arquivo S19
             }
             status = DATA_DONE;
             break;  
          
          case PROG: 
            bootloader_data[0]='S';          // armazena no vetor S
            bootloader_data[1]='S';          // armazena no vetor S
            for(k=2;k<VECTOR8_SIZE;k++)      // loop para preencher o vetor de dados
            {
                bootloader_data[k]=0x33;
            }                                               
            if(pedido==3 || pedido ==7)                   //verifica o proximo caracter
            {       

                bootloader_data[2]= pedido;        //se for 3 ou 7 armazena no vetor e muda o estado para comprimento
                estado_bootloader= COMPRIMENTO; 
                status = DATA_DONE;
            }else{
                //se nao for nenhum dos dois volta para o estado normal
                status = EXIT_BOOTLOADER;
                estado_bootloader = PROG;
            }
            break;            
                  
          default:
            status = EXIT_BOOTLOADER;
            estado_bootloader = PROG;
          break;
    }
    
    return status;
  }
  
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------polydiv---------------------------------------;
//CRC16_REV
//------------------------------------------------------------------------;
// Usado com a função gera_crc 
// entrada: dados da função gera_crc
// Saída: accum: CRC no campo crc16_teste
INT16U polydiv (INT8U data_CRC, INT16U accum)
{
  INT16U gerador = 0xa001;
  INT16U test = 0;
  INT8U       i = 0;
  
  accum = (INT16U) (accum ^ (INT16U)data_CRC);
  
  for (i = 0; i <= 7; i++)
  {
    test = (INT16U) (accum & 1);
    accum = (INT16U) (accum >> 1);
    
    if (test == 1)
    {                                                                           
        accum = (INT16U)(accum ^ gerador);
    }
  }
  return(accum);
}


INT16U gera_crc(INT16U cont, byte *frame)
{
  byte i;
  
  for(i = 0; i < cont; i++)
  {
    CRC16 = polydiv(*frame, CRC16);
    frame++;
  } 
  return (CRC16);
}







#endif




