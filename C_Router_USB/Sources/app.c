/***********************************************************************************
@file   app.c
@brief  functions to send and receive data in/from the network
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

/* Config. files */
#include "BRTOSConfig.h"
#include "BoardConfig.h"

#include "gpsnet_api.h" 
#include "app.h" 
#include "sensors.h"
//#include "comm2pc.h"

#if (BOOTLOADER_ENABLE == 1)
  #include "MCF51_Bootloader.h" 
  #include "Bootloader_Wireless.h"    
#endif

#pragma warn_implicitconv off
#pragma warn_unusedarg off  

volatile GPS_NODES gps_nodes[APP_NODES];
volatile GPSNET_NEIGHBOURHOOD_TABLE nb_table[NEIGHBOURHOOD_SIZE]; 

volatile INT8U estado_lampada = OFF;


/************************************************/

INT8U    Config_LUX_THRESHOLD_MIN = 0;
INT8U    Config_LUX_THRESHOLD_MAX = 0;
INT8U    Config_PARENT_THRESHOLD = 0;
INT8U    Config_PARENT_THRESHOLD_MIN = 0;
INT8U    Config_REPORT_PERIOD_1000MS = 2;
INT8U    Config_REPORT_JITTER_100MS = 1; 

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




void SniferReq(INT8U direction, LATITUDE *lat, LONGITUDE *lon) 
{
    
  INT8U j = 0;  
  INT8U status = 0;
  
  //acquireRadio();
  
  // copia o endereço de destino
  UserEnterCritical();
  dst_gps_latX = lat->x;
  dst_gps_longY = lon->y;
  UserExitCritical();
  
  NWKPayload[j++] = APP_02;
  NWKPayload[j++] = GENERAL_PROFILE;
  NWKPayload[j++] = SNIFER_REQ;  
  
    
  if (direction == DOWN_ROUTE)
    status = DownRoute(START_ROUTE,(INT8U)(j));
  else
    status = UpRoute(START_ROUTE,(INT8U)(j));
  
  //releaseRadio();
  //return status;
 
}


// Função de ...
INT8U NetGeneralInfo(INT8U MeasureType, INT8U Value8,INT16U Value16)
{
  INT8U i = 0;
  INT8U j = 0;
  INT8U status = 0;
  
  acquireRadio();

            
  NWKPayload[0] = APP_01;
  NWKPayload[1] = GENERAL_PROFILE;
  NWKPayload[2] = SIMPLE_METERING;
  NWKPayload[3] = MeasureType;
  
  
  switch(MeasureType)
  {
    case TEMPERATURE:
      NWKPayload[4] = (INT8U)Value8;
      j = 1;
      break;
    default:
      NWKPayload[4] = (INT8U)((Value16 & 0xFFFF) >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);
      j = 2;
      break;      
  }
  

  status = DownRoute(START_ROUTE,(INT8U)(4+j));
  
  releaseRadio();
  
  return status;
}


      
/**
* @fn void Decode_SmartEnergy_Profile(void)
* @brief   Function to decode Smart Energy profile
* @param  None
* @return None
*/

void Decode_SmartEnergy_Profile(void)
{   
    INT8U Attribute = 0;
    INT8U k = 0;
    INT16U Value16 = 0;
    INT32U Value32 = 0;
    
    switch(app_packet.APP_Command)
    {
      case SIMPLE_METERING:
        Attribute = app_packet.APP_Command_Attribute;
        
        switch(Attribute)
        {

          case CURRENT:
            Value16 = (INT16U)((app_packet.APP_Payload[0] << 8) + app_packet.APP_Payload[1]);
            LogNodeData(CURRENT,(INT8U)0, (INT16U)Value16, (INT32U)0);
            break;
            
          case VOLTAGE:
            Value16 = (INT16U)((app_packet.APP_Payload[0] << 8) + app_packet.APP_Payload[1]);
            LogNodeData(VOLTAGE,(INT8U)0, (INT16U)Value16, (INT32U)0);
            break;
            
          case ENERGY:
            Value32 = (INT32U)((app_packet.APP_Payload[0] << 24) + (app_packet.APP_Payload[1] << 16) + (app_packet.APP_Payload[2] << 8) + app_packet.APP_Payload[3]);
            LogNodeData(ENERGY,(INT8U)0, (INT16U)0, Value32);
            break;
            
          case ACTIVE_POWER:
            Value32 = (INT32U)((app_packet.APP_Payload[0] << 24) + (app_packet.APP_Payload[1] << 16) + (app_packet.APP_Payload[2] << 8) + app_packet.APP_Payload[3]);
            LogNodeData(ACTIVE_POWER,(INT8U)0, (INT16U)0, Value32);
            break;
            
          case REACTIVE_POWER:
            Value32 = (INT32U)((app_packet.APP_Payload[0] << 24) + (app_packet.APP_Payload[1] << 16) + (app_packet.APP_Payload[2] << 8) + app_packet.APP_Payload[3]);
            LogNodeData(REACTIVE_POWER,(INT8U)0, (INT16U)0, Value32);
            break;
            
          case APPARENT_POWER:
            Value32 = (INT32U)((app_packet.APP_Payload[0] << 24) + (app_packet.APP_Payload[1] << 16) + (app_packet.APP_Payload[2] << 8) + app_packet.APP_Payload[3]);
            LogNodeData(APPARENT_POWER,(INT8U)0, (INT16U)0, Value32);
            break;                                    
            
          case POWER_FACTOR:
            Value16 = (INT16U)((app_packet.APP_Payload[0] << 8) + app_packet.APP_Payload[1]);
            LogNodeData(POWER_FACTOR,(INT8U)0, (INT16U)Value16, (INT32U)0);
            break;                      
            
          default:
            break;
        }
        
        break;
        
      case MULTIPLE_METERING:
        Attribute = app_packet.APP_Command_Attribute;        
        
        while(Attribute)
        {
        
          switch(app_packet.APP_Payload[k])
          {
            case CURRENT:
              Value16 = (INT16U)((app_packet.APP_Payload[k+1] << 8) + app_packet.APP_Payload[k+2]);
              k += 3;
              LogNodeData(CURRENT,(INT8U)0, (INT16U)Value16, (INT32U)0);
              break;
              
            case VOLTAGE:
              Value16 = (INT16U)((app_packet.APP_Payload[k+1] << 8) + app_packet.APP_Payload[k+2]);
              k += 3;
              LogNodeData(VOLTAGE,(INT8U)0, (INT16U)Value16, (INT32U)0);
              break;
              
            case ENERGY:
              Value32 = (INT32U)((app_packet.APP_Payload[k+1] << 24) + (app_packet.APP_Payload[k+2] << 16) + (app_packet.APP_Payload[k+3] << 8) + app_packet.APP_Payload[k+4]);
              k +=5;
              LogNodeData(ENERGY,(INT8U)0, (INT16U)0, Value32);
              break;
              
            case ACTIVE_POWER:
              Value32 = (INT32U)((app_packet.APP_Payload[k+1] << 24) + (app_packet.APP_Payload[k+2] << 16) + (app_packet.APP_Payload[k+3] << 8) + app_packet.APP_Payload[k+4]);
              k +=5;
              LogNodeData(ACTIVE_POWER,(INT8U)0, (INT16U)0, Value32);
              break;
              
            case REACTIVE_POWER:
              Value32 = (INT32U)((app_packet.APP_Payload[k+1] << 24) + (app_packet.APP_Payload[k+2] << 16) + (app_packet.APP_Payload[k+3] << 8) + app_packet.APP_Payload[k+4]);
              k +=5;
              LogNodeData(REACTIVE_POWER,(INT8U)0, (INT16U)0, Value32);
              break;
              
            case APPARENT_POWER:
              Value32 = (INT32U)((app_packet.APP_Payload[k+1] << 24) + (app_packet.APP_Payload[k+2] << 16) + (app_packet.APP_Payload[k+3] << 8) + app_packet.APP_Payload[k+4]);
              k +=5;
              LogNodeData(APPARENT_POWER,(INT8U)0, (INT16U)0, Value32);
              break;                                    
              
            case POWER_FACTOR:
              Value16 = (INT16U)((app_packet.APP_Payload[k+1] << 8) + app_packet.APP_Payload[k+2]);
              k +=3;
              LogNodeData(POWER_FACTOR,(INT8U)0, (INT16U)Value16, (INT32U)0);
              break;            
              
            default:
              break;
          }        
        
          Attribute--;
        }
        
        
      
      default:
        break;
    }
}

void SniferRep(INT8U direction, LATITUDE *lat, LONGITUDE *lon) 
{
  INT8U i = 0;
  INT8U j = 0;
  INT8U k = 0;  
  INT8U status = 0;
  INT8U cnt = 0;
    
  
  // copia o endereço de destino
  UserEnterCritical();
    dst_gps_latX = lat->x;
    dst_gps_longY = lon->y;
  UserExitCritical();
                                                                    
  NWKPayload[j++] = APP_01;
  NWKPayload[j++] = GENERAL_PROFILE;
  NWKPayload[j++] = SNIFER_REP; 
  
  //conta quantos vizinhos tem
  cnt = 0; 
  for(i=0;i<NEIGHBOURHOOD_SIZE;i++) 
  {
    if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE) 
    {
      cnt++;  
    }
  } 
  
  NWKPayload[j++] = cnt; 
    
  for(i=0;i<NEIGHBOURHOOD_SIZE;i++) 
  {
    if (gpsnet_neighbourhood[i].Addr_16b != 0xFFFE) 
    {                      
      NWKPayload[j++] = (INT8U)(gpsnet_neighbourhood[i].Addr_16b >> 8);
      NWKPayload[j++] = (INT8U)(gpsnet_neighbourhood[i].Addr_16b & 0xFF);
      NWKPayload[j++] = gpsnet_neighbourhood[i].NeighborRSSI;
      NWKPayload[j++] = gpsnet_neighbourhood[i].NeighborStatus.bits.Symmetric;
      
      // ParentCnt
      cnt = 0;
      for(k=0;k<BaseCnt;k++) 
      {
        if (BaseStations[k].NeighborID == gpsnet_neighbourhood[i].Addr_16b) 
        {
          cnt++;
        }
      }
      
      NWKPayload[j++] = cnt;
    }
  }
  
 
  if (direction == DOWN_ROUTE)
    status = DownRoute(START_ROUTE,(INT8U)(j));
  else
    status = UpRoute(START_ROUTE,(INT8U)(j));
}




/**
* @fn void Decode_General_Profile(void)
* @brief  Function to decode General Profile
* @param  None
* @return None
*/

void Decode_General_Profile(void)
{   
    INT8U i = 0;
    INT8U j = 0;
    INT8U estado;
    INT8U Attribute = 0;
    INT8U Value8 = 0;
    INT16U Value16 = 0;
    LATITUDE   lat;
    LONGITUDE  lon;  
    
    switch(app_packet.APP_Command)
    {
      case GENERAL_ONOFF:
        estado = app_packet.APP_Command_Attribute;
        if (estado == ON)
        {
            PIN_RELAY = ON;
        }
        if(estado == OFF)
        {          
            PIN_RELAY = OFF;
        }  
        break;
      
      case SIMPLE_METERING:
        Attribute = app_packet.APP_Command_Attribute;
        
        switch(Attribute)
        {
          case TEMPERATURE:
            Value8 = (INT8U)app_packet.APP_Payload[0];
            LogNodeData(TEMPERATURE,(INT8U)Value8, (INT16U)0, (INT32U)0);
            break;
            
          case DEBUG_COUNTER:
            Value16 = (INT16U)((app_packet.APP_Payload[0] << 8) + app_packet.APP_Payload[1]);
            LogNodeData(LIGHT_LEVEL,(INT8U)0, (INT16U)Value16, (INT32U)0);
            break;            
            

          default:
            break;
        }
        
        break;
        
      case SNIFER_REQ:
        for(i=0;i<4;i++)
        {
          lat.bytes[i] = nwk_packet.NWK_Src_Lat[i];
          lon.bytes[i] = nwk_packet.NWK_Src_Long[i];
        }            
        
        SniferRep(UP_ROUTE, &lat, &lon);
        
        break;             
        
      
      case SNIFER_REP:
        Attribute = app_packet.APP_Command_Attribute;
            
        j=0;
            
        while (i<Attribute) 
        {
            
          nb_table[i].Addr_16b_1  = app_packet.APP_Payload[j++];
          nb_table[i].Addr_16b_2  = app_packet.APP_Payload[j++];
          nb_table[i].NeighborRSSI = app_packet.APP_Payload[j++];
          nb_table[i].Symmetric = app_packet.APP_Payload[j++];
          i++;
        }
        
        // Fill table with empty data
        for(i=Attribute;i<8;i++) 
        {
          nb_table[i].Addr_16b_1  = 0xFF;
          nb_table[i].Addr_16b_2  = 0xFF;    
        }
                            
        break;  
      
      default:
        break;
    }
}

/**
* @fn void Decode_Lighting_Profile(void)
* @brief  Funcao que decodifica o perfil Lighting
* @param  None
* @return None
*/

void Decode_Lighting_Profile(void)
{   
    INT8U Attribute = 0;
    INT16U Value16 = 0;
    
    switch(app_packet.APP_Command)
    {
      case SIMPLE_METERING:
        Attribute = app_packet.APP_Command_Attribute;
        
        switch(Attribute)
        {
          case LIGHT_LEVEL:
            Value16 = (INT16U)((app_packet.APP_Payload[0] << 8) + app_packet.APP_Payload[1]);
            LogNodeData(LIGHT_LEVEL,(INT8U)0, (INT16U)Value16, (INT32U)0);
            break;
            
          default:
            break;
        }
        
        break;
      
      default:
        break;
    }
}




// Function to send sensor measurements
void NetSimpleMeasure(INT8U direction, INT8U MeasureType, INT16U Value16, INT32U Value32)
{
  INT8U j = 0;
  
  
  acquireRadio();
  
          
  NWKPayload[0] = APP_01;
  NWKPayload[1] = SMART_ENERGY_PROFILE;
  NWKPayload[2] = SIMPLE_METERING;
  NWKPayload[3] = MeasureType;
  
  
  switch(MeasureType)
  {
    case CURRENT:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);
      j = 2;
      break;
    case VOLTAGE:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);            
      j = 2;
      break;
    case ENERGY:
      NWKPayload[4] = (INT8U)(Value32 >> 24);
      NWKPayload[5] = (INT8U)((Value32 & 0xFFFFFF) >> 16);
      NWKPayload[6] = (INT8U)((Value32 & 0xFFFF) >> 8);
      NWKPayload[7] = (INT8U)(Value32 & 0xFF);
      j = 4;
      break;
    
  }
  
  if (direction == DOWN_ROUTE)
    DownRoute(START_ROUTE,(INT8U)(4+j));
  else
    UpRoute(START_ROUTE,(INT8U)(4+j));
  
  releaseRadio();
}



void LogNodeData(INT8U Type, INT8U Value8, INT16U Value16, INT32U Value32)
{
   INT8U  i = 0;
   INT8U  j = 0;
   INT8U  k = 0;

     for(i=0;i<APP_NODES;i++)
     {
        k = 0;
        
        // Verifica se o no fonte do pacote ja tem posicao no vetor de recepcao
        for(j=0;j<4;j++)
        {
          if (gps_nodes[i].GPS_Latitude[j] == nwk_packet.NWK_Src_Lat[j])
            k++;
        }

        for(j=0;j<4;j++)
        {
          if (gps_nodes[i].GPS_Longitude[j] == nwk_packet.NWK_Src_Long[j])
            k++;
        }

        if (k == 8)
        {
          j = i;
          break;
        }
     }
     
     
     if (k == 8)
     {
        // Se ja esta na lista
        // Atualiza o valor da corrente
        UserEnterCritical();
        gps_nodes[j].NodeID = mac_packet.SrcAddr_16b;
        UserExitCritical();
        
        switch(Type)
        {
          case CURRENT:
            UserEnterCritical();
            gps_nodes[j].Current = Value16;
            UserExitCritical();
            break;
          case VOLTAGE:
            UserEnterCritical();
            gps_nodes[j].Voltage = Value16;
            UserExitCritical();
            break;          
          case ENERGY:
            UserEnterCritical();
            gps_nodes[j].Energy = (INT32U)Value32;
            UserExitCritical();
            break;
          case ACTIVE_POWER:
            UserEnterCritical();
            gps_nodes[j].ActivePower = (INT32U)Value32;
            UserExitCritical();
            break;
          case REACTIVE_POWER:
            UserEnterCritical();
            gps_nodes[j].ReactivePower = (INT32U)Value32;
            UserExitCritical();
            break;
          case APPARENT_POWER:
            UserEnterCritical();
            gps_nodes[j].ApparentPower = (INT32U)Value32;
            UserExitCritical();
            break;                                    
          case POWER_FACTOR:
            UserEnterCritical();
            gps_nodes[j].PowerFactor = Value16;
            UserExitCritical();
            break;            
          case TEMPERATURE:
            UserEnterCritical();
            gps_nodes[j].Temperature = (INT8U)Value8;
            UserExitCritical();
            break;
          case LIGHT_LEVEL:
            UserEnterCritical();
            gps_nodes[j].LightLevel = (INT16U)Value16;
            UserExitCritical();
            break;
          default:
            break;
        }
        
        gps_nodes[j].Hops    = nwk_packet.NWK_Packet_Life;
        gps_nodes[j].PacketsCount++;
        
     }else
     {
        // Senao entra na lista
        // Procurando posicao vazia
       for(i=0;i<APP_NODES;i++)
       {
          k = 0;
          for(j=0;j<4;j++)
          {
            if (gps_nodes[i].GPS_Latitude[j] == 0)
              k++;
          }

          for(j=0;j<4;j++)
          {
            if (gps_nodes[i].GPS_Longitude[j] == 0)
              k++;
          }

          if (k == 8)
          {
            j = i;
            break;
          }
       }
       
       
       if (k == 8)
       {
         // Se existe posiÃ§Ã£o vazia, adiciona a lista
         for(i=0;i<4;i++)
         {
           gps_nodes[j].GPS_Latitude[i] = nwk_packet.NWK_Src_Lat[i];
         }
         for(i=0;i<4;i++)
         {
           gps_nodes[j].GPS_Longitude[i] = nwk_packet.NWK_Src_Long[i];
         }
         
        // Se ja esta na lista
        // Atualiza o valor da corrente
        UserEnterCritical();
        gps_nodes[j].NodeID = mac_packet.SrcAddr_16b;      
        UserExitCritical();
         
         
        switch(Type)
        {
          case CURRENT:
            UserEnterCritical();
            gps_nodes[j].Current = Value16;
            UserExitCritical();
            break;
          case VOLTAGE:
            UserEnterCritical();
            gps_nodes[j].Voltage = Value16;
            UserExitCritical();
            break;          
          case ENERGY:
            UserEnterCritical();
            gps_nodes[j].Energy = (INT32U)Value32;
            UserExitCritical();
            break;
          case ACTIVE_POWER:
            UserEnterCritical();
            gps_nodes[j].ActivePower = (INT32U)Value32;
            UserExitCritical();
            break;
          case REACTIVE_POWER:
            UserEnterCritical();
            gps_nodes[j].ReactivePower = (INT32U)Value32;
            UserExitCritical();
            break;
          case APPARENT_POWER:
            UserEnterCritical();
            gps_nodes[j].ApparentPower = (INT32U)Value32;
            UserExitCritical();
            break;                                    
          case POWER_FACTOR:
            UserEnterCritical();
            gps_nodes[j].PowerFactor = Value16;
            UserExitCritical();
            break;            
          case TEMPERATURE:
            UserEnterCritical();
            gps_nodes[j].Temperature = (INT8U)Value8;
            UserExitCritical();
            break;
          case LIGHT_LEVEL:
            UserEnterCritical();
            gps_nodes[j].LightLevel = (INT16U)Value16;
            UserExitCritical();
            break;
          default:
            break;
        }
        gps_nodes[j].Hops    = nwk_packet.NWK_Packet_Life;
        gps_nodes[j].PacketsCount++;
       }
     }
}




// Funcao de comando simples da rede
void NetSimpleCommand(INT8U direction, INT8U CommandType, INT8U Value8, INT16U Value16)
{
  INT8U i = 0;
  INT8U j = 0;
  
  
  acquireRadio();
  
          
  NWKPayload[0] = APP_01;
  NWKPayload[1] = GENERAL_PROFILE;
  NWKPayload[2] = CommandType;
  
  
  switch(CommandType)
  {
    case GENERAL_ONOFF:
      NWKPayload[3] = (INT8U)(Value8);
      j = 1;
      break;
    case VOLTAGE:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);            
      j = 2;
      break;
  }
  
  if (direction == DOWN_ROUTE)
  {
    
  }
  else
  {
    UpSimpleRoute((INT8U)(3+j));
  }
  
  releaseRadio();    
}




// Funcao de mensagem do Lighting Profile
INT8U NetLightingProfile(INT8U direction, INT8U Command, INT8U Parameter, INT8U Value8, INT16U Value16, INT32U Value32)
{
  INT8U i = 0;
  INT8U j = 0;
  
  acquireRadio();
          
  NWKPayload[0] = APP_01;
  NWKPayload[1] = LIGHTING_PROFILE;
  NWKPayload[2] = Command;

  // Analisa qual o comando a ser executado
  switch(Command)
  {
    case SIMPLE_METERING:
      NWKPayload[3] = Parameter;
      
      switch(Parameter)
      {
        case LIGHT_LEVEL:
          NWKPayload[4] = (INT8U)(Value16 >> 8);
          NWKPayload[5] = (INT8U)(Value16 & 0xFF);
          j = 2;
          break; 
        case LAMP_STATE:
          NWKPayload[4] = (INT8U)Value8;
          j = 1;
          break;   
      }
      break;
      
    
    case LIGHTING_DIMMING:
      NWKPayload[3] = Parameter;
      break;
      
           
  }
  
  if (direction == DOWN_ROUTE)
  {
    #if (DEVICE_TYPE != PAN_COORDINATOR)
      i = DownRoute(START_ROUTE,(INT8U)(4+j));
    #else                                             
      i = ROUTE_NODE_ERROR;
    #endif
  }
  else
  {
    i = UpRoute(START_ROUTE,(INT8U)(4+j));
  }
  
  releaseRadio();
  return i;
}

/**
* @fn     NetMultiMeasureSE
* @brief  function to send message for Smart Energy profile
**/
INT8U NetMultiMeasureSE(SE_STRUCT *se)
{

  INT8U i = 0;
  INT8U j = 0;  
  INT8U count = 0; 
  INT8U status = 0;
  
  acquireRadio();
  
  NWKPayload[j++] = APP_01;
  NWKPayload[j++] = SMART_ENERGY_PROFILE;
  NWKPayload[j++] = MULTIPLE_METERING;   
  NWKPayload[j++] = se->params.Byte;
  
  //verifica se o bit voltage da variavel params é 1. Se for envia.  
  if(se->params.Bits.Voltage)      
  {
      //NWKPayload[j++] = VOLTAGE;
      NWKPayload[j++] = (INT8U)(se->v_rms >> 8);
      NWKPayload[j++] = (INT8U)(se->v_rms & 0xFF);             
  }
  //verifica se o bit current da variavel params é 1. Se for envia     
  if(se->params.Bits.Current)   
  {
      //NWKPayload[j++] = CURRENT;
      NWKPayload[j++] = (INT8U)(se->i_rms >> 8);
      NWKPayload[j++] = (INT8U)(se->i_rms & 0xFF);             
  }
  
  
  if(se->params.Bits.Power_Factor)      
  {
      //NWKPayload[j++] = POWER_FACTOR;
      NWKPayload[j++] = (INT8U)(se->power_factor >> 8);
      NWKPayload[j++] = (INT8U)(se->power_factor & 0xFF);            
  }
  
  
  if(se->params.Bits.Apparent_Power)
  {
      //NWKPayload[j++] = APPARENT_POWER;
      NWKPayload[j++] = (INT8U)(se->power_S >> 24);
      NWKPayload[j++] = (INT8U)((se->power_S & 0xFFFFFF) >> 16);
      NWKPayload[j++] = (INT8U)((se->power_S & 0xFFFF) >> 8);
      NWKPayload[j++] = (INT8U)(se->power_S & 0xFF);            
  }
  
  
  if(se->params.Bits.Reactive_Power)      
  {
      //NWKPayload[j++] = REACTIVE_POWER;
      NWKPayload[j++] = (INT8U)(se->power_Q >> 24);
      NWKPayload[j++] = (INT8U)((se->power_Q & 0xFFFFFF) >> 16);
      NWKPayload[j++] = (INT8U)((se->power_Q & 0xFFFF) >> 8);
      NWKPayload[j++] = (INT8U)(se->power_Q & 0xFF);        
  }
      
  
  if(se->params.Bits.Active_Power)      
  {
      //NWKPayload[j++] = ACTIVE_POWER;
      NWKPayload[j++] = (INT8U)(se->power_P >> 24);
      NWKPayload[j++] = (INT8U)((se->power_P & 0xFFFFFF) >> 16);
      NWKPayload[j++] = (INT8U)((se->power_P & 0xFFFF) >> 8);
      NWKPayload[j++] = (INT8U)(se->power_P & 0xFF);           
  }
  
  
  if(se->params.Bits.Energy)
  {
      //NWKPayload[j++] = ENERGY;
      NWKPayload[j++] = (INT8U)(se->energy_meter >> 24);
      NWKPayload[j++] = (INT8U)((se->energy_meter & 0xFFFFFF) >> 16);
      NWKPayload[j++] = (INT8U)((se->energy_meter & 0xFFFF) >> 8);
      NWKPayload[j++] = (INT8U)(se->energy_meter & 0xFF);           
  }  
  
  
  status = DownRoute(START_ROUTE,(INT8U)(j));  
  
  releaseRadio(); 
  
  return status;
}


/**
* @fn     NetSimpleMeasureSE
* @brief  function to send a message for Smart Energy profile
**/

INT8U NetSimpleMeasureSE(INT8U MeasureType, INT16U Value16, INT32U Value32)
{
  INT8U i = 0;
  INT8U j = 0;
  INT8U status = 0;  
  
  acquireRadio();
  
          
  NWKPayload[0] = APP_01;
  NWKPayload[1] = SMART_ENERGY_PROFILE;
  NWKPayload[2] = MULTIPLE_METERING;
  NWKPayload[3] = MeasureType;
  
  
  switch(MeasureType)
  {
    case CURRENT:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);
      j = 2;
      break;
    case VOLTAGE:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);            
      j = 2;
      break;
    case ENERGY:
      NWKPayload[4] = (INT8U)(Value32 >> 24);
      NWKPayload[5] = (INT8U)((Value32 & 0xFFFFFF) >> 16);
      NWKPayload[6] = (INT8U)((Value32 & 0xFFFF) >> 8);
      NWKPayload[7] = (INT8U)(Value32 & 0xFF);
      j = 4;
      break;
    case ACTIVE_POWER:
      NWKPayload[4] = (INT8U)(Value32 >> 24);
      NWKPayload[5] = (INT8U)((Value32 & 0xFFFFFF) >> 16);
      NWKPayload[6] = (INT8U)((Value32 & 0xFFFF) >> 8);
      NWKPayload[7] = (INT8U)(Value32 & 0xFF);
      j = 4;
      break;
    case REACTIVE_POWER:
      NWKPayload[4] = (INT8U)(Value32 >> 24);
      NWKPayload[5] = (INT8U)((Value32 & 0xFFFFFF) >> 16);
      NWKPayload[6] = (INT8U)((Value32 & 0xFFFF) >> 8);
      NWKPayload[7] = (INT8U)(Value32 & 0xFF);
      j = 4;
      break;
    case APPARENT_POWER:
      NWKPayload[4] = (INT8U)(Value32 >> 24);
      NWKPayload[5] = (INT8U)((Value32 & 0xFFFFFF) >> 16);
      NWKPayload[6] = (INT8U)((Value32 & 0xFFFF) >> 8);
      NWKPayload[7] = (INT8U)(Value32 & 0xFF);
      j = 4;
      break;
    case POWER_FACTOR:
      NWKPayload[4] = (INT8U)(Value16 >> 8);
      NWKPayload[5] = (INT8U)(Value16 & 0xFF);            
      j = 2;
      break;
    
  }
  
  
  status = DownRoute(START_ROUTE,(INT8U)(4+j));   
  releaseRadio();
  
  return status;
}


INT8U NetClearSE(SE_STRUCT *se){

  se->params.Byte = 0;
  se->v_rms = 0;
  se->i_rms = 0;
  se->power_factor=0;
  se->power_S=0;
  se->power_Q=0;
  se->power_P=0;  
  se->energy_meter=0;
}





