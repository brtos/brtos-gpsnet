/**********************************************************************************
@file   app.c
@brief  GPSNET applications file
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

#include "BRTOS.h"
#include "hardware.h"
#include "app.h"
#include "comm2pc.h"
#include "gpsnet_api.h"


#pragma warn_implicitconv off
#pragma warn_unusedarg off
                                                       

volatile GPS_NODES gps_nodes[APP_NODES];
volatile GPS_NEIGHBOURHOOD_TABLE nb_table[NEIGHBOURHOOD_SIZE]; 




////////////////////////////////////////////////////////////////////////////
// Função para enviar as linhas de código
////////////////////////////////////////////////////////////////////////////
#if 0
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
  NWKPayload[j++] = APP_01;         /* app id */
  NWKPayload[j++] = BULK_DATA_PROFILE;    /* app perfil */
  NWKPayload[j++] = FILE_S19;             /* app cmd */
  NWKPayload[j++] = 0;                    /* cmd atrib */
  
  //armazena a linha S19 no vetor  
  for(i=0;i<VECTOR8_SIZE;i++)  
  {
     NWKPayload[j++] = *p_data;
     p_data++;  
  }

  if (direction == DOWN_ROUTE)
  {      
  }    
  else                                                    //a transmissão é para um roteador da rede
  {
     i = UpRoute(START_ROUTE,(INT8U)(VECTOR8_SIZE + 4));  //envia vetor de dados
  }
  
  releaseRadio();
  return i;
}
#endif




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

/**
* @fn void Decode_General_Profile(void)
* @brief  Funcao que decodifica o perfil General
* @param  None
* @return None
*/

void Decode_General_Profile(void)
{   
    INT8U i = 0;
    INT8U j = 0;
    INT8U Attribute = 0;
    INT8U Value8 = 0;
    INT16U Value16 = 0;
    volatile INT8U x = 0;

    
    switch(app_packet.APP_Command)
    {
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
        
      
      case SNIFER_REP:
        Attribute = app_packet.APP_Command_Attribute;
            
        j=0;  // Antes estava 4
            
        while (i<Attribute) 
        {
            
          nb_table[i].Addr_16b_1  = app_packet.APP_Payload[j++];
          nb_table[i].Addr_16b_2  = app_packet.APP_Payload[j++];
          nb_table[i].NeighborRSSI = app_packet.APP_Payload[j++];
          nb_table[i].Symmetric = app_packet.APP_Payload[j++];
          i++;
        }
        
        // Preenche resto da tabela
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
* @brief  Function to decode Lighting Profile
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




// Function to send sensor measurements through GPSNET
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

// Funcao de comando simples da rede
void NetSimpleCommand(INT8U direction, INT8U CommandType, INT8U Value8, INT16U Value16)
{
  INT8U i = 0;
  INT8U j = 0;
  
  
  acquireRadio();
  
          
  NWKPayload[0] = APP_01;             /* app id */
  NWKPayload[1] = GENERAL_PROFILE;    /* app profile */
  NWKPayload[2] = CommandType;        /* app cmd */
  
  
  switch(CommandType)                  /* app cmd atrib */
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
    //UpSimpleRoute((INT8U)(3+j));
    UpRoute(START_ROUTE,(INT8U)(3+j));
  }
  
  releaseRadio();    
}

// Function to send a Lighting Profile message
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

/** Function to send command to position */
                                                                         
INT8U NetSimpleCommandXY(INT8U direction, LATITUDE *lat, LONGITUDE *lon, INT8U CommandType, INT8U CommandAttribute, INT8U *p_data, INT8U num)
{                                                                          
  INT8U i = 0;
  INT8U j = 0;
   
  acquireRadio();
  
  // copia o endereço de destino
  UserEnterCritical();
    dst_gps_latX = lat->x;
    dst_gps_longY = lon->y;
  UserExitCritical();  

  // monta vetor NWKPayload no formato APP Packet
   
  NWKPayload[j++] = APP_01;                               
  NWKPayload[j++] = GENERAL_PROFILE;
  NWKPayload[j++] = CommandType;
  NWKPayload[j++] = CommandAttribute;

  if(p_data != NULL){  
    for(i=0;i<num;i++)                             
    {
       NWKPayload[j++] = *p_data;
       p_data++;  
    }
  }

  if (direction == DOWN_ROUTE)
  {      
  }    
  else                                                     
  {
     //a transmissão é para um roteador da rede
     i = UpRoute(START_ROUTE,(INT8U)(j));  
  }
  
  releaseRadio();
  return i;
}







/***************************************************************************/
/* DEPRECATED: old functions */     
      
/**
* @fn void Decode_SmartEnergy_Profile(void)
* @brief  Function to decode Smart Energy Profile
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
  

void LogNodeData(INT8U Type, INT8U Value8, INT16U Value16, INT32U Value32)
{
   INT8U  i = 0;
   INT8U  j = 0;
   INT8U  k = 0;

     for(i=0;i<APP_NODES;i++)
     {
        k = 0;
        
        // Verifica se a fonte do pacote ja tem posicao no vetor de recepcao
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
         // Se existe posicao vazia, adiciona a lista
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




