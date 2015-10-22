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


/* MCU and OS includes */
#include "hardware.h"
#include "BRTOS.h"
#include "drivers.h"

/* Config. files */
#include "BRTOSConfig.h"
#include "AppConfig.h"
#include "NetConfig.h"
#include "BoardConfig.h"

/* Function prototypes */
#include "gpsnet_api.h" 
#include "app.h" 
#include "tasks.h"
#include "math_utils.h"
#include "sensors.h"
#include "smartmeter.h"

#pragma warn_implicitconv off 

/************************************************************************/
/* Lamp control task global variables
/************************************************************************/
static BRTOS_Sem         *Rele_Event;  
static INT32U            TurnOn_Voltage = 0;

/*************************************************/
/* Task: keep watchdog happy and system time     */
/*************************************************/
void System_Time(void)
{
   /* task setup */
   INT16U i = 0;   
   OSResetTime();
  
   /* task main loop */
   for (;;)
   {
      #if (WATCHDOG == 1)
        __RESET_WATCHDOG();
      #endif
 
      DelayTask(10); /* wait 10 ticks -> 10 ms */
      
      #if RADIO_DRIVER_WATCHDOG == 1      
           Radio_Count_states();
      #endif         
      
      
      i++;    /* count time */
      
      
      /* debug para indicar recep��o de pacotes */
      #if (defined DEBUG_EXATRON && DEBUG_EXATRON == 1)
        if (i%20 == 0){        
             if (mac_tasks_pending.bits.isDataFrameRxed == 1){
                #if (BDM_ENABLE == 0)
                  BDM_DEBUG_OUT = ~BDM_DEBUG_OUT;  /* toggle DEBUG OUT */
                #endif 
             }
        }
      #endif 
      
      if (i >= 100)
      {
      
        OSUpdateUptime();
        i = 0;
        
        #if (defined DEBUG_EXATRON && DEBUG_EXATRON == 1)     
           if (mac_tasks_pending.bits.isAssociated == 0){              
              #if (BDM_ENABLE == 0)
                BDM_DEBUG_OUT = ~BDM_DEBUG_OUT;  /* toggle DEBUG OUT */
              #endif   
           }
           
           if (mac_tasks_pending.bits.isDataFrameRxed == 1){
              if(BDM_DEBUG_OUT == 0){  /* stop toggling with LED ON */
                mac_tasks_pending.bits.isDataFrameRxed = 0;
              } 
           }           
        #endif      

      }
   }
}

/*************************************************************/
/* Task: Reports all measurements (including "smart energy") */
/* to the nearest base station at every REPORTING_PERIOD_MS  */
/* milliseconds, plus RadioRand()* REPORTING_JITTER_MS       */
/*************************************************************/

#define SMART_ENERGY_APP_REPORT_ENABLE 

void GPSNET_SensorsApp(void)
{
   /* task setup */
   INT8U        status = 0;
   INT8U        Temperature = 0; 
   INT16U       Light = 0;
   INT8U        *Stats;
   FAIL_T       Failure;  

#ifdef SMART_ENERGY_APP_REPORT_ENABLE           
   SE_STRUCT    SmartEnergy;
#endif    

   //#define TEST_PARENT_DOWN
      
   #ifdef TEST_PARENT_DOWN
      LATITUDE   lat;
      LONGITUDE  lon;
      INT8U      seq = 0;
   #endif    
   
   /* enable sensors for Temperature and Light measurements */
   SensorTemp_Enable(); 
   SensorLight_Enable();       
   
   /* wait first connection */ 
   while (BetterDepth >= ROUTE_TO_BASESTATION_LOST){
      DelayTask(1000);
   }
  
   /* task main loop */
   for (;;)
   {
      
      /* this is very important !!! do not remove !!! */
      UserEnterCritical();
        dst_gps_latX = BaseStations[NearBase].GPS_LatX;
        dst_gps_longY = BaseStations[NearBase].GPS_LongY;
      UserExitCritical();
      
      
      /* set tricker delay */
      if (Config_REPORT_PERIOD_1000MS < (MAX_REPORTING_PERIOD_MS/REPORTING_PERIOD_MS)/2){
          Config_REPORT_PERIOD_1000MS <<= 1; /* double report period */
      }
      
#if 0       
      
      /* Smart Lamp Message */
      if((status = SensorLight_GetLast(&Light)) != SENSOR_OK){
        Light = (INT16U)(-1);        
      }
      if((status = SensorTemp_Get(&Temperature)) != SENSOR_OK){
        Temperature = (INT8U)(-1); 
      }
      
      status = NetSmartLamp(Get_LampStateReg(),(INT16S)Light,(INT8S)Temperature);
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);
#endif      
      

#ifdef SMART_ENERGY_APP_REPORT_ENABLE 

      /* Smart energy reporting */ 
      Smartmeter_GetValues(&SmartEnergy);
      status = NetMultiMeasureSE(&SmartEnergy);          
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);             
     
#endif
            
      
#if 1     
      /* Lamp state reporting */            
      status = NetLightingProfile(SIMPLE_METERING, LAMP_STATE, Get_LampStateReg(), (INT16U)0);       
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);

      /* Light level measurement and reporting */
      if((status = SensorLight_GetLast(&Light)) == SENSOR_OK)
      {
          /* ajuste p/ mV para LUX */
          Light = Light*10 - 2425;
          Light = (INT16U) ((INT32S)Light/(INT32S)503);
          status = NetLightingProfile(SIMPLE_METERING, LIGHT_LEVEL, (INT8U)0, (INT16U)Light);
      }else{
          status = NetLightingProfile(SIMPLE_METERING, SENSOR_STATE, (INT8U)status, (INT16U)0);
      }      
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);
      
      
      /* Temperature measurement and reporting */                     
      if((status = SensorTemp_Get(&Temperature)) == SENSOR_OK)
      {                  
          status = NetGeneralInfo(TEMPERATURE, (INT8U)Temperature, (INT16U)0);
      }               
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);
#endif      
      
      /* GPSNET statistics reporting */                     
      if((Stats = GetGPSNET_Statistics(&status)) != NULL)
      {                                    
         GPSNET_TX_ToBaseSation(GPSNET_INFO, GPSNET_STAT, (UINT_DWORD*)Stats, 28);  
      }      
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);
      
      
      /* GPSNET connectivity reporting */
      acquireRadio();       
        SniferRep(DOWN_ROUTE, macAddr, NULL, NULL);
      releaseRadio();      
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS); 
      
      
      /* Failure reporting */
      ReportFailure_Get(&Failure);                          
      if(Failure != NO_FAIL)
      {                  
          status = NetGeneralInfo(FAILURE_REPORT, (INT8U)Failure, (INT16U)0);
      }               
      DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS); 
        
      
#ifdef TEST_PARENT_DOWN
      
        DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*REPORTING_JITTER_MS); 
      
        #if (BDM_ENABLE == 0)
          BDM_DEBUG_OUT = 0;  /* turn on DEBUG OUT */
        #endif
        
        lat.x = BaseStations[NearBase].Par_GPS_Latitude.x;
        lon.y = BaseStations[NearBase].Par_GPS_Longitude.y;
        
        DelayTask(1000);        
      
        #if (BDM_ENABLE == 0)
          BDM_DEBUG_OUT = 1;  /* turn off DEBUG OUT */
        #endif
      
        DelayTask(15000);         
        
        if (BetterDepth >= 1 && BetterDepth < 254)
        {         
          NetDebugPacket(DEBUG_PKT, DEBUG_COUNTER, (INT8U)0, (INT16U)seq, &lat, &lon);                 
          ++seq;
        }
        
        DelayTask(Config_REPORT_PERIOD_1000MS*REPORTING_PERIOD_MS + RadioRand()*Config_REPORT_JITTER_100MS*REPORTING_JITTER_MS);
      
#endif  

       
                  
   }
}



/************************************************************************/
/* GPSNET Applications                                                  */
/************************************************************************/

// Sanity check
#ifndef SIGNAL_APP1
  #error  "Please define SIGNAL_APP1 for this app"
#endif

/* For bootloader - only if enabled */ 
#if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)  
#define   SIGNAL_TIMEOUT   200
#include "MCF51_Bootloader.h"  
#include "Bootloader_wireless.h"  
#else
#define   SIGNAL_TIMEOUT   0
#endif

void GPSNET_App(void)
{
   
   INT8U  ret = 0;
   
   /* task main loop */
   for (;;)
   {
     
      /* Wait event from APP layer, with or without timeout */
      ret = OSSemPend(SIGNAL_APP1, SIGNAL_TIMEOUT);      
      
      #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)       
      if(ret != TIMEOUT){      
      #endif
       
       acquireRadio();
       
       switch(app_packet.APP_Profile) 
       {
        case GENERAL_PROFILE:
          Decode_General_Profile();
          break;
          
        case LIGHTING_PROFILE:
          Decode_Lighting_Profile();
          break;
        
        #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)  
        case BULK_DATA_PROFILE:
          WBootloader_Handler();
          break;
        #endif
        
        default:
          break;
       }        
       releaseRadio();
      
      #if (defined BOOTLOADER_ENABLE) && (BOOTLOADER_ENABLE==1)       
      }else{          
        WBootloader_Handler_Timeout();         
      }
      #endif 

   }
}

/************************************************************************/
/* Task: controls lamp/relay state (ON/OFF) according to the light level
/************************************************************************/

void RelayControl_Task(void)
{
   // task setup
   INT16U   i = 0;   
      
   INT16U   cnt_light_on = 0;
   INT16U   cnt_light_off = 0;
   INT16U   luz = 0;
   INT8U    sensorlight_state;
      
   INT32U   nivel_cc_corrente = 0;
   INT32U   Iload = 0;
   INT16U   Imax = 0;
   INT16U   Imin = 0;
   
   INT32U   nivel_cc_rede =0;
   INT32U   v_entrada = 0;
   INT32U   v_disparo = 0;
   INT16U   turnon_time = RELAY_TURN_ON_TIME; /* estimado para o rel� */
   
   INT32U   energy_meter = 0;
   INT8U    without_load_cnt = 0;
      
   
   
   if ((INT8U)OSSemCreate(0,&Rele_Event) != ALLOC_EVENT_OK)
   {
      
      while(1){UserEnterCritical();};  /* For�a reset */
   }             

   /* Habilita sensor de ilumina��o */   
   SensorLight_Enable();
   
   while (SensorLight_Get(&luz) != SENSOR_OK){   
      DelayTask(100);
   }
   
   DelayTask(100);
   
   /* Aguarda habilita��o dos sensores de tens�o e corrente */ 
   
   while(SensorCurrent_Status() != SENSOR_ENABLED && SensorVoltage_Status() != SENSOR_ENABLED){      
      ReportFailure_Set(TASK_FAIL_INIT);
      DelayTask(1000);
   } 
    
   ReportFailure_Set(NO_FAIL);              
   
   for(;;) 
   {
 
         /* 
        a l�mpada deve ser ligada entre 6-15lux e 
        desligada em no m�ximo 25lux.
        para o acionamento da carga deve haver 
        um tempo de retardo m�nimo de 5 segundos 
        e m�ximo em torno de 20 segundos,para 
        prote��o contra incid�ncia de ilumina��o 
        transit�ria, raios, far�is de carro ou 
        outras mudan�as na luz ambiente.
        */
        
        medicao_sensor_luz:
        sensorlight_state = SensorLight_Get(&luz);
               
        if (sensorlight_state != SENSOR_OK){
            if(sensorlight_state == SENSOR_ERROR_BANDGAP){                   
                   ADC_Bandgap_Set();
                   goto medicao_sensor_luz;
            }
            
            if(sensorlight_state == SENSOR_DISABLED){
            
                /* sensor foi desabilitado para for�ar o ligamento 
                da lampada durante o dia. Emula-se um valor de 
                ilumina��o baixo para for�ar ligamento. */
                SensorLight_Enable(); 
                SensorLight_Get(&luz);
                if(luz > LUX_MIN){
                   luz = LUX_MIN;
                   SensorLight_Disable();    /* desabilita sensor novamente */                   
                } 
                
                #if (defined TEST_PIN && TEST_PIN == 1)   
                  PIN_RELAY = ON;
                  Lamp_State_Reg = ON; 
                #endif
                                                                   
            }

        }   
              
        
          if(luz <= LUX_MIN)
          {        
            /* Cancela a contagem de lampada ligada */
            cnt_light_on = 0;
            
            /* Se o rele est� desligado */
            if(PIN_RELAY == OFF)
            {                
                /* Verifica se a luz ficou abaixo do valor
                 minimo por pelo menos 5 segundos */
                cnt_light_off++;
                
                /* Se ficou, a contagem ser� de 10
                 e o rel� deve ser ligado */
                if (cnt_light_off >= 10)   /* 10 x 500ms = 5s */
                {
                   cnt_light_off = 0;  
                   
                  
  #if (defined RELAY_CHECK_CC && RELAY_CHECK_CC == 1)                 
                  /* testa TENS�O CC no rel� antes de tentar lig�-lo */                          
                  do{                 
                    nivel_cc_rede = 0;
                      
                    for(i=0;i<128;i++)    
                    {
                        UserEnterCritical();
                          nivel_cc_rede += (INT16U)ADC_Conversion(CHAN_VLOAD);
                        UserExitCritical();
                        DelayTask(1);
                    }                 
                    nivel_cc_rede = (INT32U)(nivel_cc_rede>>7);
                    
                    if(nivel_cc_rede < (NIVEL_CC_SENSOR_TENSAO-10) || nivel_cc_rede> (NIVEL_CC_SENSOR_TENSAO+10)){
                        nivel_cc_rede = 0;
                        DelayTask(1000); 
                    }                 
                  }while(nivel_cc_rede == 0); /* NIVEL_CC_SENSOR_TENSAO = 2V/3,3V x 4095 = 2482 */
                 
  #endif     

  #if (defined RELAY_TURNON_VOLTAGE && RELAY_TURNON_VOLTAGE == 1)                                  
                      
                   /* mede o n�vel CC do sensor de tens�o na entrada */
                   nivel_cc_rede = 0;
                   
                   for(i=0;i<128;i++)    
                   {
                      UserEnterCritical();
                        nivel_cc_rede += (INT16U)ADC_Conversion(CHAN_VIN);
                      UserExitCritical();
                      DelayTask(1);
                   }                 
                   nivel_cc_rede = (INT32U)(nivel_cc_rede>>7);                                                                     
                   
                   /* rotina para ligar o rel� */                    
                   nivel_cc_corrente = 0;
                       
                   for(i=0;i<128;i++)
                   {
                      UserEnterCritical();
                        nivel_cc_corrente += (INT32U)ADC_Conversion(CHAN_ILOAD);
                      UserExitCritical();
                      DelayTask(1);
                   }
                  /* calcula n�vel cc do sensor de corrente */
                   nivel_cc_corrente = (INT32U)nivel_cc_corrente>>7;
                    
                  /* passa o valor do n�vel cc para Imax e Imin */
                   Imax = nivel_cc_corrente;   
                   Imin = nivel_cc_corrente;
                      
                  /* loop para medir varia��es m�ximas 
                  e m�nimas na resposta do sensor  */                
                   for(i=0;i<=1024;i++) 
                   {
                      UserEnterCritical();
                        Iload = (INT16U)ADC_Conversion(CHAN_ILOAD);
                      UserExitCritical();
                      /* se o valor lido for maior que Imax 
                      ent�o Imax recebe o valor lido */                    
                      if(Iload>Imax)   
                      {
                        Imax = Iload;
                      }
                      /* se o valor lido for menor do que Imin
                       ent�o Imin recebe o valor lido */                    
                      else if(Iload<Imin) 
                      {
                        Imin = Iload;
                      } 
                   } 
                   
                   Imax = (INT16U)(Imax - nivel_cc_corrente);        
                   Imin = (INT16U)(nivel_cc_corrente - Imin);
                   
                   /////////////////////////////////////////////////////////////////////////////////////////////////       
                   
                   
                   v_entrada = 0;
                   v_disparo = 0;                 
                   
                   /* l� tens�o da rede at� encontrar a passagem por zero  */                                
                   do   
                   {
                       UserEnterCritical();
                        v_entrada = (INT16U)ADC_Conversion(CHAN_VIN);
                       UserExitCritical();
                   } while((v_entrada>=nivel_cc_rede+5)||(v_entrada<=nivel_cc_rede-5));
                      
                  /* configura timer3 para bus clock/128 
                     com interrup��o ativa */
                   TPM3CNT = 0;     
                   TPM3MOD = turnon_time;
                   TPM3SC  = 0x4F;
                   
                  /* aguarda retorno da interrup��o com o 
                  valor do primeiro ponto de tens�o */                 
                   OSSemPend(Rele_Event,0);
                   
                   UserEnterCritical();
                    v_disparo = (INT32U)TurnOn_Voltage;
                   UserExitCritical();
                   
                  /* aguarda aprox. 2ms para iniciar a 
                  varredura do segundo ponto de tens�o */
                   DelayTask(2);
                    
                  /* loop para encontrar o segundo ponto de tens�o */              
                   do 
                   {
                      /* l� o sensor de tens�o de entrada */
                      UserEnterCritical();
                      v_entrada = (INT16U)ADC_Conversion(CHAN_VIN);                    
                      UserExitCritical();
                   }while((v_entrada>=(v_disparo + 15))||(v_entrada<=(v_disparo - 15)));
                   
  #endif                 
                   
                   
                   /* quando o segundo ponto for encontrado
                   liga o rel� e dispara o timer para 
                   a contagem de tempo */                 
                   PIN_RELAY = ON;
                   Set_LampStateReg(ON); 
                   
  #if (defined RELAY_TURNON_VOLTAGE && RELAY_TURNON_VOLTAGE == 1)                   
                                       
                   TPM3CNT = 0;      
                   TPM3MOD = 0xFFFF;  
                                                                                           
                   /* dispara o timer: bus clock/128 sem interrup��o */
                   TPM3SC = 0x0F;

                   /* loop para aguardar o rel� fechar                 
                   se n�o houver corrente em um determinado tempo, o rele abre novamente */
                   
                   do
                   {
                      UserEnterCritical();
                        Iload = (INT16U)ADC_Conversion(CHAN_ILOAD);
                      UserExitCritical();
                       
                      if(TPM3SC_TOF==1)
                      {
                         TPM3SC = 0;
                         PIN_RELAY = OFF;
                         
                         Set_LampStateReg(OFF);
                         
                         without_load_cnt++;
                         if (without_load_cnt > 3) 
                         {
                            /* Zera o contador de tentativas de 
                            ligar o rele com carga */
                            without_load_cnt = 0;
                            
                            ReportFailure_Set(NO_LOAD); 
                            
                            /* Pare de tentar ligar o rele por 1 hora */
                            DelayTaskHMSM(1,0,0,0);
                                                       
                            ReportFailure_Set(NO_FAIL); 
                         }                       
                         break; 
                      }
                   } while((Iload<(nivel_cc_corrente + 2*Imax)) && (Iload>(nivel_cc_corrente - 2*Imin)));
                                                                              
                   /* desliga o timer */
                   TPM3SC = 0;
  #endif                  
                   
                   /* ligamos a lampada ? */
                   if(PIN_RELAY == ON){
                    
  #if (defined RELAY_TURNON_VOLTAGE && RELAY_TURNON_VOLTAGE == 1)                   
                     /* adquire o valor do timer */ 
                     turnon_time = (INT16U)TPM3CNT; 
                                        
                     /* l� o valor de tens�o em que o rel� fechou */
                     UserEnterCritical();
                        v_entrada = (INT16U)ADC_Conversion(CHAN_VIN);
                     UserExitCritical(); 
  #endif                     
                     
                     /* Aciona aquisicao de corrente e tensao */
                     Smartmeter_Start();
                   }
                }
            }
          }
          else
          {
              /* Cancela a contagem de lampada apagada */
              cnt_light_off = 0;
                
              /* Verifica se a luz ficou acima do 
              valor m�ximo por pelo menos 5 segundos */
              if (luz > LUX_MAX) 
              {
                cnt_light_on++;
                
                /* Se ficou, a contagem ser� de 10
                   e o rel� deve ser desligado */
                if (cnt_light_on >= 10) 
                {
                  cnt_light_on = 0;
                  
                   /* Zera o contador de tentativas de ligar 
                    o rele com carga */
                   without_load_cnt = 0; 
                  
                  if (PIN_RELAY == ON){ 
                                    
                    /* desliga rel� */
                    PIN_RELAY = OFF;
                    Set_LampStateReg(OFF);
                    
                    /* Desliga aquisicao de corrente e tensao */
                    Smartmeter_Stop();
                    
                    /* Grava a energia acumulada no per�odo */
                    Smartmeter_GetStoredEnergy();
                  }
                  
                }
              }else 
              {
                cnt_light_on = 0;
              }
          }         
        
        /* espera a proxima medi��o de luminosidade */
        DelayTask(RELAY_CONTROL_PERIOD_MS);   
   }

}

/* function to measure relay turn on voltage */  
static void Relay_turn_on_voltage(void);  
  
static void Relay_turn_on_voltage(void)
{
    OS_SR_SAVE_VAR;
    INT8U  i=0; 
    
    OSEnterCritical();
    
    /* turn on voltage - 4 samples averaging */
    TurnOn_Voltage = 0;
    
    for(i=0;i<4;i++)
    {
      TurnOn_Voltage += (INT16U)ADC_Conversion(CHAN_VIN); // voltage sensor read
    }
    TurnOn_Voltage = (INT32U)(TurnOn_Voltage>>2);
    
    OSExitCritical();
     
    OSSemPost(Rele_Event);      
}

void Relay_TurnOn(void);

#if (NESTING_INT == 1)
#pragma TRAP_PROC
void Relay_TurnOn(void)
#else
interrupt void Relay_TurnOn(void)
#endif
{
    // ************************
    // Interrupt Entry
    // ************************  
    OS_INT_ENTER();
    
    // Interrupt flag clearing
    TPM3SC_TOF = 0;
    TPM3SC_TOIE = 0;
    TPM3SC = 0;
    
    #if (NESTING_INT == 1)
      OS_ENABLE_NESTING();
    #endif     
    
    // Interrupt handling     
    Relay_turn_on_voltage();
    
    // ************************
    // Interrupt Exit
    // ************************
    OS_INT_EXIT();  

}

