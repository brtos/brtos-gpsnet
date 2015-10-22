/***********************************************************************************
@file   smartmeter.c
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

/* MCU and OS includes */
#include "hardware.h"
#include "BRTOS.h"
#include "drivers.h"

/* Config. files */
#include "BRTOSConfig.h"
#include "AppConfig.h"
#include "NetConfig.h"
#include "BoardConfig.h"

#include "smartmeter.h"
#include "math_utils.h"
#include "app.h"
#include "sensors.h"

/* Sanity check */ 


//////////////////////////////////////////////////////////////////////////////////////////////////

/* Energy meter vars */
#define SMARTMETER_BUFFSIZE 128
static INT16U corrente1[SMARTMETER_BUFFSIZE];
static INT16U corrente2[SMARTMETER_BUFFSIZE];
static INT16U tensao1[SMARTMETER_BUFFSIZE];
static INT16U tensao2[SMARTMETER_BUFFSIZE];
static INT16U Nivel_CC_Corrente = 0;
static INT16U Nivel_CC_Tensao = 0;
static INT32S Nivel_ACC_Corrente = 0;
static INT32S Nivel_ACC_Tensao = 0;
static INT8U  Buffer = 1;
static INT32S Fator_Potencia = 0; 

/* Accumulated energy register in non-volatile memory */
const     INT32U  Energy_Meter_Reg   @ENERGY_REGISTER_ADDR = 0;

/* Calibration factor for current sensor */
const     INT32S  CurrentFactor  @CURRENT_CALIB_REGISTER_ADDR = 0xFFFFFFFF;

static    SE_STRUCT      SmartEnergyValues = {0};
static    INT32U         EnergyMeter     = 0;
static    INT8U          buffer_counter  = 0;
static    INT8U          buffer_counter_v  = 0;
static    INT8U          buffer_counter_i  = 0;
static    INT8U          SmartMeterState = SMARTMETER_OFF;  


// Event signal for power/energy processing
BRTOS_Sem    *Do_Calculation;


/* BANDGAP_SQ = (Tensao_bandgap_milivolts^2) / 1000  ~= 1369 */
#define BANDGAP_SQ    (INT32U)1369

void EnergyMetering_Task(void)
{
  INT16U      j = 0;
  INT32S      Potencia = 0;    
  INT32S      nivel_cc_vload = 0;
  INT32S      nivel_cc_vin = 0;
  INT32S      nivel_cc_corrente = 0;
  INT32S      fc = 0;
  INT8U       buffer = 0; 
  INT32S      vrms = 0;
  INT32S      corrente_rms = 0; 
  INT32S      Potencia_aparente = 0;
  INT32S      Potencia_reativa = 0;
  INT32U      energy_acc = 0;
  INT32U      energy_acc_index = 0;
  INT32U      energy_meter;
  INT32U      energy_meter_index = 0;
  INT32S      pf = 0;
  INT32U      bandgap = 0;      
  
  
  // task setup
  if (OSSemCreate(0,&Do_Calculation) != ALLOC_EVENT_OK)
  {
    while(1){UserEnterCritical();};  /* Force reset */
  };
    
  /* Get BandGap ref */
  do{    
    bandgap = ADC_Bandgap_Get();
    DelayTask(1000);
    ReportFailure_Set(TASK_FAIL_INIT);
  }while(bandgap==0);  
  
#if (!defined SMARTMETER_TEST_CALCULATIONS ||  (defined SMARTMETER_TEST_CALCULATIONS && SMARTMETER_TEST_CALCULATIONS == 0))  
  
  nivel_cc_corrente = 0; nivel_cc_vin = 0; nivel_cc_vload = 0;
   
  /* Test voltage and current sensor */
  for(j=0;j<SMARTMETER_BUFFSIZE;j++){     
    nivel_cc_corrente = nivel_cc_corrente + (INT16U)ADC_Conversion(CHAN_ILOAD);
    nivel_cc_vin     = nivel_cc_vin + (INT16U)ADC_Conversion(CHAN_VIN); 
    nivel_cc_vload   = nivel_cc_vload + (INT16U)ADC_Conversion(CHAN_VLOAD); 
    DelayTask(1); 
  }
  
  nivel_cc_corrente = nivel_cc_corrente / SMARTMETER_BUFFSIZE;
  nivel_cc_vload   = nivel_cc_vload / SMARTMETER_BUFFSIZE;
  nivel_cc_vin     = nivel_cc_vin  / SMARTMETER_BUFFSIZE;
  
  if((nivel_cc_corrente < 2000 && nivel_cc_corrente > 2100) ||
  (nivel_cc_vload < 2000 && nivel_cc_vload > 2100) ||
  (nivel_cc_vin < 2000 && nivel_cc_vin > 2100)){
  
    while(1){
      DelayTask(1000);
      ReportFailure_Set(TASK_FAIL_INIT);
    }
  }
#endif

   ReportFailure_Set(NO_FAIL);   
   SensorVoltage_Enable();  
   SensorCurrent_Enable(); 
        

  /* BANDGAP_SQ = (Tensao_bandgap_milivolts^2) / 1000  */
  if (CurrentFactor == 0xFFFFFFFF) 
  {
    Fator_Potencia = BANDGAP_SQ * FATOR_TENSAO * FATOR_CORRENTE;
    fc = FATOR_CORRENTE;
  } else
  {
    Fator_Potencia = BANDGAP_SQ * FATOR_TENSAO * CurrentFactor;
  }
  
  Fator_Potencia = (Fator_Potencia / (bandgap * bandgap)) * 10;
  
  /* Set period of 130,2 us */
  TPM2MOD = 3276;  
  (void)(TPM2SC == 0);  
  TPM2SC = 0x08;  /* Start only when relay is turned on */
  
  /* get accumulated energy */
  energy_meter = Energy_Meter_Reg;       
  UserEnterCritical();
    EnergyMeter = energy_meter;
    SmartEnergyValues.energy_meter = energy_meter;
  UserExitCritical();
  
   // task main loop
   for (;;)
   {

      /* Wait buffer full  - Para e espera buffer estar cheio */
      OSSemPend (Do_Calculation,0);
      
      /* Init vars */
      vrms = 0;
      corrente_rms = 0;      
      Potencia = 0;
      Potencia_aparente = 0;
      Potencia_reativa = 0;
      pf = 0;
      
      
      UserEnterCritical();
        buffer = Buffer;
      UserExitCritical();
      
      /* Check which buffer is full - Verifica qual buffer deve ser utilizado para os calculos */

      if (buffer == 1)
      {
        for(j=0;j<SMARTMETER_BUFFSIZE;j++)
        {          
          vrms += (INT32S)((tensao2[j]-Nivel_CC_Tensao) * (tensao2[j]-Nivel_CC_Tensao));
          corrente_rms += (INT32S)((corrente2[j]-Nivel_CC_Corrente) * (corrente2[j]-Nivel_CC_Corrente));
          Potencia += (INT32S)((corrente2[j] - Nivel_CC_Corrente) * (tensao2[j] - Nivel_CC_Tensao));          
        }
      }else
      {
        for(j=0;j<SMARTMETER_BUFFSIZE;j++)
        {          
          vrms += (INT32S)((tensao1[j]-Nivel_CC_Tensao) * (tensao1[j]-Nivel_CC_Tensao));
          corrente_rms += (INT32S)((corrente1[j]-Nivel_CC_Corrente) * (corrente1[j]-Nivel_CC_Corrente));
          Potencia += (INT32S)((corrente1[j] - Nivel_CC_Corrente) * (tensao1[j] - Nivel_CC_Tensao));          
        }        
      }
  
      /* Active power  calculation - Determina potência ativa, considerando o BandGap e o fator de ajuste dos sensores */
      Potencia = Potencia / SMARTMETER_BUFFSIZE; 
      Potencia = (INT32S)((Potencia * Fator_Potencia) / 1000);
       
      
      /* RMS voltage - Calculo da tensao RMS, valor com uma casa depois da virgula */
      vrms = vrms / SMARTMETER_BUFFSIZE;
      vrms = SquareRoot((INT32U)vrms);      
      vrms = (vrms * BANDGAP_MV * FATOR_TENSAO)/(100*bandgap);
                  
      /* RMS current - Calculo da corrente RMS */     
      corrente_rms = corrente_rms / SMARTMETER_BUFFSIZE;
      corrente_rms = SquareRoot((INT32U)corrente_rms);           
                       
      /* Current sensor calibration - Teste para calibrar sensor (se ainda não calibrado) */
      
      #if (CALIB_CURRENT == 1)
      if (CurrentFactor == 0xFFFFFFFF) 
      {
      #endif
        
        corrente_rms = (corrente_rms * BANDGAP_MV * fc)/(100*bandgap);
        
      #if (CALIB_CURRENT == 1)
        if ((corrente_rms > (CALIB_VALUE-1)) && (corrente_rms < (CALIB_VALUE+4))) 
        {
            /* Save in ROM - Grava na flash novo fator de corrente */
            UserEnterCritical();
              Flash_Prog((INT32U)&CurrentFactor, (INT32U)&fc, 1);
            UserExitCritical();                
            
            Fator_Potencia = BANDGAP_SQ * FATOR_TENSAO * fc;
            Fator_Potencia = (Fator_Potencia / (bandgap * bandgap)) * 10;            
        } else 
        {          
          /* Faixa de segurança do valor de referencia */
          if ((corrente_rms > (CALIB_VALUE-150)) && (corrente_rms < (CALIB_VALUE+150)))
          {
            /* 0,945 A é o valor de referencia de corrente  (carga resistiva de 200W @ 220V RMS) */
            fc = (CALIB_VALUE * FATOR_CORRENTE) / corrente_rms;
            fc = fc + 1;
          }else 
          {
            fc = FATOR_CORRENTE;
          }
        }
      }
      else
      {
        corrente_rms = (corrente_rms * BANDGAP_MV * CurrentFactor)/(100*bandgap);
      }
      #endif
            
      /* Apparent power - Calculo da potencia aparente com 3 digitos de precisao */
      if ((vrms * corrente_rms) > 0)
      {
        Potencia_aparente = (INT32U)((vrms * corrente_rms)/10);
      }
      

      UserEnterCritical();
        SmartEnergyValues.v_rms = (INT16U)vrms;
        SmartEnergyValues.i_rms = (INT16U)corrente_rms;
        SmartEnergyValues.power_S = (INT32U)Potencia_aparente;
        SmartEnergyValues.power_P = Positive(Potencia); 
      UserExitCritical();
      
      /* Calculo do acumulador de energia. Para o acumulador a resolução da potência 
      é diminuida para 1 casa decimal */        
      energy_acc += Positive(Potencia / 100);
      energy_acc_index++;
      
      /* Se energy_acc = 3600, significa que 
      temos 1 minuto de energia acumulada */
      if (energy_acc_index >= 3600) 
      {
        /* acumula 1 minuto de consumo */
        energy_meter = energy_meter + (energy_acc / 216000);
        
        /* Grava dados do acumulador de energia em uma variável global
         usada para gravar a energia acumulada quando o rele desligar
         */
        UserEnterCritical();
          EnergyMeter = energy_meter;
        UserExitCritical();        
        
        /* Reseta o acumulador de energia de 1 minuto e o indice do somatório */
        
        energy_acc = 0;
        energy_acc_index = 0;
        energy_meter_index++;
        
        /* Grava 1 hora de energia acumulada */
        if (energy_meter_index >= 60) 
        {
          energy_meter_index = 0;
          EEPROM_Store((INT32U)&Energy_Meter_Reg, (INT32U *) &energy_meter, 1);
        }
      }

   
      // Calculo da potencia reativa
      Potencia_reativa = 0;
      
      if ((Potencia_aparente > 0) && (Potencia > 0))
      {
        Potencia = Potencia / 10;
        Potencia_aparente = Potencia_aparente / 10;
        if ((Potencia_aparente > 0) && (Potencia > 0))
        {
          Potencia_reativa = ((Potencia_aparente * Potencia_aparente) / 10)-((Potencia * Potencia) / 10);
        }
      }
      
            
      if(Potencia_reativa > 0)
      {
        Potencia_reativa = SquareRoot((INT32U)Potencia_reativa);
      } 
      else
      {
        Potencia_reativa = 0;
      }
  
      
      /* Calculo do fator de potencia
       Resolução de três casas de precisão */
      pf = 1000;      
      if ((Potencia > 0) && (Potencia_aparente > 0))
      {
        pf = (Potencia * 1000)/Potencia_aparente;
        if (pf > 1000)
        {
          pf = 1000;
        }
      }
      
            
      UserEnterCritical();
        /* Multiplica por 10 para deixar na mesma 
        base das outras potencias */
        SmartEnergyValues.power_factor = (INT16U)pf; 
        SmartEnergyValues.power_Q = (INT32U)(Potencia_reativa*10); 
        SmartEnergyValues.energy_meter = energy_meter;
      UserExitCritical();
   } 
}

#if (defined SMARTMETER_TEST_CALCULATIONS &&  SMARTMETER_TEST_CALCULATIONS == 1)
#include "inputdata.h"
#endif 


/* ADC interrupt */

static void ADC_handler(void);

static void ADC_handler(void)
{
    INT16U current, voltage;
    OS_SR_SAVE_VAR;

#if (defined SMARTMETER_TEST_CALCULATIONS &&  SMARTMETER_TEST_CALCULATIONS == 1)
    voltage = (INT16U)((inputsignal1[buffer_counter*2] + (INT16U)(1<<15))>>3);
    current = (INT16U)((inputsignal2[buffer_counter*2] + (INT16U)(1<<15))>>7);   
#else
    /* take two measurements of voltage and current */   
    OSEnterCritical();
      current = (INT16U)ADC_Conversion(CHAN_ILOAD);
      voltage   = (INT16U)ADC_Conversion(CHAN_VIN);
      voltage   = (INT16U)(voltage + (INT16U)ADC_Conversion(CHAN_VIN));
      current = (INT16U)(current + (INT16U)ADC_Conversion(CHAN_ILOAD));  
    OSExitCritical();
    
    current = (INT16U)(current >> 1);
    voltage = (INT16U)(voltage >> 1);
#endif    
    
  
  if (Buffer == 1)
  {      
      corrente1[buffer_counter_i] = current;
      tensao1[buffer_counter_v]   = voltage;
      Nivel_ACC_Corrente += current;
      Nivel_ACC_Tensao += voltage;
      
      if (++buffer_counter_v == SMARTMETER_BUFFSIZE){
        buffer_counter_v = 0;
      }
      if (++buffer_counter_i == SMARTMETER_BUFFSIZE){
        buffer_counter_i = 0;
      }
      
      /* time to switch buffers ? */
      if (++buffer_counter == SMARTMETER_BUFFSIZE)
      {
        Buffer = 2;
        buffer_counter = 0;
        buffer_counter_v = 0;  // TODO: change to  BUFCOUNT_V_STARTINDEX
        buffer_counter_i = 0;  // TODO: change to  BUFCOUNT_I_STARTINDEX
        Nivel_CC_Corrente = (INT16U)(Nivel_ACC_Corrente / SMARTMETER_BUFFSIZE);
        Nivel_CC_Tensao = (INT16U)(Nivel_ACC_Tensao / SMARTMETER_BUFFSIZE);
        Nivel_ACC_Corrente = 0;
        Nivel_ACC_Tensao = 0;
        
        /* Buffer full, time to do the calculations */
        OSSemPost(Do_Calculation);
      }
  }else
  {
      corrente2[buffer_counter_i] = current;
      tensao2[buffer_counter_v]   = voltage;
      Nivel_ACC_Corrente += current;
      Nivel_ACC_Tensao += voltage;      
      
      if (++buffer_counter_v == SMARTMETER_BUFFSIZE){
        buffer_counter_v = 0;
      }
      if (++buffer_counter_i == SMARTMETER_BUFFSIZE){
        buffer_counter_i = 0;
      }

      
       /* time to switch buffers ? */
      if (++buffer_counter == SMARTMETER_BUFFSIZE)
      {
        Buffer = 1;
        buffer_counter = 0;
        buffer_counter_v = 0;  // TODO: change to  BUFCOUNT_V_STARTINDEX
        buffer_counter_i = 0;  // TODO: change to  BUFCOUNT_I_STARTINDEX
        Nivel_CC_Corrente = (INT16U)(Nivel_ACC_Corrente / SMARTMETER_BUFFSIZE);
        Nivel_CC_Tensao = (INT16U)(Nivel_ACC_Tensao / SMARTMETER_BUFFSIZE);
        Nivel_ACC_Corrente = 0;
        Nivel_ACC_Tensao = 0;  
              
        /* Buffer full, time to do the calculations */
        OSSemPost(Do_Calculation);
      }    
  }

}

/* Interrupt routines */
void ADC_Tick(void);
      
#if (NESTING_INT == 1)
#pragma TRAP_PROC
void ADC_Tick(void)
#else
interrupt void ADC_Tick(void)
#endif
{
  // ************************
  // Interrupt Entry
  // ************************
  OS_INT_ENTER();  

  // Interrupt flag clearing
  TPM2SC_TOF = 0;
  
  #if (NESTING_INT == 1)
    OS_ENABLE_NESTING();
  #endif   

  // Interrupt handling  
  ADC_handler();
  
  // ************************
  // Interrupt Exit
  // ************************
  OS_INT_EXIT();  
  // ************************
  
}

/* Return last measurements of power, energy, current, voltage, etc. */
INT8U Smartmeter_GetValues(SE_STRUCT* se) {
      
     *se = SmartEnergyValues;
     se->params.Byte = P_VOLTAGE + P_CURRENT + P_POWER_P + P_POWER_Q + P_POWER_S + P_PF + P_ENERGY;                               
     return SMARTMETER_OK;
  
}

/* Return last measurement of energy */
INT8U Smartmeter_GetEnergy(INT32U* value) {       
      *value =  EnergyMeter;          
      return SMARTMETER_OK;
  
}

/* Return nothing, take last measurement of energy and store */
INT8U Smartmeter_GetStoredEnergy(void) {
      
      INT32U energy_meter = EnergyMeter;                                            
      EEPROM_Store((INT32U)&Energy_Meter_Reg, (INT32U*)&energy_meter,1);          
      return  SMARTMETER_OK;
  
}

INT8U Smartmeter_Start(void){
    if(SmartMeterState == SMARTMETER_OFF){
       SmartMeterState = SMARTMETER_ON;
       
       buffer_counter = 0;
       TPM2CNT = 0;
       TPM2SC_TOIE = 1;
    }
    return SMARTMETER_ON;   
}

INT8U Smartmeter_Stop(void) {
    if(SmartMeterState == SMARTMETER_ON){
       SmartMeterState = SMARTMETER_OFF;
       
       /* Desliga aquisicao de corrente e tensao */
       TPM2SC_TOIE = 0;
    }
    return SMARTMETER_OFF;    
}

/***************************************************************************************/
