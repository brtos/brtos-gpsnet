/***********************************************************************************
@file   sensors.c
@brief  functions to handle sensor inputs
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

#include "sensors.h"

/* Sanity check */
#if (!defined CORE_TEMP) || (!defined BANDGAP_MV)
  #error "Please define 'CORE_TEMP' and 'BANDGAP_MV'"
#endif



#if (SENSOR_TEMP == 1)
/** Functions to handle Temperature sensor */

static INT8U SensorTemp_is_enable = SENSOR_ENABLED;
static INT8U SensorTemp_last_value = 0;

void SensorTemp_Enable(void){
  SensorTemp_is_enable = SENSOR_ENABLED;
}

void SensorTemp_Disable(void){
  SensorTemp_is_enable = SENSOR_DISABLED;
}

INT8U SensorTemp_Status(void){
  return SensorTemp_is_enable;
}

INT8U SensorTemp_Get(INT8U* value) {
  
      INT16U       Temperatura = 0;
      INT16U       Digital_Temp = 0;
      INT16U       bandgap = 0;
     
      if(SensorTemp_is_enable == FALSE){
        return SENSOR_DISABLED;
      }
      
      bandgap = ADC_Bandgap_Get();         
      if(bandgap == 0) {
          return SENSOR_ERROR_BANDGAP;
      }
            
      //////////////////////////////////////////////////////////////
      // Take core temperature
      //////////////////////////////////////////////////////////////          
      UserEnterCritical();
        Digital_Temp = (INT16U)ADC_Conversion(CORE_TEMP);
      UserExitCritical();  
    
      Digital_Temp = (INT16U)((BANDGAP_MV * Digital_Temp)/bandgap); 
      
      #if (defined _MCF51JM128_H)
        if (Digital_Temp > 1396)
        {
          Temperatura = (INT16U)(25 - (((Digital_Temp - 1396)*1000)/3266));
        }else
        {
          Temperatura = (INT16U)(25 - (((Digital_Temp - 1396)*1000)/3638));
        }
      #elif (defined _MCF51QE128_H) 
        if (Digital_Temp > 701)
        {
          Temperatura = (INT16U)(25 - (((Digital_Temp - 701)*1000)/1646));
        }else
        {
          Temperatura = (INT16U)(25 - (((Digital_Temp - 701)*1000)/1769));
        }
      #endif
      
      *value = (INT8U) Temperatura; 
      
      /* Keep last one */      
      SensorTemp_last_value = (INT8U)Temperatura;
      
      return  SENSOR_OK;
      ////////////////////////////////////////////////////////////// 
}

/* Return last measurement, or take a new one if the last one is not available */
INT8U SensorTemp_GetLast(INT8U* value) {
      
      if(SensorTemp_is_enable == FALSE){
        return SENSOR_DISABLED;
      }
      
      if(SensorTemp_last_value == 0){
         return (SensorTemp_Get(value));         
      }
        
      *value = SensorTemp_last_value;      
      return  SENSOR_OK;
  
}
#endif
/***************************************************************************************/

#if (SENSOR_LIGHT == 1)
/** Functions to handle Light sensor */

static INT8U SensorLight_is_enable = SENSOR_ENABLED;
static INT16U SensorLight_last_value = 0;

void SensorLight_Enable(void){
  SensorLight_is_enable = SENSOR_ENABLED;
}

void SensorLight_Disable(void){
  SensorLight_is_enable = SENSOR_DISABLED;
}

INT8U SensorLight_Status(void){
  return SensorLight_is_enable;
}

INT8U SensorLight_Get(INT16U* value) {
  
      INT32S       Light = 0;
      INT16U       bandgap = 0;
      INT8U        avg_cnt = 0;
     
      if(SensorLight_is_enable == FALSE){
        return SENSOR_DISABLED;
      }
      
      bandgap = ADC_Bandgap_Get();         
      if(bandgap == 0) {
          return SENSOR_ERROR_BANDGAP;
      }
            
      //////////////////////////////////////////////////////////////
      // Take light measurement
      //////////////////////////////////////////////////////////////          
              
      for(avg_cnt=0;avg_cnt<(1<<LIGHT_AVERAGE_COUNT);avg_cnt++)
      {
        UserEnterCritical();
          Light += (INT32S)ADC_Conversion(CHAN_LIGHT);
        UserExitCritical();
        
        DelayTask(LIGHT_AVERAGE_MS);
      }

      Light = (INT32S)(Light>>LIGHT_AVERAGE_COUNT);
      
      Light = (INT32S)((Light * BANDGAP_MV)/bandgap);  
     
      
      *value = (INT16U) Light; 
      
      /* Keep last one */      
      SensorLight_last_value = (INT16U)Light;
      
      return  SENSOR_OK;
      ////////////////////////////////////////////////////////////// 
}

/* Return last measurement, or take a new one if the last one is not available */
INT8U SensorLight_GetLast(INT16U* value) {
      
      if(SensorLight_is_enable == FALSE){
        return SENSOR_DISABLED;
      }
      
      if(SensorLight_last_value == 0){
         return (SensorLight_Get(value));         
      }
        
      *value = SensorLight_last_value;      
      return  SENSOR_OK;
  
}

#endif

/***************************************************************************************/
