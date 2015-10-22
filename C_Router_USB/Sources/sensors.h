/***********************************************************************************
@file   sensors.h
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

#ifndef _SENSORS_H
#define _SENSORS_H

/* MCU and OS includes */
#include "hardware.h"
#include "BRTOS.h"
#include "drivers.h"

/* Enable/disable sensors */
#define SENSOR_TEMP     1

/* Return codes */
#define SENSOR_ENABLED         (INT8U)(1)
#define SENSOR_OK              (INT8U)(0)
#define SENSOR_DISABLED        (INT8U)(-1)
#define SENSOR_ERROR_BANDGAP   (INT8U)(-2)  

/* Average config for Light Sensor */
#define LIGHT_AVERAGE_COUNT       4
#define LIGHT_AVERAGE_MS          50

/* Functions for Temperaure Sensor */
void SensorTemp_Enable(void);
void SensorTemp_Disable(void);
INT8U SensorTemp_Status(void); 
INT8U SensorTemp_Get(INT8U* value);
INT8U SensorTemp_GetLast(INT8U* value);

/* Functions for Light Sensor */
void SensorLight_Enable(void);
void SensorLight_Disable(void);
INT8U SensorLight_Status(void);
INT8U SensorLight_Get(INT16U* value);
INT8U SensorLight_GetLast(INT16U* value);

#endif