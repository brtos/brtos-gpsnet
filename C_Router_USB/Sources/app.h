/***********************************************************************************
@file   app.h
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

#include "gpsnet_api.h"

#include "smartmeter.h"

#define APP_NODES        (INT8U)4


typedef struct _GPS_NODES    
{
    INT32U        Energy;                     // Energy measurement
    INT32U        ActivePower;                // Active Power measurement
    INT32U        ApparentPower;              // Apparent Power measurement
    INT32U        ReactivePower;              // Reactive Power measurement
    INT16U        PowerFactor;                // Power Factor measurement        
    INT16U        Current;                    // Current measurement  
    INT16U        Voltage;                    // Current measurement    
    INT16U        LightLevel;                 // Light measurement
    INT8U         Temperature;                // Temperature measurement
    INT8U         GPS_Latitude[4];            // Neighbor GPS position
    INT8U         GPS_Longitude[4];
    INT16U        NodeID;
    INT8U         Hops;
    INT32U        PacketsCount;
} GPS_NODES;


typedef struct _GPSNET_NEIGHBOURHOOD_TABLE
{
    INT16U        Addr_16b_1;                 // 16 bit address from neighbor
    INT16U        Addr_16b_2;                 // 16 bit address from neighbor
    INT8U         NeighborRSSI;               // Index of the Neighbor signal quality vector  
    INT8U         Symmetric;                  // is neighbor symmetryc?
    INT8U         ParentCnt;                  // 
} GPSNET_NEIGHBOURHOOD_TABLE;


// Prototypes  
void NetSimpleCommand(INT8U direction, INT8U CommandType, INT8U Value8, INT16U Value16);
INT8U NetLightingProfile(INT8U direction, INT8U Command, INT8U Parameter, INT8U Value8, INT16U Value16, INT32U Value32);

void LogNodeData(INT8U Type, INT8U Value8, INT16U Value16, INT32U Value32);
void SniferReq(INT8U direction, LATITUDE *lat, LONGITUDE *lon);
void SniferRep(INT8U direction, LATITUDE *lat, LONGITUDE *lon);

INT8U NetSimpleMeasureSE(INT8U MeasureType, INT16U Value16, INT32U Value32);
INT8U NetGeneralInfo(INT8U MeasureType, INT8U Value8,INT16U Value16);
INT8U NetDebugPacket(INT8U Command, INT8U Parameter, INT8U Value8, INT16U Value16, LATITUDE *lat, LONGITUDE *lon);

INT8U NetSimpleMeasureSE(INT8U MeasureType, INT16U Value16, INT32U Value32);
INT8U NetMultiMeasureSE(SE_STRUCT *se);
INT8U NetClearSE(SE_STRUCT *se); 

extern volatile GPSNET_NEIGHBOURHOOD_TABLE nb_table[NEIGHBOURHOOD_SIZE];


/* Return codes */
typedef enum {         
  NO_FAIL,              
  TASK_FAIL_INIT,
  NO_LOAD, 
  GENERIC_FAIL   
}FAIL_T;

void ReportFailure_Get(FAIL_T *failure_kind); 
void ReportFailure_Set(FAIL_T failure_kind);

enum ParamNames
{
    LUX_THRESHOLD_MIN,
    LUX_THRESHOLD_MAX,
    PARENT_THRESHOLD,
    PARENT_THRESHOLD_MIN,
    REPORT_PERIOD_1000MS,
    REPORT_JITTER_100MS
};

extern INT8U    Config_LUX_THRESHOLD_MIN;
extern INT8U    Config_LUX_THRESHOLD_MAX;
extern INT8U    Config_PARENT_THRESHOLD;
extern INT8U    Config_PARENT_THRESHOLD_MIN;
extern INT8U    Config_REPORT_PERIOD_1000MS;
extern INT8U    Config_REPORT_JITTER_100MS;

