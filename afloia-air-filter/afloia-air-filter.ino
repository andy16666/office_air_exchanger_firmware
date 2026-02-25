/*
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
   
    Author: Andrew Somerville <andy16666@gmail.com> 
    GitHub: andy16666
 */

#define PWM_FREQUENCY 100000

//#define IN_OFFICE 1
//#define IN_MB 1
#define IN_LR 1

#ifdef IN_MB
#define HAS_AC_DATA 1
#define AC_HOSTNAME "ac1"
#define HOSTNAME "af1"
#elifdef IN_OFFICE
#define HOSTNAME "officehepa3"
#define HAS_GAS_SENSOR 1
#elifdef IN_LR
#define HOSTNAME "lrhepa"
#else
#define HOSTNAME "testhepa"
#endif

#include <Wire.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <aos.h>
#include <config.h>
#include <SparkFunCCS811.h>

#include <ArduinoJson.h>

#ifdef HAS_AC_DATA
#include <SimplicityAC.h> 
#endif

#include <PWMFan.h>

using namespace AOS; 

#define AF_FAN_PIN  6
#define AF_ON_PIN   7

#define TRANSITION_TIME_MS 5000

#ifdef HAS_AC_DATA
SimplicityAC AC(AC_HOSTNAME); 
#endif

PWMFan FAN("afFan", AF_FAN_PIN, 3, 15, 100); 

#ifdef HAS_GAS_SENSOR
//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address
CCS811 mySensor(CCS811_ADDR);
#endif


static volatile float           command                __attribute__((section(".uninitialized_data"))); 

#ifdef HAS_GAS_SENSOR
static volatile uint16_t co2Level  __attribute__((section(".uninitialized_data"))); 
static volatile uint16_t tvocLevel  __attribute__((section(".uninitialized_data"))); 
#endif

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
  command = 0; 

#ifdef HAS_GAS_SENSOR
  co2Level = 400; 
  tvocLevel = 0; 
#endif
}

void aosSetup() 
{
#ifdef HAS_GAS_SENSOR
  Wire.begin(); 
  Serial.println(mySensor.begin());
#endif

#ifdef HAS_AC_DATA
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_pollAC, 15000); 
#endif

#ifdef HAS_GAS_SENSOR
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_readGasSensor, 1000);
#endif
}

void aosSetup1()
{
  analogWriteFreq(PWM_FREQUENCY); 

#ifdef HAS_AC_DATA
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_parseAC);  
#endif
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_processCommands, TRANSITION_TIME_MS);  
}

#ifdef HAS_GAS_SENSOR
void task_readGasSensor()
{
  Serial.println(co2Level);  
  if (mySensor.dataAvailable())
  {
    mySensor.readAlgorithmResults();
    co2Level = mySensor.getCO2(); 
    tvocLevel = mySensor.getTVOC(); 
    //Serial.println(mySensor.statusString());  
  }
}
#endif

void populateHttpResponse(JsonDocument& document) 
{
  FAN.addTo("fans", document); 
#ifdef HAS_AC_DATA
  AC.addTo(AC_HOSTNAME, document); 
#endif
  document["command"] = command; 

#ifdef HAS_GAS_SENSOR
  document["gasSensor"]["co2Level"] = co2Level; 
  document["gasSensor"]["tvocLevel"] = tvocLevel; 
#endif
}

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  if (argName.equals("cmd")) 
  { 
    command = arg.toFloat(); 
    success = command <= 100.0 && command >= 0.0; 
  }

  return success; 
}

#ifdef HAS_AC_DATA
void task_pollAC()
{
  AC.execute(); 
}

void task_parseAC()
{
  DPRINTLN("                             Enter task_parseAC"); 
  AC.parse();
  DPRINTLN("                             Leave task_parseAC"); 
}
#endif

void task_processCommands()
{
#ifdef HAS_AC_DATA
  float commandFromACState = 0; 

  switch(AC.getFanState())
  {
    case AC_FAN_HIGH: commandFromACState = 50.0;  break;
    case AC_FAN_MED:  commandFromACState = 20.0;  break;
    case AC_FAN_LOW:  commandFromACState = 8.0;   break;
    case AC_FAN_OFF:  commandFromACState = 0.0;   break;
    default:          commandFromACState = 0.0;   break;
  }
  
  FAN.setCommand(fmax(command, commandFromACState)); 
#else
  FAN.setCommand(command); 
#endif
  FAN.execute(); 
}