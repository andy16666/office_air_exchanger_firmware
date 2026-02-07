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

#define HOSTNAME "officehrv"

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <aos.h>
#include <config.h>

#include <ArduinoJson.h>

#include <PWMFan.h>
#include <GPIOOutputs.h>
#include <BangProtocol.h>

using namespace AOS; 

#define TRANSITION_TIME_MS 5000

static volatile bool            cmdCool                __attribute__((section(".uninitialized_data"))); 
static volatile bool            cmdSilent              __attribute__((section(".uninitialized_data"))); 
static volatile int             cmdVentilate           __attribute__((section(".uninitialized_data"))); 
static volatile bool            cmdExhaust             __attribute__((section(".uninitialized_data"))); 

volatile float intakeInletTempC = 0; 
volatile float intakeOutletTempC = 0; 
volatile float intakeIntercoreTempC = 0; 
volatile float exhaustInletTempC = 0; 
volatile float exhaustOutletTempC = 0; 
volatile float exhaustSecondOutletTempC = 0; 


// Tx, TxEnable, Rx, RxEnable
BangChannel COPROCESSOR(8, 9, 6, 7); 

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
  cmdCool = false; 
  cmdSilent = false; 
  cmdVentilate = 0; 
  cmdExhaust = false; 
}

void aosSetup() 
{ 
  COPROCESSOR.begin(); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_bangSend, 5000); 
}

void aosSetup1()
{
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_bangReceive); 
}

void task_bangSend()
{
  String responseString = getHttpResponseString(); 
  responseString.replace(",",",\r\n"); 
  COPROCESSOR.put(responseString);
}

void task_bangReceive()
{
  if (COPROCESSOR.ready())
  {
    String jsonString = COPROCESSOR.get();
    parseJson(jsonString); 
    Serial.println(jsonString.c_str()); 
  }
}

void parseJson(String jsonString)
{
  JsonDocument document; 
  DeserializationError err = deserializeJson(document, jsonString.c_str());
  
  if (err) 
  {
      EPRINTF("deserializeJson() failed: %s\r\n", err.c_str());
      return; 
  }

  intakeInletTempC   = document["intakeInletTempC"];
  intakeIntercoreTempC  = document["intakeIntercoreTempC"];  
  intakeOutletTempC  = document["intakeOutletTempC"]; 
  exhaustInletTempC  = document["exhaustInletTempC"]; 
  exhaustOutletTempC = document["exhaustOutletTempC"]; 
  exhaustSecondOutletTempC = document["exhaustSecondOutletTempC"]; 
}

void populateHttpResponse(JsonDocument& document) 
{
  document["intakeInletTempC"]   = intakeInletTempC; 
  document["intakeIntercoreTempC"]  = intakeIntercoreTempC;
  document["intakeOutletTempC"]  = intakeOutletTempC;
  document["exhaustInletTempC"]  = exhaustInletTempC;
  document["exhaustOutletTempC"] = exhaustOutletTempC;
  document["exhaustSecondOutletTempC"] = exhaustSecondOutletTempC;
  document["hrvCommands"]["Cool"] = cmdCool; 
  document["hrvCommands"]["Silent"] = cmdSilent; 
  document["hrvCommands"]["Ventilate"] = cmdVentilate; 
  document["hrvCommands"]["Exhaust"] = cmdExhaust; 
}

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  if (argName.equals("c")) 
  { 
    cmdCool = arg.toInt(); 
  }

  if (argName.equals("s")) 
  { 
    cmdSilent = arg.toInt(); 
  }

  if (argName.equals("v")) 
  { 
    cmdVentilate = arg.toInt(); 
  }

  if (argName.equals("e")) 
  { 
    cmdExhaust = arg.toInt(); 
  }

  return success; 
}