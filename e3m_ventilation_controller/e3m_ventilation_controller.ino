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

#define PWM_FREQUENCY 25000
#define HOSTNAME "e3mvent"
#define PWM_MIN 28
#define HOTEND_PWM_MIN 50
#define TARGET_HOTEND_TEMP_C 30
#define TARGET_FRONT_STACK_TEMP_C 23
#define TARGET_PSU_TEMP_C 30
#define TARGET_ENCLOSURE_TEMP_C 35
#define TARGET_UNDER_SHELF_MAIN_TEMP_C 25
#define TARGET_UNDER_SHELF_VENT_TEMP_C 23



#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <aos.h>
#include <config.h>
#include <util.h>

#include <ArduinoJson.h>

#include <PWMFan.h>
#include <PIDController.h> 

using namespace AOS; 

PWMFan ENCLOSURE_FAN_PWM("enclosureFan", 6, PWM_MIN, 30, 100); 
PWMFan HOTEND_EXHAUST_PWM("hotendExhaust", 7, HOTEND_PWM_MIN, 30, 100); 

PWMFan E3M_FRONT_STACK_FAN_PWM("frontStackFan", 10, PWM_MIN, 30, 100); 
PWMFan UNDER_SHELF_VENT_FAN_PWM("underShelfVent", 11, PWM_MIN, 30, 100); 
PWMFan UNDER_SHELF_MAIN_PWM("underShelfMain", 12, PWM_MIN, 30, 100); 
PWMFan E3M_PSU_PWM("psuExhaust", 13, PWM_MIN, 30, 100); 

PIDController ENCLOSURE_PID_CONTROLLER(120, 0.5, 0.5, 0.5);
PIDController UNDER_SHELF_MAIN_PID_CONTROLLER(120, 0.5, 0.5, 0.5);
PIDController UNDER_SHELF_VENT_PID_CONTROLLER(120, 0.5, 0.5, 0.5);
PIDController HOTEND_EXHAUST_PID_CONTROLLER(120, 0.5, 0.5, 0.5);
PIDController E3M_PSU_PID_CONTROLLER(120, 0.5, 0.5, 0.5);
PIDController E3M_FRONT_STACK_PID_CONTROLLER(120, 0.5, 0.5, 0.5);


#define E3M_FRONT_STACK_TEMP 32
#define HOTEND_TEMP 188
#define UNDER_SHELF_MAIN_TEMP 149
#define UNDER_SHELF_VENT_TEMP 161
#define E3M_PSU_TEMP 148

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
}

void aosSetup() 
{
  TEMPERATURES.add("Front Stack Temp", "e3mFrontStackTemp", E3M_FRONT_STACK_TEMP); 
  TEMPERATURES.add("Hotend Temp", "hotendTemp", HOTEND_TEMP); 
  TEMPERATURES.add("Under Shelf Main Temp", "underShelfMainTemp", UNDER_SHELF_MAIN_TEMP);
  TEMPERATURES.add("Under Shelf Vent Tep", "underShelfVentTemp", UNDER_SHELF_VENT_TEMP);
  TEMPERATURES.add("PSU Temp", "psuTemp", E3M_PSU_TEMP);
}

void aosSetup1()
{
  analogWriteFreq(PWM_FREQUENCY); 

  CORE_1_KERNEL->add(CORE_1_KERNEL, task_processCommands, 1000);  
}

void populateHttpResponse(JsonDocument& document) 
{
  UNDER_SHELF_MAIN_PWM.addTo("fans", document); 
  UNDER_SHELF_VENT_FAN_PWM.addTo("fans", document); 
  HOTEND_EXHAUST_PWM.addTo("fans", document); 
  ENCLOSURE_FAN_PWM.addTo("fans", document); 
  E3M_PSU_PWM.addTo("fans", document); 
  E3M_FRONT_STACK_FAN_PWM.addTo("fans", document);
}

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  /*if (argName.equals("cmd") && arg.length() == 1) 
  {
    switch(arg.charAt(0))
    {
      case 'O':  command = CMD_DHR_OFF;  break;  
      case 'C':  command = CMD_DHR_COOL;  break;
      case 'H':  command = CMD_DHR_HEAT;  break;  
      default:   success = false; 
    }
  }*/

  return success; 
}

void task_processCommands()
{
  float hotendTempToTargetHotendTempC            = computeGradientC(TEMPERATURES[HOTEND_TEMP], TARGET_HOTEND_TEMP_C, 0.1);  
  HOTEND_EXHAUST_PWM.setCommand(extrapolatePWM(
        TEMPERATURES[HOTEND_TEMP] > TARGET_HOTEND_TEMP_C,
        hotendTempToTargetHotendTempC, 
        10.0, 0.1, HOTEND_PWM_MIN, 100, HOTEND_EXHAUST_PID_CONTROLLER
      ));
  HOTEND_EXHAUST_PWM.execute(); 

  float enclosureTempC = cpu.getTemperature(); 
  float e3mFrontStackTempToTargetTempC      = computeGradientC(TEMPERATURES[E3M_FRONT_STACK_TEMP],  TARGET_FRONT_STACK_TEMP_C, 0.1); 
  float underShelfMainTempToTargetTempC     = computeGradientC(TEMPERATURES[UNDER_SHELF_MAIN_TEMP], TARGET_UNDER_SHELF_MAIN_TEMP_C, 0.1);
  float underShelfVentTempToTargetTempC = computeGradientC(TEMPERATURES[UNDER_SHELF_VENT_TEMP], TARGET_UNDER_SHELF_VENT_TEMP_C, 0.1); 
  float e3mPsuTempToTargetTempC = computeGradientC(TEMPERATURES[E3M_PSU_TEMP], TARGET_PSU_TEMP_C, 0.1); 
  float enclosureTempToTargetTempC = computeGradientC(enclosureTempC, TARGET_ENCLOSURE_TEMP_C, 0.1); 


  UNDER_SHELF_MAIN_PWM.setCommand(extrapolatePWM(TEMPERATURES[UNDER_SHELF_MAIN_TEMP] > TARGET_UNDER_SHELF_MAIN_TEMP_C, underShelfMainTempToTargetTempC, 7.0, 0.1, PWM_MIN, 100, UNDER_SHELF_MAIN_PID_CONTROLLER));
  UNDER_SHELF_MAIN_PWM.execute(); 
  UNDER_SHELF_VENT_FAN_PWM.setCommand(extrapolatePWM(TEMPERATURES[UNDER_SHELF_VENT_TEMP] > TARGET_UNDER_SHELF_VENT_TEMP_C, underShelfVentTempToTargetTempC, 7.0, 0.1, PWM_MIN, 100, UNDER_SHELF_VENT_PID_CONTROLLER));
  UNDER_SHELF_VENT_FAN_PWM.execute(); 
  ENCLOSURE_FAN_PWM.setCommand(extrapolatePWM(enclosureTempC > TARGET_ENCLOSURE_TEMP_C, enclosureTempToTargetTempC, 10.0, 0.1, PWM_MIN, 100, ENCLOSURE_PID_CONTROLLER));
  ENCLOSURE_FAN_PWM.execute();  
  E3M_PSU_PWM.setCommand(extrapolatePWM(TEMPERATURES[E3M_PSU_TEMP] > TARGET_PSU_TEMP_C, e3mPsuTempToTargetTempC, 10.0, 0.1, PWM_MIN, 100, E3M_PSU_PID_CONTROLLER)); 
  E3M_PSU_PWM.execute(); 
  E3M_FRONT_STACK_FAN_PWM.setCommand(extrapolatePWM(TEMPERATURES[E3M_FRONT_STACK_TEMP] > TARGET_FRONT_STACK_TEMP_C, e3mFrontStackTempToTargetTempC, 7.0, 0.1, PWM_MIN, 100, E3M_FRONT_STACK_PID_CONTROLLER)); 
  E3M_FRONT_STACK_FAN_PWM.execute(); 
}