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

#define PWM_FREQUENCY 22000
#define HOSTNAME "e3mvent"

#define PWM_MIN 28
#define HOTEND_PWM_MIN 35
#define PWM_MIN_NOCTUA_INDUSTRIAL 20
#define PWM_MIN_4500_24V 35

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

PWMFans FANS(PWM_MIN, PWM_MIN + 20); 

PIDController ENCLOSURE_PID_CONTROLLER(120, 0.5, 0.6, 0.76);
PIDController UNDER_SHELF_MAIN_PID_CONTROLLER(120, 0.5, 0.9, 0.75);
PIDController UNDER_SHELF_VENT_PID_CONTROLLER(120, 0.5, 0.5, 0.9);
PIDController HOTEND_EXHAUST_PID_CONTROLLER(120, 0.5, 0.75, 0.9);
PIDController E3M_PSU_PID_CONTROLLER(120, 0.5, 0.5, 0.9);
PIDController E3M_FRONT_STACK_PID_CONTROLLER(120, 0.5, 0.75, 0.9);

#define E3M_FRONT_STACK_TEMP 32
#define HOTEND_TEMP 188
#define UNDER_SHELF_MAIN_TEMP 149
#define UNDER_SHELF_VENT_TEMP 161
#define E3M_PSU_TEMP 148

#define ENCLOSURE_FAN_PWM 8
#define HOTEND_EXHAUST_PWM 9

#define E3M_FRONT_STACK_FAN_PWM 10
#define UNDER_SHELF_VENT_FAN_PWM 11
#define UNDER_SHELF_MAIN_PWM 12
#define E3M_PSU_PWM 13


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

  FANS.add("enclosureFan",   ENCLOSURE_FAN_PWM, PWM_MIN, 35); 

  FANS.add("hotendExhaust",  HOTEND_EXHAUST_PWM,       HOTEND_PWM_MIN,            HOTEND_PWM_MIN + 10); 

  FANS.add("frontStackFan",  E3M_FRONT_STACK_FAN_PWM,  PWM_MIN_NOCTUA_INDUSTRIAL, 30); 
  FANS.add("underShelfVent", UNDER_SHELF_VENT_FAN_PWM, PWM_MIN_NOCTUA_INDUSTRIAL, 30); 

  FANS.add("underShelfMain", UNDER_SHELF_MAIN_PWM,     PWM_MIN_4500_24V,          PWM_MIN_4500_24V + 10); 
  FANS.add("psuExhaust",     E3M_PSU_PWM,              PWM_MIN_4500_24V,          PWM_MIN_4500_24V + 10); 

  CORE_1_KERNEL->add(CORE_1_KERNEL, task_processCommands, 1000);  
}

void populateHttpResponse(JsonDocument& document) 
{
  FANS.addTo("fans", document); 
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
  float enclosureTempC = cpu.getTemperature(); 
  float e3mFrontStackTempToTargetTempC      = computeGradientC(TEMPERATURES[E3M_FRONT_STACK_TEMP],  TARGET_FRONT_STACK_TEMP_C, 0.1); 
  float underShelfMainTempToTargetTempC     = computeGradientC(TEMPERATURES[UNDER_SHELF_MAIN_TEMP], TARGET_UNDER_SHELF_MAIN_TEMP_C, 0.1);
  float underShelfVentTempToTargetTempC = computeGradientC(TEMPERATURES[UNDER_SHELF_VENT_TEMP], TARGET_UNDER_SHELF_VENT_TEMP_C, 0.1); 
  float e3mPsuTempToTargetTempC = computeGradientC(TEMPERATURES[E3M_PSU_TEMP], TARGET_PSU_TEMP_C, 0.1); 
  float enclosureTempToTargetTempC = computeGradientC(enclosureTempC, TARGET_ENCLOSURE_TEMP_C, 0.1); 
  float hotendTempToTargetHotendTempC            = computeGradientC(TEMPERATURES[HOTEND_TEMP], TARGET_HOTEND_TEMP_C, 0.1);

  FANS.get(ENCLOSURE_FAN_PWM).setCommand(extrapolatePWM(enclosureTempC > TARGET_ENCLOSURE_TEMP_C, enclosureTempToTargetTempC, 10.0, 0.1, PWM_MIN, 100, ENCLOSURE_PID_CONTROLLER));

  FANS.get(HOTEND_EXHAUST_PWM)
    .setCommand(
      extrapolatePWM(
        TEMPERATURES[HOTEND_TEMP] > TARGET_HOTEND_TEMP_C,
        hotendTempToTargetHotendTempC, 
        10.0, 0.1, HOTEND_PWM_MIN, 100, HOTEND_EXHAUST_PID_CONTROLLER
      ));

  FANS.get(UNDER_SHELF_MAIN_PWM)
    .setCommand(
      extrapolatePWM(
        TEMPERATURES[UNDER_SHELF_MAIN_TEMP] > TARGET_UNDER_SHELF_MAIN_TEMP_C, 
        underShelfMainTempToTargetTempC, 7.0, 0.1, PWM_MIN_4500_24V, 100, UNDER_SHELF_MAIN_PID_CONTROLLER
      ));

  FANS.get(E3M_PSU_PWM)
    .setCommand(
      extrapolatePWM(TEMPERATURES[E3M_PSU_TEMP] > TARGET_PSU_TEMP_C, e3mPsuTempToTargetTempC, 10.0, 0.1, PWM_MIN_4500_24V, 100, E3M_PSU_PID_CONTROLLER)); 

  
  FANS.get(UNDER_SHELF_VENT_FAN_PWM)
    .setCommand(
      extrapolatePWM(TEMPERATURES[UNDER_SHELF_VENT_TEMP] > TARGET_UNDER_SHELF_VENT_TEMP_C, underShelfVentTempToTargetTempC, 7.0, 0.1, PWM_MIN_NOCTUA_INDUSTRIAL, 100, UNDER_SHELF_VENT_PID_CONTROLLER));
  
  FANS.get(E3M_FRONT_STACK_FAN_PWM)
    .setCommand(
      extrapolatePWM(TEMPERATURES[E3M_FRONT_STACK_TEMP] > TARGET_FRONT_STACK_TEMP_C, e3mFrontStackTempToTargetTempC, 7.0, 0.1, PWM_MIN_NOCTUA_INDUSTRIAL, 100, E3M_FRONT_STACK_PID_CONTROLLER)); 

  FANS.execute(); 
}