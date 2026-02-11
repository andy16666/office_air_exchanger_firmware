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
#define PWM_MIN 10
#define TARGET_HOTEND_TEMP_C 30
#define TARGET_FRONT_STACK_TEMP_C 20


#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <aos.h>
#include <config.h>
#include <util.h>

#include <ArduinoJson.h>

#include <PWMFan.h>

using namespace AOS; 

#define TRANSITION_TIME_MS 5000

PWMFan UNDER_SHELF_MAIN_PWM("underShelfMain", 6, PWM_MIN, 30, 100); 
PWMFan HOTEND_EXHAUST_PWM("hotendExhaust", 7, PWM_MIN, 30, 100); 

#define E3M_FRONT_STACK_TEMP 32
#define HOTEND_TEMP 188

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
}

void aosSetup() 
{
  TEMPERATURES.add("Hotend Temp 1", "e3mFrontStackTemp", E3M_FRONT_STACK_TEMP); 
  TEMPERATURES.add("Hotend Temp 2", "hotendTemp2", HOTEND_TEMP); 
}

void aosSetup1()
{
  analogWriteFreq(PWM_FREQUENCY); 

  CORE_1_KERNEL->add(CORE_1_KERNEL, task_processCommands, TRANSITION_TIME_MS);  
}

void populateHttpResponse(JsonDocument& document) 
{
  UNDER_SHELF_MAIN_PWM.addTo("fans", document); 
  HOTEND_EXHAUST_PWM.addTo("fans", document); 
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
  if (TEMPERATURES.get(HOTEND_TEMP).getAgeSeconds() < 5.0)
  {
    bool hotendCoolingEnabled = TEMPERATURES[HOTEND_TEMP] > TARGET_HOTEND_TEMP_C; 
    float hotendPwm = hotendCoolingEnabled 
        ? 
          extrapolatePWM(
            computeGradientC(TEMPERATURES[HOTEND_TEMP], TARGET_HOTEND_TEMP_C, 1.0), 
            20.0, 0.1, PWM_MIN, 100
          ) : 0; 

    HOTEND_EXHAUST_PWM.setCommand(hotendPwm); 
    HOTEND_EXHAUST_PWM.execute(); 
  }

  if (TEMPERATURES.get(E3M_FRONT_STACK_TEMP).getAgeSeconds() < 5.0)
  {
    bool frontStackCoolingEnabled = TEMPERATURES[E3M_FRONT_STACK_TEMP] > TARGET_FRONT_STACK_TEMP_C; 
    float frontStackPwm = frontStackCoolingEnabled 
      ? 
        extrapolatePWM(
          computeGradientC(TEMPERATURES[E3M_FRONT_STACK_TEMP], TARGET_FRONT_STACK_TEMP_C, 1.0), 
          15.0, 0.1, PWM_MIN, 100
        ) : 0; 

    UNDER_SHELF_MAIN_PWM.setCommand(frontStackPwm); 
    UNDER_SHELF_MAIN_PWM.execute(); 
  }
}