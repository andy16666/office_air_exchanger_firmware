#include <math.h>
#include <float.h>
#include <aos.h> 
#include <LCDPrint.h>
#include <TemperatureSensors.h> 
#include <GPIOOutputs.h> 
#include <PWMFan.h>
#include <BangProtocol.h> 

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
  This firmware is designed to run on a Raspberry Pi Pico and uses the MBed 
  RTos. It's written for a custom HRV/air exchanger which has 4 different control 
  axes: 
  - Intake, which applies a pressure gradient from an outside air intake, 
  across the intake side of the core assembly, to an inside vent. 
  - Exhaust, which applies a pressure gradient from the inside air across the 
  exhaust side of the core assembly, to an outside vent. 
  - Bypass, which applies a pressure gradient around the intake side of the core,
  bypassing it, bringing outside air directly inside without heat exchange
  - Core Assist, which increases air velocity through the intake side of the 
  core assembly in the direction of intake, and also applies a reverse flow 
  on the bypass axis, which must not exceed the intake pressure gradient. 

  Each axis has an "inlet" and "outlet" side, representing the source and 
  sink directions from the core. 

  Each axis has one or more PWM fans, Noctua Industrial are recommended, 
  controlled by the PWM channels via MOSFETS, and at least one mains powered 
  fan or "blower", which is engaged on the highest PWM level. Intake and Exhaust 
  blowers are centrifugal turbines, while bypass and core assist are either 
  axial turbines or large 120V server fans. 

  4 temperature sensors are used: 
  - Exhaust inlet, which is located anywhere on the room side of the exhaust 
  air flow path, directly in the air flow. 
  - Exhaust outlet, which is located very close to the outlet of the heat exchanger, 
  OR ideally, against the outlet side of the core. This temperature should 
  closely represent the actual core temperature. 
  - Intake inlet, which will be very close to the vent which admits outside air to the 
  system. 
  - Intake outlet, which should be somewhere in the air stream leaving the intake
  side, after the bypass air has been re-introduced. It should give an accurate 
  reading of the resulting temperature after the core air and bypass air are mixed, 
  which informs the ratio of bypass to core assist used to achieve the target 
  temperature.
  */ 

#define HOSTNAME "officeae"

#define INVALID_TEMP FLT_MIN

#define PWM_FREQUENCY      25000
#define PWM_MIN_DUTY_CYCLE    10.0 
#define TEMP_SENSOR_PIN 2

#define INTAKE_INLET_TEMP_ADDR   234
#define INTAKE_OUTLET_TEMP_ADDR  119
#define EXHAUST_INLET_TEMP_ADDR  166
#define EXHAUST_OUTLET_TEMP_ADDR  41

#define CMD_COOL_IDX      0
#define CMD_SILENT_IDX    1
#define CMD_VENTILATE_IDX 2
#define CMD_EXHAUST_IDX   3
#define NUM_CMD_PINS      4

#define NUM_PWM_PINS 4
#define INTAKE_IDX 0
#define EXHAUST_IDX 1
#define BYPASS_IDX 2
#define CA_IDX 3

float PWM_COMMANDS[(NUM_PWM_PINS)]     = {0, 0, 0, 0}; 

// Output Pins
// Puller on the outlet of the core section
#define INTAKE_BLOWER_ON         21
// Pusher on the inlet of the exhaust side 
#define EXHAUST_BLOWER_ON        20
// Puller near the fresh air intake, directly into the room 
#define BYPASS_BLOWER_ON         19
// Diverts air from the outlet to the inlet of the fresh air side, effectively increasing heat transfer from the core. 
#define CORE_ASSIST_BLOWER_ON    18

#define TRANSITION_TIME_MS 5000

#define INTAKE_PWM      10
#define EXHAUST_PWM     11
#define BYPASS_PWM      12
#define CORE_ASSIST_PWM 13

#define PWM_MIN_DUTY_CYCLE    10.0 

using namespace AOS; 
using AOS::TemperatureSensors; 
using AOS::LCDPrint; 

bool   COMMANDS[(NUM_CMD_PINS)] = {false, false, false, false};

// Target air strem temp for cooling 
const float COOL_TEMP_C            = 10; 
const float NORM_TEMP_C            = 20; 
const float HOT_TEMP_C             = 35; 

// How closely do we try to control temperatures around the core. 
const float CORE_TEMP_TOLERANCE_C  = 1.0;

LCDPrint LCD;
PWMFans PWM_FANS(PWM_MIN_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE + 5.0); 
GPIOOutputs BLOWERS("blowers"); 

// Tx, TxEnable, Rx, RxEnable
BangChannel GATEWAY(6, 7, 8, 9); 

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize() { }

volatile int displayRefreshCycle = 0; 

void aosSetup()
{
  LCD.init(); 
  LCD.printLn("Init PWM");

  analogWriteFreq(PWM_FREQUENCY); 

  LCD.printLn("Init PWM Pins"); 

  PWM_FANS.add("Intake PWM",      INTAKE_PWM     ); 
  PWM_FANS.add("Exhaust PWM",     EXHAUST_PWM    ); 
  PWM_FANS.add("Bypass PWM",      BYPASS_PWM     ); 
  PWM_FANS.add("Core Assist PWM", CORE_ASSIST_PWM); 
  PWM_FANS.execute(); 

  LCD.printLn("Init Blower Pins"); 

  BLOWERS.add("Intake",      INTAKE_BLOWER_ON,      false); 
  BLOWERS.add("Exhaust",     EXHAUST_BLOWER_ON,     false); 
  BLOWERS.add("Bypass",      BYPASS_BLOWER_ON,      false); 
  BLOWERS.add("Core Assist", CORE_ASSIST_BLOWER_ON, false); 
  BLOWERS.init(); 
  BLOWERS.setAll(); 

  TEMPERATURES.add("Intake Inlet",   "intakeInletTempC",   INTAKE_INLET_TEMP_ADDR  ); 
  TEMPERATURES.add("Intake Outlet",  "intakeOutletTempC",  INTAKE_OUTLET_TEMP_ADDR ); 
  TEMPERATURES.add("Exhaust Inlet",  "exhaustInletTempC",  EXHAUST_INLET_TEMP_ADDR ); 
  TEMPERATURES.add("Exhaust Outlet", "exhaustOutletTempC", EXHAUST_OUTLET_TEMP_ADDR); 

  LCD.printLn("Starting gateway."); 

  GATEWAY.begin(); 

  CORE_0_KERNEL->add(CORE_0_KERNEL, task_bangSend, 5000); 
  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_lcdRefresh); 
}

void aosSetup1()
{ 
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_bangReceive); 
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_recomputeMotorStates); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateDisplay, 1000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS);
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updatePWMs, TRANSITION_TIME_MS);
}

void task_bangSend()
{
  String responseString = getHttpResponseString(); 
  responseString.replace(",",",\r\n"); 
  DPRINTF("Sending %s\r\n", responseString); 
  GATEWAY.put(responseString);
}

void task_bangReceive()
{
  if (GATEWAY.ready())
  {
    String jsonString = GATEWAY.get();
    parseJson(jsonString); 
  }
}

void parseJson(String jsonString)
{
  JsonDocument gatewayJson; 
  DeserializationError err = deserializeJson(gatewayJson, jsonString.c_str());
  
  if (err) 
  {
      LCD.printfLn("deserializeJson() failed: %s", err.c_str());
      EPRINTF("deserializeJson() failed: %s\r\n", err.c_str());
      return; 
  }

  COMMANDS[CMD_COOL_IDX]      = gatewayJson["hrvCommands"]["Cool"]; 
  COMMANDS[CMD_SILENT_IDX]    = gatewayJson["hrvCommands"]["Silent"]; 
  COMMANDS[CMD_VENTILATE_IDX] = gatewayJson["hrvCommands"]["Ventilate"]; 
  COMMANDS[CMD_EXHAUST_IDX]   = gatewayJson["hrvCommands"]["Exhaust"]; 
}

void populateHttpResponse(JsonDocument& document) 
{
  BLOWERS.addTo("blowers", document); 
  PWM_FANS.addTo("blowerPWMs", document); 
  document["hrvCommands"]["Cool"]      = COMMANDS[CMD_COOL_IDX]; 
  document["hrvCommands"]["Silent"]    = COMMANDS[CMD_SILENT_IDX]; 
  document["hrvCommands"]["Ventilate"] = COMMANDS[CMD_VENTILATE_IDX]; 
  document["hrvCommands"]["Exhaust"]   = COMMANDS[CMD_EXHAUST_IDX]; 
}

void task_updatePWMs()
{
  PWM_FANS.execute(); 
}

void task_updateNextBlower()
{
  BLOWERS.setNext(); 
}

uint8_t extrapolatePWM(float gapC, float rangeC, float minGapC, float pwmMin, float pwmMax)
{
  if (gapC >= rangeC)
    return pwmMax; 
  
  if (gapC < minGapC)  
      return 0; 

  float powerLevel = (gapC - minGapC) / (rangeC - minGapC); 

  return (powerLevel * (pwmMax - pwmMin)) + pwmMin;
}

float computeGradientC(float sourceTempC, float targetTempC, float toleranceC)
{ 
  float availableGradientC = sourceTempC - targetTempC; 
  float availableAmountC = fabs(availableGradientC);

  return (availableAmountC > toleranceC) ? availableAmountC : 0.0; 
}

float clamp(float value, float min, float max)
{
  if (value > max)
    return max; 
  if (value < min)
    return min; 
  
  return value; 
}

/*
  Move the two fan speeds apart so that they don't make an interference pattern
*/ 
void pryApart(volatile float* pwm1, volatile float* pwm2, float minDifference, float maxValue)
{
  // If one is off, we don't care. 
  if (!*pwm1 || !*pwm2)
    return; 

  float differenceAbs = fabs(*pwm1 - *pwm2);

  if (differenceAbs <= minDifference)
  {
    float targetDifference = minDifference - differenceAbs;
    float direction = (*pwm1 >= *pwm2) ? 1 : -1; 

    *pwm1 += direction * targetDifference/2.0;
    *pwm2 -= direction * targetDifference/2.0; 
    
    if (fmax(*pwm1, *pwm2) > maxValue)
    {
      float decrease = fmax(*pwm1, *pwm2) - maxValue; 

      *pwm1 -= decrease;
      *pwm2 -= decrease; 
    }

    if (fmin(*pwm1, *pwm2) < PWM_MIN_DUTY_CYCLE)
    {
      float increase = PWM_MIN_DUTY_CYCLE - fmin(*pwm1, *pwm2); 

      *pwm1 += increase;
      *pwm2 += increase; 
    }
  }
}

void task_recomputeMotorStates() 
{
  // Are we heating or cooling? 
  float targetTempC = COMMANDS[CMD_COOL_IDX] ? COOL_TEMP_C : NORM_TEMP_C; 
  float maxSpeed = COMMANDS[CMD_SILENT_IDX] ? 80 : 100; 

  // Intake and outlet idle at PWM_MIN_DUTY_CYCLE. The outlet 
  // of the exhaust side is a very good indicator of the core 
  // temperature due to the placement of the sensor. 
  float coreTempC = TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR];

  // The exhaust idle flow gives us a good sample of room temperature
  // air, albeit a bit on the warm side if printers are running.
  float estimatedRoomTempC = TEMPERATURES[EXHAUST_INLET_TEMP_ADDR]; 
  
    // Try not to melt the core
  int coreHot = TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR] > HOT_TEMP_C || TEMPERATURES[EXHAUST_INLET_TEMP_ADDR] > HOT_TEMP_C;

  float intakeOutletToTargetTempC       = computeGradientC(TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], targetTempC, 0.1);  
  float coreToIntakeOutletTempC         = computeGradientC(coreTempC,                             TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], 0.1);
  float intakeInletToIntakeOutletTempC  = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], 0.1);
  float intakeInletToTargetTempC        = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  targetTempC, 0.1);
  float exhaustToCoreTempC              = computeGradientC(TEMPERATURES[EXHAUST_INLET_TEMP_ADDR], coreTempC, 0.1);
  float estimatedRoomTempCToTargetTempC = computeGradientC(estimatedRoomTempC,                    targetTempC, 0.1);

  // Target the core temp the same distance from the target as the intake outlet is, but opposite. 
  float targetCoreTempC = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > targetTempC
          ? targetTempC - 2.0 : targetTempC + 2.0;

  float coreTempToTargetCoreTempC       = computeGradientC(coreTempC,                         targetCoreTempC, 0.1);

  // Try to position the core between the target temp and the exhaust temp. 
  float exhaustTempControlAmountC    = coreTempToTargetCoreTempC;
  float intakeTempControlAmountC     = fmin(estimatedRoomTempCToTargetTempC, fmin(intakeOutletToTargetTempC, intakeInletToTargetTempC)); 
  float bypassTempControlAmountC     = fmin(intakeInletToTargetTempC, intakeOutletToTargetTempC); 
  float coreAssistTempControlAmountC = fmin(coreToIntakeOutletTempC, intakeOutletToTargetTempC);

  // Is the exhaust side useful for controlling temperature 
  int exhaustEnableTempControl = TEMPERATURES[EXHAUST_INLET_TEMP_ADDR] > coreTempC
          ? coreTempC < targetCoreTempC
          : coreTempC > targetCoreTempC;

  // Is the intake air useful for controlling temperature
  int intakeEnableTempControl = estimatedRoomTempC > targetTempC  
          ? TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] < estimatedRoomTempC 
          : TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > estimatedRoomTempC;

  int coreAssistEnable = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > targetTempC
          ? coreTempC < TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR]
          : coreTempC > TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR];

  int bypassEnable = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > targetTempC
          ? TEMPERATURES[INTAKE_INLET_TEMP_ADDR] < TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] 
          : TEMPERATURES[INTAKE_INLET_TEMP_ADDR] > TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR];

  PWM_COMMANDS[INTAKE_IDX]     = !COMMANDS[CMD_EXHAUST_IDX]
      ? 
          (intakeEnableTempControl * extrapolatePWM(intakeTempControlAmountC, 5.0, 1.0, PWM_MIN_DUTY_CYCLE, maxSpeed)) 
        + ((COMMANDS[CMD_VENTILATE_IDX] ? 1.0 : 0.0) * maxSpeed)
        + ((COMMANDS[CMD_COOL_IDX] ? 1.0 : 0.0) * maxSpeed)
        + (coreHot * maxSpeed)
      : 0
      ;

  PWM_COMMANDS[EXHAUST_IDX]     = COMMANDS[CMD_EXHAUST_IDX] || COMMANDS[CMD_VENTILATE_IDX] || exhaustEnableTempControl || coreHot
      ?
          (exhaustEnableTempControl * extrapolatePWM(exhaustTempControlAmountC, CORE_TEMP_TOLERANCE_C, 0, PWM_MIN_DUTY_CYCLE, maxSpeed))
        + ((COMMANDS[CMD_VENTILATE_IDX] ? 1.0 : 0.0) * maxSpeed)
        + (COMMANDS[CMD_EXHAUST_IDX] ? 100.0 : 0.0)
        + (coreHot * maxSpeed)
      : 0
      ;

  PWM_COMMANDS[CA_IDX]          = coreAssistEnable        ? extrapolatePWM(coreAssistTempControlAmountC,   CORE_TEMP_TOLERANCE_C, 0.1, PWM_MIN_DUTY_CYCLE, maxSpeed) : 0.0;
  PWM_COMMANDS[BYPASS_IDX]      = bypassEnable            ? extrapolatePWM(bypassTempControlAmountC,       CORE_TEMP_TOLERANCE_C, 0.1, PWM_MIN_DUTY_CYCLE, maxSpeed) : 0.0;
  
  // Always idle the intake and outlet at PWM_MIN_DUTY_CYCLE so we have an accurate temp sample. 
  PWM_COMMANDS[EXHAUST_IDX]     = clamp(PWM_COMMANDS[EXHAUST_IDX    ], PWM_MIN_DUTY_CYCLE, maxSpeed); 
  PWM_COMMANDS[INTAKE_IDX ]     = clamp(PWM_COMMANDS[INTAKE_IDX     ], PWM_MIN_DUTY_CYCLE, maxSpeed);
  PWM_COMMANDS[BYPASS_IDX ]     = clamp(PWM_COMMANDS[BYPASS_IDX     ], 0,                  maxSpeed);
  PWM_COMMANDS[CA_IDX]          = clamp(PWM_COMMANDS[CA_IDX],          0,                  maxSpeed);
  
  if (COMMANDS[CMD_SILENT_IDX])
  {
    pryApart(&PWM_COMMANDS[CA_IDX], &PWM_COMMANDS[BYPASS_IDX], 25, 80); 
    pryApart(&PWM_COMMANDS[CA_IDX], &PWM_COMMANDS[INTAKE_IDX], 15, 90);
  }
  // Safeguard against bypass during CA inversion. 
  else if (PWM_COMMANDS[CA_IDX] >= 100.0)
  {
    PWM_COMMANDS[BYPASS_IDX] = 0; 
  }

  PWM_FANS.get(INTAKE_PWM     ).setCommand(PWM_COMMANDS[INTAKE_IDX]); 
  PWM_FANS.get(EXHAUST_PWM    ).setCommand(PWM_COMMANDS[EXHAUST_IDX]); 
  PWM_FANS.get(BYPASS_PWM     ).setCommand(PWM_COMMANDS[BYPASS_IDX]); 
  PWM_FANS.get(CORE_ASSIST_PWM).setCommand(PWM_COMMANDS[CA_IDX]); 

  BLOWERS.get(INTAKE_BLOWER_ON     ).setCommand(PWM_COMMANDS[INTAKE_IDX] >= 100.0);
  BLOWERS.get(EXHAUST_BLOWER_ON    ).setCommand(PWM_COMMANDS[EXHAUST_IDX] >= 100.0);
  BLOWERS.get(BYPASS_BLOWER_ON     ).setCommand(PWM_COMMANDS[BYPASS_IDX] >= 100.0);
  BLOWERS.get(CORE_ASSIST_BLOWER_ON).setCommand(PWM_COMMANDS[CA_IDX] >= 100.0);
}

////////////////
// LCD 
////////////////

void task_lcdRefresh()
{
  LCD.refresh(); 
}

void task_updateDisplay() 
{
  displayRefreshCycle = displayRefreshCycle >= 8 ? 0 : displayRefreshCycle + 1; 
  
  if (displayRefreshCycle < 4)
  {
    LCD.printfLn(
          "I %5.1f <= %5.1f",
          TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR],
          TEMPERATURES[INTAKE_INLET_TEMP_ADDR]
    );
  }
  else 
  {
    LCD.printfLn(
            "E %5.1f => %5.1f",
            TEMPERATURES[EXHAUST_INLET_TEMP_ADDR],
            TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR]
    );
  }

  LCD.printfLn(
          "%1s%1d%1s%1d%1s%1d%1s%1d%1s %1s%1s%1s%1s%1s ",
          BLOWERS[INTAKE_BLOWER_ON].getCommand()      ? "I" : "i",
          (int)(PWM_COMMANDS[INTAKE_IDX]    /11.1),
          BLOWERS[EXHAUST_BLOWER_ON].getCommand()     ? "E" : "e",
          (int)(PWM_COMMANDS[EXHAUST_IDX]   /11.1),
          BLOWERS[BYPASS_BLOWER_ON].getCommand()      ? "B" : "b",
          (int)(PWM_COMMANDS[BYPASS_IDX]    /11.1),
          BLOWERS[CORE_ASSIST_BLOWER_ON].getCommand() ? "C" : "c",
          (int)(PWM_COMMANDS[CA_IDX]/11.1),
          !BLOWERS.isSet() ? "*" : " ",

          COMMANDS[CMD_COOL_IDX]       ? "C" : "c",
          COMMANDS[CMD_SILENT_IDX]     ? "S" : "s",
          COMMANDS[CMD_VENTILATE_IDX]  ? "V" : "v",
          COMMANDS[CMD_EXHAUST_IDX]    ? "E" : "e",
          false ? "*" : " ",

          ((displayRefreshCycle+1) % 2) == 0 ? "." : " "
  );
  
}
