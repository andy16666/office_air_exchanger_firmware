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

// 
// 124, 120, 129
#define INTAKE_INLET_TEMP_ADDR   234
#define INTAKE_OUTLET_TEMP_ADDR  129 // 119
#define EXHAUST_INLET_TEMP_ADDR  166 
#define EXHAUST_OUTLET_TEMP_ADDR  41
#define INTAKE_INTERCORE_TEMP_ADDR 124 // ?
#define EXHAUST_SECOND_OUTLET_TEMP_ADDR 120

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

using namespace AOS; 
using AOS::TemperatureSensors; 
using AOS::LCDPrint; 

int   COMMANDS[(NUM_CMD_PINS)] = {0, 0, 0, 0};

// Target air strem temp for cooling 
const float COOL_TEMP_C            = 5 ; 
const float NORM_TEMP_C            = 20; 
const float HOT_TEMP_C             = 35; 

// How closely do we try to control temperatures around the core. 
const float CORE_TEMP_TOLERANCE_C  = 1.0;

LCDPrint LCD(0x27, 20, 4);
PWMFans PWM_FANS(PWM_MIN_DUTY_CYCLE, 50.0); 
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

  PWM_FANS.add("Intake PWM",      INTAKE_PWM     , true); 
  PWM_FANS.add("Exhaust PWM",     EXHAUST_PWM    , true); 
  PWM_FANS.add("Bypass PWM",      BYPASS_PWM     , true); 
  PWM_FANS.add("Core Assist PWM", CORE_ASSIST_PWM, true); 
  PWM_FANS.execute(); 

  LCD.printLn("Init Blower Pins"); 

  BLOWERS.add("Intake",      INTAKE_BLOWER_ON,      false); 
  BLOWERS.add("Exhaust",     EXHAUST_BLOWER_ON,     false); 
  BLOWERS.add("Bypass",      BYPASS_BLOWER_ON,      false); 
  BLOWERS.add("Core Assist", CORE_ASSIST_BLOWER_ON, false); 
  BLOWERS.init(); 
  BLOWERS.setAll(); 

  TEMPERATURES.add("Intake Inlet",   "intakeInletTempC",   INTAKE_INLET_TEMP_ADDR  ); 
  TEMPERATURES.add("Intake Intercore", "intakeIntercoreTempC", INTAKE_INTERCORE_TEMP_ADDR); 
  TEMPERATURES.add("Intake Outlet",  "intakeOutletTempC",  INTAKE_OUTLET_TEMP_ADDR ); 
  TEMPERATURES.add("Exhaust Inlet",  "exhaustInletTempC",  EXHAUST_INLET_TEMP_ADDR ); 
  TEMPERATURES.add("Exhaust Outlet", "exhaustOutletTempC", EXHAUST_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("Exhaust Second Outlet", "exhaustSecondOutletTempC", EXHAUST_SECOND_OUTLET_TEMP_ADDR); 

  LCD.printLn("Starting gateway."); 

  GATEWAY.begin(); 

  CORE_0_KERNEL->add(CORE_0_KERNEL, task_bangSend, 5000); 
  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_lcdRefresh); 
}

void aosSetup1()
{ 
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_bangReceive); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_recomputeMotorStates, 1000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateDisplay, 1000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS);
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updatePWMs, 1000);
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
      //LCD.printfLn("deserializeJson() failed: %s", err.c_str());
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

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  return success; 
}

void task_updatePWMs()
{
  PWM_FANS.execute(); 
}

void task_updateNextBlower()
{
  BLOWERS.setNext(); 
}

float extrapolateGradualPWM(float gapC, float rangeC, float minGapC, float pwmMin, float pwmMax, float lastPwm, float adjustment)
{
  float up = lastPwm + adjustment < pwmMin ? pwmMin : (lastPwm + adjustment < pwmMax ? lastPwm + adjustment : pwmMax);
  float down = lastPwm - adjustment < pwmMin ? 0 : lastPwm - adjustment; 

  float result = extrapolatePWM(gapC, rangeC, minGapC, pwmMin, pwmMax);

  if (result > lastPwm)
    return up; 
  else if (result < lastPwm)
    return down;
  else return lastPwm;
}

void task_recomputeMotorStates() 
{
  double intakeOutletTempAgeSeconds = TEMPERATURES.get(INTAKE_OUTLET_TEMP_ADDR).getAgeSeconds(); 
  float tempAdjustmentSpeed = TEMPERATURES.get(INTAKE_OUTLET_TEMP_ADDR).isTempExpired() 
    ? 0.0 
    : 10.0 / ((intakeOutletTempAgeSeconds * intakeOutletTempAgeSeconds) + 1.0);
    
  // Are we heating or cooling? 
  float targetTempC = COMMANDS[CMD_COOL_IDX] ? COOL_TEMP_C : NORM_TEMP_C; 
  float maxSpeed    = COMMANDS[CMD_SILENT_IDX] ? 75 : 100; 

  float core1TempC = TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR];
  float core2TempC = TEMPERATURES[EXHAUST_SECOND_OUTLET_TEMP_ADDR];

  // Intake and outlet idle at PWM_MIN_DUTY_CYCLE. The outlet 
  // of the exhaust side is a very good indicator of the core 
  // temperature due to the placement of the sensor. 
  float avgCoreTempC = (core1TempC + core2TempC)/2;

  float exhaustInletTempC = TEMPERATURES[EXHAUST_INLET_TEMP_ADDR]; 

  // The exhaust idle flow gives us a good sample of room temperature
  // air, albeit a bit on the warm side if printers are running.
  float estimatedRoomTempC = TEMPERATURES[EXHAUST_INLET_TEMP_ADDR]; 
  
    // Try not to melt the core
  int coreHot = TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR] > HOT_TEMP_C || TEMPERATURES[EXHAUST_INLET_TEMP_ADDR] > HOT_TEMP_C;

  float intakeOutletToTargetTempC       = computeGradientC(TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], targetTempC, 0.1);  
  float avgCoreToIntakeOutletTempC      = computeGradientC(avgCoreTempC,                          TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], 0.1);
  float core1ToIntakeIntercoreTempC     = computeGradientC(core1TempC,                            TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR], 0.1);
  float core2ToIntakeOutletTempC        = computeGradientC(core2TempC,                            TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], 0.1);
  float intakeInletToIntakeOutletTempC  = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR], 0.1);
  float intakeInletToTargetTempC        = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  targetTempC, 0.1);
  float exhaustToCoreTempC              = computeGradientC(TEMPERATURES[EXHAUST_INLET_TEMP_ADDR], avgCoreTempC, 0.1);
  float exhaustInletTempCToTargetTempC = computeGradientC(exhaustInletTempC,                    targetTempC, 0.1);

  // Target the core temp the same distance from the target as the intake outlet is, but opposite. 
  float targetCoreTempC = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > targetTempC
          ? targetTempC - 2.0 : targetTempC + 2.0;

  bool intakeAboveTarget = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR] > targetTempC;

  bool coreAboveIntake = 
         core1TempC > TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR] 
      || core2TempC > TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR];
  bool bypassAboveIntake = TEMPERATURES[INTAKE_INLET_TEMP_ADDR] > TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR];

  bool bypassUseful =     ((intakeAboveTarget && !bypassAboveIntake) || (!intakeAboveTarget && bypassAboveIntake)) && intakeInletToIntakeOutletTempC > 0;
  bool coreAssistUseful = ((intakeAboveTarget && !coreAboveIntake  ) || (!intakeAboveTarget && coreAboveIntake  )) && core1ToIntakeIntercoreTempC > 0 && core2ToIntakeOutletTempC > 0; 

  bool exhaustAboveCore = exhaustInletTempC > core1TempC && exhaustInletTempC > core2TempC; 

  bool preferBypass = (bypassUseful && !coreAssistUseful) || (bypassUseful && coreAssistUseful && intakeInletToIntakeOutletTempC > core2ToIntakeOutletTempC); 

  // Try to position the core between the target temp and the exhaust temp. 
  float intakeTempControlAmountC     = intakeOutletToTargetTempC < 5.0 ? (5.0 - intakeOutletToTargetTempC) : 0.0;

  bool coolExhaust = exhaustInletTempC > 25.0; 

  float exhaustCoolAmountC    = coolExhaust ? (exhaustInletTempC - 25.0): 0.0;
  
  float bypassTempControlAmountC     = bypassUseful     ? intakeOutletToTargetTempC/(intakeInletToIntakeOutletTempC/5.0) : 0.0; 
  float coreAssistTempControlAmountC = coreAssistUseful ? intakeOutletToTargetTempC/(core2ToIntakeOutletTempC      /5.0) : 0.0;

  bool coreAssistEnable = coreAssistUseful && !preferBypass;
  bool bypassEnable = bypassUseful && preferBypass;

  // Is the intake air useful for controlling temperature
  bool intakeEnableTempControl = intakeOutletToTargetTempC < 5.0;

  bool exhaustUseful = ((exhaustAboveCore && !intakeAboveTarget) || (!exhaustAboveCore && intakeAboveTarget)) && exhaustToCoreTempC > 0 && coreAssistEnable; 
  
  float exhaustTempControlAmountC = exhaustToCoreTempC; 

  float ventilatePwm = (clampi(COMMANDS[CMD_VENTILATE_IDX], 0, 100.0)); 
  float intakeTempControlPwm = (intakeEnableTempControl 
      ? extrapolateGradualPWM(intakeTempControlAmountC, 5.0, 0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(INTAKE_PWM).getCommand(), tempAdjustmentSpeed)
      : 0.0
    ); 

  PWM_COMMANDS[INTAKE_IDX]     = !COMMANDS[CMD_EXHAUST_IDX]
      ? 
          fmax(ventilatePwm, intakeTempControlPwm)
        + (coreHot ? 100.0 : 0.0)
      : 0
      ;

  bool intakeFullyActive = PWM_COMMANDS[INTAKE_IDX] >= 50.0; 

  // Is the exhaust side useful for controlling temperature 
  bool exhaustEnableTempControl = exhaustUseful && intakeFullyActive;
  float coolExhaustPwm = (coolExhaust 
      ? extrapolateGradualPWM(exhaustCoolAmountC, 5.0, 0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(EXHAUST_PWM).getCommand(), tempAdjustmentSpeed) 
      : 0.0); 
  float exhaustTempControlPwm = (exhaustEnableTempControl 
      ? extrapolateGradualPWM(exhaustTempControlAmountC, 15.0, 0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(EXHAUST_PWM).getCommand(), tempAdjustmentSpeed) 
      : 0.0
    ); 
  
  PWM_COMMANDS[EXHAUST_IDX]     = COMMANDS[CMD_EXHAUST_IDX] || COMMANDS[CMD_VENTILATE_IDX] || exhaustEnableTempControl || coolExhaust || coreHot
      ?
          fmax(ventilatePwm, fmax(exhaustTempControlPwm, coolExhaustPwm))
        + (COMMANDS[CMD_EXHAUST_IDX] ? 100.0 : 0.0)
        + (coreHot ? 100.0 : 0.0)
      : (PWM_COMMANDS[INTAKE_IDX] / 2.0)
      ;

  float tempControlMaxSpeed = intakeFullyActive ? 100.0 : 75.0; 
  
  PWM_COMMANDS[CA_IDX]          =
    PWM_FANS.get(BYPASS_PWM).getState() > 0 ? 0.0 : 
    extrapolateGradualPWM(
      coreAssistEnable ? coreAssistTempControlAmountC : 0.0, 1, 0.1, PWM_MIN_DUTY_CYCLE, tempControlMaxSpeed, PWM_FANS.get(CORE_ASSIST_PWM).getCommand(), tempAdjustmentSpeed) 
    ;
  PWM_COMMANDS[BYPASS_IDX]      =        
    PWM_FANS.get(CORE_ASSIST_PWM).getState() > 0 ? 0.0 :      
    extrapolateGradualPWM(
      bypassEnable ? bypassTempControlAmountC : 0.0,     3, 0.1, PWM_MIN_DUTY_CYCLE, tempControlMaxSpeed, PWM_FANS.get(BYPASS_PWM).getCommand(), tempAdjustmentSpeed) 
    ;

  PWM_COMMANDS[INTAKE_IDX] = clampf(PWM_COMMANDS[INTAKE_IDX], PWM_COMMANDS[BYPASS_IDX], maxSpeed); 
  
  // Always idle the intake and outlet at PWM_MIN_DUTY_CYCLE so we have an accurate temp sample. 
  PWM_COMMANDS[EXHAUST_IDX]     = clampf(
    (PWM_FANS.get(EXHAUST_PWM).getCommand() + PWM_COMMANDS[EXHAUST_IDX    ])/2.0, 
    fmax(PWM_MIN_DUTY_CYCLE + 1.0, PWM_COMMANDS[INTAKE_IDX]/2.0),
    maxSpeed); 
  PWM_COMMANDS[INTAKE_IDX ]     = clampf(
    (PWM_FANS.get(INTAKE_PWM).getCommand() + PWM_COMMANDS[INTAKE_IDX     ])/2.0, 
    fmax(PWM_MIN_DUTY_CYCLE + 1.0, PWM_COMMANDS[EXHAUST_IDX]/2.0), 
    maxSpeed);
  PWM_COMMANDS[BYPASS_IDX ]     = clampf(PWM_COMMANDS[BYPASS_IDX     ], 0,                  tempControlMaxSpeed);
  PWM_COMMANDS[CA_IDX]          = clampf(PWM_COMMANDS[CA_IDX],          0,                  tempControlMaxSpeed);
  
  /*if (COMMANDS[CMD_SILENT_IDX])
  {
    if (bypassEnable) 
      pryApart(&PWM_COMMANDS[INTAKE_IDX], &PWM_COMMANDS[BYPASS_IDX], 10, maxSpeed + 5); 
    else if (coreAssistEnable) 
      pryApart(&PWM_COMMANDS[INTAKE_IDX], &PWM_COMMANDS[CA_IDX], 10, maxSpeed + 5); 
  }*/
  
  PWM_FANS.get(INTAKE_PWM     ).setCommand(PWM_COMMANDS[INTAKE_IDX]); 
  PWM_FANS.get(EXHAUST_PWM    ).setCommand(PWM_COMMANDS[EXHAUST_IDX]); 
  PWM_FANS.get(BYPASS_PWM     ).setCommand(PWM_COMMANDS[BYPASS_IDX]); 
  PWM_FANS.get(CORE_ASSIST_PWM).setCommand(PWM_COMMANDS[CA_IDX]); 

  BLOWERS.get(INTAKE_BLOWER_ON     ).setCommand(PWM_COMMANDS[INTAKE_IDX] >= 50.0);
  BLOWERS.get(EXHAUST_BLOWER_ON    ).setCommand(PWM_COMMANDS[EXHAUST_IDX] >= 50.0);
  BLOWERS.get(BYPASS_BLOWER_ON     ).setCommand(PWM_COMMANDS[BYPASS_IDX] >= 50.0);
  BLOWERS.get(CORE_ASSIST_BLOWER_ON).setCommand(PWM_COMMANDS[CA_IDX] >= 50.0);
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
  
  /*
  --------------------
  100% 24.8 v      
  100% -18.0    -18.3 
  -15.9 |-12.4 |-10.5 
    ^             v
  */

  LCD.printfLn(
        "E%3.0f%%%5.1fv  %1s%1s%1s%1s%1d%1s%1s",
        PWM_COMMANDS[EXHAUST_IDX],
        TEMPERATURES[EXHAUST_INLET_TEMP_ADDR],
        !BLOWERS.isSet() ? "*" : " ",
        COMMANDS[CMD_COOL_IDX]       ? "C" : "c",
        COMMANDS[CMD_SILENT_IDX]     ? "S" : "s",
        COMMANDS[CMD_VENTILATE_IDX]  ? "V" : "v",
        (int)(COMMANDS[CMD_VENTILATE_IDX]    /11.1),
        COMMANDS[CMD_EXHAUST_IDX]    ? "E" : "e",
        ((displayRefreshCycle+1) % 2) == 0 ? "." : " "
  );

  LCD.printfLn(
        "   %5.1f   %5.1f",
        TEMPERATURES[EXHAUST_SECOND_OUTLET_TEMP_ADDR], 
        TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR]
  );
  
  LCD.printfLn(
        "%5.1f%2s%5.1f %2s%5.1f",
        TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR],
        PWM_COMMANDS[CA_IDX] > 0 ? "<<" : PWM_COMMANDS[BYPASS_IDX] > 0 ? "<|" : "==",
        TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR],
        PWM_COMMANDS[CA_IDX] > 0 ? "<<" : PWM_COMMANDS[BYPASS_IDX] > 0 ? "|v" : "==",
        TEMPERATURES[INTAKE_INLET_TEMP_ADDR]
  );

  LCD.printfLn(
        "I%3.0f%%%2s%2s%3.0f%%%2s",
        PWM_COMMANDS[INTAKE_IDX],
        PWM_COMMANDS[CA_IDX] > 0 ? "  " : (PWM_COMMANDS[BYPASS_IDX] > 0 ? "^<" : " |"),
        PWM_COMMANDS[CA_IDX] > 0 ? "CA" : (PWM_COMMANDS[BYPASS_IDX] > 0 ? "BP" : "--"),
        PWM_COMMANDS[CA_IDX] > 0 ? PWM_COMMANDS[CA_IDX] : PWM_COMMANDS[BYPASS_IDX],
        PWM_COMMANDS[CA_IDX] > 0 ? "  " : (PWM_COMMANDS[BYPASS_IDX] > 0 ? "<<" : "| ")

  );

  /*LCD.printfLn(
        "I%5.1f <=%5.1f      ",
        TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR],
        TEMPERATURES[INTAKE_INLET_TEMP_ADDR]
  );
  LCD.printfLn(
        "      %5.1f         ",
        TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR]
  );
  LCD.printfLn(
          "E%5.1f =>%5.1f %5.1f",
          TEMPERATURES[EXHAUST_INLET_TEMP_ADDR],
          TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR],
          TEMPERATURES[EXHAUST_SECOND_OUTLET_TEMP_ADDR]
  );
  LCD.printfLn(
          "%1s%1d%1s%1d%1s%1d%1s%1d%1s %1s%1s%1s%1s%1s %1s   ",
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
  );*/
  
}
