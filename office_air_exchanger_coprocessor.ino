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
#define PWM_MIN_DUTY_CYCLE    5.0 

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

float calculateControlChangeMaxRatePctPerSecond(TemperatureSensor& sensor, float targetTempC)
{
  double rateOfChange = sensor.getRateOfChangeDegreesPerSecond(); 
  float tempC = sensor.getTempC(); 

  float tempToTargetTempC       = computeGradientC(tempC, targetTempC, 0.1);  
  bool aboveTarget = tempC > targetTempC;

  float movement = fabs(rateOfChange); 
  bool moving = movement > 0.1; 
  bool increasing = rateOfChange > 0; 

  bool converging = moving && tempToTargetTempC > 0 && ((aboveTarget && !increasing)
      || (!aboveTarget && increasing));

  bool diverging = moving && tempToTargetTempC > 0 && ((aboveTarget && increasing)
      || (aboveTarget && !increasing)); 

  return diverging 
    ? clampf(movement * 10.0, 1.0, 25.0)  
    : converging 
      ? clampf((tempToTargetTempC / movement)/10.0, 0.25, 25.0) 
      : clampf(tempToTargetTempC, 1.0, 25.0); 
}

bool sourceBeyondTarget(float sourceTempC, float sinkTempC, float targetTempC)
{
  float sourceToTargetTempC = computeGradientC(sourceTempC,  targetTempC, 0.1);
  
  bool sourceAboveTarget =  sourceTempC > targetTempC; 
  bool sinkAboveTarget   =  sinkTempC   > targetTempC;

  return ((sourceAboveTarget && !sinkAboveTarget) || (!sourceAboveTarget && sinkAboveTarget)) && sourceToTargetTempC > 0;
}

bool sourceUseful(float sourceTempC, float sinkTempC, float targetTempC)
{
  float sourceToSinkTempC = computeGradientC(sourceTempC,  sinkTempC, 0.1);

  bool sourceAboveTarget = sourceTempC > sinkTempC;
  bool sinkAboveTarget   = sinkTempC   > targetTempC;

  return ((sinkAboveTarget && !sourceAboveTarget) || (!sinkAboveTarget && sourceAboveTarget)) && sourceToSinkTempC > 0;
}

void task_recomputeMotorStates() 
{   
  double intakeRateOfChange = TEMPERATURES.get(INTAKE_OUTLET_TEMP_ADDR).getRateOfChangeDegreesPerSecond(); 
  float intakeTempC = TEMPERATURES[INTAKE_OUTLET_TEMP_ADDR]; 

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

  float intakeOutletToTargetTempC       = computeGradientC(intakeTempC, targetTempC, 0.1);  
  float core2ToIntakeOutletTempC        = computeGradientC(core2TempC,                            intakeTempC, 0.1);
  float core2ToTargetTempC              = computeGradientC(core2TempC,                            targetTempC, 0.1);
  float intakeInletToIntakeOutletTempC  = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  intakeTempC, 0.1);
  float intakeInletToTargetTempC        = computeGradientC(TEMPERATURES[INTAKE_INLET_TEMP_ADDR],  targetTempC, 0.1);
  float exhaustToCoreTempC              = computeGradientC(TEMPERATURES[EXHAUST_INLET_TEMP_ADDR], avgCoreTempC, 0.1);
  float exhaustToCore1TempC             = computeGradientC(TEMPERATURES[EXHAUST_INLET_TEMP_ADDR], core1TempC, 0.1);
  float exhaustToCore2TempC             = computeGradientC(TEMPERATURES[EXHAUST_INLET_TEMP_ADDR], core2TempC, 0.1);
  float exhaustInletTempCToTargetTempC  = computeGradientC(exhaustInletTempC,                     targetTempC, 0.1);

  bool intakeAboveTarget = intakeTempC > targetTempC;

  float intakeMovement = fabs(intakeRateOfChange); 
  bool intakeMoving = intakeMovement > 0.1; 
  bool intakeIncreasing = intakeRateOfChange > 0; 
  
  float tempControlMaxChange = calculateControlChangeMaxRatePctPerSecond(TEMPERATURES.get(INTAKE_OUTLET_TEMP_ADDR), targetTempC); 

  bool bypassUseful     = sourceUseful(TEMPERATURES[INTAKE_INLET_TEMP_ADDR], intakeTempC, targetTempC); 
  bool coreAssistUseful =
        sourceUseful(core2TempC, intakeTempC, targetTempC); 

  bool preferBypass = (bypassUseful && !coreAssistUseful) || (bypassUseful && coreAssistUseful && intakeInletToIntakeOutletTempC > core2ToIntakeOutletTempC); 
  bool coreAssistEnable = coreAssistUseful && !preferBypass;
  bool bypassEnable = bypassUseful && preferBypass;

  bool bypassBeyondTarget     = bypassUseful && sourceBeyondTarget(TEMPERATURES[INTAKE_INLET_TEMP_ADDR], intakeTempC, targetTempC);
  bool coreAssistBeyondTarget = coreAssistUseful && sourceBeyondTarget(core2TempC, intakeTempC, targetTempC);

  float bypassRange     = bypassBeyondTarget     ? fmax(intakeInletToTargetTempC, 1.0) : 3.0; 
  float coreAssistRange = coreAssistBeyondTarget ? fmax(core2ToTargetTempC,       1.0) : 3.0; 

  float bypassAmountC     = bypassBeyondTarget     ? intakeOutletToTargetTempC : intakeInletToIntakeOutletTempC; 
  float coreAssistAmountC = coreAssistBeyondTarget ? intakeOutletToTargetTempC : core2ToIntakeOutletTempC; 

  bool exhaustAboveCore = exhaustInletTempC > core1TempC && exhaustInletTempC > core2TempC; 
  bool exhaustBelowCore = exhaustInletTempC < core1TempC && exhaustInletTempC < core2TempC; 

  // Is the intake air useful for controlling temperature
  bool intakeEnableTempControl = intakeOutletToTargetTempC < 5.0;
  // Try to position the core between the target temp and the exhaust temp. 
  float intakeTempControlAmountC     = intakeEnableTempControl ? (5.0 - intakeOutletToTargetTempC) : 0.0;
  
  float exhaustCoolTargetC = 20.0; 
  bool coolExhaust = exhaustInletTempC > 20.0; 
  float exhaustCoolAmountC    = coolExhaust ? exhaustInletTempC - exhaustCoolTargetC: 0.0;
  
  float core2TargetC = intakeAboveTarget ? intakeTempC - 2.0 : intakeTempC + 2.0;
  float core1TargetC = intakeAboveTarget ? TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR] - 2.0 : TEMPERATURES[INTAKE_INTERCORE_TEMP_ADDR] + 2.0;

  bool exhaustUsefulCore1 = 
      sourceUseful(exhaustInletTempC, core1TempC, core1TargetC) 
      && coreAssistEnable; 

  bool exhaustUsefulCore2 = 
      sourceUseful(exhaustInletTempC, core2TempC, core2TargetC) 
      && coreAssistEnable; 

  float ventilatePwm = clampi(COMMANDS[CMD_VENTILATE_IDX], 0, 100.0); 

  bool intakeFullyActive = PWM_COMMANDS[INTAKE_IDX] >= 50.0; 

  float exhaustCoolTempControlRate = calculateControlChangeMaxRatePctPerSecond(TEMPERATURES.get(EXHAUST_INLET_TEMP_ADDR), exhaustCoolTargetC); 
  float coolExhaustPwm = (coolExhaust
      ? extrapolateGradualPWM(exhaustCoolAmountC, 10.0, 2, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(EXHAUST_PWM).getCommand(), exhaustCoolTempControlRate) 
      : 0.0); 

  float core2TempControlRate = calculateControlChangeMaxRatePctPerSecond(TEMPERATURES.get(EXHAUST_SECOND_OUTLET_TEMP_ADDR), core2TempC); 
  float core1TempControlRate = calculateControlChangeMaxRatePctPerSecond(TEMPERATURES.get(EXHAUST_OUTLET_TEMP_ADDR), core1TempC);
  
  float exhaustTempControlCore1Pwm = (exhaustUsefulCore1
      ? extrapolateGradualPWM(exhaustToCore1TempC, 15.0, 2, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(EXHAUST_PWM).getCommand(), core1TempControlRate) 
      : 0.0); 

  float exhaustTempControlCore2Pwm = (exhaustUsefulCore2 
    ? extrapolateGradualPWM(exhaustToCore1TempC, 15.0, 2, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(EXHAUST_PWM).getCommand(), core2TempControlRate) 
    : 0.0); 

  float exhaustTempControlPwm = fmax(exhaustTempControlCore2Pwm, exhaustTempControlCore1Pwm);

  PWM_COMMANDS[CA_IDX]          =
    PWM_FANS.get(BYPASS_PWM).getState() > 0 ? 0.0 : 
    extrapolateGradualPWM(
      coreAssistEnable ? coreAssistAmountC : 0.0, coreAssistRange, coreAssistRange/20.0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(CORE_ASSIST_PWM).getCommand(), tempControlMaxChange) 
    ;
  PWM_COMMANDS[BYPASS_IDX]      =        
    PWM_FANS.get(CORE_ASSIST_PWM).getState() > 0 ? 0.0 :      
    extrapolateGradualPWM(
      bypassEnable ? bypassAmountC : 0.0,     bypassRange, bypassRange/20.0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(BYPASS_PWM).getCommand(), tempControlMaxChange) 
    ;

  float intakeTempControlPwm = (intakeEnableTempControl 
    ? extrapolateGradualPWM(intakeTempControlAmountC, 5.0, 0, PWM_MIN_DUTY_CYCLE, maxSpeed, PWM_FANS.get(INTAKE_PWM).getCommand(), tempControlMaxChange)
    : 0.0
  ); 

  PWM_COMMANDS[INTAKE_IDX]  = clampf(fmax(ventilatePwm, intakeTempControlPwm), PWM_COMMANDS[BYPASS_IDX], fmax(30.0, maxSpeed - fmax(PWM_COMMANDS[CA_IDX]-30.0, 0.0)));

  PWM_COMMANDS[EXHAUST_IDX] = fmax(ventilatePwm, fmax(exhaustTempControlPwm, coolExhaustPwm));
  
  // Always idle the intake and outlet at PWM_MIN_DUTY_CYCLE so we have an accurate temp sample. 
  PWM_COMMANDS[EXHAUST_IDX]     = clampf(PWM_COMMANDS[EXHAUST_IDX], fmax(20.0, PWM_COMMANDS[INTAKE_IDX]/2.0),  maxSpeed); 
  PWM_COMMANDS[INTAKE_IDX ]     = clampf(PWM_COMMANDS[INTAKE_IDX ], fmax(20.0, PWM_COMMANDS[EXHAUST_IDX]/2.0), maxSpeed);
  PWM_COMMANDS[BYPASS_IDX ]     = clampf(PWM_COMMANDS[BYPASS_IDX ], 0,                  100.0);
  PWM_COMMANDS[CA_IDX]          = clampf(PWM_COMMANDS[CA_IDX     ], 0,                  100.0);
  
  PWM_FANS.get(INTAKE_PWM     ).setCommand((PWM_FANS.get(INTAKE_PWM ).getCommand() + PWM_COMMANDS[INTAKE_IDX     ])/2.0); 
  PWM_FANS.get(EXHAUST_PWM    ).setCommand((PWM_FANS.get(EXHAUST_PWM).getCommand() + PWM_COMMANDS[EXHAUST_IDX    ])/2.0); 
  PWM_FANS.get(BYPASS_PWM     ).setCommand(PWM_COMMANDS[BYPASS_IDX]); 
  PWM_FANS.get(CORE_ASSIST_PWM).setCommand(PWM_COMMANDS[CA_IDX]); 

  BLOWERS.get(INTAKE_BLOWER_ON     ).setCommand(PWM_COMMANDS[INTAKE_IDX]  >= 50.0);
  BLOWERS.get(EXHAUST_BLOWER_ON    ).setCommand(PWM_COMMANDS[EXHAUST_IDX] >= 50.0);
  BLOWERS.get(BYPASS_BLOWER_ON     ).setCommand(PWM_COMMANDS[BYPASS_IDX]  >= 50.0);
  BLOWERS.get(CORE_ASSIST_BLOWER_ON).setCommand(PWM_COMMANDS[CA_IDX]      >= 50.0);
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
        "     %5.1f   %5.1f",
        TEMPERATURES[EXHAUST_SECOND_OUTLET_TEMP_ADDR], 
        TEMPERATURES[EXHAUST_OUTLET_TEMP_ADDR]
  );
  
  LCD.printfLn(
        "%-5.1f%2s%5.1f %2s%5.1f",
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
