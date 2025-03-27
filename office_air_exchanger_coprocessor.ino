#include "MBED_RP2040_PWM.h"
#include <LiquidCrystal_I2C.h>
#include <DS18B20.h>
#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <math.h>

using namespace rtos; 

Semaphore temperatureLock(1); 

Thread motorThread; 
Thread displayThread;
Thread temperatureThread;  

#define PWM_FREQUENCY 25000
#define PWM_MIN_DUTY_CYCLE 10.0 

#define INVALID_TEMP NAN

#define TEMP_SENSOR_DATA 2
// Commands the system to attempt to lower the room temperature 
#define CMD_TEMP_COOL 6 // 25

#define CMD_SILENT 7 // 23
// Request air exchange
#define CMD_CO2_HIGH 8 // 24
// Request a negative pressure condition in the room (preventing air from leaving, 
// for example when a door is opened to another part of the house)
#define CMD_EXHAUST 9 // 22

//pin_size_t commandPins[] = {CMD_TEMP_COOL, CMD_SILENT, CMD_CO2_HIGH, CMD_EXHAUST}; 

// Sensor Addresses
#define INTAKE_INLET_TEMP_ADDR 234
#define EXHAUST_OUTLET_TEMP_ADDR 41
#define INTAKE_OUTLET_TEMP_ADDR 119
#define EXHAUST_INLET_TEMP_ADDR 166

// Output Pins
// Puller on the outlet of the core section
#define INTAKE_BLOWER_ON 21
// Pusher on the inlet of the exhaust side 
#define EXHAUST_BLOWER_ON 20
// Puller near the fresh air intake, directly into the room 
#define BYPASS_BLOWER_ON 19
// Diverts air from the outlet to the inlet of the fresh air side, effectively increasing heat transfer from the core. 
#define CORE_ASSIST_ON 18

//pin_size_t relayPins[] = {INTAKE_BLOWER_ON, EXHAUST_BLOWER_ON, BYPASS_BLOWER_ON, CORE_ASSIST_ON}; 

#define INTAKE_PWM 10
#define EXHAUST_PWM 11
#define BYPASS_PWM 12
#define CORE_ASSIST_PWM 13

//pin_size_t pwmPins[] = {INTAKE_PWM, EXHAUST_PWM, BYPASS_PWM, CORE_ASSIST_PWM}; 
mbed::PwmOut* pwm[]    = { NULL, NULL, NULL, NULL };

const float TEMP_ADJUST_TOLERANCE = 2.0; 
const float TARGET_TEMP_C = 20.0; // TODO: Make this an input from HK. 
// Target air strem temp for cooling 
const float COOL_TEMP_C = TARGET_TEMP_C - TEMP_ADJUST_TOLERANCE; 
// Target air stream temp for heating
const float HEAT_TEMP_C = TARGET_TEMP_C + TEMP_ADJUST_TOLERANCE; 

DS18B20 ds(TEMP_SENSOR_DATA);

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Intended motor states
volatile int intakeOn;
volatile int exhaustOn;
volatile int coreAssistOn;
volatile int bypassOn;

volatile float intakePWM = 0; 
volatile float exhaustPWM = 0; 
volatile float bypassPWM = 0; 
volatile float coreAssistPWM = 0; 

// Current motor states
volatile int intakeOnPrev = 0;
volatile int exhaustOnPrev = 0;
volatile int bypassOnPrev = 0;
volatile int coreAssistOnPrev = 0;

volatile int cmdCool    = 0; 
volatile int cmdSilent    = 0; 
volatile int cmdVentilate = 0; 
volatile int cmdExhaust = 0; 

volatile float intakeInletTempC = INVALID_TEMP; 
volatile float intakeOutletTempC = INVALID_TEMP; 
volatile float exhaustInletTempC = INVALID_TEMP; 
volatile float exhaustOutletTempC = INVALID_TEMP; 



// Toggled to 1 whenever any input changes
volatile int inputsChanged = 0; 

char printBuffer[1024];



void setup() {
  intakeOn = 0;
  exhaustOn = 0;
  coreAssistOn = 0;
  bypassOn = 0;

  // Commands from homebridge/homekit automations
  pinMode(CMD_TEMP_COOL, INPUT_PULLDOWN);
  pinMode(CMD_SILENT, INPUT_PULLDOWN);
  pinMode(CMD_CO2_HIGH, INPUT_PULLDOWN);
  pinMode(CMD_EXHAUST, INPUT_PULLDOWN);

  // Inputs from HK are interrupt driven to make the system instantly respond to changes. 
  attachInterrupt(digitalPinToInterrupt(CMD_TEMP_COOL), handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_SILENT), handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_CO2_HIGH),  handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_EXHAUST), handleInputChange, CHANGE);

  pinMode(INTAKE_BLOWER_ON, OUTPUT);
  pinMode(EXHAUST_BLOWER_ON, OUTPUT);
  pinMode(BYPASS_BLOWER_ON, OUTPUT);
  pinMode(CORE_ASSIST_ON, OUTPUT);

  pinMode(INTAKE_PWM, OUTPUT);
  pinMode(EXHAUST_PWM, OUTPUT);
  pinMode(BYPASS_PWM, OUTPUT);
  pinMode(CORE_ASSIST_PWM, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("Devices: ");
  Serial.println(ds.getNumberOfDevices());
  Serial.println();

  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();           // LCD driver initialization
  lcd.backlight();      
  lcd.setCursor(0, 0);
  lcd.print("Starting Up");
  digitalWrite(INTAKE_BLOWER_ON, LOW);
  digitalWrite(EXHAUST_BLOWER_ON, LOW);
  digitalWrite(BYPASS_BLOWER_ON, LOW);
  digitalWrite(CORE_ASSIST_ON, LOW);
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up.");
  delay(1000);
  digitalWrite(INTAKE_BLOWER_ON, HIGH);
  digitalWrite(EXHAUST_BLOWER_ON, HIGH);
  digitalWrite(BYPASS_BLOWER_ON, HIGH);
  digitalWrite(CORE_ASSIST_ON, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up..");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up...");
  delay(1000);

  temperatureThread.start(mbed::callback(temperatureThreadImpl));
  displayThread.start(mbed::callback(displayThreadImpl));
  motorThread.start(mbed::callback(motorThreadImpl));
}

void loop() { }

int countSensors() {
  int count = 0;
  while (ds.selectNext()) count++;
  return count;
}

int getSensorData(int* address, float* tempC, int nSensors) {
  int i = 0;
  while (ds.selectNext()) {
    if (i < nSensors) {
      uint8_t sensorAddress[8];
      ds.getAddress(sensorAddress);
      address[i] = sensorAddress[7];
      tempC[i] = ds.getTempC();
    }

    i++;
  }

  return i;
}

float getTempC(int requestedAddress, int* address, float* tempC, int nSensors) {
  int i;
  for (i = 0; i < nSensors; i++) {
    if (requestedAddress == address[i])
      return tempC[i];
  }

  return INVALID_TEMP;
}

void updateTemperature(int requestedAddress, int* address, float* tempC, int nSensors, volatile float* currentTempC) {
  float newTempC = getTempC(requestedAddress, address, tempC, nSensors); 

  if (
       newTempC != INVALID_TEMP 
    && newTempC == newTempC     // NaN check
  )
  {
    // Compare truncated to 1 decimal place 
    int significantChange = (int)(*currentTempC * 10.0) != (int)(newTempC * 10.0); 
    *currentTempC = newTempC; 
    if (significantChange)
    {
      inputsChanged = 1; 
    }
  }
}

uint8_t extrapolatePWM(float gapC, float rangeC, float minGapC, float pwmMin)
{
  if (gapC >= rangeC)
    return 100.0; 
  
  if (gapC <= minGapC)  
      return 0; 

  float powerLevel = (gapC - minGapC) / (rangeC - minGapC); 

  float pwmDutyCycle = powerLevel * 100.0;
  return pwmDutyCycle > pwmMin ? pwmDutyCycle : pwmMin;
}


int computeGradientAvailability(float sourceTempC, float targetTempC, float toleranceC, int heat)
{ 
  float availableGradientC = sourceTempC - targetTempC; 
  float availableAmountC = fabs(availableGradientC);

  int available = (heat ? availableGradientC > 0 : availableGradientC < 0);

  return (available && availableAmountC > toleranceC) ? 1 : 0; 
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

  float differenceAbs = abs(*pwm1 - *pwm2);

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

void recomputeMotorStates() 
{
  temperatureLock.acquire(); 
  // Are we heating or cooling? 
  int tempControlEnable = cmdCool; 

  float exhaustCoreTempC = exhaustOutletTempC;
  float coreTempC = exhaustCoreTempC;

  float approxRoomTempC = exhaustInletTempC; 

  float tcCaTolerance = tempControlEnable ? TEMP_ADJUST_TOLERANCE : 0.1;

    // Try not to melt the core
  int coreHot = exhaustOutletTempC > 40 || exhaustInletTempC > 40;

  int caAdjustCool = intakeOutletTempC > TARGET_TEMP_C;
  int caAdjustHeat = intakeOutletTempC < TARGET_TEMP_C;

  // Is the intake air useful for controlling temperature
  int intakeEnableTempControl = 
        computeGradientAvailability(intakeOutletTempC, TARGET_TEMP_C, 0.1, intakeOutletTempC < TARGET_TEMP_C && !cmdCool);
  float intakeTempControlAmountC = computeGradientC(
      intakeOutletTempC,
      TARGET_TEMP_C,  
      0.1);

  // Is the core assit feature useful for either warming or cooling the core to assist in heating or cooling? 
  float caAvailableAmountC = computeGradientC(coreTempC, intakeOutletTempC, 0.1);

  Serial.print("caAvailableAmountC: ");
  Serial.println(caAvailableAmountC);

  // Is the exhaust side useful for controlling temperature 
  int exhaustEnableTempControl = 
         computeGradientAvailability(exhaustInletTempC, coreTempC, 1.0, coreTempC < TARGET_TEMP_C);
  float exhaustTempControlAmountC = computeGradientC(
      exhaustInletTempC, 
      coreTempC, 
      1.0);

  // Bypass is available whenever the intake inlet is cool or warm enough. 
  int bypassGradientAvailable = computeGradientAvailability(intakeInletTempC, intakeOutletTempC, TEMP_ADJUST_TOLERANCE, intakeOutletTempC > TARGET_TEMP_C);
  int bypassAvailable = (tempControlEnable && bypassGradientAvailable) || (!tempControlEnable && cmdVentilate && !bypassGradientAvailable); 
  float bypassAmountC = computeGradientC(
      intakeInletTempC, 
      intakeOutletTempC, 
      TEMP_ADJUST_TOLERANCE);

  temperatureLock.release(); 

  int bypassEnable = !cmdExhaust && bypassAvailable;

  coreAssistPWM = extrapolatePWM(caAvailableAmountC, 2, 0.01, PWM_MIN_DUTY_CYCLE) * 2.0;
  bypassPWM     = bypassEnable * extrapolatePWM(bypassAmountC, TEMP_ADJUST_TOLERANCE, 0.25, PWM_MIN_DUTY_CYCLE)

              + cmdVentilate * extrapolatePWM((1.0/(1.0+caAvailableAmountC)), 1.0, 0.0, PWM_MIN_DUTY_CYCLE)
              ;
  
  exhaustPWM    = cmdExhaust || cmdVentilate || exhaustEnableTempControl || coreHot
      ?
        exhaustEnableTempControl * extrapolatePWM(exhaustTempControlAmountC, 3, 0, PWM_MIN_DUTY_CYCLE) 
        + cmdExhaust * 100.0
        + coreHot * 100.0
        + PWM_MIN_DUTY_CYCLE
      : 0
      ;

  intakePWM     = !cmdExhaust  
      ? 
          intakeEnableTempControl * extrapolatePWM(intakeTempControlAmountC, 3, 0, PWM_MIN_DUTY_CYCLE) 
        + bypassPWM
        + cmdVentilate * 90.0 
        + coreHot * 100.0
        + PWM_MIN_DUTY_CYCLE
      : 0;

  float maxSpeed = cmdSilent ? 80 : 200; 

  exhaustPWM    =  clamp(exhaustPWM,   PWM_MIN_DUTY_CYCLE, maxSpeed); 
  intakePWM     =  clamp(intakePWM,    PWM_MIN_DUTY_CYCLE, maxSpeed);
  coreAssistPWM =  clamp(coreAssistPWM, 0, maxSpeed);
  bypassPWM     =  clamp(bypassPWM,    0, maxSpeed);

  // Motors
  intakeOn     = intakePWM     >= 100;
  exhaustOn    = exhaustPWM    >= 100;
  coreAssistOn = coreAssistPWM >= 100;
  bypassOn     = bypassPWM     >= 100;
  
  maxSpeed = cmdSilent ? 80 : 100; 

  exhaustPWM    =  clamp(exhaustPWM,     PWM_MIN_DUTY_CYCLE, maxSpeed); 
  intakePWM     =  clamp(intakePWM,     PWM_MIN_DUTY_CYCLE, maxSpeed);
  coreAssistPWM =  clamp(coreAssistPWM, 0, maxSpeed);
  bypassPWM     =  clamp(bypassPWM,     0, maxSpeed);
  
  pryApart(&coreAssistPWM, &bypassPWM, 25.0, maxSpeed); 
}


void updateMotorStates() 
{
  digitalWrite(INTAKE_BLOWER_ON,  intakeOn ? LOW : HIGH);    // Active Low
  digitalWrite(EXHAUST_BLOWER_ON, exhaustOn ? LOW : HIGH);  // Active Low
  digitalWrite(BYPASS_BLOWER_ON,  bypassOn ? LOW : HIGH);    // Active Low
  digitalWrite(CORE_ASSIST_ON,    coreAssistOn ? LOW : HIGH);  // Active Low
  
  // Save current states
  intakeOnPrev     = intakeOn; 
  exhaustOnPrev    = exhaustOn;
  bypassOnPrev     = bypassOn; 
  coreAssistOnPrev = coreAssistOn;
}

void updateDisplay() 
{
  for (int i = 0; i < 8; i++)
  {
    if (i < 4)
    {
      sprintf(printBuffer,
            "I %5.1f <= %5.1f",
            intakeOutletTempC,
            intakeInletTempC);
      lcd.setCursor(0, 0);
      lcd.print(printBuffer);
    }
    else 
    {
      sprintf(printBuffer,
              "E %5.1f => %5.1f",
              exhaustInletTempC,
              exhaustOutletTempC);
      lcd.setCursor(0, 0);
      lcd.print(printBuffer);
    }

    sprintf(printBuffer,
            "%1s%1d%1s%1d%1s%1d%1s%1d%1s %1s%1s%1s%1s%1s ",
            intakeOn     ? "I" : "i",
            (int)(intakePWM    /11.1),
            exhaustOn    ? "E" : "e",
            (int)(exhaustPWM   /11.1),
            bypassOn     ? "B" : "b",
            (int)(bypassPWM    /11.1),
            coreAssistOn ? "C" : "c",
            (int)(coreAssistPWM/11.1),
            motorStatesDirty() ? "*" : " ",

            cmdCool      ? "C" : "c",
            cmdSilent      ? "S" : "s",
            cmdVentilate   ? "V" : "v",
            cmdExhaust   ? "E" : "e",
            inputsChanged ? "*" : " ",

            ((i+1) % 2) == 0 ? "." : " "
    );
    lcd.setCursor(0, 1);
    lcd.print(printBuffer);
    rtos::ThisThread::sleep_for(1000);
  }
}

/* ISR - Any homekit switch change */
void handleInputChange()
{
  inputsChanged = 1; 
}

void reReadInputs() 
{
  cmdCool    = digitalRead(CMD_TEMP_COOL) == HIGH;    // Active High
  cmdSilent    = digitalRead(CMD_SILENT) == HIGH;    // Active High
  cmdVentilate = digitalRead(CMD_CO2_HIGH) == HIGH;  // Active High
  cmdExhaust = digitalRead(CMD_EXHAUST) == HIGH;   // Active High  
}

int motorStatesDirty()
{
  return 
           intakeOn     != intakeOnPrev
        || exhaustOn    != exhaustOnPrev
        || bypassOn     != bypassOnPrev
        || coreAssistOn != coreAssistOnPrev;
}

/* Computes the states the motors should be in and updates them. Runs every 60 seconds unless inputsChanged is toggled. */
void motorThreadImpl() 
{
  int delayMotorStateChangeSeconds = 0;
  for (;;) 
  {
    if (inputsChanged)
    {
      // Attempt to do a lock free synchronization to get a pseudo-consistent result. 
      // If inputsChanged is toggled back on during the calculation, re-do it. 
      do 
      {
        inputsChanged = 0; 
        reReadInputs(); 
        recomputeMotorStates();
        setPWM(pwm[0], INTAKE_PWM, PWM_FREQUENCY, (float)intakePWM);
        setPWM(pwm[1], EXHAUST_PWM, PWM_FREQUENCY, (float)exhaustPWM);
        setPWM(pwm[2], CORE_ASSIST_PWM, PWM_FREQUENCY, (float)coreAssistPWM);
        setPWM(pwm[3], BYPASS_PWM, PWM_FREQUENCY, (float)bypassPWM);
      } 
      while (inputsChanged); 
    }

    if (motorStatesDirty())
    {
      if (delayMotorStateChangeSeconds <= 0) 
      {
        updateMotorStates();
        delayMotorStateChangeSeconds = 15; 
      } 
    }

    rtos::ThisThread::sleep_for(1000);

    if (delayMotorStateChangeSeconds > 0)
    {
      delayMotorStateChangeSeconds--;
    } 
  }
}

/* Reads temperature sensors */
void temperatureThreadImpl()
{
  for(;;)
  {
    int nSensors = countSensors();
    int* address = (int*)malloc(nSensors * sizeof(int));
    float* tempC = (float*)malloc(nSensors * sizeof(float));
    nSensors = getSensorData(address, tempC, nSensors);
    temperatureLock.acquire(); 
    updateTemperature(INTAKE_INLET_TEMP_ADDR, address, tempC, nSensors, &intakeInletTempC);
    updateTemperature(INTAKE_OUTLET_TEMP_ADDR, address, tempC, nSensors, &intakeOutletTempC);
    updateTemperature(EXHAUST_INLET_TEMP_ADDR, address, tempC, nSensors, &exhaustInletTempC);
    updateTemperature(EXHAUST_OUTLET_TEMP_ADDR, address, tempC, nSensors, &exhaustOutletTempC);
    temperatureLock.release(); 
    free(address);
    free(tempC);
    rtos::ThisThread::sleep_for(5000);
  }
}

void displayThreadImpl() 
{
  for (;;) {
    updateDisplay();
  }
}

bool i2CAddrTest(uint8_t addr) {
  Wire.begin();
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}