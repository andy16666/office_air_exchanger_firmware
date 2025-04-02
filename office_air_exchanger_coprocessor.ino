#include "MBED_RP2040_PWM.h"
#include <LiquidCrystal_I2C.h>
#include <DS18B20.h>
#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <math.h>
#include <float.h>

#define INVALID_TEMP FLT_MIN

#define STR_(X) #X
#define STR(X) STR_(X)

#define LCD_ADDRESS 0x27
#define LCD_COLS    16
#define LCD_ROWS     2

#define PWM_FREQUENCY      25000
#define PWM_MIN_DUTY_CYCLE    10.0 
#define TEMP_SENSOR_PIN 2

#define INTAKE_INLET_TEMP_ADDR 234
#define INTAKE_OUTLET_TEMP_ADDR 119
#define EXHAUST_INLET_TEMP_ADDR 166
#define EXHAUST_OUTLET_TEMP_ADDR 41

#define NUM_BLOWER_PINS 4
#define NUM_PWM_PINS 4
#define INTAKE_IDX 0
#define EXHAUST_IDX 1
#define BYPASS_IDX 2
#define CA_IDX 3

#define CMD_COOL_IDX      0
#define CMD_SILENT_IDX    1
#define CMD_VENTILATE_IDX 2
#define CMD_EXHAUST_IDX   3
#define NUM_CMD_PINS      4

#define INTAKE_INLET_IDX   0
#define INTAKE_OUTLET_IDX  1
#define EXHAUST_INLET_IDX  2
#define EXHAUST_OUTLET_IDX 3
#define NUM_SENSORS        4

// Commands the system to attempt to lower the room temperature 
#define CMD_COOL      6 // GPIO 25 on HB server 
// Request lowest possible sound pressure levels
#define CMD_SILENT    7 // GPIO 23 on HB server 
// Request air exchange
#define CMD_VENTILATE 8 // GPIO 24 on HB server 
// Request a negative pressure condition in the room (preventing air from leaving, 
// for example when a door is opened to another part of the house)
#define CMD_EXHAUST   9 // GPIO 22 on HB server 

// Output Pins
// Puller on the outlet of the core section
#define INTAKE_BLOWER_ON         21
// Pusher on the inlet of the exhaust side 
#define EXHAUST_BLOWER_ON        20
// Puller near the fresh air intake, directly into the room 
#define BYPASS_BLOWER_ON         19
// Diverts air from the outlet to the inlet of the fresh air side, effectively increasing heat transfer from the core. 
#define CORE_ASSIST_BLOWER_ON    18


#define INTAKE_PWM      10
#define EXHAUST_PWM     11
#define BYPASS_PWM      12
#define CORE_ASSIST_PWM 13

DS18B20 ds(TEMP_SENSOR_PIN);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

const pin_size_t CMD_PINS[(NUM_CMD_PINS)] = {CMD_COOL, CMD_SILENT, CMD_VENTILATE, CMD_EXHAUST}; 
int   COMMANDS[(NUM_CMD_PINS)] = {0, 0, 0, 0};

const pin_size_t BLOWER_PINS[(NUM_BLOWER_PINS)]  = {INTAKE_BLOWER_ON, EXHAUST_BLOWER_ON, BYPASS_BLOWER_ON, CORE_ASSIST_BLOWER_ON}; 
const char*      BLOWER_NAMES[(NUM_BLOWER_PINS)] = {"Intake", "Exhaust", "Bypass", "Core Assist"}; 
// Intended motor states
int BLOWER_COMMANDS[(NUM_BLOWER_PINS)] = {0, 0, 0, 0};
// Current motor states
int   BLOWER_STATES[(NUM_BLOWER_PINS)] = {0, 0, 0, 0};

const pin_size_t PWM_PINS[(NUM_PWM_PINS)] = {INTAKE_PWM, EXHAUST_PWM, BYPASS_PWM, CORE_ASSIST_PWM}; 
const char*      PWM_NAMES[(NUM_BLOWER_PINS)] = {"Intake PWM", "Exhaust PWM", "Bypass PWM", "Core Assist PWM"}; 
float PWM_COMMANDS[(NUM_PWM_PINS)]     = {0, 0, 0, 0}; 

const pin_size_t TEMP_SENSOR_ADDRESSES[(NUM_SENSORS)] = {INTAKE_INLET_TEMP_ADDR, INTAKE_OUTLET_TEMP_ADDR, EXHAUST_INLET_TEMP_ADDR, EXHAUST_OUTLET_TEMP_ADDR};
float TEMPERATURES_C[(NUM_SENSORS)]    = {INVALID_TEMP, INVALID_TEMP, INVALID_TEMP, INVALID_TEMP};
const char*      SENSOR_NAMES[(NUM_SENSORS)] = {"Intake Inlet", "Intake Outlet", "Exhaust Inlet", "Exhaust Outlet"}; 

// Target air strem temp for cooling 
const float COOL_TEMP_C            = 16; 
const float NORM_TEMP_C            = 20; 
const float HOT_TEMP_C             = 35; 

// How closely do we try to control temperatures around the core. 
const float CORE_TEMP_TOLERANCE_C  = 2.0;

// Toggled to 1 whenever any input changes, 
// either via an ISR, or via temp sensor polling. 
volatile int inputsChanged = 0; 

volatile int temperaturesChanged = 0;

volatile int rasterDirty = 0; 

char** lcdRaster;

mbed::PwmOut* pwm[]    = { NULL, NULL, NULL, NULL };
using namespace rtos; 
Semaphore temperatureLock(1); 
Semaphore rasterLock(1); 
Thread motorThread; 
Thread displayThread;
Thread temperatureThread; 
Thread lcdRefreshThread;

void setup() {
  initLcd(); 

  printLn("Init Input Pins");

  // Commands from homebridge/homekit automations
  for (int i = 0; i < NUM_CMD_PINS; i++)
  {
    pinMode(CMD_PINS[i], INPUT_PULLDOWN);
  }

  printLn("Init Blower Pins"); 

  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    pinMode(BLOWER_PINS[i], OUTPUT);
    digitalWrite(BLOWER_PINS[i], HIGH);
  }

  printLn("Init PWM Pins"); 

  for (int i = 0; i < NUM_PWM_PINS; i++)
  {
    pinMode(PWM_PINS[i], OUTPUT); 
  }

  temperatureThread.start(mbed::callback(temperatureThreadImpl));

  waitForTemperatures(); 
  delay(1000); 

  printSensorAddresses(); 
  delay(5000); 

  printLn("Testing Blowers");

  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    printfLn("%s", BLOWER_NAMES[i]);
    
    digitalWrite(BLOWER_PINS[i], LOW);
    delay(1000);
    digitalWrite(BLOWER_PINS[i], HIGH);
    delay(2000);
  }

  printLn("Testing PWMs");

  for (int i = 0; i < NUM_PWM_PINS; i++)
  {
    printfLn("%s", PWM_NAMES[i]);
    
    setPWM(pwm[i], PWM_PINS[i], PWM_FREQUENCY, 100.0);
    delay(1000);
    setPWM(pwm[i], PWM_PINS[i], PWM_FREQUENCY, PWM_MIN_DUTY_CYCLE);
    delay(1000);
    setPWM(pwm[i], PWM_PINS[i], PWM_FREQUENCY, 0.0);
  }

  printLn("Installing ISRs");

  for (int i = 0; i < NUM_CMD_PINS; i++)
  {
    // Inputs from HK are interrupt driven to make the system instantly respond to changes. 
    attachInterrupt(digitalPinToInterrupt(CMD_PINS[i]), handleInputChange, CHANGE);
  }

  printLn("Start display thread"); 
  displayThread.start(mbed::callback(displayThreadImpl));
  printLn("Start motor control"); 
  motorThread.start(mbed::callback(motorThreadImpl));
}

void loop() { }

int waitForTemperatures()
{
  printLn("Wait for sensors"); 
  printLn(" "); 
  char padding[] = "============================";
  int ready = 0; 
  for(int retry = 0; retry < 12 && !ready; retry++) 
  {
    delay(5000); 
    
    ready = 1; 
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (TEMPERATURES_C[i] == INVALID_TEMP)
        {
          reprintfLn("... %s", SENSOR_NAMES[i]); 
          ready = 0; 
          break;
        } 
        else 
        {
          printfLn("[OK] %s", SENSOR_NAMES[i]); 
        }
    }
  }

  if (ready)
  {
    reprintfLn("Sensors ready!");
  }
  else 
  {
    reprintfLn("Sensors not ready");
  }

  return ready; 
}

void printfLn(const char *format, ...)
{
  char formatBuffer[1024];   

  va_list arg;
  va_start (arg, format);
  vsprintf (formatBuffer, format, arg);
  va_end (arg);

  printLn(formatBuffer); 
}

void reprintfLn(const char *format, ...)
{
  char formatBuffer[1024];   

  va_list arg;
  va_start (arg, format);
  vsprintf (formatBuffer, format, arg);
  va_end (arg);

  reprintLn(formatBuffer); 
}

void scrollRaster()
{
  for (int i = 0; i < LCD_ROWS; i++)
  {
      if (!i) 
        free(lcdRaster[i]); 
      else 
        lcdRaster[i-1] = lcdRaster[i]; 
  }
  lcdRaster[LCD_ROWS - 1] = (char *)malloc(sizeof(char) * (LCD_COLS + 1));
}

void printLn(char * formatBuffer)
{
  while (strlen(formatBuffer) > 0)
  {
    rasterLock.acquire(); 
    scrollRaster(); 
    if (strlen(formatBuffer) >= LCD_COLS)
    {
      strncpy(lcdRaster[LCD_ROWS - 1], formatBuffer, LCD_COLS);
      formatBuffer += LCD_COLS; 
    }
    else 
    {
      sprintf(lcdRaster[LCD_ROWS - 1], "%-*.*s", LCD_COLS, LCD_COLS, formatBuffer);
      formatBuffer += strlen(formatBuffer); 
    }
    rasterDirty = 1; 
    rasterLock.release(); 
  }  
}

void reprintLn(char * formatBuffer)
{
  rasterLock.acquire(); 
  lcdRaster[LCD_ROWS - 1][0] = 0; 
  sprintf(lcdRaster[LCD_ROWS - 1], "%-*.*s", LCD_COLS, LCD_COLS, formatBuffer);
  rasterDirty = 1; 
  rasterLock.release(); 
}

void lcdRefreshThreadImpl()
{
  for(;;)
  {
    rasterLock.acquire(); 
    rasterDirty = 0; 
    for (int row = 0; row < LCD_ROWS; row++)
    {
      lcd.setCursor(0, row); 
      lcd.print(lcdRaster[row]); 
    }
    rasterLock.release(); 
    while (!rasterDirty)
    {
      rtos::ThisThread::sleep_for(1000);
    }
  }
}

void initLcd()
{
  lcdRaster = (char **)malloc(sizeof(char*) * LCD_ROWS); 
  for (int i = 0; i < LCD_ROWS; i++)
  {
    lcdRaster[i] = (char *)malloc(sizeof(char) * (LCD_COLS + 1));
    sprintf(lcdRaster[i], "%-*ls", LCD_COLS, "...");
  }

  if (!i2CAddrTest(LCD_ADDRESS)) 
  {
    lcd = LiquidCrystal_I2C(0x3F, LCD_COLS, LCD_ROWS);
  }
  lcd.init();           // LCD driver initialization
  lcd.backlight();  
  lcdRefreshThread.start(mbed::callback(lcdRefreshThreadImpl));
}

int countSensors() 
{
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

float getTempC(int requestedAddress, int* address, float* tempC, int nSensors)
{
  int i;
  for (i = 0; i < nSensors; i++) 
  {
    if (requestedAddress == address[i])
    {
      float tmpTempC = tempC[i];

      if (tmpTempC == tmpTempC)
      {
        return tmpTempC; 
      }
    }
  }

  return INVALID_TEMP;
}

void updateTemperature(int requestedAddress, int* address, float* tempC, int nSensors, volatile float* currentTempC)
{
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

void recomputeMotorStates() 
{
  // Are we heating or cooling? 
  float targetTempC = COMMANDS[CMD_COOL_IDX] ? COOL_TEMP_C : NORM_TEMP_C; 
  float maxSpeed = COMMANDS[CMD_SILENT_IDX] ? 80 : 100; 

  temperatureLock.acquire(); 

  // Intake and outlet idle at PWM_MIN_DUTY_CYCLE. The outlet 
  // of the exhaust side is a very good indicator of the core 
  // temperature due to the placement of the sensor. 
  float coreTempC = TEMPERATURES_C[EXHAUST_OUTLET_IDX];

  // The exhaust idle flow gives us a good sample of room temperature
  // air, albeit a bit on the warm side.
  float estimatedRoomTempC = TEMPERATURES_C[EXHAUST_OUTLET_IDX]; 
  
    // Try not to melt the core
  int coreHot = TEMPERATURES_C[EXHAUST_OUTLET_IDX] > HOT_TEMP_C || TEMPERATURES_C[EXHAUST_INLET_IDX] > HOT_TEMP_C;

  float intakeOutletToTargetTempC      = computeGradientC(TEMPERATURES_C[INTAKE_OUTLET_IDX], targetTempC, 0.1);  
  float coreToIntakeOutletTempC        = computeGradientC(coreTempC,                         TEMPERATURES_C[INTAKE_OUTLET_IDX], 0.1);
  float intakeInletToIntakeOutletTempC = computeGradientC(TEMPERATURES_C[INTAKE_INLET_IDX],  TEMPERATURES_C[INTAKE_OUTLET_IDX], 0.1);
  float intakeInletToTargetTempC       = computeGradientC(TEMPERATURES_C[INTAKE_INLET_IDX],  targetTempC, 0.1);

  float exhaustToCoreTempC        = computeGradientC(TEMPERATURES_C[EXHAUST_INLET_IDX], coreTempC, 0.1);
  float estimatedRoomTempCToTargetTempC = computeGradientC(estimatedRoomTempC, targetTempC, 0.1);
  
  // Try to position the core between the target temp and the exhaust temp. 
  float exhaustTempControlAmountC = fmin(intakeOutletToTargetTempC, exhaustToCoreTempC);

  float intakeTempControlAmountC   = fmin(estimatedRoomTempCToTargetTempC, intakeOutletToTargetTempC); 

  float bypassTempControlAmountC   = fmin(intakeInletToTargetTempC, intakeOutletToTargetTempC); 

  float coreAssistTempControlAmountC = fmin(coreToIntakeOutletTempC, intakeOutletToTargetTempC);

  // Is the intake air useful for controlling temperature
  int intakeEnableTempControl = estimatedRoomTempC > targetTempC  
          ? TEMPERATURES_C[INTAKE_OUTLET_IDX] < estimatedRoomTempC 
          : TEMPERATURES_C[INTAKE_OUTLET_IDX] > estimatedRoomTempC;

  // Is the exhaust side useful for controlling temperature 
  int exhaustEnableTempControl = TEMPERATURES_C[EXHAUST_INLET_IDX] > TEMPERATURES_C[INTAKE_OUTLET_IDX] 
          ? TEMPERATURES_C[INTAKE_OUTLET_IDX] < targetTempC 
          : TEMPERATURES_C[INTAKE_OUTLET_IDX] > targetTempC;

  int coreAssistEnable = TEMPERATURES_C[INTAKE_OUTLET_IDX] > targetTempC
          ? coreTempC < TEMPERATURES_C[INTAKE_OUTLET_IDX]
          : coreTempC > TEMPERATURES_C[INTAKE_OUTLET_IDX];

  int bypassEnable = TEMPERATURES_C[INTAKE_OUTLET_IDX] > targetTempC
          ? TEMPERATURES_C[INTAKE_INLET_IDX] < TEMPERATURES_C[INTAKE_OUTLET_IDX] 
          : TEMPERATURES_C[INTAKE_INLET_IDX] > TEMPERATURES_C[INTAKE_OUTLET_IDX];

  temperatureLock.release(); 

  PWM_COMMANDS[INTAKE_IDX]     = !COMMANDS[CMD_EXHAUST_IDX]
      ? 
          intakeEnableTempControl * extrapolatePWM(intakeTempControlAmountC, 3, 0, PWM_MIN_DUTY_CYCLE, maxSpeed) 
        + COMMANDS[CMD_VENTILATE_IDX] * maxSpeed
        + COMMANDS[CMD_COOL_IDX] * maxSpeed
        + coreHot * maxSpeed
      : 0
      ;

  PWM_COMMANDS[EXHAUST_IDX]     = COMMANDS[CMD_EXHAUST_IDX] || COMMANDS[CMD_VENTILATE_IDX] || exhaustEnableTempControl || coreHot
      ?
        exhaustEnableTempControl * extrapolatePWM(exhaustTempControlAmountC, 3, 0, PWM_MIN_DUTY_CYCLE, maxSpeed) 
        + COMMANDS[CMD_EXHAUST_IDX] * 100.0
        + coreHot * maxSpeed
      : 0
      ;

  PWM_COMMANDS[CA_IDX]          = coreAssistEnable        ? extrapolatePWM(coreAssistTempControlAmountC,   CORE_TEMP_TOLERANCE_C, 0.1, PWM_MIN_DUTY_CYCLE, maxSpeed) : 0.0;
  PWM_COMMANDS[BYPASS_IDX]      = bypassEnable            ? extrapolatePWM(bypassTempControlAmountC,       CORE_TEMP_TOLERANCE_C, 0.1, PWM_MIN_DUTY_CYCLE, maxSpeed) : 0.0;
  
  // Always idle the intake and outlet at PWM_MIN_DUTY_CYCLE so we have an accurate temp sample. 
  PWM_COMMANDS[EXHAUST_IDX]     = clamp(PWM_COMMANDS[EXHAUST_IDX    ], PWM_MIN_DUTY_CYCLE, maxSpeed); 
  PWM_COMMANDS[INTAKE_IDX ]     = clamp(PWM_COMMANDS[INTAKE_IDX     ], PWM_MIN_DUTY_CYCLE, maxSpeed);
  PWM_COMMANDS[BYPASS_IDX ]     = clamp(PWM_COMMANDS[BYPASS_IDX     ], 0,                  maxSpeed);
  PWM_COMMANDS[CA_IDX]          = clamp(PWM_COMMANDS[CA_IDX],          0,                  maxSpeed);

  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    BLOWER_COMMANDS[i] = PWM_COMMANDS[i] >= 100.0;
  }
  
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
}

void updateMotorStates() 
{
  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    digitalWrite(BLOWER_PINS[i],      BLOWER_COMMANDS[i]     ? LOW : HIGH); 
    BLOWER_STATES[i] = BLOWER_COMMANDS[i]; 

  }
}

void updateDisplay() 
{
  for (int i = 0; i < 8; i++)
  {
    if (i < 4)
    {
      printfLn(
            "I %5.1f <= %5.1f",
            TEMPERATURES_C[INTAKE_OUTLET_IDX],
            TEMPERATURES_C[INTAKE_INLET_IDX]);
    }
    else 
    {
      printfLn(
              "E %5.1f => %5.1f",
              TEMPERATURES_C[EXHAUST_INLET_IDX],
              TEMPERATURES_C[EXHAUST_OUTLET_IDX]);
    }

    printfLn(
            "%1s%1d%1s%1d%1s%1d%1s%1d%1s %1s%1s%1s%1s%1s ",
            BLOWER_COMMANDS[INTAKE_IDX]     ? "I" : "i",
            (int)(PWM_COMMANDS[INTAKE_IDX]    /11.1),
            BLOWER_COMMANDS[EXHAUST_IDX]    ? "E" : "e",
            (int)(PWM_COMMANDS[EXHAUST_IDX]   /11.1),
            BLOWER_COMMANDS[BYPASS_IDX]    ? "B" : "b",
            (int)(PWM_COMMANDS[BYPASS_IDX]    /11.1),
            BLOWER_COMMANDS[CA_IDX]        ? "C" : "c",
            (int)(PWM_COMMANDS[CA_IDX]/11.1),
            motorStatesDirty() ? "*" : " ",

            COMMANDS[CMD_COOL_IDX]      ? "C" : "c",
            COMMANDS[CMD_SILENT_IDX]      ? "S" : "s",
            COMMANDS[CMD_VENTILATE_IDX]   ? "V" : "v",
            COMMANDS[CMD_EXHAUST_IDX]   ? "E" : "e",
            inputsChanged ? "*" : " ",

            ((i+1) % 2) == 0 ? "." : " "
    );
  
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
  for (int i = 0; i < NUM_CMD_PINS; i++)
  {
    COMMANDS[i]      = digitalRead(CMD_PINS[i]) == HIGH;    // Active High
  }
}

int motorStatesDirty()
{  
  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    if (BLOWER_COMMANDS[i] != BLOWER_STATES[i])
      return 1; 
  }

  return 0; 
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

        for (int i = 0; i < NUM_PWM_PINS; i++)
        {
          setPWM(pwm[i], PWM_PINS[i], PWM_FREQUENCY, PWM_COMMANDS[i]);
        }
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
void printSensorAddresses()
{
  int nSensors = countSensors();
  if (nSensors == 0)
  {
    printLn("No sensors detected"); 
    return; 
  }

  int* address = (int*)malloc(nSensors * sizeof(int));
  float* tempC = (float*)malloc(nSensors * sizeof(float));
  char buffer[1024]; 
  temperatureLock.acquire(); 
  nSensors = getSensorData(address, tempC, nSensors);
  buffer[0] = 0; 
  for (int i = 0; i < nSensors; i++)
  {
      sprintf(buffer + strlen(buffer), "%d ", address[i]); 
  }
  printLn(buffer); 
  temperatureLock.release(); 
  free(address);
  free(tempC);
  
}

/* Reads temperature sensors */
void temperatureThreadImpl()
{
  for(;;)
  {
    int nSensors = countSensors();
    int* address = (int*)malloc(nSensors * sizeof(int));
    float* tempC = (float*)malloc(nSensors * sizeof(float));
    temperatureLock.acquire(); 
    nSensors = getSensorData(address, tempC, nSensors);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      updateTemperature(TEMP_SENSOR_ADDRESSES[i], address, tempC, nSensors, &(TEMPERATURES_C[i]));
    }
    
    temperatureLock.release(); 
    free(address);
    free(tempC);
    rtos::ThisThread::sleep_for(1000);
  }
}

void displayThreadImpl() 
{
  for (;;) 
  {
    updateDisplay();
  }
}

bool i2CAddrTest(uint8_t addr) 
{
  Wire.begin();
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) 
  {
    return true;
  }
  return false;
}