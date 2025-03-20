
#include <LiquidCrystal_I2C.h>
#include <DS18B20.h>
#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <math.h>

using namespace rtos; 

Semaphore motorLock(1); 
Semaphore inputLock(1); 
Semaphore temperatureLock(1); 

 
Thread t1; 
Thread t2;
Thread t3;  

#define INVALID_TEMP NAN

// Input Pins
#define TEMP_SENSOR_DATA 2
#define CMD_TEMP_COOL 6
#define CMD_TEMP_HEAT 7
#define CMD_CO2_HIGH 8
#define CMD_EXHAUST 9

// Sensor Addresses
#define INTAKE_INLET_TEMP_ADDR 234
#define EXHAUST_OUTLET_TEMP_ADDR 41
#define INTAKE_OUTLET_TEMP_ADDR 119
#define EXHAUST_INLET_TEMP_ADDR 166

// Output Pins
#define INTAKE_BLOWER_ON 21
#define EXHAUST_BLOWER_ON 20
#define BYPASS_BLOWER_ON 19
#define CORE_ASSIST_ON 18

#define CORE_ASSIST_PWM 16

DS18B20 ds(TEMP_SENSOR_DATA);

LiquidCrystal_I2C lcd(0x27, 16, 2);

int relayDebounce;
volatile int intakeOn;
volatile int exhaustOn;
volatile int coreAssistOn;
volatile int bypassOn;
volatile int cmdCool    = 0; 
volatile int cmdHeat    = 0; 
volatile int cmdCo2High = 0; 
volatile int cmdExhaust = 0; 
volatile int intakeOnPrev = 0;
volatile int exhaustOnPrev = 0;
volatile int bypassOnPrev = 0;
volatile int coreAssistOnPrev = 0;
volatile float intakeInletTempC = INVALID_TEMP; 
volatile float intakeOutletTempC = INVALID_TEMP; 
volatile float exhaustInletTempC = INVALID_TEMP; 
volatile float exhaustOutletTempC = INVALID_TEMP; 

volatile int inputsChanged = 0; 

void setup() {
  intakeOn = 0;
  exhaustOn = 0;
  coreAssistOn = 0;
  bypassOn = 0;
  relayDebounce = 0;

  pinMode(CMD_TEMP_COOL, INPUT_PULLDOWN);
  pinMode(CMD_TEMP_HEAT, INPUT_PULLDOWN);
  pinMode(CMD_CO2_HIGH, INPUT_PULLDOWN);
  pinMode(CMD_EXHAUST, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(CMD_TEMP_COOL), handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_TEMP_HEAT), handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_CO2_HIGH),  handleInputChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CMD_EXHAUST), handleInputChange, CHANGE);

  pinMode(INTAKE_BLOWER_ON, OUTPUT);
  pinMode(EXHAUST_BLOWER_ON, OUTPUT);
  pinMode(BYPASS_BLOWER_ON, OUTPUT);
  pinMode(CORE_ASSIST_ON, OUTPUT);
  pinMode(CORE_ASSIST_PWM, OUTPUT);

  Serial.begin(9600);
  Serial.print("Devices: ");
  Serial.println(ds.getNumberOfDevices());
  Serial.println();

  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();           // LCD driver initialization
  lcd.backlight();      // Open the backlight
  lcd.setCursor(0, 0);  // Move the cursor to row 0, column 0
  lcd.setCursor(0, 0);
  lcd.print("Starting Up");
  digitalWrite(INTAKE_BLOWER_ON, LOW);
  digitalWrite(EXHAUST_BLOWER_ON, LOW);
  digitalWrite(BYPASS_BLOWER_ON, LOW);
  digitalWrite(CORE_ASSIST_ON, LOW);
  digitalWrite(CORE_ASSIST_PWM, LOW);
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up.");
  delay(1000);
  digitalWrite(INTAKE_BLOWER_ON, HIGH);
  digitalWrite(EXHAUST_BLOWER_ON, HIGH);
  digitalWrite(BYPASS_BLOWER_ON, HIGH);
  digitalWrite(CORE_ASSIST_ON, HIGH);
  digitalWrite(CORE_ASSIST_PWM, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up..");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Starting Up...");
  delay(1000);

  t3.start(mbed::callback(temperatureThread));
  reReadInputs();
  t2.start(mbed::callback(displayThread));
  t1.start(mbed::callback(motorThread));
}


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
       *currentTempC != newTempC 
    && newTempC != INVALID_TEMP 
    && newTempC == newTempC     // NAN check
  )
  {
    *currentTempC = newTempC; 
    inputsChanged = 1; 
  }
}

void recomputeMotorStates() {
  temperatureLock.acquire(); 
  const float targetRoomTemp = 19.0; 
  const float coolTemp = targetRoomTemp - 1.0; 
  const float heatTemp = targetRoomTemp + 2.0; 

  int tempControlEnable = cmdHeat || cmdCool; 

  float exhaustCoreTemp = (exhaustOutletTempC + exhaustInletTempC)
                          /2.0;
  float intakeCoreTemp  = (intakeInletTempC + intakeOutletTempC)
                          /2.0;
  float coreTemp = (exhaustCoreTemp + intakeCoreTemp)
                          /2.0;

  int coolAvailableIntake = intakeOnPrev ? intakeOutletTempC < coolTemp : coreTemp < coolTemp;
  int heatAvailableIntake = intakeOnPrev ? intakeOutletTempC > heatTemp : coreTemp < heatTemp;
  int intakeEnableTempControl = (cmdCool && coolAvailableIntake) || (cmdHeat && heatAvailableIntake); 

  int coolAvailableExhaust = exhaustOnPrev ? exhaustCoreTemp < coreTemp : exhaustInletTempC < coreTemp; 
  int heatAvailableExhaust = exhaustOnPrev ? exhaustCoreTemp > coreTemp : exhaustInletTempC > coreTemp; 
  int exhaustEnableTempControl = (cmdCool && coolAvailableExhaust) || (cmdHeat && heatAvailableExhaust); 
  int exhaustHot = exhaustOutletTempC > 35;

  int coreAssistCoolAvailable = intakeOnPrev ? coreTemp < intakeOutletTempC : 0;
  int coreAssistHeatAvailable = intakeOnPrev ? coreTemp > intakeOutletTempC : 0;
  int coreAssistAvailable = (cmdCool && coreAssistCoolAvailable) || (cmdHeat && coreAssistHeatAvailable);
  int bypassAvailable = (cmdCool && coolAvailableIntake) || (cmdHeat && heatAvailableIntake); 

  temperatureLock.release(); 

  intakeOn =
    !cmdExhaust
    && (
           cmdCo2High
        || intakeEnableTempControl
    )
    ;

  exhaustOn =
    cmdExhaust
    || (cmdCo2High && (exhaustEnableTempControl || !tempControlEnable))
    || exhaustHot
    || exhaustEnableTempControl
    ;

  coreAssistOn =
       intakeOn
    && coreAssistAvailable
    ;

  bypassOn =
       !cmdExhaust
    && bypassAvailable
    ;
}

void updateMotorStates() {
  digitalWrite(INTAKE_BLOWER_ON, intakeOn ? LOW : HIGH);    // Active Low
  digitalWrite(EXHAUST_BLOWER_ON, exhaustOn ? LOW : HIGH);  // Active Low
  digitalWrite(BYPASS_BLOWER_ON, bypassOn ? LOW : HIGH);    // Active Low
  digitalWrite(CORE_ASSIST_ON, coreAssistOn ? LOW : HIGH);  // Active Low
  intakeOnPrev = digitalRead(INTAKE_BLOWER_ON) == LOW;    // Active Low
  exhaustOnPrev = digitalRead(EXHAUST_BLOWER_ON) == LOW;  // Active Low
  bypassOnPrev = digitalRead(BYPASS_BLOWER_ON) == LOW;    // Active Low
  coreAssistOnPrev = digitalRead(CORE_ASSIST_ON) == LOW;  // Active Low
}

void updateDisplay() {
  char* buffer = (char*)malloc(1024 * sizeof(char));
  
  sprintf(buffer, "BIECaCHcE       ");
  lcd.setCursor(0, 0);
  lcd.print(buffer);
  sprintf(buffer,
          "%1d%1d%1d%1d %1d%1d%1d%1d      ",
          bypassOn ? 1 : 0,
          intakeOn ? 1 : 0,
          exhaustOn ? 1 : 0,
          coreAssistOn ? 1 : 0,
          cmdCool ? 1 : 0,
          cmdHeat ? 1 : 0,
          cmdCo2High ? 1 : 0,
          cmdExhaust ? 1 : 0,
          exhaustInletTempC,
          exhaustOutletTempC);
  lcd.setCursor(0, 1);
  lcd.print(buffer);
  rtos::ThisThread::sleep_for(5000);
  temperatureLock.acquire(); 
  sprintf(buffer,
          "I%2.3f>%2.3f  ",
          intakeInletTempC,
          intakeOutletTempC);
  temperatureLock.release(); 
  lcd.setCursor(0, 0);
  lcd.print(buffer);

  temperatureLock.acquire(); 
  sprintf(buffer,
          "E%2.3f>%2.3f  ",
          exhaustInletTempC,
          exhaustOutletTempC);
  temperatureLock.release(); 
  lcd.setCursor(0, 1);
  lcd.print(buffer);

  free(buffer);
  
  digitalWrite(CORE_ASSIST_PWM, coreAssistOn ? LOW : HIGH);
}

void temperatureThread()
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

void handleInputChange()
{
  inputsChanged = 1; 
}

void reReadInputs() 
{
  cmdCool = digitalRead(CMD_TEMP_COOL) == HIGH;    // Active High
  cmdHeat = digitalRead(CMD_TEMP_HEAT) == HIGH;    // Active High
  cmdCo2High = digitalRead(CMD_CO2_HIGH) == HIGH;  // Active High
  cmdExhaust = digitalRead(CMD_EXHAUST) == HIGH;   // Active High  
}

void readMotorStates()
{
  
}

void loop() {

}

void motorThread() {
  for (;;) {
    int motorStatesChanged;
    if (inputsChanged)
    {
      do 
      {
        inputsChanged = 0; 
        reReadInputs(); 
        recomputeMotorStates();
      } 
      while (inputsChanged);

      motorStatesChanged =
           intakeOn != intakeOnPrev
        || exhaustOn != exhaustOnPrev
        || bypassOn != bypassOnPrev
        || coreAssistOn != coreAssistOnPrev;
    }
    else 
    {
      motorStatesChanged = 0; 
    } 

    if (motorStatesChanged) 
    {
      updateMotorStates();
      for (int i = 0; i < 60 && !inputsChanged; i++)
      {
        rtos::ThisThread::sleep_for(1000);
      }
    } 
    else
    {
      rtos::ThisThread::sleep_for(1000);
    }
  }
}

void displayThread() {
  for (;;) {
    updateDisplay();
    rtos::ThisThread::sleep_for(5000);
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