
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

#define INVALID_TEMP NAN

// Input Pins
#define TEMP_SENSOR_DATA 2
// Commands the system to attempt to lower the room temperature 
#define CMD_TEMP_COOL 6
// Commands the system to attempt to increase room temperature
#define CMD_TEMP_HEAT 7
// Request air exchange
#define CMD_CO2_HIGH 8
// Request a negative pressure condition in the room (preventing air from leaving, 
// for example when a door is opened to another part of the house)
#define CMD_EXHAUST 9

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

#define CORE_ASSIST_PWM 16

const float TARGET_TEMP_C = 19.0; // TODO: Make this an input from HK. 
// Target air strem temp for cooling 
const float COOL_TEMP_C = TARGET_TEMP_C - 2.0; 
// Target air stream temp for heating
const float HEAT_TEMP_C = TARGET_TEMP_C + 2.0; 

DS18B20 ds(TEMP_SENSOR_DATA);

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Intended motor states
volatile int intakeOn;
volatile int exhaustOn;
volatile int coreAssistOn;
volatile int bypassOn;

// Current motor states
volatile int intakeOnPrev = 0;
volatile int exhaustOnPrev = 0;
volatile int bypassOnPrev = 0;
volatile int coreAssistOnPrev = 0;

volatile int cmdCool    = 0; 
volatile int cmdHeat    = 0; 
volatile int cmdCo2High = 0; 
volatile int cmdExhaust = 0; 

volatile float intakeInletTempC = INVALID_TEMP; 
volatile float intakeOutletTempC = INVALID_TEMP; 
volatile float exhaustInletTempC = INVALID_TEMP; 
volatile float exhaustOutletTempC = INVALID_TEMP; 

// Toggled to 1 whenever any input changes
volatile int inputsChanged = 0; 

void setup() {
  intakeOn = 0;
  exhaustOn = 0;
  coreAssistOn = 0;
  bypassOn = 0;

  // Commands from homebridge/homekit automations
  pinMode(CMD_TEMP_COOL, INPUT_PULLDOWN);
  pinMode(CMD_TEMP_HEAT, INPUT_PULLDOWN);
  pinMode(CMD_CO2_HIGH, INPUT_PULLDOWN);
  pinMode(CMD_EXHAUST, INPUT_PULLDOWN);

  // Inputs from HK are interrupt driven to make the system instantly respond to changes. 
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

void recomputeMotorStates() {
  temperatureLock.acquire(); 
  // Are we heating or cooling? 
  int tempControlEnable = cmdHeat || cmdCool; 

  // Average temperature of the exhaust side of the core
  float exhaustCoreTempC = (exhaustOutletTempC + exhaustInletTempC)
                          /2.0;
  // Average temperature of the intake side of the core
  float intakeCoreTempC  = (intakeInletTempC + intakeOutletTempC)
                          /2.0;
  // Average core temperature
  float coreTempC = (exhaustCoreTempC + intakeCoreTempC)
                          /2.0;

  // Is cooling air available? If the intake has been off, approximate using core temp. 
  int coolAvailableIntake = intakeOnPrev ? intakeOutletTempC < COOL_TEMP_C : coreTempC < COOL_TEMP_C;
  // Is heating air available? ""
  int heatAvailableIntake = intakeOnPrev ? intakeOutletTempC > HEAT_TEMP_C : coreTempC < HEAT_TEMP_C;
  // Is the intake air useful for controlling temperature
  int intakeEnableTempControl = (cmdCool && coolAvailableIntake) || (cmdHeat && heatAvailableIntake); 

  // Is the exhaust side useful for controlling temperature 
  int coolAvailableExhaust = exhaustOnPrev ? exhaustCoreTempC < coreTempC : exhaustInletTempC < coreTempC; 
  int heatAvailableExhaust = exhaustOnPrev ? exhaustCoreTempC > coreTempC : exhaustInletTempC > coreTempC; 
  int exhaustEnableTempControl = (cmdCool && coolAvailableExhaust) || (cmdHeat && heatAvailableExhaust); 
  
  // Try not to melt the core
  int exhaustHot = exhaustOutletTempC > 40;

  // Is the core assit feature useful for either warming or cooling the core to assist in heating or cooling? 
  int coreAssistCoolAvailable = intakeOnPrev && exhaustOnPrev ? exhaustCoreTempC < intakeOutletTempC : 0;
  int coreAssistHeatAvailable = intakeOnPrev && exhaustOnPrev ? exhaustCoreTempC > intakeOutletTempC : 0;
  int coreAssistAvailable = (cmdCool && coreAssistCoolAvailable) || (cmdHeat && coreAssistHeatAvailable);

  // Bypass is available whenever the intake inlet is cool or warm enough. 
  int coolAvailableIntakeInlet = intakeInletTempC < COOL_TEMP_C; 
  int heatAvailableIntakeInlet = intakeInletTempC < HEAT_TEMP_C; 
  int bypassAvailable = (cmdCool && coolAvailableIntakeInlet) || (cmdHeat && heatAvailableIntakeInlet); 

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
    // Ignore CO2 high if we are controlling temperature and the exhaust is not helpful. 
    || (cmdCo2High && (exhaustEnableTempControl || !tempControlEnable))
    || exhaustHot
    || exhaustEnableTempControl
    ;

  // Never risk push air backward through the intake 
  coreAssistOn =
       intakeOn
    && coreAssistAvailable
    ;

  bypassOn =
       !cmdExhaust
    && bypassAvailable
    ;
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

/* ISR - Any homekit switch change */
void handleInputChange()
{
  inputsChanged = 1; 
}

void reReadInputs() 
{
  cmdCool    = digitalRead(CMD_TEMP_COOL) == HIGH;    // Active High
  cmdHeat    = digitalRead(CMD_TEMP_HEAT) == HIGH;    // Active High
  cmdCo2High = digitalRead(CMD_CO2_HIGH) == HIGH;  // Active High
  cmdExhaust = digitalRead(CMD_EXHAUST) == HIGH;   // Active High  
}



/* Computes the states the motors should be in and updates them. Runs every 60 seconds unless inputsChanged is toggled. */
void motorThreadImpl() 
{
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
      } 
      while (inputsChanged); 

      int motorStatesChanged =
           intakeOn != intakeOnPrev
        || exhaustOn != exhaustOnPrev
        || bypassOn != bypassOnPrev
        || coreAssistOn != coreAssistOnPrev;

      if (motorStatesChanged) 
      {
        updateMotorStates();
      } 
    }

    // Wait 60 seconds or until inputs change. 
    for (int i = 0; i < 60 && !inputsChanged; i++)
    {
      rtos::ThisThread::sleep_for(1000);
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