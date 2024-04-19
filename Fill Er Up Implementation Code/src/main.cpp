#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL6180X.h>
#include <TFT_eSPI.h>

// Order of Operations
// 1) Button #1 Pressed --> Activate Single Scan of the ToF (Complete Single Measurement Then Wait)
// 2) Button #2 Pressed --> Initiate Brew, Take in ToF Measurement, User Input Dial Percentage, Multiply to Use for Fill Height

// Libraries

// Function Declarations
void state0();
void state1();
float singleHeightMeasure();
float startLiquidPour(float cupHeightMeasured);
bool writeReg(uint8_t reg, const void *pBuf, size_t size);
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size);
void stopBrew();
void restartBrew();
bool monitorPump();
bool monitorBrewButton();
void brewSteadyState();

int ButtonPressExecute(void);
void DisplayBrewInProcess(void);
void DisplayBrewComplete(void);
int DialUIInteraction(void);
void DialUISetup();

// Multiplexor Address
#define TCA9548A_ADDRESS 0x70

// Pin Definitions

// Rotary Encoder
#define ENC_CLK 15
#define ENC_DT 16
#define ENC_SW 18

#define CURR_PERCENT 50

// Control for Keurig
#define kControl 1

// Button Pins
#define interruptPin0 35
#define interruptPin1 36

// LED Shows Liquid Filled
#define LiquidLED 4

// Cup Height LEDs
#define LED0 6
#define LED1 7
#define LED2 8
#define LED3 9
#define LED4 10
#define LED5 11
#define LED6 12
#define LED7 13
#define LIMPOS 37
#define MOS 40
#define BREW 39
#define RES 41
#define POW 42
#define PUMP 5

// Define LED pins Array
const int LED_PINS[] = {LED0, LED1, LED2, LED3, LED4, LED5, LED6};

// ToF Sensor Array
VL6180X sensors_vl6180[5];

// SEN0590 Definitions
#define address 0x74
uint8_t buf[2] = {0};
uint8_t dat = 0xB0;

// Button Definitions
unsigned long lastDebounceTime0 = 0;
unsigned long debounceDelay0 = 750;
unsigned long lastDebounceTime1 = 0;
unsigned long debounceDelay1 = 750;

void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(25);
}

// ***********SETUP*************

float cupHeightMeasured;
int numLiquid = 0;
int numCupMeasure = 0;
float heightOutput = 0.0;
bool noButton = true;
bool firstPress = true;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

int counter = CURR_PERCENT;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;

void setup()
{
  DialUISetup();
  delay(500);
  Serial.begin(115200);
  Serial.println("Entered Setup");

  int SCL_Pin = 14;
  int SDA_Pin = 21;
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(100000L); // L suffix denotes long type

  // Pinmode Definitions
  pinMode(interruptPin0, INPUT_PULLUP);
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(kControl, OUTPUT);

  // Interrupt Pin (Used for Button)
  attachInterrupt(digitalPinToInterrupt(interruptPin0), state0, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), state1, RISING);

  // Define LED pinMode for Cup Sensing
  for (int j = 0; j < 5; j++)
  {
    pinMode(LED_PINS[j], OUTPUT);
  }

  Serial.println("LED Pins Defined");

  delay(250);

  Serial.println("Start of ToF Config loop");
  // Configure Cup Height Sensors (i = 8 to config all 8 of them)
  for (int i = 0; i < 5; i++)
  {
    tcaselect(i + 3);
    sensors_vl6180[i].init();
    sensors_vl6180[i].configureDefault();
    Serial.println(sensors_vl6180[i].getScaling());
    sensors_vl6180[i].setAddress(0x29); // Set the correct I2C address
    sensors_vl6180[i].setTimeout(500);  // Set timeout to 500ms
  }
  Serial.println("Finished ToF Configuration");
  Serial.println("Initializing...");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Waiting for Height Measurement");
}

// ************* LOOP  ************** //

int percent = 0;
float currLiquid = 0;
bool isPumping = false;
void loop()
{

  // Create an array to store sensor measurements

  while (noButton)
  {
    delay(25);
  } // Waits in Here

  if (numCupMeasure == 1 && numLiquid == 0 && firstPress == true)
  { // Button 1 Pressed, then Button 2 Pressed
    cupHeightMeasured = singleHeightMeasure();
    firstPress = false;
    Serial.println("Waiting to Start Brew...");
  }

  else if (numCupMeasure == 1 && numLiquid == 1)
  {
    brewSteadyState();
    Serial.println(cupHeightMeasured * (float(percent) / 100.0));
    while (monitorBrewButton() == false)
    {
    }
    while (currLiquid < cupHeightMeasured * (float(percent) / 100.0))
    {
      currLiquid = startLiquidPour(cupHeightMeasured);
      Serial.print("Current Level: ");
      Serial.println(currLiquid);
      if (monitorPump() == true)
      {
        int timeStart = millis();
        int runTime = 0;
        while (runTime < 27000 && currLiquid < cupHeightMeasured * (float(percent) / 100.0))
        {
          Serial.print("Runtime: ");
          Serial.println(runTime);
          Serial.print("Water Height: ");
          runTime = millis() - timeStart;
          currLiquid = startLiquidPour(cupHeightMeasured);
        }
        if(currLiquid < cupHeightMeasured * (float(percent) / 100.0)){
        restartBrew();
        }
      }
    }
    stopBrew();

    Serial.println("Brew Completed... Reset Device to Complete Next Brew");
    while (1)
    {
    } // Sit here until reset
  }

  percent = DialUIInteraction();
}

// *********** FUNCTION DEFINITIONS *************

float singleHeightMeasure()
{
  int distances[7];
  // Read measurements for all sensors
  for (int i = 0; i < 5; i++)
  {
    tcaselect(i + 3);
    distances[i] = sensors_vl6180[i].readRangeSingle();
  }
  // Serial.println(sensors_vl6180[i].timeoutOccurred());
  delay(250); // Adjust delay as needed

  // Output all measurements
  Serial.println("Measurements:");
  for (int i = 0; i < 5; i++)
  {
    Serial.print("Sensor #");
    Serial.println(i + 1);
    Serial.print(" Distance: ");
    Serial.println(distances[i]);
  }

  // Update LEDs based on measurements
  float SensorHeight = 1;
  for (int i = 5; i > -1; i--)
  {
    if (distances[i] < 150)
    {
      SensorHeight += 1; // Assume 1cm bottom + 1cm per sensor spacing
      digitalWrite(LED_PINS[i], HIGH);
      delay(250);
    }
    else
    {
      digitalWrite(LED_PINS[i], LOW);
    }
  }
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.print("Cup Height = ");
  Serial.println(SensorHeight);
  Serial.println(" ");
  Serial.println(" ");
  delay(250);
  return SensorHeight; // Return Height of Cup in cm
}

float startLiquidPour(float cupHeightMeasured)
{

  // Output to Keurig Initializing Liquid Measurement
  // digitalWrite(kControl, HIGH); // Output to Keurig Initializing Liquid Measurement
  // Serial.println(cupHeightMeasured);
  // Serial.println(percent);
  float FillHeight = cupHeightMeasured * (percent * 1.0 / 100.0);
  // Serial.print("Fill Height: ");
  // Serial.print(FillHeight);
  // Serial.println(" cm");
  float SEN_dist;

  // Initialize Summing & Iteration Variables
  float SEN_dist_raw = 0;
  int num_iter = 4;

  // Write to & Read from Liquid Sensor 4 Times
  for (int counter = 0; counter < num_iter; counter++)
  {
    writeReg(0x10, &dat, 1);
    delay(50);
    readReg(0x02, buf, 2);
    SEN_dist_raw += buf[0] * 0x100 + buf[1] + 10; // Combine Last 4 Measurements using +=
  }

  // Convert Raw Data to Height Measurement
  float SEN_dist_mm = SEN_dist_raw / num_iter; // Average Last "num_iter" measurements in mm
  SEN_dist = (SEN_dist_mm) / 10.0;             // Convert from mm to cm
  SEN_dist_raw = 0;                            // Reset averaging variable

  // Serial Plotter Output
  Serial.println(18.65 - SEN_dist);

  delay(25);

  /* if ((18.65 - SEN_dist) > FillHeight)
   {
     //digitalWrite(kControl, LOW); // Tell Keurig Cup is Full
     Serial.println("Cup Filled");
     break;
   }
 } */
  return 18.65 - SEN_dist;
}

void state0()
{
  unsigned long currentMillis0 = millis();
  if (currentMillis0 - lastDebounceTime0 > debounceDelay0)
  {

    noButton = false;
    numCupMeasure = 1; // States Cup Has Been Measured

    lastDebounceTime0 = currentMillis0;
  }
}

void state1()
{
  unsigned long currentMillis1 = millis();
  if (currentMillis1 - lastDebounceTime1 > debounceDelay1)
  {
    // Serial.println("Button 2 Pushed");

    Serial.println("Starting to Pour");

    numLiquid = 1;
    lastDebounceTime1 = currentMillis1;
  }
}

// SEN0590 Functions
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size)
{
  if (pBuf == NULL)
  {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  Wire.beginTransmission(address);
  Wire.write(&reg, 1);
  if (Wire.endTransmission() != 0)
  {
    return 0;
  }
  delay(20);
  Wire.requestFrom((uint8_t)address, (uint8_t)size);
  for (uint16_t i = 0; i < size; i++)
  {
    _pBuf[i] = Wire.read();
  }
  return size;
}

bool writeReg(uint8_t reg, const void *pBuf, size_t size)
{
  if (pBuf == NULL)
  {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  Wire.beginTransmission(address);
  Wire.write(&reg, 1);

  for (uint16_t i = 0; i < size; i++)
  {
    Wire.write(_pBuf[i]);
  }
  if (Wire.endTransmission() != 0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int ButtonPressExecute(void)
{
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(0, 50, 4);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(1);
  tft.println("           Fill Cup To: ");

  tft.setTextColor(TFT_BLUE);
  tft.setTextSize(3);
  if (counter < 10)
  {
    tft.print("    ");
  }
  else if (counter == 100)
  {
    tft.print("  ");
  }
  else
  {
    tft.print("   ");
  }
  tft.print(counter);
  tft.println("%");
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN);
  tft.println("       SET!");

  return counter;
}

void DisplayBrewInProcess(void)
{
  tft.fillScreen(TFT_YELLOW);

  tft.setCursor(0, 95, 4);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.println("BREWING");
}

void DisplayBrewComplete(void)
{
  tft.fillScreen(TFT_GREEN);

  tft.setCursor(0, 75, 4);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.println("     BREW");
  tft.setTextSize(1);
  tft.println("           COMPLETE!");
}

int DialUIInteraction(void)
{
  // Read the current state of CLK
  currentStateCLK = digitalRead(ENC_CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1)
  {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(ENC_DT) != currentStateCLK)
    {
      if (counter > 0)
      {
        counter -= 5;
        Serial.println(counter);
      }
    }
    else
    {
      if (counter < 100)
        counter += 5;
      Serial.println(counter);
    }
    tft.fillScreen(TFT_WHITE);
    tft.setCursor(0, 75, 4);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(1);
    tft.println("           Fill Cup To: ");

    tft.setTextColor(TFT_BLUE);
    tft.setTextSize(3);
    if (counter < 10)
    {
      tft.print("    ");
    }
    else if (counter == 100)
    {
      tft.print("  ");
    }
    else
    {
      tft.print("   ");
    }
    tft.print(counter);
    tft.print("%");
    float arc = (((float)counter / 100) * 360);
    tft.drawArc(120, 120, 110, 100, 0, arc, TFT_BLUE, TFT_BLUE, true);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(ENC_SW);

  // If we detect LOW signal, button is pressed
  if (btnState == LOW)
  {
    // if 50ms have passed since last LOW pulse, it means that the
    // button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50)
    {
      percent = ButtonPressExecute();
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);

  return percent;
}

void DialUISetup(void)
{
  tft.init();

  tft.fillScreen(TFT_WHITE);

  tft.setCursor(0, 75, 4);

  tft.setTextColor(TFT_RED);
  tft.setTextSize(1);
  tft.println("           Fill Cup To: ");

  tft.setTextColor(TFT_BLUE);
  tft.setTextSize(3);
  tft.print("   ");
  tft.print(CURR_PERCENT);
  tft.print("%");

  tft.drawArc(120, 120, 110, 100, 90, 180, TFT_BLUE, TFT_BLUE, true);

  // Set encoder pins as inputs
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  // Setup Serial Monitor
  Serial.begin(115200);

  int lastStateCLK = digitalRead(ENC_CLK);
}

/*

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 14);

  Serial.println("\nI2C Scanner");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("done\n");
}

void loop() {}
*/

// Start of Brew Cycle
void brewSteadyState()
{
  // Define pinMode for Brew Control
  pinMode(LIMPOS, INPUT);
  pinMode(BREW, INPUT);
  pinMode(MOS, OUTPUT);
  pinMode(RES, OUTPUT);
  pinMode(POW, INPUT);
  pinMode(PUMP, INPUT);

  digitalWrite(MOS, HIGH);
  digitalWrite(RES, LOW);
  Serial.println("Brew Steady State.");
}

bool monitorBrewButton()
{
  pinMode(BREW, INPUT);
  if (digitalRead(BREW) == 0)
  {
    // Resevoir Sensor
    Serial.println("Brew Buton Pressed.");
    delay(12000);
    pinMode(RES, INPUT);
    Serial.println("Resevoir Sensor Opened.");
    delay(5000);
    return true;
  }
  else
    return false;
}

bool monitorPump()
{
  if (digitalRead(PUMP) == 0)
  {
    return true;
  }
  return false;
}

void restartBrew()
{

  pinMode(LIMPOS, INPUT);
  pinMode(BREW, INPUT);
  pinMode(MOS, OUTPUT);
  pinMode(RES, OUTPUT);
  pinMode(POW, INPUT);

  // Power Button
  pinMode(POW, OUTPUT);
  digitalWrite(POW, LOW);
  delay(100);
  pinMode(POW, INPUT);
  Serial.println("Power button pressed. Brew aborted.");
  pinMode(RES, OUTPUT);
  pinMode(LIMPOS, INPUT);
  delay(50);

  // Limit Switch
  pinMode(LIMPOS, INPUT);
  digitalWrite(MOS, LOW);
  delay(100);
  digitalWrite(MOS, HIGH);
  Serial.println("Limit switch opened and closed.");
  pinMode(LIMPOS, OUTPUT);
  digitalWrite(LIMPOS, LOW);
  delay(5000);


  // Brew Button
  pinMode(BREW, OUTPUT);
  delay(200);
  digitalWrite(BREW, LOW);
  delay(200);
  pinMode(BREW, INPUT);
  Serial.println("Brew button pressed. Brew Initiated");
  delay(12000);

  // Resevoir Sensor
  delay(12000);
  pinMode(RES, INPUT);
  Serial.println("Resevoir Sensor Opened.");
  delay(5000);
}

void stopBrew()
{
  // Power Button
  pinMode(POW, OUTPUT);
  digitalWrite(POW, LOW);
  delay(100);
  pinMode(POW, INPUT);
  Serial.println("Power button pressed. Brew aborted.");
  brewSteadyState();
  delay(50);
}
