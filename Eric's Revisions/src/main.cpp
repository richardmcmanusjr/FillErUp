#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL6180X.h>
#include <TFT_eSPI.h>

// Order of Operations
/* Order of Operations
A) Cup placed underneath brewing machine
B) Dial is set, Brew button is pressed
C) ESP32 communicates with Mux
D) Mux activates GPIO0_ToF1, ESP32 sends command to measure distance to ToF1
E) Measurement received, added to an array to holds ToF distances
F) MUX turns off GPIO0_TOF1 (disables ToF)
G) Repeat steps D-F for ToF1-ToF7
H) After all ToF are read, check array to see how many sensors are below threshold distance (~100mm or 4 inches?)
I) Communicating with the LED driver, power on LEDs (one by one) from bottom most LED to the LED that corresponds with last ToF that measured below threshold (D_tof)
J) (Insert Steps for Richards Setup)
K) Convert ToFs below threshold to distance measurement from bottom to top sensor
L) Subtract this distance from the TOTAL distance (D_total) from bottom to Liquid sensor
M) Start the pour (Richard's code) and start measurements, doing averages of every four measurements (or some other algorithm)
N) Pour should stop if the measured distance of the sensor, D_liquid makes this statement true: D_liquid < D_total-D_tof
O) Do multiple brews if this fails to happen
*/

// Code for LED Driver

// Base Slave Address is _____

#include "TLC59108.h"

#define driver0_addr TLC59108::I2C_ADDR::BASE
#define driver1_addr TLC59108::I2C_ADDR::BASE

TLC59108 driver0(driver0_addr);
TLC59108 driver1(driver1_addr);


// Code for multiplexor
#include "SparkFun_TCA9534.h"
TCA9534 tof_Mux[2]; // Create an array of two muxes for the two boards
bool mux_GPIO_State[2][8]= { {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}, {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW} };

//Setup


// Functions


// Libraries



// Function Declarations
void state0();
void state1();
float singleHeightMeasure();
float startLiquidPour(float cupHeightMeasured);
bool writeReg(uint8_t reg, const void *pBuf, size_t size);
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size);

// Keurig Functions
void stopBrew();
void restartBrew();
bool monitorPump();
bool monitorBrewButton();
void brewSteadyState();

// Rotary Encoder Functions
int ButtonPressExecute(void);
void DisplayBrewInProcess(void);
void DisplayBrewComplete(void);
int DialUIInteraction(void);
void DialUISetup();

// Multiplexor Functions
void mux_GPIO_HIGH(int mux_num, int GPIO_Pin);
void mux_GPIO_LOW(int mux_num, int GPIO_Pin);
void toggle_Mux(int mux_num, int GPIO_Pin);
bool readGPIO(int mux_num, int GPIO_Pin);

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
#define LIMPOS 37
#define MOS 40
#define BREW 39
#define RES 41
#define POW 42
#define PUMP 5
#define half_bright 128

// ToF Sensor Array
VL6180X sensors_vl6180[2][8];

// SEN0590 Definitions
#define address 0x74
uint8_t buf[2] = {0};
uint8_t dat = 0xB0;

// Button Definitions
unsigned long lastDebounceTime0 = 0;
unsigned long debounceDelay0 = 750;
unsigned long lastDebounceTime1 = 0;
unsigned long debounceDelay1 = 750;

/* void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(25);
}
*/

// *********** SETUP *************

float cupHeightMeasured;
int numLiquid = 0;
int dialPressed = 0;
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
  int HW_RESET_PIN = 2;
  driver0.init(HW_RESET_PIN);
  driver1.init(HW_RESET_PIN);
  driver0.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  driver1.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

  int min = 0;
  int max = 255;
  driver0.setAllBrightness(max); // Set all LEDs to 0 Brightness (valid values between 0 and 255)
  driver1.setAllBrightness(max); // Set all LEDs to 0 Brightness (valid values between 0 and 255)
  
  DialUISetup();
  delay(500);
  Serial.begin(115200);
  Serial.println("Entered Setup");

  /* Initialize all muxes
  for(int mux_num = 0; mux_num < 2; mux_num ++ ){
    for(int gpio=0; gpio<8; gpio++){
      if(sensors_vl6180[mux_num][gpio].begin() == false){
        Serial.println("Mux" + String(sensors_vl6180[mux_num][gpio]) + "Failed to Initialize");
        exit(1);
      }
    }
  }
  */

  int mux_pins = 8;
  int num_mux = 2;

  // Configure all mux pins to outputs
  for(int counter0=0; counter0 < num_mux; counter0++){
    for(int counter1 = 0; counter1 < mux_pins; counter++){
      tof_Mux[counter0].pinMode(0, GPIO_OUT); //Configure all pins to output
    }
  }

  //int SCL_Pin = 14;
  //int SDA_Pin = 21;
  Wire.begin();
  Wire.setClock(100000L); // L denotes long type

  // Pinmode Definitions
  pinMode(interruptPin0, INPUT_PULLUP);
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(kControl, OUTPUT);

  // Interrupt Pin (Used for Button)
  attachInterrupt(digitalPinToInterrupt(interruptPin0), state0, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), state1, RISING);

  // Define LED pinMode for Cup Sensing

  delay(250);

  Serial.println("Start of ToF Config loop");
  
  // Configure Cup Height Sensors (both boards)
  for(int mux_num = 0; mux_num < 2; mux_num++){ // Do for both muxes
    for (int gpio = 0; gpio < 8; gpio++){
      tof_Mux[mux_num].digitalWrite(gpio, HIGH); // Toggle Enable High to talk to the right sensor
      delay(25);
      sensors_vl6180[mux_num][gpio].init();
      sensors_vl6180[mux_num][gpio].configureDefault();
      sensors_vl6180[mux_num][gpio].setAddress(0x29); // Set the correct I2C address
      sensors_vl6180[mux_num][gpio].setTimeout(500);  // Set timeout to 500ms
    }
  }

  Serial.println("Finished ToF Configuration");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Waiting for Height Measurement");

}






//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------







// ************* LOOP  ************** //

int percent = 0;
float currLiquid = 0;
bool isPumping = false;

void loop()
{

  // Create an array to store sensor measurements

  // Convert Rotary Coder Input To First Button Press
  if (dialPressed == 1 && numLiquid == 0 && firstPress == true) // Button 1 Has Been Pressed
  { 
    firstPress = false;
    Serial.println("Height Set! Waiting to Start Brew...");
  }

  else if (dialPressed == 1 && numLiquid == 1)
  { // Button 1 and Button 2 Has Been Pressed
    cupHeightMeasured = singleHeightMeasure();
    Serial.println(cupHeightMeasured * (float(percent) / 100.0));

    brewSteadyState();

    while (monitorBrewButton() == false){}

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

  percent = DialUIInteraction(); // Percent of cup full controlled by rotary encoder
}



// *********** FUNCTION DEFINITIONS *************

float singleHeightMeasure()
{
  int distances[2][8];
  // Read measurements for all sensors

  for (int mux_num = 0; mux_num < 2; mux_num++){
    for (int gpio = 0; gpio < 8; gpio++)
    {
      tof_Mux[mux_num].digitalWrite(gpio, HIGH); // Toggle Enable High
      delay(25);
      distances[mux_num][gpio] = sensors_vl6180[mux_num][gpio].readRangeSingle();
    }
  }

  delay(50); // Adjust delay as needed

  // Output all measurements
  Serial.println("Measurements:");
  for (int mux_num=0; mux_num < 2; mux_num++){
    for (int gpio = 0; gpio < 8; gpio++)
    {
      Serial.print("Sensor #");
      Serial.println(mux_num*8 + gpio + 1);
      Serial.print(" Distance: ");
      Serial.println(distances[mux_num][gpio]);
    }
  }

  // Update LEDs based on measurements
  
  float SensorHeight = 1; // Distance from bottom to first sensor
  for (int mux_num = 0; mux_num < 2; mux_num++){
    for (int gpio = 8; gpio > -1; gpio--)
    {

      if (distances[mux_num][gpio] < 150){ // Cup Close Enough to Sensor
        SensorHeight += 1.25; // Assume 1cm bottom + 1cm per sensor spacing
        if (mux_num == 0){
          driver0.setBrightness(gpio, half_bright); // Set individual LEDs to 50% Brightness
        }
        else if (mux_num == 1){
          driver1.setBrightness(gpio, half_bright); // Set individual LEDs to 50% Brightness
        }
      }

      else{
        if (mux_num == 0){
          driver0.setBrightness(gpio, 0); // Set individual LEDs to 0% Brightness
        }
        else if (mux_num == 1){
          driver1.setBrightness(gpio, 0); // Set individual LEDs to 0% Brightness
        }
      }

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

/* void state0() //Not needed, state controlled by rotary controller

{
  unsigned long currentMillis0 = millis();
  if (currentMillis0 - lastDebounceTime0 > debounceDelay0)
  {

    noButton = false;
    dialPressed = 1; // States Cup Height Has Been Set With Dial

    lastDebounceTime0 = currentMillis0;
  }
}
*/

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
      percent = ButtonPressExecute(); // Execute when button has been pressed
      dialPressed = 1; // Change loop condition to enter loop if statement (once! with bool firstPress)
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

void mux_GPIO_HIGH(int mux_num, int GPIO_Pin){ 

  // Set MUX GPIO Output HIGH
  tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
  mux_GPIO_State[mux_num][GPIO_Pin] == HIGH;

}

void mux_GPIO_LOW(int mux_num, int GPIO_Pin){
  // Set MUX GPIO Output LOW

  tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
  mux_GPIO_State[mux_num][GPIO_Pin] == LOW;

}

void toggle_Mux(int mux_num, int GPIO_Pin){
// Swap GPIO Position --> I still have no fucking clue why I wrote this
  if(readGPIO(mux_num,GPIO_Pin) == LOW){
    tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
    mux_GPIO_State[mux_num][GPIO_Pin] == HIGH;

  }
  else if(readGPIO(mux_num,GPIO_Pin) == LOW){
    tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
    mux_GPIO_State[mux_num][GPIO_Pin] == LOW;

  }

}

bool readGPIO(int mux_num, int GPIO_Pin){

  if(mux_GPIO_State [mux_num][GPIO_Pin] == HIGH){
    return HIGH;
  }
  else{
    return LOW;
  }

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

*/