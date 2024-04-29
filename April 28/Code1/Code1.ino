#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL6180X.h>
#include <TFT_eSPI.h>
#include "SparkFun_TCA9534.h"
#include "TLC59108.h"


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

// Function Declarations
float singleHeightMeasure();
float startLiquidPour(float cupHeightMeasured, bool first_occassion);
bool writeReg(uint8_t reg, const void *pBuf, size_t size);
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size);

// Keurig Functions
void stopBrew();
void restartBrew();
bool pumpRunning();
bool monitorBrewButton();
void brewSteadyState();
void SexyLights();

// Rotary Encoder Functions
int ButtonPressExecute(void);
void DisplayBrewInProcess(void);
void DisplayBrewComplete(void);
int DialUIInteraction(void);
void DialUISetup();
float AverageLiquidHeight(float);

// Multiplexor Functions
void mux_GPIO_HIGH(int mux_num, int GPIO_Pin);
void mux_GPIO_LOW(int mux_num, int GPIO_Pin);
void toggle_Mux(int mux_num, int GPIO_Pin);
bool readGPIO(int mux_num, int GPIO_Pin);

// Rotary Encoder
#define ENC_CLK 16
#define ENC_DT 15
#define ENC_SW 18
#define CURR_PERCENT 75

// Button Pins
//#define interruptPin0 35
//#define interruptPin1 36

// LED Shows Liquid Filled
#define LiquidLED 4

// Cup Height LEDs
#define LIMPOS 4
#define MOS 5
#define BREW 2
#define RES 6
#define POW 1
#define PUMP 17

// I2C Communication
#define SDA 14
#define SCL 13
#define NUM2AVG 3

// Code for multiplexor
TCA9534 tof_Mux[2]; // Create an array of two muxes for the two boards
bool mux_GPIO_State[2][8]= { {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}, {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW} };
float liquidHeights[NUM2AVG];
int liquidHeightsCounter = 0;


// ToF Sensor Array
VL6180X sensors_vl6180; // Treat 16x as a single device 

// SEN0590 Definitions
#define address 0x74
#define MaxHeight = 27.5
uint8_t buf[2] = {0};
uint8_t dat = 0xB0;

// Button Definitions
unsigned long lastDebounceTime0 = 0;
unsigned long debounceDelay0 = 750;
unsigned long lastDebounceTime1 = 0;
unsigned long debounceDelay1 = 750;



// *********** SETUP *************


float cupHeightMeasured;
int numLiquid = 0;
int dialPressed = 0;
float heightOutput = 0.0;
bool noButton = true;
bool firstPress = true;

// Circular Display 
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
int counter = CURR_PERCENT;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;

// Led Driver 
#define ADDR_LED0 0x40
#define ADDR_LED1 0x41

TLC59108 leds0(ADDR_LED0);
TLC59108 leds1(ADDR_LED1);

int mux_pins = 8;
int num_mux = 2;

int mux_num;
int gpio;

void setup()
{

  Serial.begin(115200);
  delay(1000);

  Serial.println("Entered Setup");

  Wire.begin(SDA, SCL);
  Wire1.begin(40, 39);

  // Setup LED Drivers
  leds0.init();
  leds1.init();
  leds0.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds1.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  
  
  // Set up Multiplexors
  for(mux_num=0; mux_num < num_mux; mux_num++){
    while(tof_Mux[mux_num].begin(Wire, 0x20 + mux_num) == false){
      Serial.println("Mux not connected");
      delay(500);
      };
    for(gpio = 0; gpio < mux_pins; gpio++){
      tof_Mux[mux_num].pinMode(gpio, GPIO_OUT); //Configure all pins to output
      tof_Mux[mux_num].digitalWrite(gpio, LOW); //Set all GPIOs to LOW
    }
  }

  // Set up Time-of-Flight Sensors
  Serial.println("Start of ToF Config loop");

  // Configuration of both muxes and all TOF sensors
  for(mux_num = num_mux-1; mux_num > -1; mux_num--){ 
    for (gpio = mux_pins-1; gpio > -1; gpio--){

      // Print Current ToF and Mux Being Detected
      Serial.print("GPIO: ");
      Serial.println(gpio);
      Serial.print("    Mux: ");
      Serial.println(mux_num);


      // Enable one TOF, configure, then disable
      mux_GPIO_HIGH(mux_num, gpio);

      sensors_vl6180.init();
      sensors_vl6180.setAddress(0x29);
      sensors_vl6180.setTimeout(100);  // Set timeout to 500ms

      mux_GPIO_LOW(mux_num, gpio);

//      if (mux_num == 0){
//        for(int counter = 0; counter < 5; counter ++){
//          leds0.setBrightness(gpio, 100);
//          Serial.println("Blink");
//          delay(5);
//          leds0.setBrightness(gpio, 0);
//          Serial.println("Off");
//          delay(5);
//        }
//      }
//      else if(mux_num == 1){
//        for(int counter = 0; counter < 5; counter ++){
//          leds1.setBrightness(gpio, 100);
//          delay(5);
//          leds1.setBrightness(gpio, 0);
//          delay(5);
//        }
//      }
    }
  }
 
  SexyLights();
  tft.init();
   
  // Start up Rotary Dial
  delay(500);
  
  // Pinmode Definitions
  //pinMode(interruptPin0, INPUT_PULLUP);
  //pinMode(interruptPin1, INPUT_PULLUP);

  // Interrupt Pin (Used for Button)
  //attachInterrupt(digitalPinToInterrupt(interruptPin0), state0, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin1), state1, RISING);

  // Define LED pinMode for Cup Sensing  

  Serial.println("Waiting for Height Measurement");

}



// ************* LOOP  ************** //

int percent = 0;
float currLiquid = 0;
int maxRunTime = 45000;
int numTargetHeight = 0;
bool first = true;
bool notFirst = false;

void loop(){
  for (int element = 0; element < NUM2AVG; element++){
    liquidHeights[element] = 0;
  }
  bool filled = false;
  bool firstTime = true;
  DialUISetup();
  delay(500);
  percent = DialUIInteraction(); // Percent of cup full controlled by rotary encoder
    
  // Complete Height Measurement
  cupHeightMeasured = singleHeightMeasure(); // Complete all ToF Measurements & Trigger LED & Calculate Cup Height
  float cupOffset = 2;
  float liquidHeightTarget = (cupHeightMeasured-cupOffset)*(float(percent) / 100.0)+cupOffset;
  Serial.print("Height of Liquid to Be Poured: ");
  Serial.print(liquidHeightTarget); // Print Height of Liquid to be Poured
  Serial.println(" cm");

  brewSteadyState(); // Configure Keurig Pins - Set MOS High and RES Low

  while (monitorBrewButton() == false){} // Stay here until brew button pressed

  // Enters while loop when liquid starts pouring
  while (!filled){ // Outer While Loop --> Only Finishes When Liquid Has 

    currLiquid = startLiquidPour(cupHeightMeasured,notFirst); // Measured Height of Current Liquid Level
    Serial.print("Current Level: ");
    Serial.println(currLiquid);
    
    if (currLiquid > liquidHeightTarget){
      filled = true;
    }
    
    //brewing... function

    if (pumpRunning() == true) // Checks that pump is off before proceeding
    {

      //active display percentage
      int timeStart = millis();
      int runTime = 0;

      while (runTime < maxRunTime && !filled) // Enters if runtime is under 27 seconds and measured liquid level is not above desired height
      {
        runTime = millis() - timeStart;
        currLiquid = startLiquidPour(cupHeightMeasured,notFirst); // Continuously Measure Liquid Height
        Serial.print("Runtime: ");
        Serial.println(runTime);
        Serial.print("Water Height: ");
        Serial.println(currLiquid);

//          if (currLiquid > liquidHeightTarget*0.7 && firstTime){
//            firstTime = false;
//            Serial.println("Restarting... almost filled");
//            restartBrew();
//            break;
//          }


       if (currLiquid > liquidHeightTarget){
        filled = true;
       }
  
        if(runTime>maxRunTime){ // If reservoir empty or runtime over and not filled, start next brew
          Serial.println("Restarting... brew time exceeded");
          restartBrew();
          break;
        }
      }
    }
  }

  stopBrew();

  Serial.println("Brew Completed... Reset Device to Complete Next Brew");
}


// *********** FUNCTION DEFINITIONS *************

float singleHeightMeasure()
{
  int distances[num_mux][8];
  int maxToFGPIO;
  int maxToFMux;
  int threshold = 30;
  // Read measurements for all sensors

    for(mux_num = num_mux-1; mux_num > -1; mux_num--){ // Do for both muxes

      for (gpio = mux_pins-1; gpio > -1; gpio--){

        Serial.print("GPIO: ");
        Serial.println(gpio);
        mux_GPIO_HIGH(mux_num, gpio); // Turn on GPIO
        Serial.println(sensors_vl6180.readReg(0x000),HEX);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        sensors_vl6180.writeReg(0x018,0x01);
        sensors_vl6180.writeReg(0x02d,0x00);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        delay(150);
        Serial.println(sensors_vl6180.readReg(0x04F),BIN);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        // Complete Single ToF Measurement
        distances[mux_num][gpio] = sensors_vl6180.readReg(0x062);


        // Configure LEDs Measurements (Determine if threshold is met)
        if(distances[mux_num][gpio] < threshold && mux_num == 0){
          Serial.print("Blink");
          leds0.setBrightness(gpio,100);
          maxToFGPIO = gpio;
          maxToFMux = 0;
        }
        else if(distances[mux_num][gpio] < threshold && mux_num == 1){
          Serial.print("Blink");
          leds1.setBrightness(gpio,100);
          maxToFGPIO = gpio;
          maxToFMux = 1;

         
        }
        else if(distances[mux_num][gpio] > threshold && mux_num == 0){
          Serial.print("Off");
          leds0.setBrightness(gpio,0); 
                  
        }
        else if(distances[mux_num][gpio] > threshold && mux_num == 1){
          Serial.print("Off");
          leds1.setBrightness(gpio,0);         
        }

        else{Serial.print("Error");}

        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        Serial.print(" Distance: ");
        Serial.println(distances[mux_num][gpio]);

        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        
        mux_GPIO_LOW(mux_num, gpio);
      }
    }

    Serial.println(maxToFGPIO);
    if (maxToFMux == 0){
      leds0.setBrightness(maxToFGPIO,0); 
    }
    else if(maxToFMux == 1){
      leds1.setBrightness(maxToFGPIO,0); 
    }
    // Determine Overall Cup Height
    float Sens_Meas_Height = 2; // Distance from bottom to first sensor
    for (int mux_num = 0; mux_num < 2; mux_num++){
      for (int gpio = 8; gpio > -1; gpio--)
      {
        if (distances[mux_num][gpio] < threshold){ // Cup Close Enough to Sensor
          Sens_Meas_Height += 1.27; // Assume 1cm bottom + 1.27cm per sensor spacing
        }
      }
    }
  Sens_Meas_Height -= 2.56;

  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.print("Cup Height = ");
  Serial.println(Sens_Meas_Height);
  Serial.println(" ");
  Serial.println(" ");
  delay(250);
  return Sens_Meas_Height; // Return Height of Cup in cm
}

float startLiquidPour(float cupHeightMeasured, bool first_occassion)
{
  float curr_Liquid_Poured;
  // Output to Keurig Initializing Liquid Measurement
  // Serial.println(cupHeightMeasured);
  // Serial.println(percent);
  float FillHeight = (cupHeightMeasured * (percent * 1.0 / 100.0)); // Fill Height with 5% buffer from actual measured height
  // Serial.print("Fill Height: ");
  // Serial.print(FillHeight);
  // Serial.println(" cm");
  float SEN_dist;

  float Sen_to_bottom = 29;

  // Initialize Summing & Iteration Variables
  float SEN_dist_raw = 0;
  int num_iter = 10;
  float high = 0;
  float distance;
  // Write to & Read from Liquid Sensor 4 Times
  for (int counter = 0; counter < num_iter+1; counter++)
  {
    writeReg(0x10, &dat, 1);
    delay(50);
    readReg(0x02, buf, 2);
    distance = buf[0] * 0x100 + buf[1] + 10;
    if (distance > high){high = distance;}
    SEN_dist_raw += distance; // Combine Last 4 Measurements using +=
    delay(5);
  }

  // Convert Raw Data to Height Measurement
  float SEN_dist_mm = (SEN_dist_raw-high) / float(num_iter); // Average Last "num_iter" measurements in mm
  SEN_dist = (SEN_dist_mm) / 10.0;             // Convert from mm to cm
  SEN_dist_raw = 0;                            // Reset averaging variable

  
  // Serial Plotter Output
  curr_Liquid_Poured = Sen_to_bottom - SEN_dist; // Total distance - sensor measurement = Liquid Height

  delay(25);

  if(first_occassion){
    return(curr_Liquid_Poured);
  }
  return AverageLiquidHeight(curr_Liquid_Poured);
}


// SEN0590 Functions

uint8_t readReg(uint8_t reg, const void *pBuf, size_t size)
{
  if (pBuf == NULL)
  {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  Wire1.beginTransmission(address);
  Wire1.write(&reg, 1);
  if (Wire1.endTransmission() != 0)
  {
    return 0;
  }
  delay(20);
  Wire1.requestFrom((uint8_t)address, (uint8_t)size);
  for (uint16_t i = 0; i < size; i++)
  {
    _pBuf[i] = Wire1.read();
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
  Wire1.beginTransmission(address);
  Wire1.write(&reg, 1);

  for (uint16_t i = 0; i < size; i++)
  {
    Wire1.write(_pBuf[i]);
  }
  if (Wire1.endTransmission() != 0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


// Rotary Encoder Functions

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

int DialUIInteraction (void) {
  bool button_press = false;
  int percent = 0;
  while (button_press == false){
    // Read the current state of CLK
    currentStateCLK = digitalRead(ENC_CLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(ENC_DT) != currentStateCLK) {
        if (counter > 0){
          counter -= 5;
        }
      } else {
        if (counter < 100)
          counter += 5;
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
      float arc = (((float)counter/100)*360);
      tft.drawArc(120, 120, 110, 100, 0, arc, TFT_BLUE, TFT_BLUE, true);
    }

    // Remember last CLK state
    lastStateCLK = currentStateCLK;
    // Read the button state
    int btnState = digitalRead(ENC_SW);

    //If we detect LOW signal, button is pressed
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        ButtonPressExecute();
        button_press = true;
        }

      // Remember last button press event
      lastButtonPress = millis();
    }

    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  return counter;
}

void DialUISetup(void)
{

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



// Keurig Functions

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
    Serial.println("Brew Button Pressed.");
    delay(500);
    if (pumpRunning()){
      Serial.println("Pump is running. Waiting 12s to open resevoir sensor.");
      delay(12000);
      pinMode(RES, INPUT);
      Serial.println("Resevoir Sensor Opened.");
      delay(5000);
    }
    else{
      Serial.println("Pump not running. Will proceed assuming heater is full.");
    }
    return true;
  }
  else
    return false;
}

bool pumpRunning()
{
  if (analogRead(PUMP) < 2000)
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
  delay(50);

  // Limit Switch
  Serial.println("Limit switch opened and closed.");
  pinMode(LIMPOS, OUTPUT);
  digitalWrite(LIMPOS, HIGH);
  delay(100);
  pinMode(LIMPOS, INPUT);
  delay(5000);


  // Brew Button
  pinMode(BREW, OUTPUT);
  delay(200);
  digitalWrite(BREW, LOW);
  delay(200);
  pinMode(BREW, INPUT);
  Serial.println("Brew button pressed. Brew Initiated");

  delay(500);
  if (pumpRunning()){
    Serial.println("Pump is running. Waiting 12s to open resevoir sensor.");
    delay(12000);
    pinMode(RES, INPUT);
    Serial.println("Resevoir Sensor Opened.");
    delay(5000);
  }
  else{
    Serial.println("Pump not running. Will proceed assuming heater is full.");
  }
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




// Mux Functions

void mux_GPIO_HIGH(int mux_num, int GPIO_Pin){ 

  // Set MUX GPIO Output HIGH
  tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
  mux_GPIO_State[mux_num][GPIO_Pin] = HIGH;

}

void mux_GPIO_LOW(int mux_num, int GPIO_Pin){
  // Set MUX GPIO Output LOW

  tof_Mux[mux_num].digitalWrite(GPIO_Pin, LOW);
  mux_GPIO_State[mux_num][GPIO_Pin] = LOW;

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

void SexyLights(){
  Serial.println("Sexxxy Lights!!!");
  for(int numLasers = 0; numLasers < 3; numLasers++){
    for(int counter = 15; counter > -1; counter--){
      if(counter > 7){
        leds1.setBrightness(counter - 8, 100);
        delay(10);
        leds1.setBrightness(counter - 8, 0);
        delay(10);
      }
      else{
        leds0.setBrightness(counter, 100);
        delay(10);
        leds0.setBrightness(counter, 0);
        delay(10);
      }
     }
     
     for(int counter = 0; counter < 16; counter++){
      if(counter > 7){
        leds1.setBrightness(counter - 8, 100);
        delay(10);
        leds1.setBrightness(counter - 8, 0);
        delay(10);
      }
      else{
        leds0.setBrightness(counter, 100);
        delay(10);
        leds0.setBrightness(counter, 0);
        delay(10);
      }
     }
    }

   for(int numFlashes = 0; numFlashes < 3; numFlashes++){
     for(int counter = 0; counter < 16; counter++){
       if(counter > 7){
         leds1.setBrightness(counter - 8, 100);
       }
       else{
         leds0.setBrightness(counter, 100);
       }
     }
     delay(100);
     for(int counter = 0; counter < 16; counter++){
       if(counter > 7){
         leds1.setBrightness(counter - 8, 0);
       }
       else{
         leds0.setBrightness(counter, 0);
       }
     }
     delay(100);
   }
}

/*

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();

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
float AverageLiquidHeight(float liquidHeight){
  liquidHeights[liquidHeightsCounter] = liquidHeight;
  liquidHeightsCounter++;
  if(liquidHeightsCounter == NUM2AVG){
    liquidHeightsCounter = 0;
    }
  int i;
  float total;
  for (i = 0; i < NUM2AVG; i++){
    total += liquidHeights[i];
  }
  float averageLiquidHeight = total/NUM2AVG;
  return averageLiquidHeight;
}
