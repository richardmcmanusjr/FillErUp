#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <VL6180X.h>
#include <TFT_eSPI.h>
#include "SparkFun_TCA9534.h"
#include "TLC59108.h"

#define SDA 14
#define SCL 13
#define ADDR 0x40 // LED Driver Address


/*
void VL6180X::configureDefault()
{
  // "Recommended : Public registers"

  // readout__averaging_sample_period = 48
  writeReg(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);

  // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to table "Actual gain values" in datasheet)
  writeReg(SYSALS__ANALOGUE_GAIN, 0x46);

  // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
  writeReg(SYSRANGE__VHV_REPEAT_RATE, 0xFF);

  // sysals__integration_period = 99 (100 ms)
  writeReg16Bit(SYSALS__INTEGRATION_PERIOD, 0x0063);

  // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
  writeReg(SYSRANGE__VHV_RECALIBRATE, 0x01);


  // "Optional: Public registers"

  // sysrange__intermeasurement_period = 9 (100 ms)
  writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);

  // sysals__intermeasurement_period = 49 (500 ms)
  writeReg(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);

  // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
  writeReg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);


  // Reset other settings to power-on defaults

  // sysrange__max_convergence_time = 49 (49 ms)
  writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);

  // disable interleaved mode
  writeReg(INTERLEAVED_MODE__ENABLE, 0);

  // reset range scaling factor to 1x
  setScaling(1);
}
*/







TLC59108 leds(ADDR); // Define TLC59108 object using I2C pointer and slave address

TCA9534 tof_Mux[1]; // An array of "one" mux
bool mux_GPIO_State[1][8]= {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

// ToF Sensor Array
VL6180X sensors_vl6180;

// Multiplexor Functions
void mux_GPIO_HIGH(int mux_num, int GPIO_Pin);
void mux_GPIO_LOW(int mux_num, int GPIO_Pin);
void toggle_Mux(int mux_num, int GPIO_Pin);
bool readGPIO(int mux_num, int GPIO_Pin);

int mux_pins = 8;
int num_mux = 1;

void setup() {
  Serial.begin(115200);
  delay(1000);
    // Configure all mux pins to outputs
  int mux_num;
  int gpio;

  leds.init();
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  Wire.begin(14,13); // (SDA, SCL)
  
  for(mux_num=0; mux_num < num_mux; mux_num++){
    while(tof_Mux[mux_num].begin(Wire, 0x20) == false){
      Serial.println("Mux not connected");
      delay(500);
      };
    for(gpio = 0; gpio < mux_pins; gpio++){
      tof_Mux[mux_num].pinMode(gpio, GPIO_OUT); //Configure all pins to output
      tof_Mux[mux_num].digitalWrite(gpio, LOW); // Toggle Enable High to talk to the right sensor
    }
  }


  Serial.println("Start of ToF Config loop");

  while(1){

    for(mux_num = 0; mux_num < num_mux; mux_num++){ // Do for both muxes

      // VL6180X_WaitDeviceBooted (Can be called ONLY "fresh out of reset" as otherwise will create infinite loop)
      // Software Standby
      // VL6180X_InitData()
      // VL6180X_Prepare()
      // VL6180X_RangePoll
      
      // Polling (Host requests and waits) --> VL6180x_RangePollMeasurement
      //Interrupt (Ranging results retrieved from interrupts) 
      // Asynchronous (Host requests then regularly check to see if result is ready) --> VL6180x_RangeStartSingleShot & VL6180x_RangeGetMeasurementIfReady

      // VL6180x_RangeSetMaxConvergenceTime() - sets maximum time to run measurement in all ranging modes
      
      for (gpio = 0; gpio < mux_pins; gpio++){
        Serial.print("GPIO: ");
        Serial.println(gpio);
        mux_GPIO_HIGH(mux_num, gpio);
        delay(25);
        sensors_vl6180.writeReg(0x016,0x1);
        sensors_vl6180.init();
        delay(25);
        sensors_vl6180.configureDefault();
        delay(25);
        sensors_vl6180.setAddress(0x29+gpio);
        delay(25);
        sensors_vl6180.setTimeout(500);  // Set timeout to 500ms
        delay(25);



        // Identification Register
        Serial.println(sensors_vl6180.readReg(0x000),HEX);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        // SYSRANGE START
        sensors_vl6180.writeReg(0x018,0x01); // Sends message to complete Single Shot Measurement
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }


        uint8_t distance_RAW = sensors_vl6180.readReg(0x064);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        uint8_t Range_Val = sensors_vl6180.readReg(0x062);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        int rangeStatus = sensors_vl6180.readReg(0x04D);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        int Sensor_Address = sensors_vl6180.readReg(0x212);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        int GPIO0_Mode = sensors_vl6180.readReg(0x010);
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        // RESULT_RANGE_STATUS1
        Serial.print(" Range Status:");
        Serial.print(rangeStatus, BIN);
        Serial.println("       ----      0  = No Error, 10001 = VCSEL Continuity Test");
        if (sensors_vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        
        // RESULT_RANGE_RAW      
        Serial.print(" Raw Distance: ");
        Serial.println(distance_RAW);
        Serial.println(" ");

        // RESULT_RANGE_VAL
        Serial.print(" Measured Distance: ");
        Serial.println(Range_Val);     
        Serial.println(" ");

        // Sensor Address  
        Serial.print(" Sensor Address: ");
        Serial.println(Sensor_Address, HEX);     
        Serial.println(" ");

        // GPIO0 Mode  
        Serial.print(" GPIO0 Mode: ");
        Serial.println(GPIO0_Mode, BIN);     
        Serial.println(" ");

        Serial.println(" ");
        delay(2000);
        mux_GPIO_LOW(mux_num, gpio);
      }

    }

  }

  Serial.println("Finished ToF Configuration");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Waiting for Height Measurement");
  mux_GPIO_HIGH(0, 4);
  delay(25);
  
}

float cupHeightMeasured = 0;
float distance;

void loop() {
//    
}

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

//float singleHeightMeasure()
//{
//  int distances[2][8];
//  // Read measurements for all sensors
//  int mux_num;
//  int gpio;
//  for (mux_num = 0; mux_num < num_mux; mux_num++){
//    for (gpio = 0; gpio < mux_pins; gpio++)
//    {
//      mux_GPIO_HIGH(mux_num, gpio);
//      delay(25);
//      distances[mux_num][gpio] = sensors_vl6180[mux_num][gpio].readRangeSingle();
//      if (sensors_vl6180[mux_num][gpio].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
//      mux_GPIO_LOW(mux_num, gpio);
//      delay(25);
//
//    }
//  }
//
//  delay(50); // Adjust delay as needed
//
//  // Output all measurements
//  Serial.println("Measurements:");
//  for (mux_num=0; mux_num < num_mux; mux_num++){
//    for (gpio = 0; gpio < mux_pins; gpio++)
//    {
//      Serial.print("Sensor #");
//      Serial.println(mux_num*8 + gpio + 1);
//      Serial.print(" Distance: ");
//      Serial.println(distances[mux_num][gpio]);
//    }
//  }
//
//  // Update LEDs based on measurements
//  
//  float SensorHeight = 1; // Distance from bottom to first sensor
//  for (mux_num = 0; mux_num < num_mux; mux_num++){
//    for (gpio = mux_pins-1; gpio > -1; gpio--){
//      if (distances[mux_num][gpio] < 150){ // Cup Close Enough to Sensor
//        SensorHeight += 1.25; // Assume 1cm bottom + 1cm per sensor spacing
//        leds.setBrightness(gpio,50);
//      }
//      else{
//        leds.setBrightness(gpio,0);
//      }
//    }
//  }
//
//  Serial.println(" ");
//  Serial.println(" ");
//  Serial.println(" ");
//  Serial.print("Cup Height = ");
//  Serial.println(SensorHeight);
//  Serial.println(" ");
//  Serial.println(" ");
//  delay(250);
//  return SensorHeight; // Return Height of Cup in cm
//}
