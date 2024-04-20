#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "SparkFun_TCA9534.h"

#define SDA 14
#define SCL 13
#define ADDR 0x40

TCA9534 tof_Mux[1]; // Create an array of two muxes for the two boards
bool mux_GPIO_State[1][8]= {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

void mux_GPIO_HIGH(int mux_num, int GPIO_Pin);
void mux_GPIO_LOW(int mux_num, int GPIO_Pin);
void toggle_Mux(int mux_num, int GPIO_Pin);
bool readGPIO(int mux_num, int GPIO_Pin);

Adafruit_VL6180X vl = Adafruit_VL6180X();
int mux_pins = 8;
int num_mux = 1;
void setup() {
  Serial.begin(115200);
  Serial.println("Here");
  int mux_num;
  int gpio;

  Wire.begin(14,13);
  
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

  mux_GPIO_HIGH(0, 4);
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() {
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  Serial.print("Lux: "); Serial.println(lux);
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
  }

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  delay(50);
}

void mux_GPIO_HIGH(int mux_num, int GPIO_Pin){ 

  // Set MUX GPIO Output HIGH
  tof_Mux[mux_num].digitalWrite(GPIO_Pin, HIGH);
  mux_GPIO_State[mux_num][GPIO_Pin] == HIGH;

}

void mux_GPIO_LOW(int mux_num, int GPIO_Pin){
  // Set MUX GPIO Output LOW

  tof_Mux[mux_num].digitalWrite(GPIO_Pin, LOW);
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
