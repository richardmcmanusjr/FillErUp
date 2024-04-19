#include "Wire.h"
#include "TLC59108.h"

#define ADDR  0x40            // Address of TLC59108 (default is 0x40)
#define SDA0  14               // SDA pin of RPi Pico
#define SCL0  13               // SCL pin of RPi Pico
#define TLC59108_HWRESET 15   // Pin wired to TLC59108 reset

//Arduino_h::MbedI2C I2C0(SDA0,SCL0);

TLC59108 leds(ADDR); // Define TLC59108 object using I2C pointer and slave address

void setup() {

  Wire.begin(14,13); // Begin chosen interface
  digitalWrite(SDA0, LOW);
  digitalWrite(SCL0, LOW);
  leds.init();
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

}

void loop(){
  blink();
}

void blink() {

  // Blink each channel on and off one by one
  for (byte i=0; i<8; i++)
  {
    leds.setBrightness(i-1,0);
    leds.setBrightness(i,100);
    delay(200);
  }
  
  leds.setAllBrightness(0);

}
