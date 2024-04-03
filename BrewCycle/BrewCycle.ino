# define LIMPOS 10
# define MOS 9
# define BREW 8
# define RES 7
# define POW 6

void setup() {
Serial.begin(115200);
pinMode(LIMPOS,INPUT);
pinMode(BREW,INPUT);
pinMode(MOS, OUTPUT);
pinMode(RES, OUTPUT);
pinMode(POW, INPUT);
}

void loop() {
digitalWrite(MOS, HIGH);
digitalWrite(RES, LOW);

while(Serial.available() ==0){}
digitalWrite(MOS,LOW);
Serial.println("Limit Switch Closed");
Serial.readString();
pinMode(LIMPOS,OUTPUT);
digitalWrite(LIMPOS, LOW);

while(Serial.available() ==0){}
Serial.readString();
pinMode(BREW, OUTPUT);
digitalWrite(BREW,LOW);
delay(100);
pinMode(BREW, INPUT);
Serial.println("Brew button pressed. Brew Initiatied");
delay(12000);
pinMode(RES,INPUT);
Serial.println("Resevoir Sensor Opened.");

// Change below condition to while(height > full && dispense_complete != true) 
// rather than while(Serial.available() ==0).
// Will need to monitor when water height begins to rise and restart
// cycle after heated water has dispensed (~27s after water begins dispensing)

while(Serial.available() ==0){}
Serial.readString();
pinMode(POW, OUTPUT);
digitalWrite(POW, LOW);
delay(100);
pinMode(POW, INPUT);
Serial.println("Power button pressed. Brew aborted.");
pinMode(RES,OUTPUT);
pinMode(LIMPOS,INPUT);

}
