#include <Wire.h>

byte val = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Join I2C bus
}

void loop() {
    Wire.beginTransmission(97);  // Transmit to device number 44 (0x2C)
    Serial.println("Before Write");
    Wire.write(val);             // Sends value byte
    Serial.println("After Write");
    Wire.endTransmission();      // Stop transmitting
    val++;                       // Increment value
    // if reached 64th position (max)
    if(val == 64) {
      val = 0;                   // Start over from lowest value
    }
    delay(500);
}