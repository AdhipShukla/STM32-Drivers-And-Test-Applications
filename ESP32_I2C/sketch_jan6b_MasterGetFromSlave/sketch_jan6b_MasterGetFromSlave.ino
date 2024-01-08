#include <Wire.h>
byte val = 0;
int NumByte;
void setup() {
  Wire.begin();             // Join I2C bus (address is optional for controller device)
  Serial.begin(9600);       // Start serial for output
}

void loop() {
    NumByte = Wire.requestFrom(97, 17, 1);    // Request 6 bytes from slave device number two
    Serial.println("Bytes :" + String(NumByte)); 
    // Slave may send less than requested
    while(Wire.available()) {
        char c = Wire.read();    // Receive a byte as character
        Serial.print(c);         // Print the character
    }
    Wire.endTransmission();      // Stop transmitting
    delay(500);
}