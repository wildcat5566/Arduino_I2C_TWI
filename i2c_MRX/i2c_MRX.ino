#include <Wire.h>

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  Wire.requestFrom(0x22, 6);    // request 6 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    Serial.print(Wire.read());         // print the character
  }
  Serial.println();
}
