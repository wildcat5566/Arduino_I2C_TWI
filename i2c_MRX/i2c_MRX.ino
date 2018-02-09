#include <Wire.h>

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  scan();
}

void loop() {
  Wire.requestFrom(34, 6);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    //char c = Wire.read(); // receive a byte as character
    Serial.print(Wire.read());         // print the character
  }

  delay(500);
  Serial.println();
}

void scan(){
  byte error, address;
  int nDevices;
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(100); 
}
