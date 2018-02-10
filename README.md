# Arduino_I2C_TWI
Arduino I2C implementation (Master receiver &amp; Slave transmitter), direct access to TWI library.

## i2c_MRX
MASTER side, serving as data receive.r <br />
### i2c_MRX.ino
Tested on Arduino Due, with IDE v1.6.7
Modified based on official code example (master reader). <br />
Didn't bother, only modified function arguments of Wire.requestFrom(). <br />
 <br />
Refs: <br />
https://www.arduino.cc/en/Tutorial/MasterReader <br />
https://playground.arduino.cc/Main/I2cScanner <br />
 <br />
 
## i2c_STX_twi
SLAVE side, serving as data transmitter. <br />
### i2c_STX_twi.ino
Direct access to the twi library (leaving the Wire library behind). <br />
Corresponding function calls with the Wire library are described as following: <br />
<br />
void setup() { <br />
  // Wire.begin(34); <br />
  twi_setAddress(0x22); <br />
  twi_attachSlaveTxEvent(ReqHandler); <br />
  txBufferIndex = txBufferLength = 0; <br />
  twi_init(); <br />
<br />  
  // Wire.onRequest(requestEvent); <br /> 
  user_onRequest = requestEvent; <br />
} <br />
...... <br />
void requestEvent() { <br />
  // Wire.write("113547"); <br />
  byte buf[6] = {1,1,3,5,4,7}; <br />
  twi_transmit(buf, 6); <br />
} <br />
### twi_STX.cpp and twi_STX.h
Simplified & modified version of the twi library (source and header files.) <br />
Only essential parts of slave-transmitter mode are left. <br />
