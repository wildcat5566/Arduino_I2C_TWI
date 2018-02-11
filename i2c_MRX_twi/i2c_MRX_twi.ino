#include "twi_MRX.h"

uint8_t rxBuffer[32];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

void setup() {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  twi_init();

  Serial.begin(9600);
}

void loop() {
  requestFrom(0x22, 6, 0, 0, 1);
  while (rxBufferLength > rxBufferIndex) { // available
    Serial.print(read());
  }
  delay(1);
  Serial.println();
}

uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  if (isize > 0) {
  }
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop); // perform blocking read into buffer
  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

int read(void) {
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];  // get each successive byte on each call
    ++rxBufferIndex;
  }
  return value;
}
