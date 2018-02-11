#include "twi_MRX.h"
#include <inttypes.h>
#include "Stream.h"

uint8_t rxBuffer[32];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[32];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;
uint8_t transmitting = 0;

void setup() {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  twi_init();

  Serial.begin(9600);
}

void loop() {
  requestFrom(0x1A, 6);
  while (available()) { 
    Serial.print(read());
  }
  delay(500);
  Serial.println();
}

void end(void) {
  twi_disable();
}

uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  if (isize > 0) {
  beginTransmission(address);
  endTransmission(false);
  }
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop); // perform blocking read into buffer
  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

uint8_t requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

void beginTransmission(uint8_t address) {
  transmitting = 1;// indicate that we are transmitting
  txAddress = address;// set address of targeted slave
  txBufferIndex = txBufferLength = 0;
}

uint8_t endTransmission(uint8_t sendStop) {
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
  txBufferIndex = 0;
  txBufferLength = 0;
  transmitting = 0;
  return ret;
}

int available(void) {
  return rxBufferLength - rxBufferIndex;
}

int read(void) {
  int value = -1;
  
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];  // get each successive byte on each call
    ++rxBufferIndex;
  }
  return value;
}
