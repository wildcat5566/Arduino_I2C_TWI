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
  requestFrom(0x22, 6, 0, 0, 1);
  while (rxBufferLength > rxBufferIndex) { // available
    Serial.print(read());
  }
  delay(10);
  Serial.println();
}

uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  if (isize > 0) {
  beginTransmission(address);
  //endTransmission(false);
  }
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop); // perform blocking read into buffer
  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

void beginTransmission(uint8_t address) {
  transmitting = 1;
  txAddress = address;                                               // set address of targeted slave
  txBufferIndex = txBufferLength = 0;
}

int read(void) {
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];  // get each successive byte on each call
    ++rxBufferIndex;
  }
  return value;
}
