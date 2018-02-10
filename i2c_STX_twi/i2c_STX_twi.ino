// I2C slave transmitter
// Wire library break down: direct access to twi library.

#include "twi_STX.h"
    static uint8_t txAddress;
    static uint8_t txBuffer[32];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static void (*user_onRequest)(void);
    static void onRequestService();

void setup(){
  twi_setAddress(0x3F);
  twi_attachSlaveTxEvent(ReqHandler);
  txBufferIndex = txBufferLength = 0;
  twi_init();
  
  user_onRequest = requestEvent;
}
void loop(){
  delay(100);
}
void requestEvent() {
  byte buf[6] = {7,5,7,4,3,4};
  twi_transmit(buf, 6); // respond with message of 6 bytes as expected by master
}

void ReqHandler(){
  if(!user_onRequest){
    return;
  }
  txBufferIndex = 0;
  txBufferLength = 0;
  user_onRequest();// alert user program
}

