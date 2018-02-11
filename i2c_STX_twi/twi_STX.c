#include "twi_STX.h"
#include <avr/interrupt.h>

static volatile uint8_t twi_state;
static void (*twi_onSlaveTransmit)(void);
static uint8_t twi_txBuffer[32];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;
static volatile uint8_t twi_error;

void twi_setAddress(uint8_t address){
  TWAR = address << 1;                           // set twi slave address (skip over TWGCE bit)
}

void twi_attachSlaveTxEvent( void (*function)(void) ){
  twi_onSlaveTransmit = function;                // Set callback function.
}

void twi_init(void){
  twi_state = 0;                                 // TWI_READY: initialize state
  PORTC = 48;                                    // (1) activate internal pullups for twi.
                                                 //     byte num = B00110000, i.e. digitalWrite(SDA, 1); (SCL, 1);
  _SFR_BYTE(TWSR) &= ~_BV(TWPS0);                // (2) initialize twi prescaler and bit rate
  _SFR_BYTE(TWSR) &= ~_BV(TWPS1);
  TWBR = ((F_CPU / 100000) - 16) / 2;            // (3) set SCL clock speed = 100 kHz
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);      // (4) enable twi module, acks, and twi interrupt
}

uint8_t twi_transmit(const uint8_t* data, uint8_t length){
  uint8_t i;
  twi_txBufferLength = length;                   // set length
  for(i = 0; i < twi_txBufferLength; ++i){
    twi_txBuffer[i] = data[i];                   // populate data into tx buffer
  }
  return 0;
}

void twi_reply(uint8_t ack){
  // transmit master read ready signal, with or without ack
  if(ack){                                       // Reply ack (in progress)
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{                                         // Reply nack (send complete)
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

#define TW_STATUS_MASK    (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
ISR(TWI_vect){
  switch((TWSR & TW_STATUS_MASK)){ //TW_STATUS
    case 0xA8:                                    // (1) TW_ST_SLA_ACK(168): addressed, returned ack
    case 0xB0:                                    // (2) TW_ST_ARB_LOST_SLA_ACK(176): arbitration lost, returned ack
      twi_state = 4;                              //     TWI_STX: enter slave transmitter mode
      twi_txBufferIndex = 0;                      //     ready the tx buffer index for iteration
      twi_txBufferLength = 0;                     //     set tx buffer length to be zero
      twi_onSlaveTransmit();                      //     call twi_transmit to populate tx_buffer
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }                                           //     transmit first byte from buffer, fall
    case 0xB8:                                    // (3) TW_ST_DATA_ACK(184): byte transmitted, ack returned
      TWDR = twi_txBuffer[twi_txBufferIndex++];   //     copy data to output register
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);                             //     ack if in progress
      }else{
        twi_reply(0);                             //     nack if complete
      }
      break;
    case 0xC0:                                    // (4) TW_ST_DATA_NACK(192): last byte transmitted, NACK received 
    case 0xC8:                                    // (5) TW_ST_LAST_DATA(200): last byte transmitted, received ACK
      twi_reply(1);
      twi_state = 0;                              //     TWI_READY
      break;
  }
}

