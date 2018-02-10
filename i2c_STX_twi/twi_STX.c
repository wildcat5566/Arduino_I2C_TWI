#include "twi_STX.h"
#include <avr/interrupt.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

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
                                                 // byte num = B00110000, i.e. digitalWrite(SDA, 1); (SCL, 1);
  cbi(TWSR, TWPS0);                              // (2) initialize twi prescaler and bit rate
  cbi(TWSR, TWPS1);
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

/*void twi_disable(void){
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));  // disable twi module, acks, and twi interrupt
  PORTC = 0;
}

void twi_stop(void){
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
  // send stop condition
  // wait for stop condition to be executed on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }
  twi_state = 0; //TWI_READY
}

void twi_releaseBus(void){
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);  // release bus
  twi_state = 0; //TWI_READY
}*/

#define TW_STATUS_MASK    (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
ISR(TWI_vect){
  switch((TWSR & TW_STATUS_MASK)){ //TW_STATUS
    // Slave Transmitter
    case 0xA8:                                    // (1) TW_ST_SLA_ACK: addressed, returned ack
    case 0xB0:                                    // (2) TW_ST_ARB_LOST_SLA_ACK: arbitration lost, returned ack
      twi_state = 4;                        //     TWI_STX: enter slave transmitter mode
      twi_txBufferIndex = 0;                      //     ready the tx buffer index for iteration
      twi_txBufferLength = 0;                     //     set tx buffer length to be zero
      twi_onSlaveTransmit();                      //     call twi_transmit to populate tx_buffer
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
    case 0xB8:                                    // (3) TW_ST_DATA_ACK: byte transmitted, ack returned
      TWDR = twi_txBuffer[twi_txBufferIndex++];   //     copy data to output register
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);                             //     ack if in progress
      }else{
        twi_reply(0);                             //     nack if complete
      }
      break;
    case 0xC0:                                    // (4) TW_ST_DATA_NACK: last byte tranmitted, NACK received 
    case 0xC8:                                    // (5) TW_ST_LAST_DATA: last byte transmitted, however received ACK
      twi_reply(1);
      twi_state = 0; //TWI_READY
      break;

    // General
    case 0xF8:   // TW_NO_INFO, no state information
      break;
    case 0x00: // TW_BUS_ERROR, illegal stop/start
      twi_error = 0x00;
      //twi_stop();
      break;
  }
}

