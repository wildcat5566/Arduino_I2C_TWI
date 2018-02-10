#include "twi_STX.h"
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include "Arduino.h" // for digitalWrite
#include "pins_arduino.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static volatile uint8_t twi_state;
static volatile uint8_t twi_slarw;
static volatile uint8_t twi_sendStop;			// should the transaction end with a stop
static volatile uint8_t twi_inRepStart;			// in the middle of a repeated start

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static volatile uint8_t twi_error;

void twi_init(void){
  twi_state = TWI_READY;                         // initialize state
  twi_sendStop = true;		                       // default value
  twi_inRepStart = false;
  
  digitalWrite(SDA, 1);                          // (1) activate internal pullups for twi.
  digitalWrite(SCL, 1);

  
  cbi(TWSR, TWPS0);                              // (2) initialize twi prescaler and bit rate
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;          // (3) set SCL clock speed = 100 kHz

  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);      // (4) enable twi module, acks, and twi interrupt
}

void twi_disable(void){
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));  // disable twi module, acks, and twi interrupt
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
}

void twi_setAddress(uint8_t address){
  TWAR = address << 1;                           // set twi slave address (skip over TWGCE bit)
}

uint8_t twi_transmit(const uint8_t* data, uint8_t length){
  uint8_t i;
  if(TWI_BUFFER_LENGTH < length){
    return 1;                                    // data length exceeds buffer length
  }
  if(TWI_STX != twi_state){
    return 2;                                    // not a slave transmitter
  }
  twi_txBufferLength = length;                   // set length
  for(i = 0; i < length; ++i){
    twi_txBuffer[i] = data[i];                   // populate data into tx buffer
  }
  return 0;
}

void twi_attachSlaveTxEvent( void (*function)(void) ){
  twi_onSlaveTransmit = function;                // Set callback function.
}


void twi_reply(uint8_t ack){
  // transmit master read ready signal, with or without ack
  if(ack){                                       // Reply ack (in progress)
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{                                         // Reply nack (send complete)
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

void twi_stop(void){
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
  // send stop condition
  // wait for stop condition to be executed on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }
  twi_state = TWI_READY;
}

void twi_releaseBus(void){
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);// release bus
  twi_state = TWI_READY;
}

ISR(TWI_vect){
  switch(TW_STATUS){
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      twi_state = TWI_STX;// enter slave transmitter mode
      twi_txBufferIndex = 0;// ready the tx buffer index for iteration
      twi_txBufferLength = 0;// set tx buffer length to be zero
      twi_onSlaveTransmit(); // call twi_transmit to populate tx_buffer
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      TWDR = twi_txBuffer[twi_txBufferIndex++];// copy data to output register
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1); // ack if in progress
      }else{
        twi_reply(0); // nack if complete
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      twi_reply(1);// ack future responses
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}

