#include "twi_MRX.h"
#include <avr/interrupt.h>

static volatile uint8_t twi_state;
static volatile uint8_t twi_slarw;
static volatile uint8_t twi_sendStop;			// should the transaction end with a stop
static volatile uint8_t twi_inRepStart;			// in the middle of a repeated start

static uint8_t twi_masterBuffer[32];
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;
static uint8_t twi_rxBuffer[32];
static volatile uint8_t twi_rxBufferIndex;
static volatile uint8_t twi_error;

void twi_setAddress(uint8_t address) {
  TWAR = address << 1; // set twi slave address (skip over TWGCE bit)
}

void twi_init(void) {
  twi_state = TWI_READY;
  twi_sendStop = 1;
  twi_inRepStart = 0;
  PORTC = 48;                                    // (1) activate internal pullups for twi.
                                                 //     byte num = B00110000, i.e. digitalWrite(SDA, 1); (SCL, 1);
  _SFR_BYTE(TWSR) &= ~_BV(TWPS0);                // (2) initialize twi prescaler and bit rate
  _SFR_BYTE(TWSR) &= ~_BV(TWPS1);
  TWBR = ((F_CPU / 100000) - 16) / 2;            // (3) set SCL clock speed = 100 kHz
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);      // (4) enable twi module, acks, and twi interrupt
}

uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  uint8_t i;
  while(TWI_READY != twi_state){
    continue;                                    // wait until twi is ready, become master receiver
  }
  twi_state = TWI_MRX;
  twi_sendStop = sendStop;  
  twi_error = 0xFF;                              // reset error state (0xFF.. no error occured)

  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;
  
  
  twi_slarw = 1;                                 // build sla+w, slave device address + w bit
  twi_slarw |= address << 1;

  if (1 == twi_inRepStart) {
    twi_inRepStart = 0;			                     // remember, we're dealing with an ASYNC ISR
    do {
      TWDR = twi_slarw;
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    
  while(TWI_MRX == twi_state){
    continue;                                    // wait for read operation to complete
  }
  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;
 
  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];               // copy twi buffer to data
  }
  return length;
}

void twi_reply(uint8_t ack) {
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

void twi_stop(void) {
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
  while(TWCR & _BV(TWSTO)){
    continue;
  }
  twi_state = TWI_READY;
}

#define TW_STATUS_MASK    (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
ISR(TWI_vect)
{
  switch(TWSR & TW_STATUS_MASK){
    // All Master
    case 0x08:                                            // (1) TW_START: sent start condition 
    case 0x10:                                            // (2) TW_REP_START: sent repeated start condition
      TWDR = twi_slarw;                                   //     copy device address and r/w bit to output register and ack
      twi_reply(1);
      break;
    case 0x50:                                            // (3) TW_MR_DATA_ACK: data received, ack sent
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;   //     put byte into buffer
    case 0x40:                                            // (4) TW_MR_SLA_ACK: address sent, ack received
      if(twi_masterBufferIndex < twi_masterBufferLength){ //     ack if more bytes are expected, otherwise nack
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case 0x58:                                            // (5) TW_MR_DATA_NACK: data received, nack sent
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;   //     put final byte into buffer
	    if (twi_sendStop)
          twi_stop();
	    else {
	        twi_inRepStart = 1;	                            // we're gonna send the START
	        // don't enable the interrupt. We'll generate the start, but we 
	        // avoid handling the interrupt until we're in the next transaction,
	        // at the point where we would normally issue the start.
	        TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
	        twi_state = TWI_READY;
	    }    
	    break;
    case 0x48:                                            // (6) TW_MR_SLA_NACK: address sent, nack received. 
      twi_stop();
      break;
    // General
    case 0xF8:                                            // TW_NO_INFO: no state information
      break;
    case 0x00:                                            // TW_BUS_ERROR: bus error, illegal stop/start
      twi_error = 0x00;
      twi_stop();
      break;
  }
}

