// I2C master receiver on arduino UNO (atmega328)
// Wire library break down: direct access to twi library and registers.

uint8_t rxBuffer[32];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;
static uint8_t masterBuffer[32];
static volatile uint8_t masterBufferIndex;
static volatile uint8_t masterBufferLength;

static volatile uint8_t state;
static volatile uint8_t slarw;
static volatile uint8_t sendStop;      // should the transaction end with a stop
static volatile uint8_t inRepStart;     // in the middle of a repeated start
static volatile uint8_t err;

void setup() {
  sendStop = 1;
  inRepStart = 0;
  rxBufferIndex = rxBufferLength = 0;
  
  state = 0;
  PORTC = 48;                                    // (1) activate internal pullups for twi.
                                                 //     byte num = B00110000, i.e. digitalWrite(SDA, 1); (SCL, 1);
  _SFR_BYTE(TWSR) &= ~_BV(TWPS0);                // (2) initialize twi prescaler and bit rate
  _SFR_BYTE(TWSR) &= ~_BV(TWPS1);
  TWBR = ((F_CPU / 100000) - 16) / 2;            // (3) set SCL clock speed = 100 kHz
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);      // (4) enable twi module, acks, and twi interrupt
  
  Serial.begin(9600);
} // </setup>

void loop() {
  requestFrom(0x29, 6, 0, 0, 1);
  while (rxBufferLength > rxBufferIndex) { // available
    Serial.print(rxBuffer[rxBufferIndex]);
    ++rxBufferIndex;
  }
  delay(1);
  Serial.println();
} // </loop>

uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  if (isize > 0) {
  }
  uint8_t message = readFrom(address, rxBuffer, quantity, sendStop); // perform blocking read into buffer
  rxBufferIndex = 0;
  rxBufferLength = message;
  return message;
} // </requestFrom>

uint8_t readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop) {
  uint8_t i;
  while(0 != state){
    continue;                                    // wait until twi is ready, become master receiver
  }
  state = 1;
  sendStop = sendStop;  
  err = 0xFF;                              // reset error state (0xFF.. no error occured)
  masterBufferIndex = 0;
  masterBufferLength = length - 1;
  slarw = 1;                                 // build sla+w, slave device address + w bit
  slarw |= address << 1;

  if (1 == inRepStart) {
    inRepStart = 0;                           // remember, we're dealing with an ASYNC ISR
    do {
      TWDR = slarw;
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    
  while(1 == state){
    continue;                                    // wait for read operation to complete
  }
  if (masterBufferIndex < length)
    length = masterBufferIndex;
 
  for(i = 0; i < length; ++i){
    data[i] = masterBuffer[i];               // copy twi buffer to data
  }
  return length;
} // </readFrom>

#define TW_STATUS_MASK    (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
ISR(TWI_vect) {
  switch(TWSR & TW_STATUS_MASK) {
    case 0x08:                                                  // (1) TW_START: sent start condition 
    
    case 0x10:                                                  // (2) TW_REP_START: sent repeated start condition
      TWDR = slarw;                                             //     copy address and populate register
      TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);    //     ack
      break;
    
    case 0x50:                                                  // (3) TW_MR_DATA_ACK: data received, ack sent
      masterBuffer[masterBufferIndex++] = TWDR;                 //     populate buffer
    
    case 0x40:                                                  // (4) TW_MR_SLA_ACK: address sent, ack received
      if(masterBufferIndex < masterBufferLength){
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);  // ack if in progress
      }else{
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);              // else nack
      }
      break;
      
    case 0x58:                                                  // (5) TW_MR_DATA_NACK: data received, nack sent
      masterBuffer[masterBufferIndex++] = TWDR;                 //     put final byte into buffer
      if (sendStop){
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO); // stop and stare
        while(TWCR & _BV(TWSTO)){continue;}
        state = 0;
      }else{
          inRepStart = 1; 
          TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;           // send start but don't enable interrupt
          state = 0;
      }    
      break;
      
    case 0x48:                                                  // (6) TW_MR_SLA_NACK: address sent, nack received. 
      TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO); // stop and stare
      while(TWCR & _BV(TWSTO)){continue;}
      state = 0;
      break;
      
    // General
    case 0xF8:                                                  // (X1) TW_NO_INFO: no state information
      break;
    case 0x00:                                                  // (X2) TW_BUS_ERROR: bus error, illegal stop/start
      err = 0x00;
      TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO); // stop and stare
      while(TWCR & _BV(TWSTO)){continue;}
      state = 0;
      break;
  }
} // </ISR>
