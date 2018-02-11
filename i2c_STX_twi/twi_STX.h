#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
  
  void twi_setAddress(uint8_t);
  void twi_attachSlaveTxEvent( void (*)(void) );  
  void twi_init(void);
  uint8_t twi_transmit(const uint8_t*, uint8_t);
  void twi_reply(uint8_t);

#ifdef __cplusplus
}
#endif
