#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  #define TWI_READY 0
  #define TWI_MRX   1

  void twi_init(void);
  void twi_setAddress(uint8_t);
  uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
  void twi_reply(uint8_t);
  void twi_stop(void);

#ifdef __cplusplus
}
#endif
