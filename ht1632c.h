#ifndef HT1632C_H
#define HT1632C_H

#include <inttypes.h>


#define SPI_FREQ 2560000

#define HT1632_CMD_8NMOS 0x20	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_16NMOS 0x24	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_8PMOS 0x28	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_16PMOS 0x2C	/* CMD= 0010-ABxx-x commons options */



//
// public functions
//

/// Initializes library and display.
/// Commons mode is either 8/16 NMOS/PMOS (see #define)
int ht1632c_init(const uint8_t commonsMode);

/// Shuts down library.
int ht1632c_close();

/// Sets display brightness.
void ht1632c_pwm(const uint8_t value);

/// Sends frame buffer to display; required to bring any drawing operations to the display.
void ht1632c_sendframe();

/// Clears the whole frame. Also reset clipping area.
void ht1632c_clear();

/// Puts a single value at HT1632 addr, at COM bit index
/// Note: only sets bit in frame buffer. a call to ht1632_sendframe() is required to update the display.
void ht1632c_update_framebuffer(const int addr, const uint8_t bitIndex, const uint8_t bitValue);

uint8_t ht1632c_get_framebuffer(const int addr, const uint8_t bitIndex);

#endif
