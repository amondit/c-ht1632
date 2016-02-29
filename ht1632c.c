#include "ht1632c.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

/*#include <wiringPiSPI.h>
#include <wiringPi.h>

#include "panelconfig.h"*/
#include "wiringPiSPI.h"
#include "wiringPi.h"

/*
 * commands written to the chip consist of a 3 bit "ID", followed by
 * either 9 bits of "Command code" or 7 bits of address + 4 bits of data.
 */
#define HT1632_ID_CMD        4	/* ID = 100 - Commands */
#define HT1632_ID_RD         6	/* ID = 110 - Read RAM */
#define HT1632_ID_WR         5	/* ID = 101 - Write RAM */

#define HT1632_CMD_SYSDIS 0x00	/* CMD= 0000-0000-x Turn off oscil */
#define HT1632_CMD_SYSON  0x01	/* CMD= 0000-0001-x Enable system oscil */
#define HT1632_CMD_LEDOFF 0x02	/* CMD= 0000-0010-x LED duty cycle gen off */
#define HT1632_CMD_LEDON  0x03	/* CMD= 0000-0011-x LEDs ON */
#define HT1632_CMD_BLOFF  0x08	/* CMD= 0000-1000-x Blink ON */
#define HT1632_CMD_BLON   0x09	/* CMD= 0000-1001-x Blink Off */
#define HT1632_CMD_SLVMD  0x10	/* CMD= 0001-00xx-x Slave Mode */
#define HT1632_CMD_MSTMD  0x14	/* CMD= 0001-01xx-x Master Mode */
#define HT1632_CMD_RCCLK  0x18	/* CMD= 0001-10xx-x Use on-chip clock */
#define HT1632_CMD_EXTCLK 0x1C	/* CMD= 0001-11xx-x Use external clock */
#define HT1632_CMD_PWM    0xA0	/* CMD= 101x-PPPP-x PWM duty cycle */

#define HT1632_ID_LEN     3  /* IDs are 3 bits */
#define HT1632_CMD_LEN    8  /* CMDs are 8 bits */
#define HT1632_DATA_LEN   8  /* Data are 4*2 bits */
#define HT1632_ADDR_LEN   7  /* Address are 7 bits */

// panel parameters
#define PANEL_HEADER_BITS (HT1632_ID_LEN + HT1632_ADDR_LEN)
#define HT1632_CS   10

// use this WiringPi lock ID
#define LOCK_ID 0

#define toBit(v) ((v) ? 1 : 0)


/// framebuffer
static uint8_t** ht1632c_framebuffer = 0;
static uint8_t ht1632c_framebuffer_len = 96;

static int ht1632c_spifd = -1;

//
// internal functions
//

void *reverse_endian(void *p, size_t size) {
  char *head = (char *)p;
  char *tail = head + size -1;

  for(; tail > head; --tail, ++head) {
    char temp = *head;
    *head = *tail;
    *tail = temp;
  }
  return p;
}


void ht1632c_chipselect(const int value)
{
	digitalWrite(HT1632_CS, toBit(value));
}

void ht1632c_sendcmd(const uint8_t cmd) {
	uint16_t data = HT1632_ID_CMD;
	data <<= HT1632_CMD_LEN;
	data |= cmd;
	data <<= 5;
	reverse_endian(&data, sizeof(data));

	piLock(LOCK_ID);

	ht1632c_chipselect(1);
	write(ht1632c_spifd, &data, 2);
	ht1632c_chipselect(0);

	piUnlock(LOCK_ID);
}

void ht1632c_update_framebuffer(const int addr, const uint8_t bitIndex, const uint8_t bitValue)
{
	if(addr <0 || addr > (ht1632c_framebuffer_len-1) || bitIndex > 3)
		return;
	ht1632c_framebuffer[addr][bitIndex] = toBit(bitValue);
}

uint8_t ht1632c_get_framebuffer(const int addr, const uint8_t bitIndex)
{
	if(addr <0 || addr > (ht1632c_framebuffer_len-1) || bitIndex > 3)
			return 0;
	return ht1632c_framebuffer[addr][bitIndex];
}

//
// public functions
//

int ht1632c_init(const uint8_t commonsMode)
{
	// init WiringPi, SPI
	if (wiringPiSetup() == -1) {
		printf( "WiringPi Setup Failed: %s\n", strerror(errno));
		return 1;
	}
	if ((ht1632c_spifd = wiringPiSPISetup(0, SPI_FREQ)) < 0) {
		printf( "SPI Setup Failed: %s\n", strerror(errno));
		return 1;
	}


	switch (commonsMode) {
		case HT1632_CMD_8NMOS:
		case HT1632_CMD_8PMOS:
			ht1632c_framebuffer_len = 64;
			break;
		case HT1632_CMD_16NMOS:
		case HT1632_CMD_16PMOS:
			ht1632c_framebuffer_len = 96;
			break;
		default:
			break;
	}


	ht1632c_framebuffer = (uint8_t**) malloc(ht1632c_framebuffer_len*sizeof(uint8_t*));
	if (!ht1632c_framebuffer) {
		printf( "Framebuffer allocation failed.");
		return 3;
	}

	for (int i = 0; i < ht1632c_framebuffer_len; ++i) {
		ht1632c_framebuffer[i] = (uint8_t*) malloc(4*sizeof(uint8_t));
		if (!ht1632c_framebuffer[i]) {
				printf( "Framebuffer allocation failed.");
				return 3;
			}
		ht1632c_framebuffer[i][0] = ht1632c_framebuffer[i][1] = ht1632c_framebuffer[i][1] = ht1632c_framebuffer[i][3] = 0;
	}

	// configure CS pin
	pinMode(HT1632_CS, OUTPUT);

	// init display
	ht1632c_sendcmd(HT1632_CMD_SYSDIS);
	ht1632c_sendcmd(commonsMode);
	ht1632c_sendcmd(HT1632_CMD_MSTMD);
	ht1632c_sendcmd(HT1632_CMD_RCCLK);
	ht1632c_sendcmd(HT1632_CMD_SYSON);
	ht1632c_sendcmd(HT1632_CMD_LEDON);
	ht1632c_sendcmd(HT1632_CMD_BLOFF);
	ht1632c_sendcmd(HT1632_CMD_PWM);

	ht1632c_clear();
	ht1632c_sendframe();

	return 0;
}

int ht1632c_close()
{
	close(ht1632c_spifd);

	if (ht1632c_framebuffer) {
		for (int i = 0; i < ht1632c_framebuffer_len; ++i) {
			if (ht1632c_framebuffer[i]) {
				free(ht1632c_framebuffer[i]);
				ht1632c_framebuffer[i] = 0;
			}
		}
		free(ht1632c_framebuffer);
		ht1632c_framebuffer = 0;
	}
}

void ht1632c_pwm(const uint8_t value)
{
	ht1632c_sendcmd(HT1632_CMD_PWM | (value & 0x0f));
}

void ht1632c_sendframe()
{
	uint16_t data = HT1632_ID_WR;
	data <<= HT1632_ADDR_LEN;
	data <<= 6;

	//We append the first 1.5 addresses values, to fill the 16bit buffer
	uint8_t bitValues = 0;
	for (int j = 0; j < 4; ++j) {
		bitValues <<= 1;
		bitValues |= toBit(ht1632c_framebuffer[0][j]);
	}
	for (int j = 0; j < 2; ++j) {
		bitValues <<= 1;
		bitValues |= toBit(ht1632c_framebuffer[1][j]);
	}
	data |= bitValues;
	reverse_endian(&data, sizeof(data));

	piLock(LOCK_ID);
	ht1632c_chipselect(1);
	//Write WRITE command, initialized at addr 0
	write(ht1632c_spifd, &data, sizeof(data));


	//Loop to write the frame buffer
	int i = 1;
	while (i < ht1632c_framebuffer_len) {
		bitValues = 0;
		//we copy the last 2 bits of the previous address,
		//as they are truncated due to message size of 16b
		//
		//(format of bit values : AABB BBCC)
		for (int j = 2; j < 4; ++j) {
			bitValues <<= 1;
			bitValues |= toBit(ht1632c_framebuffer[i][j]);
		}
		i++;
		if (i == ht1632c_framebuffer_len) {
			bitValues <<=6;
		}else {
			for (int j = 0; j < 4; ++j) {
				bitValues <<= 1;
				bitValues |= toBit(ht1632c_framebuffer[i][j]);
			}
			i++;
			if (i == ht1632c_framebuffer_len) {
				bitValues <<= 2;
			}else {
				for (int j = 0; j < 2; ++j) {
					bitValues <<= 1;
					bitValues |= toBit(ht1632c_framebuffer[i][j]);
				}
			}

		}

		write(ht1632c_spifd, &bitValues, sizeof(bitValues));
	}

	ht1632c_chipselect(0);
	piUnlock(LOCK_ID);
}

void ht1632c_clear()
{
	// clear buffer
	if (ht1632c_framebuffer) {
		for (int i = 0; i < ht1632c_framebuffer_len; ++i) {
			if (ht1632c_framebuffer[i]) {
				ht1632c_framebuffer[i][0] = ht1632c_framebuffer[i][1] = ht1632c_framebuffer[i][1] = ht1632c_framebuffer[i][3] = 0;
			}
		}
	}

}
