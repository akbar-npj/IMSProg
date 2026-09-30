/*
 * ezp2023_spi.h
 *
 * Driver interface for EZP2023+ / EZP2019 / EZP2019+ USB High-Speed Flash Programmers
 * in IMSProg.
 */

#ifndef __EZP2023_SPI_H__
#define __EZP2023_SPI_H__

#include <stdint.h>
#include <stdbool.h>

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* EZP USB identifiers */
#define EZP_VID           0x1fc8
#define EZP2019_PID       0x310b  /* EZP2019 & EZP2023+ */
#define EZP2019_PLUS_PID  0x310c  /* EZP2019+ */
#define EZP2023_PID       0x310d  /* EZP2023 */

/* USB Endpoints */
#define EZP_EP_DATA_OUT   0x01
#define EZP_EP_CMD_OUT    0x02
#define EZP_EP_IN         0x82

int ezp2023_spi_init(uint8_t chipType, uint16_t speed);
int ezp2023_spi_shutdown(void);
int ezp2023_spi_send_command(unsigned int writecnt, unsigned int readcnt,
                             const unsigned char *writearr, unsigned char *readarr);
int ezp2023_enable_pins(bool enable);
int ezp2023_config_stream(unsigned int speed);
int ezp2023_get_descriptor(uint8_t *buf);

#endif /* __EZP2023_SPI_H__ */
