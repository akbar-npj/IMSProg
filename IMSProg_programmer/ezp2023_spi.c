/*
 * ezp2023_spi.c
 *
 * Driver implementation for EZP2023+ / EZP2019 / EZP2019+ USB High-Speed Flash Programmers
 * in IMSProg.
 */

#include "ezp2023_spi.h"
#include <libusb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define EZP_PACKET_SIZE   64
#define EZP_READ_SIZE     256
#define EZP_USB_TIMEOUT   5000

/* Command Codes */
#define EZP_CMD_ERASE     0x02
#define EZP_CMD_TRIGGER   0x05
#define EZP_CMD_READ_SPI  0x07
#define EZP_CMD_RESET     0x08
#define EZP_CMD_CONNECT   0x09
#define EZP_CMD_STATUS    0x0A
#define EZP_CMD_WRITE     0x0B

/* SPI opcodes */
#define OPCODE_WREN       0x06
#define OPCODE_WRDI       0x04
#define OPCODE_RDSR       0x05
#define OPCODE_READ       0x03
#define OPCODE_FAST_READ  0x0B
#define OPCODE_PP         0x02
#define OPCODE_QPP        0x32
#define OPCODE_SE         0xD8
#define OPCODE_BE         0x60
#define OPCODE_BE1        0xC7
#define OPCODE_P4E        0x20
#define OPCODE_RES        0xAB
#define OPCODE_RDID       0x9F
#define OPCODE_READ_ID    0x90

static struct libusb_context *ezp_ctx = NULL;
static struct libusb_device_handle *ezp_handle = NULL;

static uint32_t ezp_chip_id = 0;
static uint32_t ezp_chip_size = 16 * 1024 * 1024; /* 16 MB default */
static uint16_t ezp_chip_pagesize = 256;
static uint16_t ezp_chip_protocol = 0;
static uint16_t ezp_chip_timeout = 1000;
static bool ezp_chip_configured = false;

/* Buffer for accumulating SPI write bytes */
#define EZP_CMD_BUF_SIZE 1024
static uint8_t ezp_cmd_buf[EZP_CMD_BUF_SIZE];
static uint32_t ezp_cmd_len = 0;

static void prepare_command_packet(uint8_t packet[EZP_PACKET_SIZE],
                                   uint8_t command_id,
                                   uint32_t size,
                                   uint16_t pagesize,
                                   uint16_t protocol,
                                   uint16_t timeout,
                                   uint32_t chip_id_val)
{
    memset(packet, 0, EZP_PACKET_SIZE);
    packet[1] = command_id;
    packet[2] = protocol & 0xFF;
    packet[3] = (protocol >> 8) & 0xFF;
    packet[4] = (pagesize >> 8) & 0xFF;
    packet[5] = pagesize & 0xFF;
    packet[6] = (timeout >> 8) & 0xFF;
    packet[7] = timeout & 0xFF;
    packet[8]  = (size >> 24) & 0xFF;
    packet[9]  = (size >> 16) & 0xFF;
    packet[10] = (size >> 8) & 0xFF;
    packet[11] = size & 0xFF;
    packet[12] = (chip_id_val >> 24) & 0xFF;
    packet[13] = (chip_id_val >> 16) & 0xFF;
    packet[14] = (chip_id_val >> 8) & 0xFF;
    packet[15] = chip_id_val & 0xFF;
}

static int ezp_send_raw_command(const uint8_t command[EZP_PACKET_SIZE],
                                uint8_t result[EZP_PACKET_SIZE])
{
    int sent = 0, received = 0, ret;

    if (!ezp_handle) return -1;

    ret = libusb_bulk_transfer(ezp_handle, EZP_EP_CMD_OUT,
                               (unsigned char *)command, EZP_PACKET_SIZE,
                               &sent, EZP_USB_TIMEOUT);
    if (ret != 0 || sent != EZP_PACKET_SIZE) {
        if (ret == LIBUSB_ERROR_PIPE)
            libusb_clear_halt(ezp_handle, EZP_EP_CMD_OUT);
        return -1;
    }

    if (result) {
        usleep(50000); /* 50ms wait required by EZP MCU firmware */
        ret = libusb_bulk_transfer(ezp_handle, EZP_EP_IN,
                                   result, EZP_PACKET_SIZE,
                                   &received, EZP_USB_TIMEOUT);
        if (ret != 0 || received != EZP_PACKET_SIZE) {
            if (ret == LIBUSB_ERROR_PIPE)
                libusb_clear_halt(ezp_handle, EZP_EP_IN);
            return -1;
        }
    }
    return 0;
}

static int ezp_reset_device(void)
{
    uint8_t packet[EZP_PACKET_SIZE];
    memset(packet, 0, EZP_PACKET_SIZE);
    packet[0] = 0x01;
    packet[1] = EZP_CMD_RESET;
    return ezp_send_raw_command(packet, NULL);
}

static int ezp_do_connect(void)
{
    uint8_t packet[EZP_PACKET_SIZE];
    uint8_t result[EZP_PACKET_SIZE];
    int ret;

    prepare_command_packet(packet, EZP_CMD_CONNECT,
                           ezp_chip_size, ezp_chip_pagesize,
                           ezp_chip_protocol, ezp_chip_timeout,
                           ezp_chip_id);

    /* First connect wakes SPI bus */
    ret = ezp_send_raw_command(packet, result);
    if (ret < 0) return -1;

    usleep(50000);

    /* Second connect returns stable JEDEC ID */
    ret = ezp_send_raw_command(packet, result);
    if (ret < 0) return -1;

    ezp_chip_id = ((uint32_t)result[1] << 16) |
                  ((uint32_t)result[2] << 8) |
                   (uint32_t)result[3];
    ezp_chip_configured = true;

    return 0;
}

static int ezp_poll_status(int max_retries, int sleep_us)
{
    uint8_t spacket[EZP_PACKET_SIZE];
    uint8_t sresult[EZP_PACKET_SIZE];
    int total_polls = 0;
    int ret;

    memset(spacket, 0, EZP_PACKET_SIZE);
    spacket[1] = EZP_CMD_STATUS;

    while (max_retries-- > 0) {
        total_polls++;
        usleep(sleep_us);
        memset(sresult, 0xFF, sizeof(sresult));
        ret = ezp_send_raw_command(spacket, sresult);
        if (ret == 0) {
            if ((sresult[0] & 0x01) == 0 && total_polls > 1) {
                return 0;
            }
        }
    }
    return -1;
}

static int ezp_do_read(uint32_t addr, uint32_t len, uint8_t *buf)
{
    uint8_t packet[EZP_PACKET_SIZE];
    uint8_t result[EZP_PACKET_SIZE];
    uint8_t chunk_buf[EZP_READ_SIZE];
    int ret;

    if (!ezp_chip_configured) {
        if (ezp_do_connect() < 0) return -1;
    }

    /* Send READ_SPI setup */
    prepare_command_packet(packet, EZP_CMD_READ_SPI,
                           ezp_chip_size, ezp_chip_pagesize,
                           ezp_chip_protocol, ezp_chip_timeout,
                           ezp_chip_id);
    if (ezp_send_raw_command(packet, result) < 0) return -1;

    /* Send trigger with starting address */
    memset(packet, 0, EZP_PACKET_SIZE);
    packet[1] = EZP_CMD_TRIGGER;
    packet[8]  = (addr >> 24) & 0xFF;
    packet[9]  = (addr >> 16) & 0xFF;
    packet[10] = (addr >> 8) & 0xFF;
    packet[11] = addr & 0xFF;
    if (ezp_send_raw_command(packet, result) < 0) return -1;

    /* Read stream in 256-byte chunks */
    uint32_t remaining = len;
    uint32_t offset = 0;

    while (remaining > 0) {
        int chunk = (remaining > EZP_READ_SIZE) ? EZP_READ_SIZE : (int)remaining;
        int received = 0;

        ret = libusb_bulk_transfer(ezp_handle, EZP_EP_IN,
                                   chunk_buf, EZP_READ_SIZE,
                                   &received, EZP_USB_TIMEOUT);
        if (ret == 0 && received >= chunk) {
            memcpy(buf + offset, chunk_buf, chunk);
            offset += chunk;
            remaining -= chunk;
        } else {
            if (ret == LIBUSB_ERROR_PIPE)
                libusb_clear_halt(ezp_handle, EZP_EP_IN);
            return -1;
        }
    }
    return 0;
}

static int ezp_do_write(uint32_t addr, uint32_t len, const uint8_t *data)
{
    uint8_t packet[EZP_PACKET_SIZE];
    uint8_t result[EZP_PACKET_SIZE];
    int ret, written = 0;

    if (ezp_do_connect() < 0) return -1;

    prepare_command_packet(packet, EZP_CMD_READ_SPI,
                           ezp_chip_size, ezp_chip_pagesize,
                           ezp_chip_protocol, ezp_chip_timeout,
                           ezp_chip_id);
    if (ezp_send_raw_command(packet, result) < 0) return -1;

    /* Trigger starting address for write stream */
    memset(packet, 0, EZP_PACKET_SIZE);
    packet[1] = EZP_CMD_TRIGGER;
    packet[8]  = (addr >> 24) & 0xFF;
    packet[9]  = (addr >> 16) & 0xFF;
    packet[10] = (addr >> 8) & 0xFF;
    packet[11] = addr & 0xFF;
    if (ezp_send_raw_command(packet, NULL) < 0) return -1;

    /* Fast data write to DATA_OUT endpoint */
    ret = libusb_bulk_transfer(ezp_handle, EZP_EP_DATA_OUT,
                               (unsigned char *)data, (int)len,
                               &written, EZP_USB_TIMEOUT);
    if (ret != 0 || written != (int)len) {
        if (ret == LIBUSB_ERROR_PIPE)
            libusb_clear_halt(ezp_handle, EZP_EP_DATA_OUT);
        return -1;
    }

    usleep(100000);
    return ezp_poll_status(1000, 20000);
}

static int ezp_do_erase(void)
{
    uint8_t packet[EZP_PACKET_SIZE];
    uint8_t result[EZP_PACKET_SIZE];

    if (ezp_do_connect() < 0) return -1;

    prepare_command_packet(packet, EZP_CMD_READ_SPI,
                           ezp_chip_size, ezp_chip_pagesize,
                           ezp_chip_protocol, ezp_chip_timeout,
                           ezp_chip_id);
    if (ezp_send_raw_command(packet, result) < 0) return -1;

    /* Chip Erase */
    memset(packet, 0, EZP_PACKET_SIZE);
    packet[0] = 0x01;
    packet[1] = EZP_CMD_ERASE;
    packet[0x1A] = 0x80;
    packet[0x1B] = 0x00;
    if (ezp_send_raw_command(packet, NULL) < 0) return -1;

    usleep(100000);
    return ezp_poll_status(12000, 5000);
}

/* ------------------------------------------------------------------ */
/* Exported Interface                                                 */

int ezp2023_spi_init(uint8_t chipType, uint16_t speed)
{
    (void)chipType;
    (void)speed;
    int ret;

    if (ezp_handle != NULL) return 0;

    ret = libusb_init(&ezp_ctx);
    if (ret < 0) return -1;

    static const uint16_t ezp_pids[] = {
        EZP2019_PID,
        EZP2019_PLUS_PID,
        EZP2023_PID,
        0
    };

    for (const uint16_t *pid = ezp_pids; *pid; pid++) {
        ezp_handle = libusb_open_device_with_vid_pid(ezp_ctx, EZP_VID, *pid);
        if (ezp_handle != NULL) break;
    }

    if (ezp_handle == NULL) {
        if (ezp_ctx) {
            libusb_exit(ezp_ctx);
            ezp_ctx = NULL;
        }
        return -1;
    }

    libusb_set_auto_detach_kernel_driver(ezp_handle, 1);

    /* Mandatory handshake: query string descriptors 1 & 2 */
    uint8_t tmp[256];
    libusb_control_transfer(ezp_handle, 0x80, 0x06, (0x03 << 8) | 0x01, 0x0409, tmp, sizeof(tmp), 1000);
    libusb_control_transfer(ezp_handle, 0x80, 0x06, (0x03 << 8) | 0x02, 0x0409, tmp, sizeof(tmp), 1000);

    ret = libusb_claim_interface(ezp_handle, 0);
    if (ret != 0) {
        libusb_close(ezp_handle);
        ezp_handle = NULL;
        if (ezp_ctx) {
            libusb_exit(ezp_ctx);
            ezp_ctx = NULL;
        }
        return -1;
    }

    ezp_cmd_len = 0;
    ezp_chip_configured = false;
    ezp_chip_id = 0;

    return 0;
}

int ezp2023_spi_shutdown(void)
{
    if (ezp_handle == NULL) return 0;

    ezp_reset_device();

    libusb_release_interface(ezp_handle, 0);
    libusb_close(ezp_handle);
    ezp_handle = NULL;

    if (ezp_ctx) {
        libusb_exit(ezp_ctx);
        ezp_ctx = NULL;
    }

    ezp_cmd_len = 0;
    ezp_chip_configured = false;
    ezp_chip_id = 0;

    return 0;
}

int ezp2023_enable_pins(bool enable)
{
    if (enable) {
        /* CS goes low: transaction begins */
        ezp_cmd_len = 0;
    } else {
        /* CS goes high: execute accumulated write or erase */
        if (ezp_cmd_len > 0) {
            uint8_t opcode = ezp_cmd_buf[0];
            if (opcode == OPCODE_PP || opcode == OPCODE_QPP) {
                uint32_t addr = 0;
                int addr_len = 3;
                if (ezp_cmd_len >= 5) {
                    addr = ((uint32_t)ezp_cmd_buf[1] << 16) |
                           ((uint32_t)ezp_cmd_buf[2] << 8) |
                            (uint32_t)ezp_cmd_buf[3];
                    addr_len = 3;
                }
                const uint8_t *data = ezp_cmd_buf + 1 + addr_len;
                int data_len = ezp_cmd_len - (1 + addr_len);
                if (data_len > 0) {
                    ezp_do_write(addr, (uint32_t)data_len, data);
                }
            } else if (opcode == OPCODE_BE1 || opcode == OPCODE_BE ||
                       opcode == OPCODE_SE || opcode == OPCODE_P4E) {
                ezp_do_erase();
            }
        }
        ezp_cmd_len = 0;
    }
    return 0;
}

int ezp2023_spi_send_command(unsigned int writecnt, unsigned int readcnt,
                             const unsigned char *writearr, unsigned char *readarr)
{
    if (!ezp_handle) return -1;

    if (writecnt > 0) {
        if (ezp_cmd_len + writecnt <= EZP_CMD_BUF_SIZE) {
            memcpy(ezp_cmd_buf + ezp_cmd_len, writearr, writecnt);
            ezp_cmd_len += writecnt;
        }
    }

    if (readcnt > 0) {
        if (ezp_cmd_len == 0) {
            memset(readarr, 0xFF, readcnt);
            return 0;
        }

        uint8_t opcode = ezp_cmd_buf[0];

        if (opcode == OPCODE_RDID) {
            if (ezp_do_connect() < 0) {
                memset(readarr, 0xFF, readcnt);
                return -1;
            }
            memset(readarr, 0, readcnt);
            if (readcnt >= 1) readarr[0] = (ezp_chip_id >> 16) & 0xFF;
            if (readcnt >= 2) readarr[1] = (ezp_chip_id >> 8) & 0xFF;
            if (readcnt >= 3) readarr[2] = ezp_chip_id & 0xFF;
        } else if (opcode == OPCODE_READ_ID) {
            if (ezp_do_connect() < 0) {
                memset(readarr, 0xFF, readcnt);
                return -1;
            }
            memset(readarr, 0, readcnt);
            if (readcnt >= 1) readarr[0] = (ezp_chip_id >> 16) & 0xFF;
            if (readcnt >= 2) readarr[1] = (ezp_chip_id >> 8) & 0xFF;
        } else if (opcode == OPCODE_RDSR || opcode == 0x35 || opcode == 0x15) {
            memset(readarr, 0, readcnt);
        } else if (opcode == OPCODE_READ || opcode == OPCODE_FAST_READ ||
                   opcode == 0x3B || opcode == 0x6B || opcode == 0xBB || opcode == 0xEB) {
            uint32_t addr = 0;
            if (ezp_cmd_len >= 4) {
                addr = ((uint32_t)ezp_cmd_buf[1] << 16) |
                       ((uint32_t)ezp_cmd_buf[2] << 8) |
                        (uint32_t)ezp_cmd_buf[3];
            }
            if (ezp_do_read(addr, readcnt, readarr) < 0) return -1;
        } else if (opcode == OPCODE_RES) {
            memset(readarr, 0, readcnt);
        } else {
            memset(readarr, 0xFF, readcnt);
        }

        ezp_cmd_len = 0;
    }
    return 0;
}

int ezp2023_config_stream(unsigned int speed)
{
    (void)speed;
    return 0;
}

int ezp2023_get_descriptor(uint8_t *buf)
{
    if (!ezp_handle) return -1;
    return libusb_get_descriptor(ezp_handle, LIBUSB_DT_DEVICE, 0x00, buf, 0x12);
}
