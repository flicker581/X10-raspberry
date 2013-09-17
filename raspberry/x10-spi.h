/*
 * X10 control via SPI, Linux part of the picture
 *
 * Copyright (c) 2013 pavel@levshin.spb.ru
 *
 * derived from:
 *
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */

#ifndef x10_spi_h
#define x10_spi_h

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

// Maximum is 32 due to stream_tail size, but RAM restricts it further
#define X10_BITSTREAM_OCTETS 24

struct __attribute__((__packed__)) x10_bitstream {
	uint8_t data[X10_BITSTREAM_OCTETS];
	uint8_t tail; // pointer to the bit after the stream
};

struct __attribute__((__packed__)) spi_message {
	uint8_t rr_code;
	uint8_t rr_id;
	struct x10_bitstream x10_data;
	uint16_t crc16;
};

#define SPI_REQUEST_POLL 0
#define SPI_REQUEST_CANCEL 1
#define SPI_REQUEST_TRANSMIT 2

#define SPI_RESPONSE_SEEN 1
#define SPI_RESPONSE_INPROGRESS 2
#define SPI_RESPONSE_COMPLETE 3

extern const uint8_t _x10_code[];

extern const uint8_t _x10_decode[];

extern const char *_x10_function[];

#define X10_FUNC_ALLUNITSOFF	12
#define X10_FUNC_ALLLIGHTSOFF 	0
#define X10_FUNC_ALLLIGHTSON	4
#define X10_FUNC_OFF			10
#define X10_FUNC_ON				2
#define X10_FUNC_DIM			14
#define X10_FUNC_BRIGHT			6
#define X10_FUNC_STATUSREQUEST	9
#define X10_FUNC_STATUSON		7
#define X10_FUNC_STATUSOFF		1
#define X10_FUNC_HAILREQUEST	13
#define X10_FUNC_HAILACK		5
#define X10_FUNC_PRESETDIM1		3
#define X10_FUNC_PRESETDIM2		11
#define X10_FUNC_EXTENDEDCODE	8
#define X10_FUNC_EXTENDEDDATA	15

struct x10_command {
	int hc;
	int uc;
	int fc;
	int addr_rpt;
	int func_rpt;
	int x_byte_1;
	int x_byte_2;
	int sticky;
};

extern void (*feed_bit_callback)(uint8_t);
extern void (*commit_x10_callback)(struct x10_command*);
void x10_decode_bit(uint8_t bit);
void prepare_x10_transmit(struct spi_message *msg, struct x10_command *p_cmd);
int reliable_spi_transfer(int fd, struct spi_message *spi_tx_message,
        struct spi_message *spi_rx_message, int target_code );
void spi_x10_poll(int fd);

void fail(const char *s);
void plog(int level, char *str, ...);
void pabort(const char *s);
void log_command(int level, struct x10_command *p_cmd);

#endif /* x10_spi_h */
