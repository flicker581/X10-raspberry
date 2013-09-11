/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

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

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 100000;
static uint32_t rspeed = 0;
static uint16_t delay;

// Maximum is 32 due to stream_tail size, but RAM restricts it further
#define X10_BITSTREAM_OCTETS 24

typedef struct __attribute__((__packed__)) _x10_bitstream {
 uint8_t data[X10_BITSTREAM_OCTETS];
 uint8_t tail; // pointer to the bit after the stream
} x10_bitstream_t;

typedef struct __attribute__((__packed__)) _spi_message {
 uint8_t rr_code;
 uint8_t rr_id;
 x10_bitstream_t x10_data;
 uint16_t crc16;
} spi_message_t;

#define SPI_REQUEST_POLL 0
#define SPI_REQUEST_CANCEL 1
#define SPI_REQUEST_TRANSMIT 2

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	int i;

	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

#define lo8(a) ((uint16_t)a&0xFF)
#define hi8(a) ((uint16_t)a >> 8)

uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)
    {
        data ^= lo8 (crc);
        data ^= data << 4;

        return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) 
                ^ ((uint16_t)data << 3));
    }

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
    {
        int i;

        crc = crc ^ ((uint16_t)data << 8);
        for (i=0; i<8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }

        return crc;
    }

static uint16_t u16_reverse(uint16_t word)
{
    uint16_t tmp;
    uint8_t i;
    for (i=16; i--;) {
        tmp <<= 1;
        if (word & 1)
            tmp |= 1;
        word >>= 1;
    }
    return tmp;
}

uint16_t spi_crc16(const spi_message_t *spi_buffer) 
{
	uint16_t crc = 0xffff;
	int i;

	for (i=0; i<sizeof(spi_message_t)-2; i++)
		crc = crc_ccitt_update(crc, ((uint8_t*)spi_buffer)[i]);

	return u16_reverse(crc);
}

/*
 * Concatenate bitstream b to a.
 * Returns: NULL if cannot concatenate;
 *          a if success
 */

x10_bitstream_t* x10concat( x10_bitstream_t *a, const x10_bitstream_t *b ) {

    uint16_t tmp;
    short b_tail;
    short dst_index, dst_shift, src_index;

    if (a->tail+b->tail > X10_BITSTREAM_OCTETS*8)
	return NULL;

    dst_index = a->tail / 8;
    dst_shift = a->tail % 8;
    src_index = 0;
    tmp = a->data[dst_index] << dst_shift; // meaningful bits now in MSB
    tmp &= 0xff00; // clear meaningless bits from a
    
    for (b_tail = b->tail; b_tail > 0; b_tail-=8) {
	tmp |= b->data[src_index++];
	a->data[dst_index++] = (tmp >> dst_shift) & 0xff;
	tmp <<= 8;
    }

    // save bits left in the buffer, if any
    if (dst_shift + b_tail > 0) {
	a->data[dst_index] = (tmp >> dst_shift) & 0xff;
    }
    
    a->tail += b->tail;
    
    return a;
}

const uint8_t _x10_code[16] = {
    0b0110, // A, 1, All Lights Off
    0b1110, // B, 2, Status = off
    0b0010, // C, 3, On
    0b1010, // D, 4, Pre-set Dim 1
    0b0001, // E, 5, All Lights On
    0b1001, // F, 6, Hail Acknowledge
    0b0101, // G, 7, Bright
    0b1101, // H, 8, Status = on
    0b0111, // I, 9, Extended code
    0b1111, // J, 10, Status Request
    0b0011, // K, 11, Off
    0b1011, // L, 12, Pre-set Dim 2
    0b0000, // M, 13, All Units Off
    0b1000, // N, 14, Hail Request
    0b0100, // O, 15, Dim
    0b1100, // P, 16, Extended Data (analog)
    };

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


/*
 * Add basic code to existing bitstream.
 * Returns: NULL if bitstream is full,
 *			bs on success
 */

x10_bitstream_t* x10_basic( x10_bitstream_t* bs, uint8_t hc, uint8_t uc, uint8_t is_function ) {

    short bs_tail = bs->tail;
    uint16_t tmp;
    short dst_index, dst_shift;

    if (bs_tail + 22 > X10_BITSTREAM_OCTETS*8)
	return NULL;
    
    dst_index = bs_tail / 8;
    dst_shift = bs_tail % 8;

    tmp = bs->data[dst_index] << dst_shift; // meaningful bits now in MSB
    tmp &= 0xff00; // clear meaningless bits
    tmp |= 0b11100101; // prepare for encoding

    if (_x10_code[hc] & 0b1000)
	tmp ^= 0b00001100;
    if (_x10_code[hc] & 0b0100)
	tmp ^= 0b00000011;

    bs->data[dst_index++] = (tmp >> dst_shift) & 0xff;
    tmp <<= 8;
    tmp |= 0b01010101;

    if (_x10_code[hc] & 0b0010)
	tmp ^= 0b11000000;
    if (_x10_code[hc] & 0b0001)
	tmp ^= 0b00110000;
	
    if (_x10_code[uc] & 0b1000)
	tmp ^= 0b00001100;
    if (_x10_code[uc] & 0b0100)
	tmp ^= 0b00000011;

    bs->data[dst_index++] = (tmp >> dst_shift) & 0xff;
    tmp <<= 8;
    tmp |= 0b01010100;
    
    if (_x10_code[uc] & 0b0010)
	tmp ^= 0b11000000;
    if (_x10_code[uc] & 0b0001)
	tmp ^= 0b00110000;
    if (is_function)
	tmp ^= 0b00001100;
	
    bs->data[dst_index++] = (tmp >> dst_shift) & 0xff;
    tmp <<= 8;

    if (dst_shift - 2 > 0)
	bs->data[dst_index] = (tmp >> dst_shift) & 0xff;
    
    bs->tail += 22;

    return bs;

}

/*
 * Add pause to existing bitstream.
 * Returns: NULL if bitstream is full,
 *			bs on success
 */

x10_bitstream_t* x10_pause( x10_bitstream_t* bs, unsigned short bits ) {

    short bs_tail = bs->tail;
    short dst_index, dst_shift;
    short bits_left;

    if (bs_tail + bits > X10_BITSTREAM_OCTETS*8)
	return NULL;
    
    dst_index = bs_tail / 8;
    dst_shift = bs_tail % 8;

    // Clean pause bits
    bs->data[dst_index++] &= ~((1<<(8-dst_shift))-1);  

    for (bits_left=bits-8+dst_shift; bits_left>0 ; bits_left-=8) {
	bs->data[dst_index++] = 0;
    }

    bs->tail += bits;

    return bs;

}

void print_spi_message( const spi_message_t* spi_message)
{
	int j;

	printf("=============================================\n");
	printf("rr code = %hhu\n", spi_message->rr_code);
	printf("rr id   = %hhu\n", spi_message->rr_id);
	printf("x10 data:\n");
	for (j = 0; j < sizeof(spi_message->x10_data.data)*8; j++) {
		printf("%c", (j==spi_message->x10_data.tail) ? ' ' :
			((spi_message->x10_data.data[j/8])&(1<<(7-j%8))) ? '1' : '0');
		if (!((j+1) % 48))
			printf("\n");
	}
	printf("tail    = %hhu\n", spi_message->x10_data.tail);
	printf("crc     = %.4X/%.4X\n", spi_message->crc16, spi_crc16(spi_message));
	printf("================================================\n");
        for (j = 0; j < sizeof(spi_message_t); j++) {
                if (!(j % 6))
                        puts("");
                printf("%.2X ", ((uint8_t*)spi_message)[j]);
        }
        puts("");
}

static void spi_transfer(int fd, const spi_message_t *spi_tx_message,
	spi_message_t *spi_rx_message)
{
	int ret;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)spi_tx_message,
		.rx_buf = (unsigned long)spi_rx_message,
		.len = sizeof(spi_message_t),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int init(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rspeed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", rspeed, rspeed/1000);
	return fd;
}

void prepare_x10_transmit(spi_message_t *spi_message, char* cmd)
{
	char *cmd_ptr;
	int hc = -1, uc = -1, fc = -1;
	int continuous = 0;
	char x;

	for (cmd_ptr = cmd; *cmd_ptr; cmd_ptr++)
		*cmd_ptr = tolower(*cmd_ptr);
	for (cmd_ptr = cmd; *cmd_ptr; cmd_ptr++) {
		x = *cmd_ptr;
		if ( x>='a' && x<='p' ) {
			// house code
			if ( hc != -1 )
				pabort("House code already set");
			hc = x - 'a';
		}
		else if ( x >= '0' && x <= '9' ) {
			// unit code
			if ( uc != -1 )
				pabort("Unit code already set");
			uc = x - '0';
			while ( *(cmd_ptr+1) >='0' && *(cmd_ptr+1) <= '9' ) {
				uc = uc * 10 + *(++cmd_ptr) - '0';
			}
			uc -= 1;
			if (uc < 0 || uc > 15)
				pabort("Unit code out of bounds");
		}
		else if ( x == ':' ) {
			++cmd_ptr;
			if (strcmp(cmd_ptr, "allunitsoff") == 0) {
				fc = X10_FUNC_ALLUNITSOFF;
			} else
			if (strcmp(cmd_ptr, "alllightsoff") == 0) {
				fc = X10_FUNC_ALLLIGHTSOFF;
			} else
			if (strcmp(cmd_ptr, "alllightson") == 0) {
				fc = X10_FUNC_ALLLIGHTSON;
			} else
			if (strcmp(cmd_ptr, "off") == 0) {
				fc = X10_FUNC_OFF;
			} else
			if (strcmp(cmd_ptr, "on") == 0) {
				fc = X10_FUNC_ON;
			} else
			if (strcmp(cmd_ptr, "dim") == 0) {
				fc = X10_FUNC_DIM;
			} else
			if (strcmp(cmd_ptr, "bright") == 0) {
				fc = X10_FUNC_BRIGHT;
			} else
			if (strcmp(cmd_ptr, "microdim") == 0) {
				fc = X10_FUNC_DIM;
				continuous = 1;
			} else
			if (strcmp(cmd_ptr, "microbright") == 0) {
				fc = X10_FUNC_BRIGHT;
				continuous = 1;
			} else
			if (strcmp(cmd_ptr, "status") == 0) {
				fc = X10_FUNC_STATUSREQUEST;
			} else
			if (strcmp(cmd_ptr, "hail") == 0) {
				fc = X10_FUNC_HAILREQUEST;
			} else {
				pabort("Command not understood");
			}
			break;
		}
	}

	printf ("hc = %d, uc = %d, fc = %d\n", hc, uc, fc);

	memset(spi_message, 0, sizeof(*spi_message));
	spi_message->rr_code = SPI_REQUEST_TRANSMIT;

	if ( hc == -1 ) {
	    printf("Warning: house code not set, cannot encode the message\n");
	    return;
	}

	if ( uc!=-1 ) {
		if (!x10_basic(&spi_message->x10_data, hc, uc, 0))
			pabort("Failed to encode command");
		if (!x10_basic(&spi_message->x10_data, hc, uc, 0))
			pabort("Failed to encode command");
		if (!x10_pause(&spi_message->x10_data, 6))
			pabort("Failed to encode command");
	}
	if ( fc!=-1 ) {
		if (!x10_basic(&spi_message->x10_data, hc, fc, 1))
			pabort("Failed to encode command");
		if (!continuous)
			{
			if (!x10_basic(&spi_message->x10_data, hc, fc, 1))
				pabort("Failed to encode command");
			if (!x10_pause(&spi_message->x10_data, 6))
				pabort("Failed to encode command");
			}
	}

}

#define MAX_SPI_TRIES 10

int reliable_spi_transfer(int fd, spi_message_t *spi_tx_message,
	spi_message_t *spi_rx_message)
{
	int try;
	spi_message_t spi_poll_message;
	struct timespec ts_rq;

	memset(&spi_poll_message, 0, sizeof(spi_poll_message));
	// Just poll and receive rr_id
	for (try=MAX_SPI_TRIES; try>0; --try)
	{
		spi_transfer(fd, &spi_poll_message, spi_rx_message);
		printf("<<<");
		print_spi_message(spi_rx_message);
		if (spi_crc16(spi_rx_message) == spi_rx_message->crc16 ) {
		    break;
		}
	}
	if ( try < MAX_SPI_TRIES )
		printf("Warning: %d poll tries have failed\n", MAX_SPI_TRIES - try);

	if ( try == 0 || spi_tx_message == NULL )
		return try;

	// Use the rr_id we just received
	spi_tx_message->rr_id = (spi_rx_message->rr_id+1) % 256;
	spi_tx_message->crc16 = spi_crc16(spi_tx_message);

	for (try=MAX_SPI_TRIES; try>0; --try)
	{
		printf(">>>");
		print_spi_message(spi_tx_message);
		spi_transfer(fd, spi_tx_message, spi_rx_message);
		printf("<<<");
		print_spi_message(spi_rx_message);

		ts_rq.tv_sec = 0;
		ts_rq.tv_nsec = 1000000L; // Allow 1 ms for processing
		while(nanosleep(&ts_rq, &ts_rq));
		// Poll now
		spi_transfer(fd, &spi_poll_message, spi_rx_message);
		printf("<<<");
		print_spi_message(spi_rx_message);
		// Check if rr_id is known to Tiny now
		if (spi_crc16(spi_rx_message) == spi_rx_message->crc16 
			&& spi_rx_message->rr_id == spi_tx_message->rr_id) {
		    break;
		}
	}
	if ( try < MAX_SPI_TRIES ) {
		printf("Warning: %d trx tries have failed\n", MAX_SPI_TRIES - try);
	}
	return try;
}


int main(int argc, char *argv[])
{
	int fd;
	spi_message_t spi_tx_message;
	spi_message_t spi_rx_message;
	int ret;


	fd = init(argc, argv);

	while (optind<argc) {

		printf("Processing command: %s\n", argv[optind]);

		if (strcmp(argv[optind], "poll") == 0) {
			ret = reliable_spi_transfer(fd, NULL, &spi_rx_message);
			if (!ret)
				printf("Poll has failed!\n");
			else
				printf("Poll has succeeded, the result follows\n");
			print_spi_message(&spi_rx_message);
		} else {
			prepare_x10_transmit(&spi_tx_message, argv[optind]);

			ret = reliable_spi_transfer(fd, &spi_tx_message, &spi_rx_message);
			if (!ret)
				printf("Transaction has failed!\n");
			else
				printf("Transaction has succeeded\n");
			print_spi_message(&spi_rx_message);

		}

		optind++;
	}

	close(fd);

	return 0;
}

