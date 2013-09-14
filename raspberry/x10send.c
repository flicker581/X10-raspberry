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

static void fail(const char *s)
{
	fprintf(stderr, "Fatal error: %s\n", s);
	abort();
}

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 130000;
static uint32_t rspeed = 0;
static uint16_t delay;

static int verbosity = 0;

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

#define SPI_RESPONSE_SEEN 1
#define SPI_RESPONSE_INPROGRESS 2
#define SPI_RESPONSE_COMPLETE 3

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
	for (i=0; i<8; i++) {
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

x10_bitstream_t* x10concat( x10_bitstream_t *a, const x10_bitstream_t *b )
{
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

const uint8_t _x10_decode[16] = {
	12, 4, 2, 10, 14, 6, 0, 8, 13, 5, 3, 11, 15, 7, 1, 9,
};

const char *_x10_function[16] = {
	"AllLightsOff",
	"StatusOff",
	"On",
	"PresetDim1",
	"AllLightsOn",
	"HailAck",
	"Bright",
	"StatusOn",
	"ExtendedCode",
	"Status",
	"Off",
	"PresetDim2",
	"AllUnitsOff",
	"Hail",
	"Dim",
	"ExtendedData",
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

x10_bitstream_t* x10_basic( x10_bitstream_t* bs, uint8_t hc, uint8_t uc, uint8_t is_function )
{
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
 * Add two extended bytes to existing bitstream.
 * Returns: NULL if bitstream is full,
 *			bs on success
 */

x10_bitstream_t* x10_extended_code( x10_bitstream_t* bs, uint8_t uc, uint8_t byte1, uint8_t byte2 )
{
	short bs_tail = bs->tail;
	uint16_t tmp;
	short dst_index, dst_shift;
	uint32_t word = (_x10_code[uc]<<16) + (byte1 << 8) + byte2 ;
	short i;
	uint32_t bit;

	if (bs_tail + 40 > X10_BITSTREAM_OCTETS*8)
		return NULL;

	dst_index = bs_tail / 8;
	dst_shift = bs_tail % 8;

	tmp = bs->data[dst_index] >> (8-dst_shift);
	for (i=40; i>0; i--) {
		bit = 1 << (i-1)/2;
		tmp = (tmp << 1) ^ ((word&bit) ? 1 : 0) ^ (i&1);
		if (++dst_shift == 8) {
			bs->data[dst_index] = tmp;
			dst_index++;
			dst_shift = 0;
		}
	}

	if (dst_shift > 0)
		bs->data[dst_index] = tmp << (8-dst_shift);

	bs->tail += 40;

	return bs;
}

/*
 * Add pause to existing bitstream.
 * Returns: NULL if bitstream is full,
 *			bs on success
 */

x10_bitstream_t* x10_pause( x10_bitstream_t* bs, unsigned short bits )
{

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
	short j;

	if (spi_message->crc16 != spi_crc16(spi_message))
		printf("= SPI message CORRUPTED ========================\n");
	else
		printf("= SPI message ==================================\n");
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
	printf("hex dump:\n");
        for (j = 0; j < sizeof(spi_message_t); j++) {
                if ( j>0 && (j % 15) == 0 )
                        printf("\n");
                printf("%.2X ", ((uint8_t*)spi_message)[j]);
        }
        printf("\n");
	printf("= SPI message end ==============================\n");
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

	if ( verbosity >= 1)
		printf("******************* SPI transfer ********************\n");

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3] command ...\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -N --no-cs    disable chip select\n"
	     "  -R --ready    use SPI ready input\n"
	     "  -v --verbose  increase verbosity level\n"
);
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
			{ "verbose", 0, 0, 'v' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NRv", lopts, NULL);

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
		case 'v':
			verbosity++;
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
	if ( verbosity >= 2 ) {
		printf("spi mode: %d\n", mode);
		printf("bits per word: %d\n", bits);
		printf("max speed: %d Hz (%d KHz)\n", rspeed, rspeed/1000);
	}
	return fd;
}

struct a_command {
	int hc;
	int uc;
	int fc;
	int addr_rpt;
	int func_rpt;
	int x_byte_1;
	int x_byte_2;
	int sticky;
	} a_cmd;

void print_command( struct a_command *p_cmd )
{

	printf( "= Command ======================================\n" );
	printf( "HC = %c\n", 'A' + p_cmd->hc );
	if (p_cmd->addr_rpt 
		|| (p_cmd->func_rpt && p_cmd->fc == X10_FUNC_EXTENDEDCODE))
		printf( "UC = %d\n", p_cmd->uc + 1 );
	if (p_cmd->func_rpt)
		printf("Function = %s\n", _x10_function[p_cmd->fc]);
	printf( "Address repeats = %d, function repeats = %d\n", p_cmd->addr_rpt, p_cmd->func_rpt );
	if (p_cmd->func_rpt && p_cmd->fc == X10_FUNC_EXTENDEDCODE)
		printf( "Extended byte 1 = 0x%.2X(%d), byte 2 = 0x%.2X(%d)\n", 
			p_cmd->x_byte_1, p_cmd->x_byte_1, 
			p_cmd->x_byte_2, p_cmd->x_byte_2 ); 
	if ( p_cmd->sticky )
		printf( "The command is sticky\n" );
	printf( "= End of command ===============================\n" );
}

int parse_decimal(char **ptr)
{
	int x = 0;

	while (isdigit(**ptr)) {
		x = x*10 + **ptr - '0';
		*ptr += 1;
	}
	return x;
}

void parse_command(const char* orig_cmd, struct a_command* p_cmd)
{
	char *cmd;
	char *c_ptr, *c_ptr_e;
	int x; // temporary number
	int has_uc;

	cmd = strdup( orig_cmd );
	p_cmd->hc = p_cmd->uc = p_cmd->fc = -1;
	p_cmd->addr_rpt = p_cmd->func_rpt = 0;
	p_cmd->sticky = 0;
	p_cmd->x_byte_1 = p_cmd->x_byte_2 = 0;

	for (c_ptr = cmd; *c_ptr; c_ptr++)
		*c_ptr = tolower(*c_ptr);

	c_ptr = cmd;
	c_ptr_e = strchr( cmd, ':' );
	if (c_ptr_e != NULL && c_ptr < c_ptr_e ) {
		// we have address in the command
		if ( *c_ptr >= 'a' && *c_ptr <= 'p' ) {
			p_cmd->hc = *c_ptr - 'a';
		} else {
			fail("X10 address should begin with HC");
		}

		x = has_uc = 0;
		while (++c_ptr < c_ptr_e) {
			if ( isdigit(*c_ptr) ) {
				x = x * 10 + *c_ptr - '0';
			} else {
				fail("X10 unit number should be a number");
			}
			has_uc = 1;
		}
		x -= 1;
		if (has_uc) {
			if (x >=0 && x <= 15) {
				p_cmd->uc = x;
				p_cmd->addr_rpt = 2;
			} else {
				fail("Unit code out of bounds [1..16]");
			}
		}
		++c_ptr;
	}
	if ( *c_ptr ) {
		// has a function
		p_cmd->func_rpt = 2;
		if (strcmp(c_ptr, "allunitsoff") == 0) {
			p_cmd->fc = X10_FUNC_ALLUNITSOFF;
		} else if (strcmp(c_ptr, "alllightsoff") == 0) {
			p_cmd->fc = X10_FUNC_ALLLIGHTSOFF;
		} else if (strcmp(c_ptr, "alllightson") == 0) {
			p_cmd->fc = X10_FUNC_ALLLIGHTSON;
		} else if (strcmp(c_ptr, "off") == 0) {
			p_cmd->fc = X10_FUNC_OFF;
		} else if (strcmp(c_ptr, "on") == 0) {
			p_cmd->fc = X10_FUNC_ON;
		} else if (strcmp(c_ptr, "dim") == 0) {
			p_cmd->fc = X10_FUNC_DIM;
		} else if (strcmp(c_ptr, "bright") == 0) {
			p_cmd->fc = X10_FUNC_BRIGHT;
		} else if (strcmp(c_ptr, "microdim") == 0) {
			p_cmd->fc = X10_FUNC_DIM;
			p_cmd->func_rpt = 1;
			p_cmd->sticky = 1;
		} else if (strcmp(c_ptr, "microbright") == 0) {
			p_cmd->fc = X10_FUNC_BRIGHT;
			p_cmd->func_rpt = 1;
			p_cmd->sticky = 1;
		} else if (strcmp(c_ptr, "status") == 0) {
			p_cmd->fc = X10_FUNC_STATUSREQUEST;
		} else if (strcmp(c_ptr, "statuson") == 0) {
			p_cmd->fc = X10_FUNC_STATUSON;
		} else if (strcmp(c_ptr, "statusoff") == 0) {
			p_cmd->fc = X10_FUNC_STATUSOFF;
		} else if (strcmp(c_ptr, "hail") == 0) {
			p_cmd->fc = X10_FUNC_HAILREQUEST;
		} else if (strcmp(c_ptr, "hailack") == 0) {
			p_cmd->fc = X10_FUNC_HAILACK;
		} else if (strncmp(c_ptr, "xpreset[", 8) == 0) {
			c_ptr += 8;
			x = parse_decimal(&c_ptr);
			if ( x<0 || x>63 )
				fail("Xpreset value not in range [0..63]");
			if (strcmp(c_ptr, "]") != 0)
				fail("Xpreset command malformed");
			p_cmd->fc = X10_FUNC_EXTENDEDCODE;
			p_cmd->x_byte_2 = 0x31; // XPreset code
			p_cmd->x_byte_1 = x; // XPreset code
			p_cmd->addr_rpt = 0; // no need to address
		} else {
			fail("Command not understood");
		}
	}
	free( cmd );
}

void prepare_x10_transmit(spi_message_t *spi_message, char* cmd)
{
	int i;

	parse_command(cmd, &a_cmd);

	if ( verbosity >= 1 )
		print_command( &a_cmd );

	//printf ("hc = %d, uc = %d, fc = %d\n", hc, uc, fc);

	memset(spi_message, 0, sizeof(*spi_message));
	spi_message->rr_code = SPI_REQUEST_TRANSMIT;

	if ( a_cmd.hc == -1 )
	    fail("House code not set");

	if ( a_cmd.uc == -1 && a_cmd.fc == -1 )
	    fail("Unit code or a function need to be set");

	for ( i = a_cmd.addr_rpt; i>0; --i )
		if (!x10_basic(&spi_message->x10_data, a_cmd.hc, a_cmd.uc, 0))
			fail("Failed to encode command");

	if ( a_cmd.addr_rpt )
		if (!x10_pause(&spi_message->x10_data, 6))
			fail("Failed to encode command");

	for ( i = a_cmd.func_rpt; i>0; --i )
		if ( a_cmd.fc == X10_FUNC_EXTENDEDCODE ) {
			if ( a_cmd.uc == -1 )
				fail("Extended command needs unit code");
			if (!x10_basic(&spi_message->x10_data, a_cmd.hc, a_cmd.fc, 1))
				fail("Failed to encode command");
			if (!x10_extended_code(&spi_message->x10_data, a_cmd.uc, a_cmd.x_byte_1, a_cmd.x_byte_2))
				fail("Failed to encode command");
		}
		else {
			if (!x10_basic(&spi_message->x10_data, a_cmd.hc, a_cmd.fc, 1))
				fail("Failed to encode command");
		}
	if (a_cmd.func_rpt && !a_cmd.sticky)
		if (!x10_pause(&spi_message->x10_data, 6))
			fail("Failed to encode command");

}

#define MAX_SPI_TRIES 10

int checked_spi_receive(int fd, spi_message_t *spi_rx_message)
{
	int try;
	spi_message_t spi_poll_message;

	memset(&spi_poll_message, 0, sizeof(spi_poll_message));
	for (try=MAX_SPI_TRIES; try>0; --try)
	{
		spi_transfer(fd, &spi_poll_message, spi_rx_message);
		if (spi_crc16(spi_rx_message) == spi_rx_message->crc16 ) {
			break;
		}
		if ( verbosity >= 1 ) {
			printf("<<< Corrupted incoming message <<<\n");
			print_spi_message(spi_rx_message);
		}
	}
	if ( try > 0 )
	{
		if ( verbosity >= 2 ) {
			printf("<<< Incoming message <<<\n");
			print_spi_message(spi_rx_message);
		}
	}
	return try;
}

int reliable_spi_transfer(int fd, spi_message_t *spi_tx_message,
	spi_message_t *spi_rx_message, int target_code )
{
	int try;
	spi_message_t spi_poll_message;
	struct timespec ts_rq;

	memset(&spi_poll_message, 0, sizeof(spi_poll_message));
	// Just poll and receive rr_id
	try = checked_spi_receive(fd, spi_rx_message);

	if ( try < MAX_SPI_TRIES && verbosity >= 1 )
		printf("Warning: %d poll tries have failed\n", MAX_SPI_TRIES - try);

	if ( try == 0 || spi_tx_message == NULL )
		return try;

	// Use the rr_id we just received
	spi_tx_message->rr_id = (spi_rx_message->rr_id+1) % 256;
	spi_tx_message->crc16 = spi_crc16(spi_tx_message);

	for (try = MAX_SPI_TRIES+1; try>0; --try)
	{
		if ( verbosity >= 2 ) {
			printf(">>> Outgoing message >>>\n");
			print_spi_message(spi_tx_message);
		}
		spi_transfer(fd, spi_tx_message, spi_rx_message);
		if ( verbosity >= 2 ) {
			printf("<<< Incoming message <<<\n");
			print_spi_message(spi_rx_message);
		}

		// Check if rr_id is known to Tiny now
		if (spi_crc16(spi_rx_message) == spi_rx_message->crc16 
			&& spi_rx_message->rr_id == spi_tx_message->rr_id) {
			break;
		}

		ts_rq.tv_sec = 0;
		ts_rq.tv_nsec = 1000000L; // Allow 1 ms for processing
		while(nanosleep(&ts_rq, &ts_rq));
	}

	if ( try < MAX_SPI_TRIES && verbosity >= 1 ) {
		printf("Warning: %d trx tries have failed\n", MAX_SPI_TRIES - try);
	}

	if ( try == 0 )
		return 0;

	while ( spi_rx_message->rr_code < target_code ) {
		ts_rq.tv_sec = 0;
		ts_rq.tv_nsec = 200000000L; // Allow 200 ms for processing
		while(nanosleep(&ts_rq, &ts_rq));
		// Poll now
		try = checked_spi_receive(fd, spi_rx_message);
		if (try == 0)
			return 0;
		// Check if final code has been reached
		if (spi_rx_message->rr_id != spi_tx_message->rr_id) {
			printf("Strange thing has happened, wrong rr_id received");
			break;
		}
	}

	return try;
}

void x10_feed_bit(uint8_t bit)
{
	static int pos = 0;
	printf("%d", bit);
	if (++pos == 48) {
		printf("\n");
		pos = 0;
	}
	fflush(stdout);
}

#define X10_STATE_IDLE 0
#define X10_STATE_BASIC 1
#define X10_STATE_EXTENDED 2
#define X10_STATE_RECOVER 3
#define X10_STATE_RECEIVED 4

int x10_deinterleave(uint32_t buf, uint8_t bits)
{
	int tmp = 0;
	int check = 0;
	int i;

	for (i = bits ; i > 0; --i) {
		tmp <<= 1;
		check <<= 1;
		if (buf & (1L << (i * 2 - 1)))
			tmp += 1;
		if (!(buf & (1L << (i * 2 - 2))))
			check += 1;
		if (check != tmp)
			return -1;
	}
	return tmp;
}

void x10_decode_bit(uint8_t bit)
{
	static int x10_state = X10_STATE_IDLE;
	static uint32_t buf = 0;
	int tmp;
	static uint32_t rbuf, last_rbuf;
	static struct a_command a_cmd;
	static int counter = 0;
	int commit_command = 0;
	static int repeats = 0;

	if (verbosity >=2)
		x10_feed_bit(bit);

	buf = (buf << 1) + bit;
	counter++;

	if (x10_state != X10_STATE_IDLE && (buf & 0b111111) == 0) {
		if (verbosity >= 1)
			printf("Force return to idle state\n");
		x10_state = X10_STATE_IDLE;
		buf = 0;
	}

	switch (x10_state) {
	case X10_STATE_IDLE:
		if (last_rbuf && counter == 5)
			commit_command = 1;
		if ((buf & 0xF) != 0xE)
			break;
		if (verbosity >= 1)
			printf("Start condition detected\n");
		counter = 0;
		rbuf = 0;
		x10_state = X10_STATE_BASIC;
		break;
	case X10_STATE_BASIC:
		if (counter % 2)
			break;
		tmp = x10_deinterleave(buf, 1);
		if (tmp == -1) {
			if (verbosity >= 1)
				printf("The transmission is invalid\n");
			x10_state = X10_STATE_RECOVER;
			break;
		}
		rbuf = (rbuf << 1) + tmp;
		if (counter < 18)
			break;
		if ((rbuf & 1) && _x10_decode[(rbuf >> 1) & 0xF] 
			== X10_FUNC_EXTENDEDCODE) {
			x10_state = X10_STATE_EXTENDED;
			break;
		}
		rbuf <<= 20;
		x10_state = X10_STATE_RECEIVED;
		break;
	case X10_STATE_EXTENDED:
		if (counter % 2)
			break;
		tmp = x10_deinterleave(buf, 1);
		if (tmp == -1) {
			if (verbosity >= 1)
				printf("The transmission is invalid\n");
			x10_state = X10_STATE_RECOVER;
			break;
		}
		rbuf = (rbuf << 1) + tmp;
		if (counter < 58)
			break;
		x10_state = X10_STATE_RECEIVED;
		break;
	}
	if (x10_state == X10_STATE_RECEIVED) {
		if (verbosity >= 1)
			printf("The received code seems valid: %.8X\n", rbuf);
		rbuf |= 1<<31; // there is a command in the rbuf
		if (last_rbuf) {
			if (last_rbuf == rbuf) {
				repeats++;
				if (verbosity >= 1)
					printf("The same as before!\n");
			} else {
				commit_command = 1;
				repeats = 1;
			}
		} else {
			repeats = 1;
		}
	}

	if (last_rbuf && x10_state == X10_STATE_RECOVER)
		commit_command = 1;

	if (commit_command) {
		if (verbosity >= 1)
			printf("Committing the command!\n");
		memset(&a_cmd, 0, sizeof(a_cmd));
		a_cmd.hc=_x10_decode[(last_rbuf >> 25) & 0xF];
		if ((last_rbuf >> 20) & 1) {
			a_cmd.fc = _x10_decode[(last_rbuf >> 21) & 0xF];
			a_cmd.func_rpt = repeats;
		} else {
			a_cmd.uc = _x10_decode[(last_rbuf >> 21) & 0xF];
			a_cmd.addr_rpt = repeats;
		}
		if (a_cmd.fc == X10_FUNC_EXTENDEDCODE) {
			a_cmd.uc = _x10_decode[(last_rbuf >> 16) & 0xF];
			a_cmd.x_byte_1 = (last_rbuf >> 8) & 0xFF;
			a_cmd.x_byte_2 = last_rbuf & 0xFF;
		}
		print_command(&a_cmd);
		last_rbuf = 0;
		repeats = 0;
	}
	if (x10_state == X10_STATE_RECEIVED) {
		last_rbuf = rbuf;
		buf = 0;
		counter = 0;
		x10_state = X10_STATE_IDLE;
	}
}

void spi_x10_listen(int fd, void (*feed_bit)(uint8_t))
{
	spi_message_t spi_rx;
	int ret;
	struct timespec ts_rq;
	int rx_tail = -1;
	uint8_t bit;

	while (1) {
		ret = reliable_spi_transfer(fd, NULL, &spi_rx, 0);
		if (!ret)
			fail("SPI receive has failed");
		if (verbosity >= 1)
			print_spi_message(&spi_rx);
		if (rx_tail == -1) {
			// feed the whole buffer
			rx_tail = spi_rx.x10_data.tail+1;
		}
		while (rx_tail != spi_rx.x10_data.tail) {
			bit = (spi_rx.x10_data.data[rx_tail/8] 
				>> (7 - rx_tail % 8)) & 1;
			(*feed_bit)(bit);
			if(++rx_tail == X10_BITSTREAM_OCTETS*8)
				rx_tail = 0;
		}

		ts_rq.tv_sec = 0;
		ts_rq.tv_nsec = 100000000L; // 100ms
		while(nanosleep(&ts_rq, &ts_rq));
	}

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
			ret = reliable_spi_transfer(fd, NULL, &spi_rx_message, 0);
			if (!ret)
				printf("Poll has failed!\n");
			else
				printf("Poll has succeeded, the result follows\n");
			print_spi_message(&spi_rx_message);
		} else if (strcmp(argv[optind], "listenraw") == 0) {
			spi_x10_listen(fd, &x10_feed_bit);
		} else if (strcmp(argv[optind], "listen") == 0) {
			spi_x10_listen(fd, &x10_decode_bit);
		} else {
			prepare_x10_transmit(&spi_tx_message, argv[optind]);

			ret = reliable_spi_transfer(fd, &spi_tx_message, &spi_rx_message, SPI_RESPONSE_INPROGRESS);
			if (!ret)
				printf("Transaction has failed!\n");
			else
				printf("Transaction has succeeded\n");

		}

		optind++;
	}

	close(fd);

	return 0;
}

