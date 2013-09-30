/*
 * X10 control via SPI, Linux part of the picture.
 *
 * CM11 emulator code.
 *
 * Copyright (c) 2013 pavel@levshin.spb.ru
 *
 */


#include <time.h>

#include "x10-spi.h"
#include "cm11.h"

#define CM11_WBUF_OCTETS 10

static uint8_t cm11_cbuf[CM11_WBUF_OCTETS];
static int cm11_has_cbuf = 0;
static int cm11_fresh_rbuf = 0;
static uint8_t cm11_rbuf[100];
static uint8_t cm11_wbuf[20];
static int cm11_wbuf_bytes, cm11_rbuf_bytes;

enum cm11_state {
	cm11_state_ready,
	cm11_state_tx_ack,
	cm11_state_rx_poll,
};

static int cm11_command_parse(uint8_t *buf, int bytes, struct x10_command *p_cmd)
{
	uint8_t hdr;
	uint8_t code;
	int dims;
	int is_function;
	int is_extended;
	int length = 2;

	memset(p_cmd, 0, sizeof(*p_cmd));
	hdr = *buf;
	if (bytes < 2)
		return 0; // too few data in the buffer
	if ((hdr & 0x04) == 0)
		return -1; // this cannot be a transfer
	dims = (hdr >> 3) & 0x1F;
	is_function = hdr & 0x02;
	is_extended = hdr & 0x01;
	code = *(buf+1);
	p_cmd->hc = _x10_decode[(code >> 4) & 0xF];
	if (is_function) {
		p_cmd->fc = _x10_decode[code & 0xF];
		if (p_cmd->fc == X10_FUNC_DIM || p_cmd->fc == X10_FUNC_BRIGHT)
			p_cmd->func_rpt = dims;
		else
			p_cmd->func_rpt = 2;
	} else {
		p_cmd->uc = _x10_decode[code & 0xF];
		p_cmd->addr_rpt = 2;
	}
	if (is_extended) {
		if (bytes < 5)
			return 0; // too few data in the buffer
		p_cmd->uc = _x10_decode[*(buf+2) & 0xF];
		p_cmd->x_byte_1 = *(buf+3);
		p_cmd->x_byte_2 = *(buf+4);
		length = 5;
	}
	return length;
}

static void cm11_command_tobuffer(struct x10_command *p_cmd, uint8_t *wbuf)
{
	int i, j;
	int dimlevel;

	i = wbuf[0];
	if (i == 0)
		i = 1;

	if (i + 1 > CM11_WBUF_OCTETS - 1)
		return;
	if (p_cmd->addr_rpt) {
		wbuf[++i] = (_x10_code[p_cmd->hc] << 4)
			+ _x10_code[p_cmd->uc];
	}
	if (p_cmd->func_rpt) {
		wbuf[++i] = (_x10_code[p_cmd->hc] << 4)
			+ _x10_code[p_cmd->fc];
		wbuf[1] |= 1 << (i - 2);
		switch (p_cmd->fc) {
		case X10_FUNC_DIM:
		case X10_FUNC_BRIGHT:
			if (i + 1 > CM11_WBUF_OCTETS - 1)
				return;
			// 1 -> 2.5, >=2 -> 13,5*(i-1)
			dimlevel = (p_cmd->func_rpt - 1) * 11 + 3;
			wbuf[++i] = (dimlevel < 210) ? dimlevel : 210;
			break;
		case X10_FUNC_EXTENDEDCODE:
			if (i + 3 > CM11_WBUF_OCTETS - 1)
				return;
			wbuf[++i] = _x10_code[p_cmd->uc];
			wbuf[++i] = p_cmd->x_byte_1;
			wbuf[++i] = p_cmd->x_byte_2;
			break;
		}
	}
	wbuf[0] = i;
	plog(1, "Ready to send X10 command to PC [");
	for (j = 0; j < i + 1; j++)
		plog(1, "%.2X ", wbuf[j]);
	plog(1, "]\n");

}

static void cm11_x10_receive(struct x10_command *p_cmd)
{
	plog(1, "CM11 have received a command from PLC\n");
	cm11_command_tobuffer(p_cmd, cm11_cbuf);
	cm11_has_cbuf = 1;
}

static void cm11_init(void)
{
	feed_bit_callback = &x10_decode_bit;
	commit_x10_callback = &cm11_x10_receive;
	memset(cm11_rbuf, 0, sizeof(cm11_rbuf));
	memset(cm11_cbuf, 0, sizeof(cm11_cbuf));
	cm11_wbuf_bytes = 0;
	cm11_rbuf_bytes = 0;
}


static uint8_t cm11_checksum(uint8_t *buf, int bytes)
{
	uint8_t cs = 0;
	int i;
	
	for (i=0; i<bytes; i++)
		cs += buf[i];

	return cs;
}

static void cm11_execute(int fd, struct x10_command *p_cmd)
{
	int ret, i;
	struct spi_message spi_tx_message, spi_rx_message;
	int repetitions = 0;

	if (p_cmd->func_rpt > 2) {
		// This is a special case, likely DIM or BRIGHT
		plog(1, "Splitting the command to multiple transmissions\n");
		repetitions = p_cmd->func_rpt;
		p_cmd->func_rpt = 1;
		p_cmd->sticky = 1;
		prepare_x10_transmit(&spi_tx_message, p_cmd);
		for (i = repetitions; i > 1; --i) {
			ret = reliable_spi_transfer(fd, &spi_tx_message,
				&spi_rx_message, SPI_RESPONSE_INPROGRESS);
			if (!ret)
				plog(0, "SPI transaction has failed!\n");
			else
				plog(1, "SPI transaction has succeeded\n");
		}
	}
	// final transfer
	p_cmd->sticky = 0;
	prepare_x10_transmit(&spi_tx_message, p_cmd);
	ret = reliable_spi_transfer(fd, &spi_tx_message, &spi_rx_message,
		SPI_RESPONSE_COMPLETE);
	if (!ret)
		plog(0, "SPI transaction has failed!\n");
	else
		plog(1, "SPI transaction has succeeded\n");

}

static struct timespec timespec_diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

static int cm11_state_machine(int fd)
{
	static enum cm11_state state = cm11_state_ready;
	int parsed_bytes;
	static struct x10_command a_cmd;
	static struct timespec cm11_timer, ts_tmp, ts_diff;

	plog(1, "State %d, rbuf %d\n", state, cm11_rbuf_bytes);

	if (cm11_fresh_rbuf) {
		clock_gettime(CLOCK_MONOTONIC, &cm11_timer);
	} else {
		clock_gettime(CLOCK_MONOTONIC, &ts_tmp);
		ts_diff = timespec_diff(cm11_timer, ts_tmp);
		if (ts_diff.tv_sec >= 1) {
			if ( state != cm11_state_ready
				|| cm11_rbuf_bytes > 0 ) {
				plog(1, "UART idle timeout\n");
				// Flush the buffer
				cm11_rbuf_bytes = 0;
				state = cm11_state_ready;
			}
			clock_gettime(CLOCK_MONOTONIC, &cm11_timer);
		}
	}

	switch (state) {
	case cm11_state_ready:
		// Try to parse the command
		// If it is incomlete, wait more
		parsed_bytes = 0;
		if (cm11_fresh_rbuf)
			parsed_bytes = cm11_command_parse(cm11_rbuf, 
				cm11_rbuf_bytes, &a_cmd);
		if (parsed_bytes > 0) {
			plog(1, "Just parsed the command\n");
			log_command(1, &a_cmd);
			cm11_wbuf[0] = cm11_checksum(cm11_rbuf, cm11_rbuf_bytes);
			cm11_wbuf_bytes = 1;
			// flush the buffer
			cm11_rbuf_bytes = 0;
			state = cm11_state_tx_ack;
			break;
		}
		if (parsed_bytes < 0) {
			// flush the buffer, it is broken
			cm11_rbuf_bytes = 0;
		}
		if (cm11_has_cbuf) {
			plog(1, "Going to poll PC\n");
			cm11_wbuf[0] = 0x5A;
			cm11_wbuf_bytes = 1;
			state = cm11_state_rx_poll;
			break;
		}
		break;
	case cm11_state_tx_ack:
		if (cm11_fresh_rbuf) {
			if (cm11_rbuf[0] == 0) {
				plog(1, "Going to execute the transmission\n");
				cm11_execute(fd, &a_cmd);
				cm11_rbuf_bytes = 0;
				cm11_wbuf[0] = 0x55;
				cm11_wbuf_bytes = 1;
				state = cm11_state_ready;
				break;
			}
			// looks like a new transmission
			state = cm11_state_ready;
			return 1;
		}
		break;
	case cm11_state_rx_poll:
		if (cm11_fresh_rbuf) {
			if (cm11_rbuf[0] == 0xC3) {
				plog(1, "Poll answered from PC\n");
				cm11_rbuf_bytes = 0;
				memcpy(cm11_wbuf, cm11_cbuf, cm11_cbuf[0] + 1);
				cm11_wbuf_bytes = cm11_cbuf[0] + 1;
				memset(cm11_cbuf, 0, sizeof(cm11_cbuf));
				cm11_has_cbuf = 0;
				state = cm11_state_ready;
				break;
			}
			// looks like a new transmission
			state = cm11_state_ready;
			return 1;
		}
		break;
	}
	return 0;
}

void cm11(int fd)
{
	fd_set readset;
	struct timeval tv;
	int i;
	int rx;

	cm11_init();

	while(1) {
		FD_ZERO(&readset);
		FD_SET(fileno(stdin), &readset);
		tv.tv_sec = 0;
		tv.tv_usec = 200000; // 200ms
		cm11_fresh_rbuf = 0;
		if (select(fileno(stdin) + 1, &readset, NULL, NULL, &tv) > 0
			&& FD_ISSET(fileno(stdin), &readset)) {
			rx = read(fileno(stdin), cm11_rbuf + cm11_rbuf_bytes, 
				sizeof(cm11_rbuf) - cm11_rbuf_bytes);
			if (rx < 0)
				pabort("Error reading stdin");
			if (rx == 0) {
				plog(0, "Pipe has been closed by remote\n");
				return;
			}
			plog(1, "RX %d bytes, ", rx);
			for (i = cm11_rbuf_bytes; i < cm11_rbuf_bytes + rx; i++)
				plog(1, "%.2x ", cm11_rbuf[i]);
			plog(1, "\n");
			cm11_rbuf_bytes += rx;
			cm11_fresh_rbuf = 1;
		}
		// check for incoming X10
		// sets cm11_has_cbuf
		spi_x10_poll(fd);

		while (cm11_state_machine(fd));

		if (cm11_wbuf_bytes) {
			write(fileno(stdout), cm11_wbuf, cm11_wbuf_bytes);
			cm11_wbuf_bytes = 0;
		}
	}
}
