
#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define PIN_SPI PINB
#define SPI_MOSI PB0
#define SPI_MISO PB1
#define SPI_SCK PB2
#define SPI_SS PB3

// is also INT0 pin
#define X10_ZERO PB6 
#define X10_IN PB5
#define X10_OUT PB4
#define PIN_X10 PINB
#define PORT_X10 PORTB
#define DDR_X10 DDRB

#define T1_TICKS(us) ((uint8_t)(1.0*F_CPU/64*us/1000000))
#define X10_SAMPLE_DELAY 500
#define X10_TRANSMIT_LENGTH 1000

// Maximum is 32 due to stream_tail size, but RAM restricts it further
// 24 is reasonable minimum due to extended command size being 22*2+6+62*2=174 bits
#define X10_BITSTREAM_OCTETS 24

typedef struct _x10_bitstream {
	uint8_t data[X10_BITSTREAM_OCTETS];
	uint8_t tail; // pointer to the bit after the stream
 } x10_bitstream_t;

typedef struct _spi_message {
 uint8_t rr_code;
 uint8_t rr_id;
 x10_bitstream_t x10_data;
 uint16_t crc16;
} spi_message_t;

#define REQUEST_POLL 0
#define REQUEST_CANCEL 1
#define REQUEST_TRANSMIT 2

#define RESPONSE_SEEN 1
#define RESPONSE_INPROGRESS 2
#define RESPONSE_COMPLETE 3


typedef struct _struct_spi_status {
 uint8_t running : 1;
 uint8_t rx_body : 1;
 uint8_t rx_done : 1;
 uint8_t rx_enabled : 1;
 uint8_t tx_enabled : 1;
} spi_status_t;

volatile spi_status_t spi_status;

volatile spi_message_t spi_rx_message;
spi_message_t spi_tx_message;

x10_bitstream_t tx_bitstream; // currently transmitted

uint8_t spi_counter; // counts bytes in a SPI transaction, up to spi_message size

volatile uint8_t x10_rx;
volatile uint8_t x10_tx;
volatile uint8_t x10_rx_counter = 0;
volatile uint8_t x10_tx_counter = 0;

static void spi_enable(void) {
 // MISO is output
 DDR_SPI |= _BV(SPI_MISO);
 // 3-wire mode, external clock, shift on positive edge (SPI mode 0)
 // Enable interrupt on overflow
 USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USIOIE);
 // Clear clock counter (set to 0)
 // Clear counter overflow flag
 USISR = _BV(USIOIF);
 // Preload first octet to send
 USIDR = (spi_status.tx_enabled) ? ((uint8_t*)&spi_tx_message)[0] : 0xFE;
 // Init transaction engine
 spi_counter = 0;
 // Raise flag
 spi_status.running = 1;
 // start the timer
 // prescaler is CS/64, reset it
 TCCR0 = _BV(PSR0) | _BV(CS02);
 TCNT0 = 0;
}

static void spi_disable(void) {
 DDR_SPI &= ~_BV(SPI_MISO); // now MISO is input
 // and SPI is disabled
 USICR = 0;
 spi_status.running = 0;
 if (spi_status.rx_body) {
  spi_status.rx_done = 1;
 }
}

/*
 *
 * This interrupt should be called on Slave Select change only.
 *
 */
ISR(IO_PINS_vect) {
 if (bit_is_clear(PIN_SPI, SPI_SS)) {
  spi_enable();
 }
 else {
  spi_disable();
 }
}

/*
 *
 * This ISR is called when a byte is received/transmitted
 *
 */

ISR(USI_OVF_vect) {
 
 // Time-critical section
 uint8_t tmp_rx = USIDR;

  // tx for index 1..sizeof(spi_message_t)-1
  // and only if tx_enabled is set
 USIDR = (spi_status.tx_enabled && (spi_counter < sizeof(spi_message_t)-1)) ? 
  ((uint8_t*)&spi_tx_message)[spi_counter+1] : 0xFE;
 
 // End of time-critical section

 // Do not overwrite previous request if the new request is POLL
 if (spi_counter == 0) {
  spi_status.rx_body = (tmp_rx == REQUEST_POLL) ? 0 : 1;
 }
    
 if (spi_counter<sizeof(spi_message_t)) {
  // rx for index 0..sizeof(spi_message_t)-1
  if (spi_status.rx_enabled && spi_status.rx_body) {
   ((uint8_t*) &spi_rx_message)[spi_counter] = tmp_rx;
  }
  ++spi_counter;
 }
 
 USISR |= _BV(USIOIF);
}

/*
 *
 * Zero crossing interrupt
 *
 */

ISR(INT0_vect) {
 // Set up for next sampling interrupt
 OCR1A = TCNT1 + T1_TICKS(X10_SAMPLE_DELAY);
 TIMSK |= _BV(OCIE1A);
 TIFR = _BV(OCF1A);
 // Transmit
 if (x10_tx_counter) {
  if (x10_tx & 0x80) {
   PORT_X10 |= _BV(X10_OUT);
   OCR1B = TCNT1 + T1_TICKS(X10_TRANSMIT_LENGTH);
   TIMSK |= _BV(OCIE1B);
   TIFR = _BV(OCF1B);
  } 
  x10_tx <<= 1;
  --x10_tx_counter;
 }
}

/*
 *
 * Time to switch off X10 output
 *
 */

ISR(TIMER1_CMPB_vect) {

 // End of X10 pulse
 PORT_X10 &= ~_BV(X10_OUT);
 
 // One-shot interrupt
 TIMSK &= ~_BV(OCIE1B);
}

/*
 *
 * Time to sample a X10 bit
 *
 */

ISR(TIMER1_CMPA_vect) {
 
 x10_rx = (x10_rx << 1) + ( bit_is_clear(PIN_X10, X10_IN) ? 1 : 0 );
 x10_rx_counter++;
 /*
 x10_tmp <<= 1;
 if (bit_is_clear(PIN_X10, X10_IN)) {
  x10_tmp += 1;
 }
 switch (x10_state) {
  case X10_STATE_IDLE:
   if ((x10_tmp & 0b1111) == 0b1110) {
    x10_state = X10_STATE_COMMAND;
   }
   break;
  case X10_STATE_COMMAND:
  case X10_STATE_EXTENDED:
   x10_bits++;
   if (!(x10_bits&1)) {
    x10_rx <<= 1;
    switch (x10_tmp&0b11) {
	 case 0b10:
	  x10_rx += 1;
	 case 0b01:
	  break;
	 default:
	  // Houston, we have a problem
	  x10_state = X10_STATE_IDLE;
	  x10_bits = 0;
	  x10_rx = 0;
	}
   }
   if (x10_bits == 18) {
    switch (x10_rx & 0b11111) {
	 case 0b01111: // Extended code 1
//	 case 0b10101: // Extended code 3
//	 case 0b11001: // Extended code 2
	  x10_state = X10_STATE_EXTENDED;
	  break;
	 default:
	  x10_state = X10_STATE_RECEIVED;
	}
   } else if (x10_bits == 58) {
    x10_state = X10_STATE_RECEIVED;
   }
 }
 */
 
 // One-shot interrupt
 TIMSK &= ~_BV(OCIE1A);
 
}

static void spi_init(void) {
 
 // All pins are input
 DDR_SPI &= ~(_BV(SPI_SCK) | _BV(SPI_MOSI) | _BV(SPI_MISO) | _BV(SPI_SS));
 // All are pullup
 PORT_SPI |= _BV(SPI_SCK) | _BV(SPI_MOSI) | _BV(SPI_MISO) | _BV(SPI_SS);

 if (bit_is_clear(PIN_SPI, SPI_SS)) {
  spi_enable();
 }
 
 // Enable pin change interrupt on Chip Select
 GIMSK |= _BV(PCIE0);
}

inline void spi_enable_rx(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.rx_done = 0;
 spi_status.rx_enabled = 1;
 SREG = tmp_sreg;
}

inline void spi_disable_rx(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.rx_enabled = 0;
 SREG = tmp_sreg;
}

/*
 * Checksum the message
 */

uint16_t spi_crc16(spi_message_t *spi_buffer) {
 uint16_t crc = 0xffff;
 
 for (uint8_t i=0; i<sizeof(spi_message_t)-2; i++) {
  crc = _crc16_update(crc, ((uint8_t*)spi_buffer)[i]);
 }
 return crc;
}

/*
 * Enable SPI transmission
 */

static void spi_enable_tx(void) {
 uint8_t tmp_sreg = SREG;

 spi_tx_message.crc16 = spi_crc16( (spi_message_t*)&spi_tx_message );

 cli();
 spi_status.tx_enabled = 1;
 SREG = tmp_sreg;
}

static void spi_disable_tx(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.tx_enabled = 0;
 SREG = tmp_sreg;
}

static void x10_init(void) {
 DDRB |= _BV(X10_OUT);
 MCUCR |= _BV(ISC00); // INT0 on any change
 GIMSK |= _BV(INT0); // enable INT0
 // Using Timer1 for measurements
 TCCR1B |= _BV(CS12) | _BV(CS11) | _BV(CS10); // prescaler at 64
}

int main(void) {
 uint8_t rx_x10_index = 0;
 struct _x10_tx_state {
  uint8_t has_bitstream : 1;
  uint8_t has_postponed_rq : 1;
  uint8_t bitstream_index : 5;
 } x10_tx_state;

 // Pullup all unused pins at PORTA.
 DDRA = 0x00;
 PORTA = 0xff;

 spi_init();
 x10_init();
 
 x10_tx_state.has_bitstream = 0;
 
 sei();
 spi_enable_tx();
 while (1) {
  
  // Just received 8 bits of X10 stream
  if (x10_rx_counter >= 8) {
   spi_disable_tx(); // we will modify spi_tx_message.
   cli(); // delay x10 interrupts, just in case...
   spi_tx_message.x10_data.data[rx_x10_index++] = x10_rx;
   x10_rx_counter = 0;
   sei();
   // Wraparound
   if (rx_x10_index == X10_BITSTREAM_OCTETS) {
    rx_x10_index = 0;
   }
   spi_tx_message.x10_data.tail = rx_x10_index * 8;
  }

  // We have data to transmit and previous X10 chunk is sent
  if (x10_tx_state.has_bitstream && !x10_tx_counter) {

   if (tx_bitstream.tail == 0) {
    // This transmission is over
    spi_disable_tx();
    spi_tx_message.rr_code = RESPONSE_COMPLETE;
	x10_tx_state.has_bitstream = 0;
   }
   else {
    // The transmission is not finished yet.
    cli(); // delay X10 interrupts
	x10_tx = tx_bitstream.data[x10_tx_state.bitstream_index++];
	if (tx_bitstream.tail < 8) {
	 // this is last and incomplete octet
	 x10_tx_counter = tx_bitstream.tail;
	 tx_bitstream.tail = 0;
	}
	else {
	 // one more complete octet to send
	 x10_tx_counter = 8;
	 tx_bitstream.tail -= 8;
	}
	sei();
   }
  }

  // A new SPI message has arrived
  if (spi_status.rx_done ||
   // or there was a postponed message, and it's time to look at it.
   ( x10_tx_state.has_postponed_rq && !x10_tx_state.has_bitstream )) {
   // In all cases, postponed request cannot stay in the same state.
   x10_tx_state.has_postponed_rq = 0;
   // From this point, there is need for integrity protection against SPI
   spi_disable_rx();
   // Check CRC of the message
   if (spi_crc16((spi_message_t*)&spi_rx_message) == spi_rx_message.crc16) {
    // CRC is correct
	// We will change tx_message, so it can become invalid.
	spi_disable_tx();
	switch (spi_rx_message.rr_code) {
     case REQUEST_CANCEL:
	  // Cancel current transmission, if any. Done.
	  x10_tx_state.has_bitstream = 0;
	  spi_tx_message.rr_id = spi_rx_message.rr_id;
	  spi_tx_message.rr_code = RESPONSE_COMPLETE;
	  break;
	 case REQUEST_TRANSMIT:
	  // It's a valid request, need to ack it.
      spi_tx_message.rr_id = spi_rx_message.rr_id;
	  if (!x10_tx_state.has_bitstream) {
	   // Init bitstream sender
       tx_bitstream = spi_rx_message.x10_data;
	   x10_tx_state.has_bitstream = 1;
	   x10_tx_state.bitstream_index = 0;
       // Now we are transmitting
	   spi_tx_message.rr_code = RESPONSE_INPROGRESS;
	  }
	  else {
	   // the request should be chained, where possible
       x10_tx_state.has_postponed_rq = 1; 
       // This request can still be overwritten by host
	   spi_tx_message.rr_code = RESPONSE_SEEN;   
	  }
	  break;
	  // Default is to ignore the request
	}
   }
   // Wait fot a new SPI message
   spi_enable_rx();
  }

  spi_enable_tx();
 }
}
