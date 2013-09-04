
#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <string.h>
#include <avr/pgmspace.h>

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

#define X10_STATE_IDLE 0
#define X10_STATE_COMMAND 1
#define X10_STATE_EXTENDED 2
#define X10_STATE_RECEIVED 3


typedef struct _struct_spi_status {
 uint8_t running : 1;
 uint8_t rx_ready : 1;
 uint8_t tx_ready : 1;
} spi_status_type;

volatile spi_status_type spi_status;

volatile uint8_t spi_rx_buffer[38];
volatile uint8_t spi_tx_buffer[38];
volatile uint8_t spi_index;

volatile uint32_t x10_rx;
volatile uint8_t x10_tmp;
volatile uint8_t x10_state = X10_STATE_IDLE;
volatile uint8_t x10_bits = 0;

static void spi_enable(void) {
 // MISO is output
 DDR_SPI |= _BV(SPI_MISO);
 // 3-wire mode, external clock, shift on positive edge (SPI mode 0)
 // Enable interrupt on overflow
 USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USIOIE);
 // Clear clock counter (set to 0)
 // Clear counter overflow flag
 USISR = _BV(USIOIF);
 // Place first octet to send
 USIDR = (spi_status.tx_ready) ? spi_tx_buffer[0] : 0xfe;
 // Init buffer
 spi_index = 0;
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
 spi_status.rx_ready = 1;
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
 
 spi_rx_buffer[spi_index] = USIDR;
 if (++spi_index >= sizeof(spi_rx_buffer)) {
  spi_index = 0;
 }
 USIDR = (spi_status.tx_ready) ? spi_tx_buffer[spi_index] : 0xFE;
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
}

/*
 *
 * Time to sample a x10 bit
 *
 */

ISR(TIMER1_CMPA_vect) {
 
 x10_rx = (x10_rx << 1) + ( bit_is_clear(PIN_X10, X10_IN) ? 1 : 0 );
 if (!((++x10_bits) & 0x7)) {
  x10_state = X10_STATE_RECEIVED;
 }
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

inline uint8_t spi_rx_ready(void) {
 return spi_status.rx_ready;
}

inline void spi_rx_wait(void) {
 while (!spi_status.rx_ready);
}

inline void spi_cleanup(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.rx_ready = 0;
 SREG = tmp_sreg;
}

inline void spi_enable_tx(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.tx_ready = 1;
 SREG = tmp_sreg;
}

inline void spi_disable_tx(void) {
 uint8_t tmp_sreg = SREG;
 
 cli();
 spi_status.tx_ready = 1;
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
 uint8_t index = 0;

 //pullup all unused pins at PORTA
 DDRA = 0x00;
 PORTA = 0xff;

 spi_init();
 x10_init();
 
 sei();
 while (1) {
  //spi_rx_wait();
  if (x10_state == X10_STATE_RECEIVED) {
   spi_tx_buffer[index] = x10_rx;
   if (index < 36) {
    index++;
   }
   else {
    index = 0;
   }
   spi_tx_buffer[36] = index;
   x10_rx = 0;
   x10_state = X10_STATE_IDLE;
  }
  spi_enable_tx();
  spi_cleanup();
 }
}
