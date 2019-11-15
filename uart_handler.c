/**
* author: Oskar Piechowski
* date: 29-paź-2019 10:28:31 PM
*/

#define F_CPU 8000000UL
/* AVR Includes */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>
/* Other includes */
#include <stdbool.h>
#include <string.h>

/* Configuration */
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define UART_BUF_SIZE 256

#define SOFTUART_BAUDRATE 9600
#define SOFTUART_PORT PORTB
#define SOFTUART_PIN 7

/* Variables */
volatile uint8_t in_buffer[UART_BUF_SIZE];
volatile uint8_t c_len = 0;

/** Interrupt Service Routine for Receive Complete
 *  Drops incoming chars if buffer full
 */
ISR(USART_RXC_vect) {
  if (c_len < UART_BUF_SIZE) {
    in_buffer[c_len++] = UDR;
  }
}

/** Initializes the UART.
 *   Default frame format is 8 data bits, no parity, 1 stop bit
 *   to change use UCSRC, see AVR datasheet
 */
void USART_Init(void) {
  /* Load lower 8 - bits into the low byte of the UBRR register */
  UBRRL = BAUD_PRESCALE;
  /* Load upper 8-bits into the high byte of the UBRR register */
  UBRRH = (BAUD_PRESCALE >> 8);
  /* Enable receiver and transmitter and receive complete interrupt */
  UCSRB = ((1 << TXEN) | (1 << RXEN) | (1 << RXCIE));
}

void USART_SendByte(uint8_t u8Data) {
  // Wait until last byte has been transmitted
  while ((UCSRA & (1 << UDRE)) == 0)
    ;

  // Transmit data
  UDR = u8Data;
}

void portb_init(void) {
  // outputs, all off
  DDRB = 0xFF;
  PORTB = 0xFF;
}

/* T = 1/f, 1.000.000 microseconds in a second. */
double baud_to_us(int baudrate) { return 1000000 / baudrate; }

/* Bit magic */
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))
#define SET_BIT(p, n) ((p) |= (1 << (n)))
#define CLR_BIT(p, n) ((p) &= ~((1) << (n)))

/**
 * Configured via #SOFTUART_PORT, #SOFTUART_PIN and $SOFTUART_BAUDRATE
 * definitions. Besides we have hardcoded no parity and 1 stop bit for now.
 */
void softuart_send_byte(uint8_t tx_byte) {
  int8_t i;

  /* Send the start bit */
  CLR_BIT(SOFTUART_PORT, SOFTUART_PIN);
  _delay_us(baud_to_us(SOFTUART_BAUDRATE));

  /* Transfer the byte consisting of 8 bits */
  for (i = 0; i < 8; i++) {
    if (CHECK_BIT(tx_byte, i)) {
      SET_BIT(SOFTUART_PORT, SOFTUART_PIN);
    } else {
      CLR_BIT(SOFTUART_PORT, SOFTUART_PIN);
    }
    _delay_us(baud_to_us(SOFTUART_BAUDRATE));
  }

  /* Send the stop bit */
  SET_BIT(SOFTUART_PORT, SOFTUART_PIN);
  _delay_us(baud_to_us(SOFTUART_BAUDRATE));
}

static inline void clear_buffer(void *buffer) {
  memset(buffer, 0, UART_BUF_SIZE);
  if (buffer == in_buffer) {
    c_len = 0;
  }
}

static inline void send_string(uint8_t *str, void (*char_tx)(uint8_t)) {
  while (*str) {
    char_tx(*str);
    str++;
  }
}

static void esp_wait_for_ok() {
  uint8_t *found;
  uint8_t offset;
  for (;;) {
    found = strstr(in_buffer, "OK\r\n");
    if (found) {
      offset = found + 4 - in_buffer;
      memmove(in_buffer, found + 4, UART_BUF_SIZE - offset);
      c_len = c_len - offset;
      return;
    }
    _delay_ms(1000);
  }
}

#define ESP_CONFIG_CMDS 2
void send_esp_config() {
  uint8_t *esp_init_cmds[ESP_CONFIG_CMDS] = {"AT+CIPMUX=1\r\n",
                                             "AT+CIPSERVER=1,1234\r\n "};
  uint8_t i;

  for (i = 0; i < ESP_CONFIG_CMDS; i++) {
    send_string(esp_init_cmds[i], USART_SendByte);
    esp_wait_for_ok();
  }
}

/* Returns filtered data len */
int filter_data(uint8_t *dst_buff, uint8_t *src_buff) {
  uint8_t *colon;
  uint8_t *found;
  uint8_t offset;

  found = strstr(in_buffer, "OK\r\n");
  if (found) {
    colon = strchr(in_buffer, ':');
    if (colon) {
      memccpy(dst_buff, colon + 1, found - colon - 1, found - colon - 1);
    }
    offset = found + 4 - in_buffer;
    memmove(in_buffer, found + 4, UART_BUF_SIZE - offset);
    c_len = c_len - offset;
    return colon ? found - colon : 0;
  }
  return 0;
}

int main(void) {
  char out_buffer[UART_BUF_SIZE];
  int recv_count;

  USART_Init();
  portb_init();

  /* Enable interrupts. */
  sei();

  /* Wait until ESP boots. */
  _delay_ms(5000);

  clear_buffer(in_buffer);
  clear_buffer(out_buffer);

  /* Use the HW uart to send the config string */
  send_esp_config();

  while (true) {
    recv_count = filter_data(out_buffer, in_buffer);
    if (recv_count > 0) {
      send_string(out_buffer, softuart_send_byte);
      clear_buffer(out_buffer);
    }
    _delay_ms(100);
  }
}
