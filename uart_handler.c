/**
* author: Oskar Piechowski
* date: 29-10-2019 10:28:31 PM
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
uint8_t in_buffer[UART_BUF_SIZE];
volatile uint8_t c_len = 0;

/** Interrupt Service Routine for Receive Complete
 *  Drops incoming chars if buffer full
 */
ISR(USART_RXC_vect)
{
  in_buffer[c_len] = UDR;
  c_len += 1;
  if (c_len >= UART_BUF_SIZE)
  {
    c_len = 0;
  }
}

/** Initializes the UART.
 *   Default frame format is 8 data bits, no parity, 1 stop bit
 *   to change use UCSRC, see AVR datasheet
 */
void USART_Init(void)
{
  /* Load lower 8 - bits into the low byte of the UBRR register */
  UBRRL = BAUD_PRESCALE;
  /* Load upper 8-bits into the high byte of the UBRR register */
  UBRRH = (BAUD_PRESCALE >> 8);
  /* Enable receiver and transmitter and receive complete interrupt */
  UCSRB = ((1 << TXEN) | (1 << RXEN) | (1 << RXCIE));
}

void USART_SendByte(uint8_t u8Data)
{
  // Wait until last byte has been transmitted
  while ((UCSRA & (1 << UDRE)) == 0)
  {
    ;
  }

  // Transmit data
  UDR = u8Data;
}

void buzz(void)
{

  OCR1A = 0x01FF;
  // set PWM for 50% duty cycle @ 10bit

  TCCR1A |= (1 << COM1A1);
  // set none-inverting mode

  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  // set 10bit phase corrected PWM Mode

  TCCR1B |= (1 << CS11);
  // set prescaler to 8 and starts PWM
}

void portb_init(void)
{
  // outputs, all zeroed
  DDRB = 0xFF;
  PORTB = 0x00;
}

/* T = 1/f, 1.000.000 microseconds in a second. */
static inline double baud_to_us(int baudrate) { return 1000000 / baudrate; }

/* Bit magic */
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))
#define SET_BIT(p, n) ((p) |= (1 << (n)))
#define CLR_BIT(p, n) ((p) &= ~((1) << (n)))

/**
 * Configured via #SOFTUART_PORT, #SOFTUART_PIN and $SOFTUART_BAUDRATE
 * definitions. Besides we have hardcoded no parity and 1 stop bit for now.
 */
void softuart_send_byte(uint8_t tx_byte)
{
  int8_t i;

  /* Send the start bit */
  CLR_BIT(SOFTUART_PORT, SOFTUART_PIN);
  _delay_us(baud_to_us(SOFTUART_BAUDRATE));

  /* Transfer the byte consisting of 8 bits */
  for (i = 0; i < 8; i++)
  {
    if (CHECK_BIT(tx_byte, i))
    {
      SET_BIT(SOFTUART_PORT, SOFTUART_PIN);
    }
    else
    {
      CLR_BIT(SOFTUART_PORT, SOFTUART_PIN);
    }
    _delay_us(baud_to_us(SOFTUART_BAUDRATE));
  }

  /* Send the stop bit */
  SET_BIT(SOFTUART_PORT, SOFTUART_PIN);
  _delay_us(baud_to_us(SOFTUART_BAUDRATE));
}

static inline void clear_in_buffer()
{
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    memset(in_buffer, 0, UART_BUF_SIZE);
    c_len = 0;
  }
}

static inline void send_string(uint8_t *str, void (*char_tx)(uint8_t))
{
  while (*str)
  {
    char_tx(*str);
    str++;
  }
}

static void esp_wait_for_ok(unsigned int timeout, uint8_t num_retries)
{
  uint8_t *found;
  uint8_t offset;
  uint8_t i;

  for (i = 0; i < num_retries; i++)
  {
    _delay_ms(timeout);
    found = strstr(in_buffer, "OK\r\n");
    if (found)
    {
      offset = found + 4 - in_buffer;
      memmove(in_buffer, found + 4, UART_BUF_SIZE - offset);
      c_len = c_len - offset;
      return;
    }
  }
}

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

void send_esp_config()
{
  uint8_t *esp_init_cmds[] = {
      "AT+CWSAP=\"TalkingBox\",\"TalkToMe\",1,3\r\n",
      "AT+CIPMUX=1\r\n",
      "AT+CIPSERVER=1,1234\r\n "};
  uint8_t i;

  for (i = 0; i < ARRAY_SIZE(esp_init_cmds); i++)
  {
    clear_in_buffer();
    send_string(esp_init_cmds[i], USART_SendByte);
    esp_wait_for_ok(500, 3);
  }
}

/* Returns filtered data len */
int filter_data(uint8_t *dst_buff, uint8_t *src_buff)
{
  uint8_t *colon;
  uint8_t *found;
  uint8_t offset;

  found = strstr(in_buffer, "OK\r\n");
  if (found)
  {
    colon = strchr(in_buffer, ':');
    if (colon)
    {
      memccpy(dst_buff, colon + 1, found - colon - 1, found - colon - 1);
    }
    offset = found + 4 - in_buffer;
    memmove(in_buffer, found + 4, UART_BUF_SIZE - offset);
    c_len = c_len - offset;
    return colon ? found - colon : 0;
  }
  return 0;
}

/** Adjusting the output to make the FPGA + LCD for following effects:
  * 1) Do not change lines when writing text (remove CR LF from strings)
  * 3) When sending just CR LF, do send CR without LF.
  * 4) There is no point 2.
  * 
  * @out_buf: buffer to modify in-place :)
  */
void output_adjustments(char *out_buf)
{
  char *cur;

  /* This funny ''Lovelace'' sequence is sent on empty string by ESP. */
  if (out_buf[0] == 0xA &&
      out_buf[1] == 0xD &&
      out_buf[2] == 0xA)
  {
    out_buf[0] = '\r';
    out_buf[1] = 0;
    out_buf[2] = 0;
    return;
  }

  cur = out_buf;
  /* Assumption: CR LF are the last chars in the string, make them 0. */
  while (*cur)
  {
    if (*cur == '\n' || *cur == '\r')
    {
      *cur = 0;
    }
    cur++;
  }
}

int main(void)
{
  char out_buffer[UART_BUF_SIZE] = {0};
  int recv_count;

  USART_Init();
  portb_init();

  /* Enable interrupts. */
  sei();

  /* Wait until ESP boots. */
  _delay_ms(5000);

  /* Use the HW uart to send the config string */
  send_esp_config();

  while (true)
  {
    recv_count = filter_data(out_buffer, in_buffer);
    if (recv_count > 0)
    {
      output_adjustments(out_buffer);
      send_string(out_buffer, softuart_send_byte);
      memset(out_buffer, 0, UART_BUF_SIZE);
    }
    _delay_ms(100);
  }
}
