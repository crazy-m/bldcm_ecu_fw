#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/power.h>

#include "config.h"
#include "uart.h"

#if DEBUG_ME

static int uart_putc(char c, FILE *stream);
static int uart_getc(FILE *stream);

static FILE usart_stdio = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);

void uart_init(uart_baudrate_t baudrate)
{
	#if \
		defined(__AVR_ATmega32C1__)  || \
		defined(__AVR_ATmega64C1__)  || \
		defined(__AVR_ATmega16M1__)  || \
		defined(__AVR_ATmega32M1__)  || \
		defined(__AVR_ATmega64M1__)
	power_lin_enable();

	LINCR  &= ~_BV(LENA);
	LINBTR  = (_BV(LDISR) | 0x08);
	LINBRR = (short)(((((long)F_CPU)/((long)baudrate*8/2))+1)/2)-1;
	LINCR  |= _BV(LENA);

	LINCR = _BV(LENA)|_BV(LCMD2)|_BV(LCMD1)|_BV(LCMD0);
	LINDAT = 0xFF;
	#endif

	stdout = stdin = stderr = &usart_stdio;

	// Set terminal cursor to 0,0 - ESC [ Pl ; Pc H
	printf_P(PSTR("\x1B[0;0H"));
	// Clear terminal screen - ESC [ 2 J
	printf_P(PSTR("\x1B[2J"));
}

static int uart_putc(char c, FILE *stream)
{
	if (c == '\n') uart_putc('\r',stream);
	#if \
		defined(__AVR_ATmega32C1__)  || \
		defined(__AVR_ATmega64C1__)  || \
		defined(__AVR_ATmega16M1__)  || \
		defined(__AVR_ATmega32M1__)  || \
		defined(__AVR_ATmega64M1__)
	loop_until_bit_is_set(LINSIR,LTXOK);
	LINDAT = c;
	#endif
	return 0;
}

static int uart_getc(FILE *stream)
{
	#if \
		defined(__AVR_ATmega32C1__)  || \
		defined(__AVR_ATmega64C1__)  || \
		defined(__AVR_ATmega16M1__)  || \
		defined(__AVR_ATmega32M1__)  || \
		defined(__AVR_ATmega64M1__)
	loop_until_bit_is_set(LINSIR,LRXOK);
	return LINDAT;
	#endif
}
#endif
