#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>

#include "usart_common.h"
#include "usart_atmegaxxm1c1.h"

void usart_atmegaxxm1c1_init(usart_baudrate_t baudrate)
{
	LINCR  &= ~_BV(LENA);
	LINBTR  = (_BV(LDISR) | 0x08);
	LINBRR = (short)(((((long)F_CPU)/((long)baudrate*8/2))+1)/2)-1;
	LINCR  |= _BV(LENA);

	LINCR = _BV(LENA)|_BV(LCMD2)|_BV(LCMD1)|_BV(LCMD0);
	LINDAT = 0xFF;
}

uint8_t usart_atmegaxxm1c1_getc(void)
{
	loop_until_bit_is_set(LINSIR,LRXOK);
	return LINDAT;
}

void usart_atmegaxxm1c1_putc(uint8_t c)
{
	if (c == '\n') usart_atmegaxxm1c1_putc('\r');
	loop_until_bit_is_set(LINSIR,LTXOK);
	LINDAT = c;
}
