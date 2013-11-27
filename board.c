#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

#include "board.h"

FILE usart_stdio = FDEV_SETUP_STREAM(usart_atmegaxxm1c1_putc, usart_atmegaxxm1c1_getc, _FDEV_SETUP_RW);

void board_init(void)
{
	usart_atmegaxxm1c1_init(USART_BAUDRATE_115200);
	stdout = stdin = &usart_stdio;
	// Set terminal cursor to 0,0 - ESC [ Pl ; Pc H
	printf("\x1B[0;0H");
	// Clear terminal screen - ESC [ 2 J
	printf("\x1B[2J");
}
