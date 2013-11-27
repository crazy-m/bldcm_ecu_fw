#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

#include "board.h"

int main(void)
{
	board_init();

	printf("Ovo je test USARTA!!!\n\r");

	for(;;)
	{
		printf("Test!\n\r");
		_delay_ms(1000);
	}

	return 0;
}
