#include <avr/io.h>
#include <avr/interrupt.h>

#include "ata6844.h"

void ata6844_init(void)
{
	// set EN pin
	ATA6844_EN_DDR	|=	_BV(ATA6844_EN_PIN);
	ATA6844_EN_PORT	&=	~_BV(ATA6844_EN_PIN);

	// set SLEEP
	ATA6844_SLEEP_DDR	|=	_BV(ATA6844_SLEEP_PIN);
	ATA6844_SLEEP_PORT	&=	~_BV(ATA6844_SLEEP_PIN);

	//set COAST
	ATA6844_COAST_DDR	|=	_BV(ATA6844_COAST_PIN);
	ATA6844_COAST_PORT	|=	_BV(ATA6844_COAST_PIN);

	// set WD
	ATA6844_WD_DDR		|=	_BV(ATA6844_WD_PIN);
	ATA6844_WD_PORT		&=	~_BV(ATA6844_WD_PIN);

	// set WD_EN
	ATA6844_WDEN_DDR	|=	_BV(ATA6844_WDEN_PIN);
	ATA6844_WDEN_PORT	&=	~_BV(ATA6844_WDEN_PIN);

	// set DG pins as inputs without int. pull-ups
	DDRB	&=	~_BV(PB5) & ~_BV(PB4) & ~_BV(PB3);
	PORTB	&=	~_BV(PB5) & ~_BV(PB4) & ~_BV(PB3);

	// set PCINT for DG
	PCICR	|=	_BV(PCIE0);
	PCMSK0	|=	_BV(PCINT5)|_BV(PCINT4)|_BV(PCINT3);

	ata6844_wd_enable(0);
	ata6844_reset_dg1();
}

void ata6844_reset_dg1(void)
{
	PORTB	|=	_BV(PB3);
	PORTB	&=	~_BV(PB3);
}

void ata6844_enabled(uint8_t enable)
{
	if(enable)
	{
		ATA6844_EN_PORT	|=	_BV(ATA6844_EN_PIN);
	}else{
		ATA6844_EN_PORT	&=	~_BV(ATA6844_EN_PIN);
	}
}

void ata6844_sleep(uint8_t sleep)
{
	if(sleep)
	{
		ATA6844_SLEEP_PORT	|=	_BV(ATA6844_SLEEP_PIN);
	}else{
		ATA6844_SLEEP_PORT	&=	~_BV(ATA6844_SLEEP_PIN);
	}
}

void ata6844_coast(uint8_t coast)
{
	if(coast)
	{
		ATA6844_COAST_PORT	&=	~_BV(ATA6844_COAST_PIN);
	}else{
		ATA6844_COAST_PORT	|=	_BV(ATA6844_COAST_PIN);
	}
}

void ata6844_wd_enable(uint8_t enable)
{
	if(enable)
	{
		ATA6844_WDEN_PORT	|=	_BV(ATA6844_WDEN_PIN);
	}else{
		ATA6844_WDEN_PORT	&=	~_BV(ATA6844_WDEN_PIN);
	}
}

uint8_t ata6844_get_dg(void)
{
	uint8_t ret=PINB;
	ret	&=	0x38;
	ret	=	ret>>3;
	return ret;
}

ISR(PCINT0_vect)
{
	ata6844_enabled(0);
}
