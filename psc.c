#include <avr/io.h>
#include <avr/power.h>

#include "psc.h"

#if \
defined(__AVR_ATmega16M1__)  || \
defined(__AVR_ATmega32M1__)  || \
defined(__AVR_ATmega64M1__)

void psc_init(void)
{
	power_psc_enable();

	// set PSC pins
	DDRB	|=	_BV(PB7) | _BV(PB6) | _BV(PB1) | _BV(PB0);
	DDRC	|=	_BV(PC0);
	DDRD	|=	_BV(PD0);
	PORTB	&=	~_BV(PB7) & ~_BV(PB6) & ~ _BV(PB1);
	PORTB	|= _BV(PB0);
	PORTC	|= _BV(PC0);
	PORTD	|= _BV(PD0);

	// Set PLL=64MHz, start PLL
	PLLCSR	=	_BV(PLLF) | _BV(PLLE);

	// wait for PLL to lock
	loop_until_bit_is_set(PLLCSR, PLOCK);

	// initial values for psc registers
	POCR0RA	=	1;
	POCR1RA	=	0;
	POCR2RA	=	0;

	POCR_RB	=	PSC_DUTY_MAX;

	POCR0SA	=	0;
	POCR1SA	=	0;
	POCR2SA	=	0;
	POCR0SB	=	0;
	POCR1SB	=	0;
	POCR2SB	=	0;

	#if !PSC_HARD_CHOPPING
	PCNF	|=	_BV(POPB);	// PSC outputs B are active high
	#endif
	PCNF	|=	_BV(PMODE); // center aligned mode

	PCTL	|=	_BV(PCLKSEL);	// set PLL as input clock, no prescalers

	// disable Overlap protection
	PMIC0	|=	_BV(POVEN0);
	PMIC1	|=	_BV(POVEN1);
	PMIC2	|=	_BV(POVEN2);

	// Send signal on match with OCRnRA (during counting up of PSC)
	//PSYNC	=	_BV(PSYNC00);
}

void psc_run(uint8_t run)
{
	if(run)
	{
		PCTL	|=	_BV(PRUN);
	}else{
		PCTL	&=	~_BV(PRUN);
	}
}

void psc_set_dutycycle(int16_t dutycycle)
{
	if(dutycycle>PSC_DUTY_MAX) dutycycle=PSC_DUTY_MAX;
	if(dutycycle<PSC_DUTY_MIN) dutycycle=PSC_DUTY_MIN;

	POCR0SA	=	dutycycle;
	POCR2SA	=	dutycycle;
	POCR1SA	=	dutycycle;
	#if PSC_HARD_CHOPPING
	POCR0SB	=	dutycycle;
	POCR1SB	=	dutycycle;
	POCR2SB	=	dutycycle;
	#else
	POCR0SB	=	0;
	POCR1SB	=	0;
	POCR2SB	=	0;
	#endif
	
}
#endif
