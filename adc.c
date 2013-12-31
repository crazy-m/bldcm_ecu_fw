#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

#include "adc.h"

void adc_init(void)
{
	#if \
		defined(__AVR_ATmega32C1__)  || \
		defined(__AVR_ATmega64C1__)  || \
		defined(__AVR_ATmega16M1__)  || \
		defined(__AVR_ATmega32M1__)  || \
		defined(__AVR_ATmega64M1__)
	power_adc_enable();
	#endif

	DDRC	&= ~_BV(PC5) & ~_BV(PC4);
	PORTC	&= ~_BV(PC5) & ~_BV(PC4);

	ADMUX	= _BV(REFS0) | _BV(MUX3); // set CH8
	ADCSRA	= _BV(ADEN) |  _BV(ADATE) |  _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // set ADC Enable, Int. Flag and prescale F_CPU/64
	ADCSRB	= _BV(AREFEN) | _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0);

	DIDR1 = _BV(ADC9D) | _BV(ADC8D);
	
	ADCSRA	|= _BV(ADSC);
}

uint16_t adc_get_channel(adc_channel_t channel)
{
	uint8_t save_ADMUX, save_ADCSRA, save_ADCSRB;

	save_ADMUX	=	ADMUX;
	save_ADCSRA	=	ADCSRA;
	save_ADCSRB	=	ADCSRB;

	ADMUX	= _BV(REFS0);
	ADMUX	|=	channel;
	ADCSRA	= _BV(ADEN) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1);
	ADCSRB	= _BV(AREFEN);

	if(channel==ADC_CHANNEL_TEMP)
	{
		ADMUX	|= _BV(REFS1); // set to Int. 2.56V ref., On-chip Temp. sensor
		ADCSRB	&= ~_BV(AREFEN);
	}

	ADCSRA	|= _BV(ADSC);
	loop_until_bit_is_set(ADCSRA,ADIF);
	ADCSRA	|=	_BV(ADIF);

	ADMUX	=	save_ADMUX;
	ADCSRA	=	save_ADCSRA;
	ADCSRB	=	save_ADCSRB;

	return ADC;
}

/*
ISR(ADC_vect)
{
	// implemented in mc.c
}
*/
