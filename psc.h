#ifndef PSC_H_
#define PSC_H_

#if \
defined(__AVR_ATmega16M1__)  || \
defined(__AVR_ATmega32M1__)  || \
defined(__AVR_ATmega64M1__)

#define PSC_OUT_0A2B		( _BV(POEN2B)|_BV(POEN0A) )
#define PSC_OUT_1A0B		( _BV(POEN1A)|_BV(POEN0B) )
#define PSC_OUT_1A2B		( _BV(POEN2B)|_BV(POEN1A) )
#define PSC_OUT_2A1B		( _BV(POEN2A)|_BV(POEN1B) )
#define PSC_OUT_0A1B		( _BV(POEN1B)|_BV(POEN0A) )
#define PSC_OUT_2A0B		( _BV(POEN2A)|_BV(POEN0B) )

#define PSC_DUTY_MIN		0x0000
#define PSC_DUTY_MAX		0x03FF
#define PSC_DUTY_DEF		PSC_DUTY_MIN
#define PSC_HARD_CHOPPING	0

void	psc_init			(void);
void	psc_run				(uint8_t run);
void	psc_set_dutycycle	(int16_t dutycycle);

#endif

#endif /* PSC_H_ */
