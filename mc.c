#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "adc.h"
#include "ata6844.h"
#include "mc.h"
#include "config.h"
#include "pid.h"
#include "psc.h"

static 			mc_mode_t		mc_mode					=	MC_MODE_POSITION;

static volatile uint16_t		mc_hall_rpm_ticks		=	0;
static volatile	int32_t			mc_hall_rev_ticks		=	0;

static volatile uint8_t			mc_hall_last_input		=	0xFF;
static volatile mc_direction_t	mc_hall_dir				=	MC_DIR_CW;
static const	uint8_t			mc_hall_states_next[8]	=	{0xFF,0x05,0x03,0x01,0x06,0x04,0x02,0xFF};
static const	uint8_t			mc_hall_states_prev[8]	=	{0xFF,0x03,0x06,0x02,0x05,0x01,0x04,0xFF};

static volatile	mc_direction_t	mc_dir_now				=	MC_DIR_CW;

static volatile	int16_t			mc_rpm_ref			=	0;
static volatile	uint16_t		mc_rpm_now			=	0;
static volatile double			mc_rpm_last			=	0;
static volatile	double			mc_rpm_filter_last	=	0;

static volatile	int16_t			mc_rev_ref			=	0;
static volatile	int32_t			mc_rev_now			=	0;

static			pid_data_t		pid_speed;
static volatile	double			pid_speed_out;

static			pid_data_t		pid_position;
static volatile	double			pid_position_out;

static			void			mc_hall_sensors_irq	(void);

static			void			timer0_start		(void);
static			void			timer0_stop			(void);
/*
static			void			timer1_start		(void);
static			void			timer1_stop			(void);
*/

void mc_init(void)
{
	adc_init();
	ata6844_init();
	psc_init();

	// set Hall sensors pins as inputs without int. pull-ups
	DDRD	&=	~_BV(PD7) & ~_BV(PD6) & ~_BV(PD5);
	PORTD	&=	~_BV(PD7) & ~_BV(PD6) & ~_BV(PD5);

	// set PCINT for Hall sensors
	PCICR	|=	_BV(PCIE2);
	PCMSK2	|=	_BV(PCINT23) | _BV(PCINT22) | _BV(PCINT21);

	pid_speed.ErrorMax		=	PID_SPEED_KP*MOTOR_MAX_SPEED;
	pid_speed.IntSumMax		=	PID_SPEED_KP*MOTOR_MAX_SPEED/(PID_SPEED_KI+1);
	pid_speed.IntSum		=	0;
	pid_speed.OutputLast	=	0;
	pid_speed.Kp			=	PID_SPEED_KP;
	pid_speed.Ki			=	PID_SPEED_KI;
	pid_speed.Kd			=	PID_SPEED_KD;

	pid_position.ErrorMax	=	PID_ANGLE_KP*MOTOR_MAX_SPEED;
	pid_position.IntSumMax	=	PID_ANGLE_KP*MOTOR_MAX_SPEED/(PID_ANGLE_KI+1);
	pid_position.IntSum		=	0;
	pid_position.OutputLast	=	0;
	pid_position.Kp			=	PID_ANGLE_KP;
	pid_position.Ki			=	PID_ANGLE_KI;
	pid_position.Kd			=	PID_ANGLE_KD;

	ata6844_enabled(0);
	ata6844_coast(1);
}

void mc_run(uint8_t run)
{
	if (run)
	{
		ata6844_coast(0);
		ata6844_enabled(1);
		psc_run(1);
		mc_hall_sensors_irq();
		timer0_start();
		//timer1_start();
	}else{
		ata6844_coast(1);
		ata6844_enabled(0);
		psc_set_dutycycle(0x0000);
		psc_run(0);
		timer0_stop();
		//timer1_stop();
		mc_rpm_ref=0;
		mc_rpm_now=0;
		mc_rev_ref=0;
		mc_rev_now=0;
		mc_hall_rev_ticks=0;
	}
	pid_speed.IntSum=0;
	pid_position.IntSum=0;
}

void mc_coast(void)
{
	ata6844_coast(1);
}

void mc_rpm_set(int16_t speed)
{
	if (speed>MOTOR_MAX_SPEED)	speed = MOTOR_MAX_SPEED;
	if(speed<-MOTOR_MAX_SPEED)	speed = -MOTOR_MAX_SPEED;

	mc_rpm_ref	=	speed;
	mc_mode		=	MC_MODE_SPEED;
}

int16_t mc_rpm_get(void)
{
	if(mc_hall_dir==MC_DIR_CCW)
		return -mc_rpm_now;
	return mc_rpm_now;
}

void mc_rev_set(int16_t revs)
{
	mc_rev_ref	=	revs;
	mc_mode		=	MC_MODE_POSITION;
}

int16_t mc_rev_get(void)
{
	return mc_rev_now;
}

uint16_t mc_current_get(void)
{
	uint32_t current;
	current	= ( (uint32_t)(adc_get_ch8())*MOTOR_MAX_CURRENT*1000/1023 );
	return current;
}

uint8_t mc_voltage_get(void)
{
	uint32_t voltage;
	voltage	= ( (uint32_t)(adc_get_channel(ADC_CHANNEL_9))*MOTOR_MAX_VOLTAGE/1023 );
	return voltage;
}

int8_t mc_temp_get(void)
{
	int32_t temp;
	// (ADC*(IntRefVol[mV]/Res[1])-ZeroDeg[mV])/VpD[mV/degC]
	temp	= (int32_t)((double)(adc_get_channel(ADC_CHANNEL_TEMP))*(2560.0000/1023.0000)-699.6923)/2.4923;
	return (int8_t)temp;
}

static void timer0_start(void)
{
	power_timer0_enable();

	TCCR0A	=	_BV(WGM01); // CTC mode
	TCCR0B	=	_BV(CS02) | _BV(CS00); // prescale 1024 = 15625Hz = 0.064ms
	OCR0A	=	250; //   set overflow value for 16ms @ 16MHz
	TIMSK0	=	_BV(OCIE0A);
}

static void timer0_stop(void)
{
	TCCR0B	&=	~_BV(CS02) & ~_BV(CS01) & ~_BV(CS00);
}

/*
static void timer1_start(void)
{
	power_timer1_enable();

	TCCR1A	=	_BV(WGM12); // CTC mode
	TCCR1B	=	_BV(CS11) | _BV(CS10); // prescale 64 = 250kHz = 4us
	OCR1A	=	1000; // set overflow for 4ms @ 16MHz
	TIMSK1	=	_BV(OCIE1A);
}

static void timer1_stop(void)
{
	TCCR1B	&=	~_BV(CS12) & ~_BV(CS11) & ~_BV(CS10);
}
*/
static void mc_hall_sensors_irq(void)
{
	uint8_t hall_sensors_input;
	uint8_t	hall_sensors_tmp;

	hall_sensors_input	=	PIND;
	hall_sensors_input	&=	0xE0;
	hall_sensors_input	=	hall_sensors_input>>5;

	hall_sensors_tmp	=	mc_hall_last_input%8;
	mc_hall_last_input	=	hall_sensors_input;

	if(hall_sensors_input == *(mc_hall_states_next+hall_sensors_tmp))
	{
		mc_hall_dir = MC_DIR_CW;
		mc_hall_rev_ticks += 1;
	}else if(hall_sensors_input == *(mc_hall_states_prev+hall_sensors_tmp)){
		mc_hall_dir = MC_DIR_CCW;
		mc_hall_rev_ticks -= 1;
	}

	switch(mc_dir_now)
	{
		case MC_DIR_CW:
			switch(hall_sensors_input)
			{
				case 0x03:
					POC	= PSC_OUT_1A2B;
					break;
				case 0x01:
					POC	= PSC_OUT_0A2B;
					break;
				case 0x05:
					POC	= PSC_OUT_0A1B;
					break;
				case 0x04:
					POC	= PSC_OUT_2A1B;
					break;
				case 0x06:
					POC	= PSC_OUT_2A0B;
					break;
				case 0x02:
					POC	= PSC_OUT_1A0B;
					break;
				default:
					break;
			}
			break;
		case MC_DIR_CCW:
			switch(hall_sensors_input)
			{
				case 0x02:
					POC	= PSC_OUT_0A1B;
					break;
				case 0x06:
					POC	= PSC_OUT_0A2B;
					break;
				case 0x04:
					POC	= PSC_OUT_1A2B;
					break;
				case 0x05:
					POC	= PSC_OUT_1A0B;
					break;
				case 0x01:
					POC	= PSC_OUT_2A0B;
					break;
				case 0x03:
					POC	= PSC_OUT_2A1B;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

ISR(PCINT2_vect)
{
	mc_hall_rpm_ticks++;
	mc_hall_sensors_irq();
}

ISR(TIMER0_COMPA_vect)
{
	double		mc_rpm_filter;
	double		mc_rpm_tmp;
	double		mc_rev_tmp;
	double		mc_rev_hall;

	mc_rpm_tmp				=	(double)(mc_hall_rpm_ticks)*3750.00/48.00;
	mc_hall_rpm_ticks		=	0;

	// Digital filter for speed calc
	mc_rpm_filter		=	0.8519*mc_rpm_filter_last + 0.07407*mc_rpm_tmp + 0.07407*mc_rpm_last;
	mc_rpm_last			=	mc_rpm_tmp;
	mc_rpm_filter_last	=	mc_rpm_filter;
	mc_rpm_now			=	(int16_t)(mc_rpm_filter);

	if(mc_mode==MC_MODE_POSITION)
	{
		// P position controller

		mc_rev_tmp		=	(double)(mc_rev_ref)*24.00;
		mc_rev_hall		=	(double)(mc_hall_rev_ticks);
		mc_rev_now		=	(int32_t)(mc_rev_hall/24.00);

		pid_position_out	=	pid_controller(&mc_rev_tmp, &mc_rev_hall, &pid_position);

		if(pid_position_out<0)
		{
			mc_dir_now	=	MC_DIR_CCW;
			pid_position_out = -pid_position_out;
		}else{
			mc_dir_now	=	MC_DIR_CW;
		}

		// PI speed controller
		pid_speed_out = pid_controller(&pid_position_out, &mc_rpm_filter, &pid_speed);
	}else{
		// PI speed controller
		mc_rpm_tmp = (double)mc_rpm_ref;
		pid_speed_out = pid_controller(&mc_rpm_tmp, &mc_rpm_filter, &pid_speed);
	}

	if(pid_speed_out<0)
	{
		mc_dir_now	=	MC_DIR_CCW;
		pid_speed_out = -pid_speed_out;
	}else{
		mc_dir_now	=	MC_DIR_CW;
	}

	pid_speed_out = pid_speed_out*PSC_DUTY_MAX/MOTOR_MAX_SPEED;

	psc_set_dutycycle( (int16_t)(pid_speed_out) );
}

/*
ISR(TIMER1_COMPA_vect)
{
	//empty
}
*/
