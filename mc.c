#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "adc.h"
#include "ata6844.h"
#include "mc.h"
#include "config.h"
#include "pid.h"
#include "psc.h"

static 			mc_mode_t		mc_mode					=	MC_MODE_POSITION;

static volatile int16_t			mc_hall_rpm_ticks		=	0;
static volatile	int32_t			mc_hall_rev_ticks		=	0;

static volatile uint8_t			mc_hall_last_input		=	0xFF;
static const	uint8_t			mc_hall_states_next[8]	=	{0xFF,0x05,0x03,0x01,0x06,0x04,0x02,0xFF};
static const	uint8_t			mc_hall_states_prev[8]	=	{0xFF,0x03,0x06,0x02,0x05,0x01,0x04,0xFF};

static volatile mc_direction_t	mc_hall_dir				=	MC_DIR_CW;
static volatile	mc_direction_t	mc_dir_now				=	MC_DIR_CW;

static volatile	int16_t			mc_rpm_ref				=	0;
static volatile	int16_t			mc_rpm_now				=	0;
static volatile double			mc_rpm_last				=	0;
static volatile	double			mc_rpm_filter_last		=	0;

static volatile	int16_t			mc_rev_ref				=	0;
static volatile	int16_t			mc_rev_now				=	0;

static volatile	double			mc_current_now			=	0;

static			pid_data_t		pid_position;
static 			double			pid_position_out		=	0;

static			pid_data_t		pid_speed;
static 			double			pid_speed_out			=	0;

static			pid_data_t		pid_current;
static 			double			pid_current_out			=	0;

static void _timer0_start(void)
{
	power_timer0_enable();

	TCCR0A	=	_BV(WGM01); // CTC mode
	TCCR0B	=	_BV(CS02) | _BV(CS00); // prescale 1024 = 15625Hz = 0.064ms
	OCR0A	=	250; //   set overflow value for 16ms @ 16MHz
	TIMSK0	=	_BV(OCIE0A);
}

static void _timer0_stop(void)
{
	TCCR0B	&=	~_BV(CS02) & ~_BV(CS01) & ~_BV(CS00);
}

/*
static void _timer1_start(void)
{
	power_timer1_enable();

	TCCR1A	=	_BV(WGM12); // CTC mode
	TCCR1B	=	_BV(CS11) | _BV(CS10); // prescale 64 = 250kHz = 4us
	OCR1A	=	1000; // set overflow for 4ms @ 16MHz
	TIMSK1	=	_BV(OCIE1A);
}

static void _timer1_stop(void)
{
	TCCR1B	&=	~_BV(CS12) & ~_BV(CS11) & ~_BV(CS10);
}
*/
static void _mc_hall_sensors_irq(void)
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
	
	pid_position.ErrorMax	=	PID_POSITION_KP*MOTOR_MAX_SPEED;
	pid_position.IntSumMax	=	PID_POSITION_KP*MOTOR_MAX_SPEED/(PID_POSITION_KI+1);
	pid_position.IntSum		=	0;
	pid_position.FeedbackLast	=	0;
	pid_position.Kp			=	PID_POSITION_KP;
	pid_position.Ki			=	PID_POSITION_KI;
	pid_position.Kd			=	PID_POSITION_KD;
	
	pid_speed.ErrorMax		=	PID_SPEED_KP*MOTOR_MAX_SPEED;
	pid_speed.IntSumMax		=	PID_SPEED_KP*MOTOR_MAX_SPEED/(PID_SPEED_KI+1);
	pid_speed.IntSum		=	0;
	pid_speed.FeedbackLast	=	0;
	pid_speed.Kp			=	PID_SPEED_KP;
	pid_speed.Ki			=	PID_SPEED_KI;
	pid_speed.Kd			=	PID_SPEED_KD;
	
	pid_current.ErrorMax	=	MOTOR_MAX_CURRENT;
	pid_current.IntSumMax	=	PID_CURRENT_KP*MOTOR_MAX_CURRENT/(PID_CURRENT_KI+1);
	pid_current.IntSum		=	0;
	pid_current.FeedbackLast=	0;
	pid_current.Kp			=	PID_CURRENT_KP;
	pid_current.Ki			=	PID_CURRENT_KI;
	pid_current.Kd			=	PID_CURRENT_KD;

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
		_mc_hall_sensors_irq();
		_timer0_start();
		//_timer1_start();
	}else{
		ata6844_coast(1);
		ata6844_enabled(0);
		psc_set_dutycycle(0x0000);
		psc_run(0);
		_timer0_stop();
		//_timer1_stop();
		mc_rpm_ref=0;
		mc_rpm_now=0;
		mc_rev_ref=0;
		mc_rev_now=0;
		mc_hall_rev_ticks=0;
		mc_hall_rpm_ticks=0;
	}
	pid_speed.IntSum=0;
	pid_speed.FeedbackLast=0;
	pid_position.IntSum=0;
	pid_position.FeedbackLast=0;
	pid_current.IntSum=0;
	pid_current.FeedbackLast=0;
}

void mc_coast(void)
{
	ata6844_coast(1);
}

void mc_rpm_set(int16_t speed)
{
	if (speed>MOTOR_MAX_SPEED)	speed = MOTOR_MAX_SPEED;
	if (speed<-MOTOR_MAX_SPEED)	speed = -MOTOR_MAX_SPEED;

	mc_rpm_ref	=	speed;
	mc_mode		=	MC_MODE_SPEED;
}

int16_t mc_rpm_get(void)
{
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
	return (uint16_t)mc_current_now;
}

uint8_t mc_voltage_get(void)
{
	uint32_t voltage;
	voltage	= ( (uint32_t)(adc_get_channel(ADC_CHANNEL_9))*MOTOR_MAX_VOLTAGE/1023 );
	return (uint8_t)voltage;
}

int8_t mc_temp_get(void)
{
	int32_t temp;
	// (ADC*(IntRefVol[mV]/Res[1])-ZeroDeg[mV])/VpD[mV/degC]
	temp	= (int32_t)((double)(adc_get_channel(ADC_CHANNEL_TEMP))*(2560.0000/1023.0000)-699.6923)/2.4923;
	return (int8_t)temp;
}

ISR(PCINT2_vect)
{
	mc_hall_rpm_ticks++;
	_mc_hall_sensors_irq();
}

ISR(TIMER0_COMPA_vect)
{
	double		mc_rpm_filter;
	double		mc_rpm_tmp;
	double		mc_rev_tmp;
	double		mc_rev_hall;
	double		mc_current_tmp;

	// Calc speed from Hall interrupts
	mc_rpm_tmp				=	(double)(mc_hall_rpm_ticks)*3750.00/(MOTOR_NUM_POLES*6.00);
	mc_hall_rpm_ticks		=	0;

	// Check for direction
	if(mc_hall_dir==MC_DIR_CCW)
		mc_rpm_tmp=-mc_rpm_tmp;
		
	// Digital filter for speed calc
	mc_rpm_filter		=	0.8519*mc_rpm_filter_last + 0.07407*mc_rpm_tmp + 0.07407*mc_rpm_last;
	//mc_rpm_filter		=	0.9841*mc_rpm_filter_last + 0.007937*mc_rpm_tmp + 0.007937*mc_rpm_last;
	mc_rpm_last			=	mc_rpm_tmp;
	mc_rpm_filter_last	=	mc_rpm_filter;
	mc_rpm_now			=	(int16_t)(mc_rpm_filter);
	
	// Position calc
	mc_rev_tmp		=	(double)(mc_rev_ref)*(MOTOR_NUM_POLES*6.00);
	mc_rev_hall		=	(double)(mc_hall_rev_ticks);
	mc_rev_now		=	(int16_t)(mc_rev_hall/(MOTOR_NUM_POLES*6.00));

	if(mc_mode==MC_MODE_POSITION)
	{
		// P position controller
		pid_position_out	=	pid_controller(&mc_rev_tmp, &mc_rev_hall, &pid_position);

		// PI speed controller
		pid_speed_out		=	pid_controller(&pid_position_out, &mc_rpm_filter, &pid_speed);
	}else{
		// PI speed controller
		mc_rpm_tmp			=	(double)mc_rpm_ref;
		pid_speed_out		=	pid_controller(&mc_rpm_tmp, &mc_rpm_filter, &pid_speed);
	}
	
	/*
	if(pid_speed_out<0)
	{
		mc_dir_now	=	MC_DIR_CCW;
		pid_speed_out = -pid_speed_out;
	}else{
		mc_dir_now	=	MC_DIR_CW;
	}
	
	pid_speed_out = pid_speed_out*PSC_DUTY_MAX/MOTOR_MAX_SPEED;
	psc_set_dutycycle( (int16_t)(pid_speed_out) );
	*/
	
	// PI current controller
	//pid_speed_out	=	pid_speed_out*MOTOR_MAX_CURRENT/MOTOR_MAX_SPEED;
	mc_current_tmp	=	mc_current_now;
	pid_current_out	=	pid_controller(&pid_speed_out, &mc_current_tmp, &pid_current);
	
	if(pid_current_out<0)
	{
		mc_dir_now	=	MC_DIR_CCW;
		pid_current_out = -pid_current_out;
	}else{
		mc_dir_now	=	MC_DIR_CW;
	}
	
	pid_current_out	=	pid_current_out*PSC_DUTY_MAX/MOTOR_MAX_CURRENT;
	psc_set_dutycycle( (int16_t)(pid_current_out) );
}

/*
ISR(TIMER1_COMPA_vect)
{
	//empty
}
*/

ISR(ADC_vect)
{
	mc_current_now	=	(double)(ADC)*MOTOR_MAX_CURRENT/1023.00;
}