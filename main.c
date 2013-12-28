#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <util/delay.h>

#include "adc.h"
#include "can.h"
#include "config.h"
#include "mc.h"
#include "uart.h"

#include <util/delay.h>

static void board_init(void);

int main(void)
{
	can_message_t	msg_cmd;
	can_message_t	msg_response;
	can_message_t	msg_position;
	int8_t			temp;

	int16_t			speed, angle;
	uint16_t		current;
	uint8_t			voltage;

	board_init();

	#if DEBUG_ME
	printf_P(PSTR("UART: 115200,8,n,1\n\r"));
	#endif
/*
	while(1)
	{
		msg_cmd.id			=	CAN_BASE_ID;
		msg_cmd.mask		=	0x000007f0;
		msg_cmd.ide			=	0;
		msg_cmd.rtr			=	1;
		msg_cmd.dlc			=	8;

		msg_position.id		=	CAN_MOTOR_POS_SET;
		msg_position.mask	=	0x000007ff;
		msg_position.ide	=	0;
		msg_position.rtr	=	0;
		msg_position.dlc	=	2;

		while(can_msg_rx(&msg_cmd,0)!=CAN_MSG_ACCEPTED);
		_delay_ms(1);
		if(can_msg_status(&msg_cmd)==CAN_MSG_COMPLETED)
		{
			switch (msg_cmd.id)
			{
				case CAN_BASE_ID:
					#if DEBUG_CAN
					printf_P(PSTR("motor id: 0x%x"), msg_cmd.id);
					#endif
					break;

				case CAN_MOTOR_START:
					#if DEBUG_CAN
					printf_P(PSTR("motor run"));
					#endif
					mc_run(1);
					break;

				case CAN_MOTOR_STOP:
					#if DEBUG_CAN
					printf_P(PSTR("motor stop"));
					#endif
					mc_run(0);
					break;

				case CAN_MOTOR_COAST:
					#if DEBUG_CAN
					printf_P(PSTR("motor coast"));
					#endif
					mc_coast();
					break;
			}
			#if DEBUG_CAN
			printf_P(PSTR("\n\r"));
			#endif
		}else{
			can_msg_abort(&msg_cmd);
		}

		while(can_msg_rx(&msg_position,0)!=CAN_MSG_ACCEPTED);
		_delay_ms(1);
		if(can_msg_status(&msg_position)==CAN_MSG_COMPLETED)
		{
			angle	=	((int16_t)(msg_position.data[1])<<8) + (int16_t)(msg_position.data[0]);
			mc_angle_set(angle);
			#if DEBUG_CAN
			printf_P(PSTR("motor angle set: %d\n\r"),angle);
			#endif
		}else{
			can_msg_abort(&msg_position);
		}

	}
	*/

	while(1)
	{
		msg_cmd.id			=	CAN_BASE_ID;
		msg_cmd.mask		=	0x000007f0;
		msg_cmd.ide			=	0;
		msg_cmd.rtr			=	0;
		msg_cmd.dlc			=	8;


		while(can_msg_rx(&msg_cmd,0)!=CAN_MSG_ACCEPTED);
		while(can_msg_status(&msg_cmd)==CAN_MSG_NOT_COMPLETED);

		#if DEBUG_CAN
		printf_P(PSTR("id: %ld, dlc: %d, ide: %d, rtr: %d --> "),msg_cmd.id,msg_cmd.dlc,msg_cmd.ide,msg_cmd.rtr);
		#endif
		switch (msg_cmd.id)
		{
			case CAN_BASE_ID:
				#if DEBUG_CAN
				printf_P(PSTR("motor id: 0x%x"), msg_cmd.id);
				#endif
				break;

			case CAN_MOTOR_START:
				#if DEBUG_CAN
				printf_P(PSTR("motor run"));
				#endif
				mc_run(1);
				break;

			case CAN_MOTOR_STOP:
				#if DEBUG_CAN
				printf_P(PSTR("motor stop"));
				#endif
				mc_run(0);
				break;

			case CAN_MOTOR_COAST:
				#if DEBUG_CAN
				printf_P(PSTR("motor coast"));
				#endif
				mc_coast();
				break;

			case CAN_MOTOR_SPEED_SET:
				speed	=	((int16_t)(msg_cmd.data[1])<<8) + (int16_t)(msg_cmd.data[0]);
				mc_rpm_set(speed);
				#if DEBUG_CAN
				printf_P(PSTR("motor speed set: %drpm"),speed);
				#endif
				break;

			case CAN_MOTOR_SPEED_GET:
				msg_response.id=msg_cmd.id;
				msg_response.mask=0xffffffff;
				msg_response.ide=0;
				msg_response.rtr=0;
				msg_response.dlc=2;
				speed = mc_rpm_get();
				msg_response.data[1]=(speed>>8);
				msg_response.data[0]=(uint8_t)(speed);
				while(can_msg_tx(&msg_response)!=CAN_MSG_ACCEPTED);
				while(can_msg_status(&msg_response)==CAN_MSG_NOT_COMPLETED);
				#if DEBUG_CAN
				printf_P(PSTR("motor speed get: %drpm"),speed);
				#endif
				break;

			case CAN_MOTOR_CURRENT_GET:
				msg_response.id=msg_cmd.id;
				msg_response.mask=0xffffffff;
				msg_response.ide=0;
				msg_response.rtr=0;
				msg_response.dlc=2;
				current	= mc_current_get();
				//msg_respone.data[3]=(uint8_t)(current>>24);
				//msg_respone.data[2]=(uint8_t)(current>>16);
				msg_response.data[1]=(uint8_t)(current>>8);
				msg_response.data[0]=(uint8_t)(current);
				while(can_msg_tx(&msg_response)!=CAN_MSG_ACCEPTED);
				while(can_msg_status(&msg_response)==CAN_MSG_NOT_COMPLETED);
				#if DEBUG_CAN
				printf_P(PSTR("motor current sent: %4.2fmA"),(double)current);
				#endif
				break;

			case CAN_MOTOR_VOLTAGE_GET:
				msg_response.id=msg_cmd.id;
				msg_response.mask=0xffffffff;
				msg_response.ide=0;
				msg_response.rtr=0;
				msg_response.dlc=1;
				voltage	= mc_voltage_get();
				//msg_respone.data[3]=(uint8_t)(voltage>>24);
				//msg_respone.data[2]=(uint8_t)(voltage>>16);
				//msg_respone.data[1]=(uint8_t)(voltage>>8);
				msg_response.data[0]=voltage;
				while(can_msg_tx(&msg_response)!=CAN_MSG_ACCEPTED);
				while(can_msg_status(&msg_response)==CAN_MSG_NOT_COMPLETED);
				#if DEBUG_CAN
				printf_P(PSTR("motor voltage sent: %4.2fV"),(double)voltage);
				#endif
				break;

			case CAN_MOTOR_ANGLE_SET:
				angle	=	((int16_t)(msg_cmd.data[1])<<8) + (int16_t)(msg_cmd.data[0]);
				mc_rev_set(angle);
				#if DEBUG_CAN
				printf_P(PSTR("motor angle set: %d"),angle);
				#endif
				break;

			case CAN_MOTOR_RESERVED2:
				break;

			case CAN_MOTOR_RESERVED3:
				break;

			case CAN_MOTOR_RESERVED4:
				break;

			case CAN_MOTOR_RESERVED5:
				break;

			case CAN_MOTOR_RESERVED6:
				break;

			case CAN_MOTOR_TEMP_GET:
				msg_response.id=msg_cmd.id;
				msg_response.mask=0xffffffff;
				msg_response.ide=0;
				msg_response.rtr=0;
				msg_response.dlc=1;
				temp	= mc_temp_get();
				msg_response.data[0]=temp;
				while(can_msg_tx(&msg_response)!=CAN_MSG_ACCEPTED);
				while(can_msg_status(&msg_response)==CAN_MSG_NOT_COMPLETED);
				#if DEBUG_CAN
				printf_P(PSTR("motor temp out: %d"), temp);
				#endif
				break;

			default:
				break;
		}
		#if DEBUG_CAN
		printf_P(PSTR("\n\r"));
		#endif

		//printf("%ld\n\r",mc_get_hall_cycles());
		//_delay_ms(10);

	}

}

static void board_init(void)
{
	#if \
		defined(__AVR_ATmega32C1__)  || \
		defined(__AVR_ATmega64C1__)  || \
		defined(__AVR_ATmega16M1__)  || \
		defined(__AVR_ATmega32M1__)  || \
		defined(__AVR_ATmega64M1__)
	power_all_disable();

	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	PORTB = 0xFF;
	PORTC = 0xFF;
	PORTD = 0xFF;
	#endif

	can_init(CAN_BITRATE_250);
	mc_init();
	#if DEBUG_ME
	uart_init(UART_BAUDRATE_115200);
	#endif

	//enable interrupts
	sei();
}
