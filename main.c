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

static void _board_init(void)
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

	#if DEBUG_ME
	uart_init(UART_BAUDRATE);
	#endif
	mc_init();
	can_init(CAN_BAUDRATE);

	//enable interrupts
	sei();
}

int main(void)
{
	can_message_t	cmd_set_msg;
	can_message_t	cmd_get_msg;
	can_message_t	cmd_resp_msg;
	uint8_t	cmd_set_data[3];
	uint8_t	cmd_get_data[8];
	
	int16_t revs, rpms, current;
	
	_board_init();
	
	#if DEBUG_ME
	printf_P(PSTR("Debug: UART->115200,8,n,1\n\r"));
	#endif
	
	cmd_set_msg.id		=	CAN_CMD_SET;
	cmd_set_msg.mask	=	0x000007ff;
	cmd_set_msg.ide		=	0;
	cmd_set_msg.rtr		=	0;
	cmd_set_msg.dlc		=	3;
	cmd_set_msg.pt_data	=	&cmd_set_data[0];
	
	cmd_get_msg.id		=	CAN_CMD_GET;
	cmd_get_msg.mask	=	0x000007ff;
	cmd_get_msg.ide		=	0;
	cmd_get_msg.rtr		=	1;
	cmd_get_msg.dlc		=	0;
	
	can_msg_rx(&cmd_set_msg);
	can_msg_rx(&cmd_get_msg);
	
	while(1)
	{
		switch(cmd_set_msg.status)
		{
			case CAN_MSG_COMPLETED:
				switch(cmd_set_data[0])
				{
					case CMD_MOTOR_STOP:
						mc_run(0);
						#if DEBUG_CAN
						printf_P(PSTR("motor stop"));
						#endif
						break;
					case CMD_MOTOR_START:
						mc_run(1);
						#if DEBUG_CAN
						printf_P(PSTR("motor start"));
						#endif
						break;
					case CMD_MOTOR_COAST:
						mc_coast();
						#if DEBUG_CAN
						printf_P(PSTR("motor coast"));
						#endif
						break;
					case CMD_MOTOR_BREAK:
						#if DEBUG_CAN
						printf_P(PSTR("motor break"));
						#endif
						break;
					case CMD_REV_SET:
						revs = ((int16_t)(cmd_set_data[2])<<8) + (int16_t)(cmd_set_data[1]);
						mc_rev_set(revs);
						#if DEBUG_CAN
						printf_P(PSTR("motor rev set: %d"),revs);
						#endif
						break;
					case CMD_RPM_SET:
						rpms = ((int16_t)(cmd_set_data[2])<<8) + (int16_t)(cmd_set_data[1]);
						mc_rpm_set(rpms);
						#if DEBUG_CAN
						printf_P(PSTR("motor rpm set: %d"),rpms);
						#endif
						break;
						
					default:
						#if DEBUG_CAN
						printf_P(PSTR("unknown cmd"));
						#endif
						break;
				}
				#if DEBUG_CAN
				printf_P(PSTR("\n\r"));
				#endif
				cmd_set_msg.status = CAN_MSG_PENDING;
				break;
			
			case CAN_MSG_ERROR:
				can_msg_rx(&cmd_set_msg);
				break;
		}
		
		switch(cmd_get_msg.status)
		{
			case CAN_MSG_COMPLETED:
				revs	=	mc_rev_get();
				rpms	=	mc_rpm_get();
				current	=	mc_current_get();
				
				cmd_get_data[0]		=	(uint8_t)(revs);
				cmd_get_data[1]		=	(uint8_t)(revs>>8);
				cmd_get_data[2]		=	(uint8_t)(rpms);
				cmd_get_data[3]		=	(uint8_t)(rpms>>8);
				cmd_get_data[4]		=	(uint8_t)(current);
				cmd_get_data[5]		=	(uint8_t)(current>>8);
				cmd_get_data[6]		=	0;
				cmd_get_data[7]		=	0;
				
				cmd_resp_msg.id		=	CAN_CMD_GET;
				cmd_resp_msg.mask	=	0x000007ff;
				cmd_resp_msg.ide	=	0;
				cmd_resp_msg.rtr	=	0;
				cmd_resp_msg.dlc	=	8;
				cmd_resp_msg.pt_data=	&cmd_get_data[0];
				
				can_msg_tx(&cmd_resp_msg);
				//_delay_us(5);
				#if DEBUG_CAN
				printf_P(PSTR("get data\n\r"));
				#endif
				cmd_get_msg.status = CAN_MSG_PENDING;
				break;
			
			case CAN_MSG_ERROR:
				can_msg_rx(&cmd_get_msg);
				break;
		}
		_delay_us(5);
	}	
}


