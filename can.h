#ifndef CAN_H_
#define CAN_H_

#if	\
	defined(__AVR_AT90CAN32__)  || \
	defined(__AVR_AT90CAN64__)  || \
	defined(__AVR_AT90CAN128__)

#define CAN_MOB_MAX		15

#elif \
	defined(__AVR_ATmega32C1__)  || \
	defined(__AVR_ATmega64C1__)  || \
	defined(__AVR_ATmega16M1__)  || \
	defined(__AVR_ATmega32M1__)  || \
	defined(__AVR_ATmega64M1__)

#define CAN_MOB_MAX		6

#else
#error "CAN is not supported on this MCU!"
#endif

#define CAN_MSG_ACCEPTED				0x01
#define CAN_MSG_PENDING					0x02
#define CAN_MSG_REFUSED					0x03
#define CAN_MSG_COMPLETED				0x04
#define CAN_MSG_NOT_COMPLETED			0x05
#define CAN_MSG_ERROR					0x06
#define CAN_MSG_ABORTED					0x07

typedef struct can_message
{
	uint8_t		mob;
	uint8_t		status;
	uint32_t	id;
	uint32_t	mask;
	uint8_t		rtr;
	uint8_t		ide;
	uint8_t		dlc;
	uint8_t*	pt_data;
	uint16_t	timestamp;
} can_message_t;

void	can_init		(uint32_t baudrate);
uint8_t can_msg_tx		(can_message_t* pt_can_msg);
uint8_t can_msg_rx		(can_message_t* pt_can_msg);
uint8_t	can_msg_abort	(can_message_t* pt_can_msg);

#endif /* CAN_H_ */
