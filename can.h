#ifndef CAN_H_
#define CAN_H_

#if	\
	defined(__AVR_AT90CAN32__)  || \
	defined(__AVR_AT90CAN64__)  || \
	defined(__AVR_AT90CAN128__)

#define CAN_MOB_NUM		15

#elif \
	defined(__AVR_ATmega32C1__)  || \
	defined(__AVR_ATmega64C1__)  || \
	defined(__AVR_ATmega16M1__)  || \
	defined(__AVR_ATmega32M1__)  || \
	defined(__AVR_ATmega64M1__)

#define CAN_MOB_NUM		6

#else
#error "CAN is not supported on this MCU!"
#endif

#define CAN_MSG_ACCEPTED				0x00
#define CAN_MSG_REFUSED					0xFF
#define CAN_MSG_COMPLETED				0x00
#define CAN_MSG_NOT_COMPLETED			0x01
#define CAN_MSG_ERROR					0x02
#define CAN_MSG_ABORTED					0xFF

#define CAN_MOB_TX_OK					_BV(TXOK)											// 0x40
#define CAN_MOB_RX_OK					_BV(RXOK)											// 0x20
#define CAN_MOB_RX_DLCW					( _BV(RXOK) | _BV(DLCW) )							// 0xA0
#define CAN_MOB_PENDING					( _BV(RXOK) | _BV(TXOK) )							// 0x60
#define CAN_MOB_ACK_ERROR				(_BV(AERR))											// 0x01
#define CAN_MOB_FORM_ERROR				(_BV(FERR))											// 0x02
#define CAN_MOB_CRC_ERROR				(_BV(CERR))											// 0x04
#define CAN_MOB_STUFF_ERROR				(_BV(SERR))											// 0x08
#define CAN_MOB_BIT_ERROR				(_BV(BERR))											// 0x10
#define CAN_MOB_NOT_REACHED				(_BV(BERR)|_BV(SERR)|_BV(CERR)|_BV(FERR)|_BV(AERR))	// 0x1F
#define CAN_MOB_NOT_OK					0x00												// 0x00
#define CAN_MOB_DISABLED				0xFF												// 0xFF
#define CAN_MOB_NOT_FREE				0xFF												// 0xFF

typedef enum can_bitrate
{
	CAN_BITRATE_10   = 0x00,
	CAN_BITRATE_20   = 0x01,
	CAN_BITRATE_50	 = 0x02,
	CAN_BITRATE_100	 = 0x03,
	CAN_BITRATE_125	 = 0x04,
	CAN_BITRATE_200	 = 0x05,
	CAN_BITRATE_250	 = 0x06,
	CAN_BITRATE_500  = 0x07,
	CAN_BITRATE_1000 = 0x08,
	CAN_BITRATE_AUTO = 0xFF
} can_bitrate_t;

typedef struct can_message
{
	uint8_t		mob;
	uint8_t		status;
	uint32_t	id;
	uint32_t	mask;
	uint8_t		rtr;
	uint8_t		ide;
	uint8_t		dlc;
	uint8_t		data[8];
	uint16_t	timestamp;

} can_message_t;

void	can_init		(uint8_t baudrate);
uint8_t can_msg_tx		(can_message_t* pt_can_msg);
uint8_t can_msg_rx		(can_message_t* pt_can_msg, uint8_t reply);
uint8_t can_msg_status	(can_message_t* pt_can_msg);
void	can_msg_abort	(can_message_t* pt_can_msg);

#endif /* CAN_H_ */
