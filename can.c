#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "can.h"

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

#define CAN_MIN_TQ                          8
#define CAN_MAX_TQ                          25

static volatile	can_message_t* can_messages[CAN_MOB_MAX];

static uint8_t _can_set_baudrate(uint32_t Baudrate, uint8_t SamplingPoint, uint8_t Tsjw)
{
	uint8_t divider;
	uint8_t Tbit;
	uint8_t Tprs;
	uint8_t Tphs1;
	uint8_t Tphs2;
	uint8_t BRP = 1;

	divider = F_CPU / Baudrate;

	Tbit = divider;
	while ((Tbit >= CAN_MAX_TQ) | (Tbit <= CAN_MIN_TQ))
	{
		Tbit = (Tbit >> 1);
		BRP *= 2;
	}

	Tphs2 = (Tbit * (100 - SamplingPoint) / 100);
	Tphs1 = 0;

	for(Tprs = 8; Tprs > 0; Tprs--)
	{
		if((1 + Tprs + Tphs2 + Tphs2) <= (Tbit + 8 - Tphs2) \
		&& ((Tbit - 1 - Tprs - Tphs2) >= Tphs2))
		{
			Tphs1 = Tbit - 1 - Tprs - Tphs2;
			break;
		}
	}

	if (1 == BRP)
	{
		Tphs1 = Tphs2 + 1;
		Tphs2 -= 1;
	}

	if (divider == (BRP) * (Tprs + Tphs1 + Tphs2 + 1))
	{
		CANBT1 = ((BRP-1) << BRP0);
		CANBT2 = ((Tprs-1) << PRS0) | ((Tsjw-1) << SJW0);
		CANBT3 = ((Tphs2-1) << PHS20) | ((Tphs1-1) << PHS10) | (1 << SMP);
		return 0;
	}

	return 0xFF;
}

static uint8_t _find_free_mob(void)
{
	uint8_t save_canpage, mob;
	
	save_canpage = CANPAGE;
	
	for (mob = 0;mob < CAN_MOB_MAX;mob++)
	{
		CANPAGE = mob << 4;
		if ( (CANCDMOB & 0xC0) == 0x00 )
		{
			CANPAGE = save_canpage;
			return (mob);
		}
	}
	CANPAGE = save_canpage;
	return 0xFF;
}

static __attribute__ ((gnu_inline)) inline void _disable_mob_interrupt(uint8_t mob)
{
	if (mob < 8)
	{
		CANIE2 &= ~(1 << mob);
	}else{
		CANIE1 &= ~(1 << (mob - 8));
	}
}

static __attribute__ ((gnu_inline)) inline void _enable_mob_interrupt(uint8_t mob)
{
	if (mob < 8)
	{
		CANIE2 |= (1 << mob);
	}else{
		CANIE1 |= (1 << (mob - 8));
	}
}

static __attribute__ ((gnu_inline)) inline void _clear_mob(void)
{
	CANSTMOB = 0x00;
	CANCDMOB = 0x00;
	CANIDT4  = 0X00;
	CANIDT3  = 0x00;
	CANIDT2  = 0x00;
	CANIDT1  = 0x00;
	CANIDM4  = 0x00;
	CANIDM3  = 0x00;
	CANIDM2  = 0x00;
	CANIDM1  = 0x00;
}

static void	_can_tx_irq(uint8_t mob)
{
	CANSTMOB = 0x00; // clear MOb status
	CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
	_disable_mob_interrupt(mob);
	can_messages[mob]->mob		=	CAN_MOB_DISABLED;
	can_messages[mob]->status	=	CAN_MSG_COMPLETED;
}

static void _can_rx_irq(uint8_t mob)
{
	uint8_t cpt;
	can_messages[mob]->dlc	=	((CANCDMOB & 0x0F)>>DLC0);
	can_messages[mob]->rtr	=	((CANIDT4 & _BV(RTRTAG)) >> RTRTAG);
	can_messages[mob]->ide	=	((CANCDMOB & _BV(IDE))>>IDE);

	//get data for data frame
	if (!can_messages[mob]->rtr)
	{
		for (cpt=0;cpt<can_messages[mob]->dlc;cpt++)
			*(can_messages[mob]->pt_data+cpt) = CANMSG;
	}

	if (can_messages[mob]->ide)
	{
		//get ext id
		can_messages[mob]->id	=	(uint32_t)(CANIDT4)>>3;
		can_messages[mob]->id	|=	(uint32_t)(CANIDT3)<<5;
		can_messages[mob]->id	|=	(uint32_t)(CANIDT2)<<13;
		can_messages[mob]->id	|=	(uint32_t)(CANIDT1)<<21;
	}else{
		//get std id
		can_messages[mob]->id	=	(uint32_t)(CANIDT2)>>5;
		can_messages[mob]->id	|=	(uint32_t)(CANIDT1)<<3;
	}
	can_messages[mob]->timestamp = CANSTM; // get timestamp
	can_messages[mob]->status	=	CAN_MSG_COMPLETED;
	CANSTMOB = 0x00; // clear MOb status
	_enable_mob_interrupt(mob);
	CANCDMOB |= _BV(CONMOB1);
}

void can_init(uint32_t baudrate)
{
	uint8_t mob;
	#if	\
	defined(__AVR_AT90CAN32__)  || \
	defined(__AVR_AT90CAN64__)  || \
	defined(__AVR_AT90CAN128__)
	DDRD	|=	_BV(PD5);
	DDRD	&=	~_BV(PD6);
	PORTD	&= ~_BV(PD6) & ~_BV(PD5);
	#elif \
	defined(__AVR_ATmega32C1__)  || \
	defined(__AVR_ATmega64C1__)  || \
	defined(__AVR_ATmega16M1__)  || \
	defined(__AVR_ATmega32M1__)  || \
	defined(__AVR_ATmega64M1__)
	power_can_enable();
	DDRC	|=	_BV(PC2);
	DDRC	&=	~_BV(PC3);
	PORTC	&=	~_BV(PC3) & ~_BV(PC2);
	#endif

	// CAN reset
	CANGCON  =  _BV(SWRES);

	// Clear all MObs
	for(mob = 0; mob < CAN_MOB_MAX; mob++)
	{
		CANPAGE = (mob<<4);
		_clear_mob();
	}

	_can_set_baudrate(baudrate, 75, 1);

	// enable general can, receive and transmit interrupt
	CANGIT = 0;
	CANGIE	=	_BV(ENIT) | _BV(ENRX) | _BV(ENTX);

	// CAN enable
	CANGCON |= _BV(ENASTB);
}

uint8_t	can_msg_tx(can_message_t* pt_can_msg)
{
	uint8_t mob,cpt;
	
	mob = _find_free_mob();
	if (mob >= CAN_MOB_MAX)
		return CAN_MSG_REFUSED;
		
	CANPAGE = (mob << 4);
	_clear_mob();
	
	can_messages[mob]			=	pt_can_msg;
	can_messages[mob]->mob		=	mob;
	can_messages[mob]->status	=	CAN_MSG_PENDING;
	
	CANCDMOB = can_messages[mob]->dlc; // set DLC

	if (can_messages[mob]->ide)
	{
		CANIDT4=(uint8_t)(can_messages[mob]->id<<3);
		CANIDT3=(uint8_t)(can_messages[mob]->id>>5);
		CANIDT2=(uint8_t)(can_messages[mob]->id>>13);
		CANIDT1=(uint8_t)(can_messages[mob]->id>>21);
		CANCDMOB |= _BV(IDE);
	}else{
		CANIDT4=0x00;
		CANIDT3=0x00;
		CANIDT2=(uint8_t)(can_messages[mob]->id)<<5;
		CANIDT1=(uint8_t)(can_messages[mob]->id>>3);
	}

	if(can_messages[mob]->rtr)
	{
		CANIDT4  |= _BV(RTRTAG);
	}else{
		for (cpt=0;cpt<can_messages[mob]->dlc;cpt++)
			CANMSG = *(can_messages[mob]->pt_data + cpt);
	}
	
	_enable_mob_interrupt(mob);
	 CANCDMOB |= (1<<CONMOB0);

	 return CAN_MSG_ACCEPTED;
}

uint8_t can_msg_rx (can_message_t* pt_can_msg)
{
	uint8_t mob;
	
	mob = _find_free_mob();
	if (mob >= CAN_MOB_MAX)
		return CAN_MSG_REFUSED;
	
	CANPAGE = (mob << 4);
	_clear_mob();

	can_messages[mob]			=	pt_can_msg;
	can_messages[mob]->mob		=	mob;
	can_messages[mob]->status	=	CAN_MSG_PENDING;
	
	CANCDMOB = can_messages[mob]->dlc;

	if (can_messages[mob]->ide)
	{
		CANIDT4=(uint8_t)(can_messages[mob]->id<<3);
		CANIDT3=(uint8_t)(can_messages[mob]->id>>5);
		CANIDT2=(uint8_t)(can_messages[mob]->id>>13);
		CANIDT1=(uint8_t)(can_messages[mob]->id>>21);
		CANCDMOB |= _BV(IDE);
	}else{
		CANIDT4=0x00;
		CANIDT3=0x00;
		CANIDT2=(uint8_t)(can_messages[mob]->id)<<5;
		CANIDT1=(uint8_t)(can_messages[mob]->id>>3);
	}

	if(can_messages[mob]->mask!=0)
	{
		if (can_messages[mob]->ide)
		{
			CANIDM4=(uint8_t)(can_messages[mob]->mask<<3);
			CANIDM3=(uint8_t)(can_messages[mob]->mask>>5);
			CANIDM2=(uint8_t)(can_messages[mob]->mask>>13);
			CANIDM1=(uint8_t)(can_messages[mob]->mask>>21);
		}else{
			CANIDM4=0x00;
			CANIDM3=0x00;
			CANIDM2=(uint8_t)(can_messages[mob]->mask)<<5;
			CANIDM1=(uint8_t)(can_messages[mob]->mask>>3);
		}
		CANIDM4 |= _BV(IDEMSK);
	}

	if(can_messages[mob]->rtr)
	{
		CANIDT4  |= _BV(RTRTAG);
		CANIDM4	 |= _BV(RTRMSK);
	}
	
	_enable_mob_interrupt(mob);
	CANCDMOB |= _BV(CONMOB1); // enable receive

	return CAN_MSG_ACCEPTED;
}

uint8_t can_msg_abort(can_message_t* pt_can_msg)
{
	uint8_t save_canpage;
	save_canpage = CANPAGE;
	if (pt_can_msg->status==CAN_MSG_PENDING)
	{
		CANPAGE  = ((pt_can_msg->mob)<<4); // set MOb
		CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
		CANSTMOB = 0x00;  // clear MOb status
		pt_can_msg->mob = CAN_MOB_DISABLED;
	}
	CANPAGE = save_canpage;
	return CAN_MSG_ABORTED;	
}

ISR(CAN_INT_vect)
{
	uint8_t save_canpage, mob, mob_status;
	
	if( (CANHPMOB & 0xF0) != 0xF0 )
	{
		save_canpage = CANPAGE;
		
		CANPAGE = CANHPMOB & 0xF0; // set MOb
		mob = (CANHPMOB >> 4);

		mob_status = CANSTMOB & (_BV(DLCW)|_BV(TXOK)|_BV(RXOK));
		
		switch(mob_status)
		{
			case CAN_MOB_NOT_OK:
				can_messages[mob]->status	=	CAN_MSG_NOT_COMPLETED;
				break;
			case CAN_MOB_TX_OK:
				_can_tx_irq(mob);
				break;
			case CAN_MOB_RX_OK:
			case CAN_MOB_RX_DLCW:
				_can_rx_irq(mob);
				break;
				
			default:
				CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
				CANSTMOB = 0x00; // clear MOb status
				can_messages[mob]->mob		=	CAN_MOB_DISABLED;
				can_messages[mob]->status	=	CAN_MSG_ERROR;
				break;
		}
		CANPAGE = save_canpage;
	}else{
		CANGIT	&=	~0x7F; // clear general int
	}
}
