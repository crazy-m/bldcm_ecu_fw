#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sfr_defs.h>

#include "can.h"

#if F_CPU == 16000000UL
const uint8_t PROGMEM can_bitrate_table[9][3] = {
	// 10 kbps
	{0x7E,0x6E,0x7F},
	// 20 kbps
	{0x62,0x0C,0x37},
	// 50 kbps
	{0x26,0x0C,0x37},
	// 100 kbps
	{0x12,0x0C,0x37},
	// 125 kbps
	{0x0E,0x0C,0x37},
	// 200 kbps
	{0x08,0x0C,0x37},
	// 250 kbps
	{0x06,0x0C,0x37},
	// 500 kbps
	{0x02,0x0C,0x37},
	// 1 Mbps
	{0x00,0x0C,0x37}
};
#else
	#error "F_CPU frequency is not supported"
#endif

static uint8_t can_mob_status(void);
static void can_mob_clear(void);
static void can_mob_clear_all(void);
static uint8_t can_mob_get_free(void);

void can_init(can_bitrate_t bitrate)
{
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
	can_mob_clear_all();

	if (bitrate!=CAN_BITRATE_AUTO)
	{
		CANBT1 = pgm_read_byte(&can_bitrate_table[bitrate][0]);
		CANBT2 = pgm_read_byte(&can_bitrate_table[bitrate][1]);
		CANBT3 = pgm_read_byte(&can_bitrate_table[bitrate][2]);
	}else{
		//#error "CAN autobitrate not implemented!"
	}

	// CAN enable
	CANGCON |= _BV(ENASTB);
}

uint8_t can_msg_tx(can_message_t* pt_can_msg)
{
	uint8_t mob;
	uint8_t cpt;

	mob = can_mob_get_free();

	if(mob!=CAN_MOB_NOT_FREE)
	{
		pt_can_msg->mob=mob;
		pt_can_msg->status = CAN_MOB_PENDING;

		CANPAGE  = (mob<<4); // set mob
		can_mob_clear(); // clear mob

		CANCDMOB = pt_can_msg->dlc; // set DLC

		if (pt_can_msg->ide)
		{
			CANIDT4=(uint8_t)(pt_can_msg->id<<3);
			CANIDT3=(uint8_t)(pt_can_msg->id>>5);
			CANIDT2=(uint8_t)(pt_can_msg->id>>13);
			CANIDT1=(uint8_t)(pt_can_msg->id>>21);
			CANCDMOB |= _BV(IDE);
		}else{
			CANIDT4=0x00;
			CANIDT3=0x00;
			CANIDT2=(uint8_t)(pt_can_msg->id)<<5;
			CANIDT1=(uint8_t)(pt_can_msg->id>>3);
		}

		if(pt_can_msg->rtr)
		{
			CANIDT4  |= _BV(RTRTAG);
		}else{
			for (cpt=0;cpt<pt_can_msg->dlc;cpt++) \
				CANMSG = pt_can_msg->data[cpt];
		}

		CANCDMOB |= _BV(CONMOB0); // enable transmission
	}
	else
	{
		return CAN_MSG_REFUSED;
	}
	return CAN_MSG_ACCEPTED;
}

uint8_t can_msg_rx(can_message_t* pt_can_msg, uint8_t reply)
{
	uint8_t mob;
	uint8_t cpt;

	mob = can_mob_get_free();

	if(mob!=CAN_MOB_NOT_FREE)
	{
		pt_can_msg->mob=mob;
		pt_can_msg->status = CAN_MOB_PENDING;

		CANPAGE  = (mob<<4); // set mob
		can_mob_clear(); // clear mob

		CANCDMOB = pt_can_msg->dlc; // set DLC

		if (pt_can_msg->ide)
		{
			CANIDT4=(uint8_t)(pt_can_msg->id<<3);
			CANIDT3=(uint8_t)(pt_can_msg->id>>5);
			CANIDT2=(uint8_t)(pt_can_msg->id>>13);
			CANIDT1=(uint8_t)(pt_can_msg->id>>21);
			CANCDMOB |= _BV(IDE);
		}else{
			CANIDT4=0x00;
			CANIDT3=0x00;
			CANIDT2=(uint8_t)(pt_can_msg->id)<<5;
			CANIDT1=(uint8_t)(pt_can_msg->id>>3);
		}

		if(pt_can_msg->mask!=0)
		{
			if (pt_can_msg->ide)
			{
				CANIDM4=(uint8_t)(pt_can_msg->mask<<3);
				CANIDM3=(uint8_t)(pt_can_msg->mask>>5);
				CANIDM2=(uint8_t)(pt_can_msg->mask>>13);
				CANIDM1=(uint8_t)(pt_can_msg->mask>>21);
			}else{
				CANIDM4=0x00;
				CANIDM3=0x00;
				CANIDM2=(uint8_t)(pt_can_msg->mask)<<5;
				CANIDM1=(uint8_t)(pt_can_msg->mask>>3);
			}
			CANIDM4 |= _BV(IDEMSK);
		}

		if(pt_can_msg->rtr)
		{
			CANIDT4  |= _BV(RTRTAG);
			CANIDM4	 |= _BV(RTRMSK);
			if(reply)
			{
				for (cpt=0;cpt<pt_can_msg->dlc;cpt++) \
					CANMSG = pt_can_msg->data[cpt];
				CANCDMOB |= _BV(RPLV);
			}
		}

		CANCDMOB |= _BV(CONMOB1); // enable receive
	}
	else
	{
		return CAN_MSG_REFUSED;
	}
	return CAN_MSG_ACCEPTED;
}

uint8_t can_msg_status(can_message_t* pt_can_msg)
{
	uint8_t ret;
	uint8_t msg_status;
	uint8_t cpt;

	msg_status=pt_can_msg->status;

	if(	msg_status==CAN_MSG_ABORTED ||
		msg_status==CAN_MOB_NOT_REACHED ||
		msg_status==CAN_MOB_DISABLED)
		return CAN_MSG_ERROR;

	CANPAGE  = ((pt_can_msg->mob)<<4); // set MOb

	msg_status = can_mob_status();

	switch(msg_status)
	{
		case CAN_MOB_NOT_OK:
			ret=CAN_MSG_NOT_COMPLETED;
			break;

		case CAN_MOB_TX_OK:
			pt_can_msg->status=msg_status;
			pt_can_msg->timestamp = CANSTM; // get timestamp
			CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
			CANSTMOB = 0x00; // clear MOb status
			ret=CAN_MSG_COMPLETED;
			break;

		case CAN_MOB_RX_OK:
		case CAN_MOB_RX_DLCW:
			pt_can_msg->dlc	=	((CANCDMOB & 0x0F)>>DLC0);

			pt_can_msg->rtr	=	((CANIDT4 & _BV(RTRTAG)) >> RTRTAG);
			pt_can_msg->ide	=	((CANCDMOB & _BV(IDE))>>IDE);

			//get data for data frame
			if (!pt_can_msg->rtr)
			{
				for (cpt=0;cpt<pt_can_msg->dlc;cpt++)
					pt_can_msg->data[cpt]=CANMSG;
			}

			if (pt_can_msg->ide)
			{
				//get ext id
				pt_can_msg->id	=	(uint32_t)(CANIDT4)>>3;
				pt_can_msg->id	|=	(uint32_t)(CANIDT3)<<5;
				pt_can_msg->id	|=	(uint32_t)(CANIDT2)<<13;
				pt_can_msg->id	|=	(uint32_t)(CANIDT1)<<21;
			}else{
				//get std id
				pt_can_msg->id	=	(uint32_t)(CANIDT2)>>5;
				pt_can_msg->id	|=	(uint32_t)(CANIDT1)<<3;
			}
			pt_can_msg->status=msg_status;
			pt_can_msg->timestamp = CANSTM; // get timestamp
			CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
			CANSTMOB = 0x00; // clear MOb status
			ret=CAN_MSG_COMPLETED;
			break;

		default:
			pt_can_msg->status=msg_status;
			CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
			CANSTMOB = 0x00; // clear MOb status
			ret=CAN_MSG_ERROR;
			break;
	}
	return ret;
}

void can_msg_abort(can_message_t* pt_can_msg)
{
	uint8_t save_canpage;
	save_canpage = CANPAGE;
	if (pt_can_msg->status==CAN_MOB_PENDING)
	{
		CANPAGE  = ((pt_can_msg->mob)<<4); // set MOb
		CANCDMOB &= ~_BV(CONMOB1) & ~_BV(CONMOB0); // disable MOb
		CANSTMOB = 0x00;  // clear MOb status
		pt_can_msg->mob=0;
	}
	CANPAGE = save_canpage;
	pt_can_msg->status=CAN_MSG_ABORTED;
}

static uint8_t can_mob_status(void)
{
	uint8_t CANSTMOB_copy;
	uint8_t mob_status;

	if ((CANCDMOB & 0xC0) == 0x00) return CAN_MOB_DISABLED;

	CANSTMOB_copy = CANSTMOB;

	mob_status = CANSTMOB_copy & (_BV(DLCW)|_BV(TXOK)|_BV(RXOK));

	if ( mob_status == CAN_MOB_TX_OK ||
		 mob_status == CAN_MOB_RX_OK ||
		 mob_status == CAN_MOB_RX_DLCW )
		 return mob_status;

	mob_status = CANSTMOB_copy & CAN_MOB_NOT_REACHED;
	if (mob_status != 0) return mob_status;

	return CAN_MOB_NOT_OK;
}

static void can_mob_clear(void)
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

static void can_mob_clear_all(void)
{
	uint8_t mob_num;
	uint8_t save_canpage;
	save_canpage = CANPAGE;
	for(mob_num = 0; mob_num < CAN_MOB_NUM; mob_num++)
	{
		CANPAGE = (mob_num<<4);
		can_mob_clear();
	}
	CANPAGE = save_canpage;
}

static uint8_t can_mob_get_free(void)
{
	uint8_t save_canpage;
	uint8_t mob;

	save_canpage = CANPAGE;

	for (mob = 0; mob < CAN_MOB_NUM; mob++)
	{
		CANPAGE = (mob << 4); // set MOb
		if ((CANCDMOB & 0xC0) == 0x00) //! Disable configuration
		{
			CANPAGE = save_canpage;
			return (mob);
		}
	}

	CANPAGE = save_canpage;
	return (CAN_MOB_NOT_FREE);
}
