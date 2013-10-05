#ifndef _BOARD_H_
#define _BOARD_H_

// #define MCU_TYPE AT90CAN
#define MCU_TYPE ATMEGAXXM1

#if MCU_TYPE==AT90CAN
	#include "mcu_at90can.h"
#elif  MCU_TYPE==ATMEGAXXM1
	#include "mcu_atmegaxxm1.h"
#else
	#error "Unsupported MCU!"
#endif

#endif /* _BOARD_H_ */
