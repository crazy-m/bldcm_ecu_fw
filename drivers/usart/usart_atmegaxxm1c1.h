#ifndef _USART_ATMEGAXXM1C1_H_
#define _USART_ATMEGAXXM1C1_H_

#include "usart_common.h"

void usart_atmegaxxm1c1_init(usart_baudrate_t);
uint8_t usart_atmegaxxm1c1_getc(void);
void usart_atmegaxxm1c1_putc(uint8_t);

#endif /* _USART_ATMEGAXXM1C1_H_ */
