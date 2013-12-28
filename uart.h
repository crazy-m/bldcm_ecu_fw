#ifndef UART_H_
#define UART_H_

typedef enum uart_baudrate
{
	UART_BAUDRATE_9600		=	9600,
	UART_BAUDRATE_19200		=	19200,
	UART_BAUDRATE_38400		=	38400,
	UART_BAUDRATE_57600		=	57600,
	UART_BAUDRATE_115200	=	115200
} uart_baudrate_t;

void	uart_init		(uart_baudrate_t);

#endif /* UART_H_ */
