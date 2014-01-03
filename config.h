#ifndef CONFIG_H_
#define CONFIG_H_

#define DEBUG_ME	0
#define DEBUG_CAN	DEBUG_ME

#define UART_BAUDRATE		115200
#define CAN_BAUDRATE		500000

#define MOTOR_MAX_SPEED		3450	//[rpm]
#define MOTOR_MAX_VOLTAGE	24.00	//[V]
#define MOTOR_MAX_CURRENT	10000	//[mA]
#define MOTOR_NUM_POLES		4

#define PID_POSITION_KP		2.0
#define PID_POSITION_KI		0.0
#define PID_POSITION_KD		0.0

#define PID_SPEED_KP		2.0
#define PID_SPEED_KI		0.3
#define PID_SPEED_KD		0.0

#define PID_CURRENT_KP		0.8
#define PID_CURRENT_KI		0.01
#define PID_CURRENT_KD		0.0

#define CAN_CMD_BASE_ID		0x64
#define CAN_CMD_SET			CAN_CMD_BASE_ID + 1
#define CAN_CMD_GET			CAN_CMD_BASE_ID + 2	

#define CMD_MOTOR_STOP		0x01
#define CMD_MOTOR_START		0x02
#define CMD_MOTOR_COAST		0x03
#define CMD_MOTOR_BREAK		0x04
#define CMD_REV_SET			0x0A
#define CMD_RPM_SET			0x0B

#endif /* CONFIG_H_ */
