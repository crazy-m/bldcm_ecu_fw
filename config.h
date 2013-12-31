#ifndef CONFIG_H_
#define CONFIG_H_

#define DEBUG_ME	0
#define DEBUG_CAN	0

#define MOTOR_MAX_SPEED		3450
#define MOTOR_MIN_SPEED		0
#define MOTOR_MAX_VOLTAGE	24.00
#define MOTOR_MAX_CURRENT	20.00
#define MOTOR_NUM_POLES		4

#define PID_POSITION_KP		2.0
#define PID_POSITION_KI		0.0
#define PID_POSITION_KD		0.0

#define PID_SPEED_KP		2.0
#define PID_SPEED_KI		0.3
#define PID_SPEED_KD		0.0

#define PID_CURRENT_KP		0.8
#define PID_CURRENT_KI		0.002
#define PID_CURRENT_KD		0.0

#define CAN_BASE_ID				0x120
#define	CAN_MOTOR_START			CAN_BASE_ID+1
#define CAN_MOTOR_STOP			CAN_BASE_ID+2
#define CAN_MOTOR_COAST			CAN_BASE_ID+3
#define CAN_MOTOR_SPEED_SET		CAN_BASE_ID+4
#define CAN_MOTOR_SPEED_GET		CAN_BASE_ID+5
#define CAN_MOTOR_CURRENT_GET	CAN_BASE_ID+6
#define CAN_MOTOR_VOLTAGE_GET	CAN_BASE_ID+7
#define CAN_MOTOR_ANGLE_SET		CAN_BASE_ID+8
#define CAN_MOTOR_ANGLE_GET		CAN_BASE_ID+9
#define CAN_MOTOR_RESERVED3		CAN_BASE_ID+0xA
#define CAN_MOTOR_RESERVED4		CAN_BASE_ID+0xB
#define CAN_MOTOR_RESERVED5		CAN_BASE_ID+0xC
#define CAN_MOTOR_RESERVED6		CAN_BASE_ID+0xD
#define CAN_MOTOR_TEMP_GET		CAN_BASE_ID+0xE
#define CAN_MOTOR_PID_GET		CAN_BASE_ID+0xF

#endif /* CONFIG_H_ */
