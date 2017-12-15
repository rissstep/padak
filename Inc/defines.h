/*
 * defines.h
 *
 *  Created on: Dec 7, 2017
 *      Author: stepan
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define TOGGLE(pin) HAL_GPIO_TogglePin(pin ## _GPIO_Port, pin ## _Pin)
#define READ_PIN(pin) HAL_GPIO_ReadPin(pin ## _GPIO_Port, pin ## _Pin)
#define WRITE_PIN(pin, lvl) HAL_GPIO_WritePin(pin ## _GPIO_Port, pin ## _Pin, lvl)
#define GET_BIT(REG, bit) ((REG >> bit) & 0x01)

typedef enum {
	STATE_DISARMED,
	STATE_ERROR,
	STATE_ARMING,
	STATE_ARMED,
	STATE_FIRED
}STATE;

typedef enum {
	SIGNAL_NONE,
	SIGNAL_TO_DISARM,
	SIGNAL_TO_ARM,
	SIGNAL_START,
	SIGNAL_DISARMED,
	SIGNAL_ARMING,
	SIGNAL_ARMED,
	SIGNAL_ARMED_ERROR,
	SIGNAL_ERROR_UIN,
	SIGNAL_ERROR_PWM,
	SIGNAL_ERROR_SCAP,
	SIGNAL_ERROR_SQUIB,
	SIGNAL_ERROR_PIN,
	SIGNAL_ERROR,
	SIGNAL_FIRED
}SIGNAL;

typedef enum {
	ERR_SCAP,
	ERR_VOLTAGE_IN,
	ERR_RESIST_SQUIB,
	ERR_PIN_LOW,
	ERR_PIN_HIGH,
	ERR_PWM
}ERROR_TYPE;

typedef enum{
  OK,
  FAIL
}ERROR_STATUS;




#endif /* DEFINES_H_ */
