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

typedef enum {
	STATE_DISARMED,
	STATE_ERROR,
	STATE_DISARMING,
	STATE_ARMED

}STATE;

typedef enum {
	ERR_SCAP,
	ERR_VOLTAGE_IN,
	ERR_RESIST_SQUIB,
	ERR_PIN_LOW,
	ERR_PIN_HIGH,
	ERR_PWM
}ERR;

typedef enum{
  PIN_IN,
  PIN_OUT
}PIN;

typedef enum{
  PWM_FIRE,
  PWM_STEADY
}PWM;




#endif /* DEFINES_H_ */
