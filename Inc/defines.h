/*
 * defines.h
 *
 *  Created on: Dec 7, 2017
 *      Author: stepan
 */

#ifndef DEFINES_H_
#define DEFINES_H_

typedef enum {
	STATE_DISARMED = 0,
	STATE_ERROR = 1,
	STATE_ARMED = -1
}STATE;

typedef enum {
	ERR_SCAP,
	ERR_VOLTAGE_IN,
	ERR_RESIST_SQUIB,
	ERR_PIN,
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
