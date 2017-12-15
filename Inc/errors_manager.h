#ifndef ERRORS_MANAGER_H_
#define ERRORS_MANAGER_H_

#include "stm32f3xx_hal.h"
#include "defines.h"

#define NSS_DOWN HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,0)
#define NSS_UP HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,1)

#define ERRORS_OK 	errors[ERR_VOLTAGE_IN] == 0 && \
					errors[ERR_SCAP] == 0 && \
					errors[ERR_PWM] == 0 && \
					errors[ERR_RESIST_SQUIB] == 0

#define ERRORS_OK_NOT_SQUIB 	errors[ERR_VOLTAGE_IN] == 0 && \
								errors[ERR_SCAP] == 0 && \
								errors[ERR_PWM] == 0 && \
								errors[ERR_RESIST_SQUIB] == 1

#define OK_PIN_IN	errors[ERR_PIN_HIGH] == 0 && \
					errors[ERR_PIN_LOW] == 0

#define OK_PIN_OUT 	errors[ERR_PIN_HIGH] == 1 && \
					errors[ERR_PIN_LOW] == 1

#define PIN_PULLING_OUT (errors[ERR_VOLTAGE_IN] == 0 && \
						errors[ERR_SCAP] == 0 && \
						errors[ERR_PWM] == 0) \
						&& (timetick_ms <= pulling_time + PULLING_OUT_TIME) && pulling_timeout == 0

#define BUZZER_TIMER htim16
#define BUZZER_ON HAL_TIM_Base_Start_IT(&BUZZER_TIMER)
#define BUZZER_OFF HAL_TIM_Base_Stop_IT(&BUZZER_TIMER); WRITE_PIN(BUZZ,0)
#define BUZZ_WRITE(bit) if(bit == 1 ) BUZZER_ON; else BUZZER_OFF;


#define TURN_OFF_ALL BUZZ_WRITE(0); \
		WRITE_PIN(LED_GREEN, 0);\
		WRITE_PIN(LED_RED, 0);



#define SIGNAL_START_DEF 	(uint16_t[64]){60, 60, 60, 60, 60},\
							0b10101, \
							0b10101, \
							0b10101, \
							5,\
							0,\
							1


#define SIGNAL_ARMING_DEF (uint16_t[64]){60, 940,60, 940, 60, 940,60, 940, 60, 940, 60, 940, 60, 440, 60, 440, 60, 440, 60, 440, 60, 140, 60, 140, 60, 140, 60, 140, 60, 140, 60, 40, 60, 40, 60, 40, 60, 40, 60, 500,400},\
								0b10101010101010101010101010101010101010101, \
								0b10101010101010101010101010101010101010101, \
								0, \
								42,\
								0,\
								0


#define SIGNAL_ARMED_DEF 	(uint16_t[64]){30, 100,30, 100,30, 100,30,500},\
							0, \
							0b01010101, \
							0, \
							8,\
							1,\
							0

#define SIGNAL_ARMED_ERROR_DEF 	(uint16_t[64]){30, 100,30, 100,30, 100,30,500},\
							0b00000000, \
							0b01010101, \
							0b00000010, \
							8,\
							1,\
							0

#define SIGNAL_ARMED_DEF 	(uint16_t[64]){30, 100,30, 100,30, 100,30,500},\
							0, \
							0b01010101, \
							0, \
							8,\
							1,\
							0



#define SIGNAL_DISARMED_DEF (uint16_t[64]){500, 500},\
							0, \
							0b01, \
							0, \
							2,\
							1,\
							0

#define SIGNAL_FIRED_DEF (uint16_t[64]){500, 500},\
							0, \
							0b01, \
							0b10, \
							2,\
							1,\
							0


#define SIGNAL_ERROR_UIN_DEF (uint16_t[64]){500, 500,500, 500,60, 500,60, 500},\
							0b001010101, \
							0, \
							0b001010101, \
							9,\
							1,\
							0

#define SIGNAL_ERROR_SCAP_DEF (uint16_t[64]){500, 500,500, 500,500, 500},\
							0b000101, \
							0b010101, \
							0b010101, \
							6,\
							1,\
							0

#define SIGNAL_ERROR_SQUIB_DEF (uint16_t[64]){500, 500,100, 500,60, 500,60, 500},\
							0b01010101, \
							0, \
							0b01010101, \
							8,\
							1,\
							0

#define SIGNAL_ERROR_PWM_DEF (uint16_t[64]){30, 100,30, 100,30, 100,30,500},\
							0, \
							0, \
							0b01010101, \
							8,\
							1,\
							0

#define SIGNAL_ERROR_PIN_DEF (uint16_t[64]){60, 200,60, 200,30, 200,30,1000},\
							0b00000001, \
							0b01010101, \
							0b01010101, \
							6,\
							1,\
							0


//definovano pro us
#define PWM_LOW 100
#define PWM_HIGH 200
#define PWM_INTERVAL 20

#define ARMING_TIME 10000
#define PULLING_OUT_TIME 1000

extern SPI_HandleTypeDef hspi1;

uint8_t get_SPI(unsigned char p);
void get_errors(uint8_t error[]);
void set_state(STATE * state,ERROR_STATUS * err_status,uint8_t errors[]);
void signal_master();
void cast_signal(SIGNAL signal, uint8_t errors[]);
uint8_t Fire();


#endif /* ERRORS_MANAGER_H_ */


