#ifndef ERRORS_MANAGER_H_
#define ERRORS_MANAGER_H_

#include "stm32f3xx_hal.h"
#include "defines.h"

#define NSS_DOWN HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,0)
#define NSS_UP HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,1)

//definovano pro us
#define PWM_LOW 1000
#define PWM_HIGH
#define PWM_INTERVAL 100


extern SPI_HandleTypeDef hspi1;

uint8_t get_SPI(unsigned char p);
void get_errors(uint8_t error[]);



#endif /* ERRORS_MANAGER_H_ */


