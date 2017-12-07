#ifndef ERRORS_MANAGER_H_
#define ERRORS_MANAGER_H_

#include "stm32f3xx_hal.h"
#include "defines.h"

#define NSS_DOWN HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,0)
#define NSS_UP HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,1)


extern SPI_HandleTypeDef hspi1;

unsigned char SPI_RESIST = 0xd0; //viz datasheet https://www.nxp.com/docs/en/data-sheet/MC33797.pdf  pg. 10
unsigned char SPI_PIN_IN = 0xc1; //dtto


#endif /* ERRORS_MANAGER_H_ */


