/*
 * errors_manager.c
 *
 *  Created on: Dec 7, 2017
 *      Author: stepan
 */


#include "errors_manager.h"


uint8_t get_SCAP(){
	return READ_PIN(CPGOOD);
}

uint8_t get_UIN(){
	return READ_PIN(PFOB);
}

uint8_t get_RESIST_SQ(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_TransmitReceive(&hspi1, &SPI_RESIST, &response,1,10);
	NSS_UP;


	return response;
	if(response == 0x07 || response == 0x03){
		return 0;
	}else{
		return 1;
	}
}

uint8_t get_PIN_IN(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_TransmitReceive(&hspi1, &SPI_PIN_IN, &response,1,10);
	NSS_UP;

	return response;

}

