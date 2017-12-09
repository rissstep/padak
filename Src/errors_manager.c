/*
 * errors_manager.c
 *
 *  Created on: Dec 7, 2017
 *      Author: stepan
 */


#include "errors_manager.h"

unsigned char SPI_RESIST = 0xd0; //viz datasheet https://www.nxp.com/docs/en/data-sheet/MC33797.pdf  pg. 10
unsigned char SPI_PIN_IN = 0xc1; //dtto
unsigned char SPI_PIN_LOW = 0xc3; //dtto
unsigned char SPI_NOP = 0x00; //dtto

extern volatile uint32_t timetick_cus;

extern uint32_t pwm1_interval_us;
extern uint32_t pwm2_interval_us;

uint8_t get_SCAP(){
	return READ_PIN(CPGOOD);
}

uint8_t get_UIN(){
	return READ_PIN(PFOB); // 1 - napeti OK, 0 - FAIL
}

//cus - stovky micro sekund
void Delay_cus(uint16_t times){
	uint32_t init_timetick = timetick_cus;

	for(;;){
		if(timetick_cus >= init_timetick + times) break;
	}

}



uint8_t get_RESIST_SQ(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1, &SPI_RESIST,1,1);
	NSS_UP;

	Delay_cus(5);

	NSS_DOWN;
	HAL_SPI_Receive(&hspi1,&response,1,1);
	NSS_UP;

	return response;

}

uint8_t resist_not_interval(uint8_t r){
	if(r == 0x03 || r == 0x07) return 0;
	else return 1;
}

uint8_t get_PIN_IN(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1, &SPI_PIN_IN,1,1);
	NSS_UP;

	Delay_cus(5);

	NSS_DOWN;
	HAL_SPI_Receive(&hspi1,&response,1,1);
	NSS_UP;

	return response;

}

uint8_t get_PIN_LOW(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1, &SPI_PIN_LOW,1,1);
	NSS_UP;

	Delay_cus(5);

	NSS_DOWN;
	HAL_SPI_Receive(&hspi1,&response,1,1);
	NSS_UP;

	return response;

}

uint8_t get_SPI(unsigned char p){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1, &p,1,1);
	NSS_UP;

	Delay_cus(5);

	NSS_DOWN;
	HAL_SPI_Receive(&hspi1,&response,1,1);
	NSS_UP;

	return response;

}

uint8_t pwm_low(uint32_t pwm){
	if( (PWM_LOW - PWM_INTERVAL/2) < pwm && pwm < (PWM_LOW+PWM_INTERVAL/2)) return 1;
	else return 0;
}

uint8_t pwm_high(uint32_t pwm){
	if( (PWM_HIGH - PWM_INTERVAL/2) < pwm && pwm < (PWM_HIGH+PWM_INTERVAL/2)) return 1;
	else return 0;
}

uint8_t is_PWM_steady(){
	if(pwm_low(pwm1_interval_us) && pwm_high(pwm2_interval_us)) return 1;
	else return 0;
}



void get_errors(uint8_t error[]){

		//Nabity SCAP
		if(get_SCAP() == 0){
			error[ERR_SCAP] = 1;
		}else{
			error[ERR_SCAP] = 0;
		}

		//Kolik je polozasunuty
		if(get_PIN_LOW() == 0xFD){
			error[ERR_PIN_LOW] = 0;
		}else{
			error[ERR_PIN_LOW] = 1;
		}

		//Kolik je tam az na doraz - low + resist squibu je 0
		if(error[ERR_PIN_LOW] == 0 && get_RESIST_SQ() == 0x00){
			error[ERR_PIN_HIGH] = 0;
		}else{
			error[ERR_PIN_HIGH] = 1;
		}

		//Odpor musit byt v intevalu 1.6 - 2.6ohm (0x03 - 0x07)
		if(resist_not_interval(get_RESIST_SQ())){
			error[ERR_RESIST_SQUIB] = 1;
		}else{
			error[ERR_RESIST_SQUIB] = 0;
		}

		//PWM signaly jsou v zakladni pozici
		if(is_PWM_steady() == 0){
			error[ERR_PWM] = 1;
		}else{
			error[ERR_PWM] = 0;
		}

		//Kontrola vstupniho napeti
		if(get_UIN() == 0){
			error[ERR_VOLTAGE_IN] = 1;
		}else{
			error[ERR_VOLTAGE_IN] = 0;
		}

}


