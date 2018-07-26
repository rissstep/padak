/*
 * errors_manager.c
 *
 *  Created on: Dec 7, 2017
 *      Author: stepan
 */


#include "errors_manager.h"
#include <string.h>

unsigned char SPI_RESIST = 0xd0; //viz datasheet https://www.nxp.com/docs/en/data-sheet/MC33797.pdf  pg. 10
unsigned char SPI_PIN_IN = 0xc1; //dtto
unsigned char SPI_PIN_LOW = 0xc3; //dtto
unsigned char SPI_NOP = 0x00; //dtto

unsigned char SPI_ARM = 0xA0; //dtto
unsigned char SPI_FIRE = 0x57; //dtto


extern volatile uint32_t timetick_cus;
extern volatile uint32_t timetick_ms;

extern uint32_t pwm1_interval_us;
extern uint32_t pwm2_interval_us;
extern CAN_HandleTypeDef hcan;

extern TIM_HandleTypeDef BUZZER_TIMER;

volatile uint32_t pulling_time = 0;
volatile uint8_t pulling_timeout = 1;

volatile uint32_t arming_time = 0;
volatile uint8_t arming_timeout = 1;

uint16_t signal_inverval[64] = {0};
uint64_t signal_buzzer =0;
uint64_t signal_led_green=0;
uint64_t signal_led_red=0;
uint8_t signal_lenght=0;
uint8_t signal_repete=0;
uint8_t signal_block=0;

SIGNAL signal_type = SIGNAL_NONE;



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
		if(get_RESIST_SQ() == 0x00){
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

void set_state(STATE * state,ERROR_STATUS * err_status,uint8_t errors[]){


	if(*state == STATE_FIRED) {
		CAN_stop();
		*err_status = FAIL;
		cast_signal(SIGNAL_FIRED, NULL);

		return;
	}


	if(*state == STATE_DISARMED){
		//cast_signal(SIGNAL_DISARMED, NULL);

		if(OK_PIN_OUT && ERRORS_OK){
			*state = STATE_ARMING;
			*err_status = OK;
			cast_signal(SIGNAL_ARMING, NULL);


			arming_time = timetick_ms;
			arming_timeout = 0;

		}else if((OK_PIN_IN && ERRORS_OK_NOT_SQUIB) || (PIN_PULLING_OUT)){
			*state = STATE_DISARMED;
			*err_status = OK;
			cast_signal(SIGNAL_DISARMED, NULL);


			if(OK_PIN_IN) {
				pulling_time = timetick_ms;
				pulling_timeout = 0;
			}
		}else {
			if(pulling_timeout == 0){
				pulling_timeout = 1;
			}
			*state = STATE_ERROR;
			cast_signal(SIGNAL_ERROR, errors);
			*err_status = FAIL;
		}


	}else if(*state == STATE_ERROR){
		//CAN_stop();
		if(OK_PIN_IN && ERRORS_OK_NOT_SQUIB){
			*state = STATE_DISARMED;
			*err_status = OK;
			//cast_signal(SIGNAL_DISARMED, NULL);
			//cast_signal(SIGNAL_START, NULL);
			 cast_signal(SIGNAL_ACTION_DISARMED, NULL);
		}else{
			*state = STATE_ERROR;
			*err_status = FAIL;
			cast_signal(SIGNAL_ERROR, errors);
		}


	}else if(*state == STATE_ARMING){
		if(OK_PIN_IN){
			*state = STATE_DISARMED;
			*err_status = OK;
			cast_signal(SIGNAL_ACTION_DISARMED, NULL);

		}else if(OK_PIN_OUT && ERRORS_OK){
			if(timetick_ms >= arming_time + ARMING_TIME && arming_timeout == 0){
				*state = STATE_ARMED;
				cast_signal(SIGNAL_ARMED, NULL);
				arming_timeout = 1;
			}else{
				*state = STATE_ARMING;
			}

			*err_status = OK;
		}else{
			*state = STATE_ERROR;
			*err_status = FAIL;
			cast_signal(SIGNAL_ERROR, errors);
		}

	}else if(*state == STATE_ARMED){
		if(OK_PIN_IN){
			*state = STATE_DISARMED;
			cast_signal(SIGNAL_ACTION_DISARMED, NULL);
			//*err_status = OK;
		}else if(OK_PIN_OUT && ERRORS_OK && *err_status == OK){
			*state = STATE_ARMED;
			cast_signal(SIGNAL_ARMED, NULL);
			//*err_status = OK;
		}else{
			*state = STATE_ARMED;
			*err_status = FAIL;
			cast_signal(SIGNAL_ARMED_ERROR, NULL);
		}
	}

}

void set_signal(uint16_t interval[static 64], uint64_t buzzer, uint64_t led_green, uint64_t led_red, uint8_t length,uint8_t repete,uint8_t block){

	memcpy(signal_inverval,interval, 2*length);
	signal_buzzer =buzzer;
	signal_led_green = led_green;
	signal_led_red = led_red;
	signal_lenght = length;
	signal_repete = repete;
	signal_block= block;

}

void signal_master(){
	static SIGNAL signal_current = SIGNAL_NONE;
	static int8_t i = -1;
	static uint8_t next_step = 1;
	static uint32_t timer =0;


	if(signal_current != signal_type){
		if(signal_block == 0 || (signal_block == 1 && signal_current == SIGNAL_NONE)){
				i = -1;
				next_step = 1;
				signal_current = signal_type;

				switch(signal_current){
					case SIGNAL_START: set_signal(SIGNAL_START_DEF); break;
					case SIGNAL_ARMING: set_signal(SIGNAL_ARMING_DEF); break;
					case SIGNAL_ARMED: set_signal(SIGNAL_ARMED_DEF); break;
					case SIGNAL_DISARMED: set_signal(SIGNAL_DISARMED_DEF); break;
					case SIGNAL_ARMED_ERROR: set_signal(SIGNAL_ARMED_ERROR_DEF); break;
					case SIGNAL_ERROR_UIN: set_signal(SIGNAL_ERROR_UIN_DEF); break;
					case SIGNAL_ERROR_PWM: set_signal(SIGNAL_ERROR_PWM_DEF); break;
					case SIGNAL_ERROR_SCAP: set_signal(SIGNAL_ERROR_SCAP_DEF); break;
					case SIGNAL_ERROR_SQUIB: set_signal(SIGNAL_ERROR_SQUIB_DEF); break;
					case SIGNAL_ERROR_PIN: set_signal(SIGNAL_ERROR_PIN_DEF); break;
					case SIGNAL_FIRED: set_signal(SIGNAL_FIRED_DEF); break;
					case SIGNAL_ACTION_DISARMED: set_signal(SIGNAL_ACTION_DISARMED_DEF); break;
					default: break;
				}
		}
	}

	if(signal_current == SIGNAL_NONE) {
		return;
	}

	if(next_step){

		i++;
		BUZZ_WRITE(GET_BIT(signal_buzzer,i));
		WRITE_PIN(LED_GREEN, GET_BIT(signal_led_green,i));
		WRITE_PIN(LED_RED, GET_BIT(signal_led_red,i));

		next_step = 0;
		timer = timetick_ms;
	}

	if(i >= signal_lenght) {
		if(signal_repete == 0){
			TURN_OFF_ALL;
			signal_current = SIGNAL_NONE;
			signal_type = SIGNAL_NONE;
			return;
		}else{
			i = -1;
			next_step = 1;
			return;
		}

	}

	if(timetick_ms >= timer+signal_inverval[i]){

		next_step = 1;
		timer = timetick_ms;
	}



}

void cast_signal(SIGNAL signal, uint8_t errors[]){

	if(signal == SIGNAL_ERROR){
		if(errors[ERR_VOLTAGE_IN]) {
			signal_type = SIGNAL_ERROR_UIN; return;
		}
		else if(errors[ERR_SCAP]) {
			signal_type = SIGNAL_ERROR_SCAP; return;
		}
		if(errors[ERR_PWM]) {
			signal_type = SIGNAL_ERROR_PWM; return;
		}
		if(OK_PIN_OUT && errors[ERR_RESIST_SQUIB]) {
			signal_type = SIGNAL_ERROR_SQUIB; return;
		}
		if((OK_PIN_OUT) || errors[ERR_PIN_LOW] == 0 ) {
			signal_type = SIGNAL_ERROR_PIN; return;
		}

	}else{
		signal_type = signal;
	}
}

uint8_t is_PWM_fire(){
	if(pwm_low(pwm2_interval_us) && pwm_high(pwm1_interval_us)) return 1;
	else return 0;
}

uint8_t set_FIRE(){
	unsigned char response = 0;

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1, &SPI_ARM,1,1);
	NSS_UP;

	Delay_cus(5);

	NSS_DOWN;
	HAL_SPI_Transmit(&hspi1,&SPI_FIRE,1,1);
	NSS_UP;


	return response;

}


void CAN_stop(){

	static CanTxMsgTypeDef msgBreak = { 0x110, 0, 0, 0, 6, { 0, 15, 0,0, 0, 0, 0, 0 } };
	static float motor_breaking;
	static uint8_t flag = 1;

	if(flag){
		hcan.pTxMsg = &msgBreak;

		motor_breaking = 1;

		unsigned char const * const p = (unsigned char const *) &motor_breaking;
		for (size_t i = 0; i != 4; ++i) {
		  msgBreak.Data[2 + i] = p[i];
		}

		flag = 0;
	}

	for(int i = 0;i<8;i++){
		msgBreak.Data[0] = i;
		HAL_CAN_Transmit(&hcan,1);
	}
}

uint8_t Fire(){
	if(is_PWM_fire()){
		CAN_stop();
		set_FIRE();
		cast_signal(SIGNAL_START, NULL);
		return 1;
	}else{
		return 0;
	}
}


