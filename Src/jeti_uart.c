/*
 * jeti_uart.c
 *
 *  Created on: Nov 2, 2017
 *      Author: stepan
 */

#include "jeti_uart.h"
#include <string.h>

uint16_t jeti_uart_count = 0;
uint8_t jeti_uart_start = 0;

 uint16_t stack_seq[MSG_BUFF_SIZE][2*MAX_MSG_LENGHT];
uint8_t stack_lenght[MSG_BUFF_SIZE];

uint16_t p_stack_low = 0;
uint16_t p_stack_high = 0;

JETI_EX_DATA ex_data;
JETI_EX_TEXT ex_text1;
JETI_EX_TEXT ex_text2;
JETI_EX_TEXT ex_text3;
JETI_EX_TEXT ex_text_plain;

extern uint32_t timetick_ms;

unsigned char update_crc (unsigned char crc, unsigned char crc_seed){
	unsigned char crc_u;
	unsigned char i;
	crc_u = crc;
	crc_u ^= crc_seed;
	for (i=0; i<8; i++)
	{
		crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
	}
	return crc_u;
}

unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght){
	unsigned char crc_up = 0;
	unsigned char c;
	for(c=0;c < crc_lenght; c++) {
		crc_up = update_crc (crc[c], crc_up);
	}
	return crc_up;
}

int has_odd_parity(uint16_t x){
    unsigned int count = 0, i, b = 1;

    for(i = 0; i < 16; i++){
        if( x & (b << i) ){count++;}
    }

    if( (count % 2) ){return 0;}

    return 1;
}

void generate_seq(uint8_t * msg, uint16_t * seq, uint8_t full_length){
	int i;
	uint8_t postfix;
	uint8_t tmp = 0;
	for(i =0;i<full_length;++i){

		tmp = 0;
		for(int j = 0; j<8;++j){
			if((msg[i] >> j) & 1){
				tmp |= 0x1;
			}
			if(j!=7)tmp<<=1;
		}

		seq[i] = tmp;
		seq[i] <<= (JETI_PROTOCOL_LENGHT == 12) ? 3 : 4;

		postfix =0b0011;
		if(JETI_PROTOCOL_LENGHT == 13){
			if(msg[i] != 0xff && msg[i] != 0x7e && msg[i] != 0xfe){
				seq[i] |= 0b1000;
			}
		}
		if(has_odd_parity(seq[i])){
			postfix |= 0b100;
		}

		seq[i] |= postfix;

	}

}

int send_jeti_data( JETI_EX_DATA * msg, JETI_EX_TEXT * text){

	if((p_stack_low-1) == p_stack_high){
		//pointer high dohonil low zespoda -> neni mozne pridat zpravu
		//HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
		return 1;

	}else{
		memcpy(&stack_seq[p_stack_high][0],&msg->_seq[0],2*msg->_msg_lenght);

		memcpy(&stack_seq[p_stack_high][msg->_msg_lenght],&text->_seq[0],2*text->_msg_lenght);

		stack_lenght[p_stack_high] = msg->_msg_lenght+text->_msg_lenght;
		p_stack_high++;

		if(p_stack_high >= MSG_BUFF_SIZE) p_stack_high=0;
	}

	return 0;
}


int send_jeti_text(JETI_EX_TEXT * msg, JETI_EX_TEXT * text){

	if((p_stack_low-1) == p_stack_high){
		//pointer high dohonil low zespoda -> neni mozne pridat zpravu
		//HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
		return 1;

	}else{
		memcpy(&stack_seq[p_stack_high][0],&msg->_seq[0],2*msg->_msg_lenght);
		if(text !=NULL){
			memcpy(&stack_seq[p_stack_high][msg->_msg_lenght],&text->_seq[0],2*text->_msg_lenght);
			stack_lenght[p_stack_high] = msg->_msg_lenght+text->_msg_lenght;
		}else{
			stack_lenght[p_stack_high] = msg->_msg_lenght;
		}


		p_stack_high++;

		if(p_stack_high >= MSG_BUFF_SIZE) p_stack_high=0;
	}


	return 0;
}

//UNUSED
void jeti_uart(){

	static uint8_t seq_p = 0;
	static uint8_t seq_shift = JETI_PROTOCOL_LENGHT;

	static uint32_t time_between = 0;
	static uint8_t interval_gone = 1;
	static int16_t i = 1;

	static uint8_t msg_on_sending = 0;

	static uint16_t * seq;

	if(timetick_ms < time_between && interval_gone == 0 ){
		return;
	}else{
		interval_gone = 1;
	}

	if(p_stack_low != p_stack_high && msg_on_sending == 0){
		msg_on_sending = 1;
		jeti_uart_start = 1;
		seq = &stack_seq[p_stack_low][0];

	}

	if(jeti_uart_count >= i && msg_on_sending){
		//HAL_GPIO_TogglePin(LD8_GPIO_Port,LD8_Pin);

		seq_shift--;

		HAL_GPIO_WritePin(SW_TX_GPIO_Port,SW_TX_Pin,(seq[seq_p] >> seq_shift) & 1);
		//print_bite((seq[seq_p] >> seq_shift) & 1);

		if(seq_shift == 0 && seq_p == stack_lenght[p_stack_low]-1){
			jeti_uart_start = 0;
			seq_p = 0;
			seq_shift = JETI_PROTOCOL_LENGHT;
			i = 1;
			jeti_uart_count = 0;
			msg_on_sending = 0;
			p_stack_low++;

			if(p_stack_low >= MSG_BUFF_SIZE) p_stack_low = 0;

			HAL_GPIO_WritePin(SW_TX_GPIO_Port,SW_TX_Pin,1);

			time_between = timetick_ms + 20;	// tady mel byt jeste zajisten 20ms interval
			interval_gone =0;


			return;
		}

		if(seq_shift == 0){
			seq_p++;
			seq_shift = JETI_PROTOCOL_LENGHT;
			//HAL_UART_Transmit(&huart2, "\n",1,1);
		}

		i++;

	}

}

void esemble_seq_data(JETI_EX_DATA * msg){

	msg->_msg_lenght = 13; // viz datasheet s jednim datovym bytem, max 29

	uint8_t array[msg->_msg_lenght];

	array[0] = EX_SEPARATOR;
	array[1] = EX_ID;
	array[2] = (msg->_msg_lenght-3) | 0x40;
	array[3] = (uint8_t)(msg->man_ID & 0x00FF);
	array[4] = (uint8_t)(msg->man_ID >> 8);
	array[5] = (uint8_t)(msg->dev_ID & 0x00FF);
	array[6] = (uint8_t)(msg->dev_ID  >> 8);
	array[7] = EX_NULL;
	array[8] = (msg->identifier_1 << 4) | msg->data_type_1;
	array[9] = msg->data_1;
	array[10] = (msg->identifier_2 << 4) | msg->data_type_2;
	array[11] = msg->data_2;
	array[12] = crc8(&array[2],msg->_msg_lenght-3);

	generate_seq(array,msg->_seq,msg->_msg_lenght);

}

void esemble_seq_text(JETI_EX_TEXT * msg){

	int value_len = strlen(msg->label_value);
	int unit_len = strlen(msg->label_unit);
	msg->_msg_lenght = 8+3+unit_len+value_len; // viz datasheet s jednim datovym bytem, max 29

	uint8_t array[18];

	array[0] = EX_SEPARATOR;
	array[1] = EX_ID;
	array[2] = (msg->_msg_lenght-3);
	array[3] = (uint8_t)(msg->man_ID & 0x00FF);
	array[4] = (uint8_t)(msg->man_ID >> 8);
	array[5] = (uint8_t)(msg->dev_ID & 0x00FF);
	array[6] = (uint8_t)(msg->dev_ID  >> 8);
	array[7] = EX_NULL;
	array[8] = msg->identifier;

	array[9] = (value_len << 3) | unit_len;
	memcpy(&array[10],msg->label_value,value_len);
	memcpy(&array[10+value_len],msg->label_unit,unit_len);

	array[msg->_msg_lenght-1] = crc8(&array[2],msg->_msg_lenght-3);

	generate_seq(array,msg->_seq,msg->_msg_lenght);

}

void init_jeti_msgs(){

	  ex_data.man_ID = 0xA401;
	  ex_data.dev_ID = 0x1212;
	  ex_data.identifier_1 = 1;
	  ex_data.data_type_1 = 0;
	  ex_data.data_1 = 0x1F & 0;
	  ex_data.identifier_2 = 2;
	  ex_data.data_type_2 = 0;
	  ex_data.data_2 = 0x1F & 1;


	  ex_text1.man_ID = 0xA401;
	  ex_text1.dev_ID = 0x1212;
	  ex_text1.identifier = 1;
	  ex_text1.label_value = "STATE";
	  ex_text1.label_unit = " ";

	  ex_text2.man_ID = 0xA401;
	  ex_text2.dev_ID = 0x1212;
	  ex_text2.identifier = 2;
	  ex_text2.label_value = "Error #";
	  ex_text2.label_unit = " ";

	  ex_text3.man_ID = 0xA401;
	  ex_text3.dev_ID = 0x1212;
	  ex_text3.identifier = 0;
	  ex_text3.label_value = "Parachutte";
	  ex_text3.label_unit = "";

	  uint8_t text[MAX_MSG_LENGHT] = {0x20};
	  text[0] = JETI_TEXT_START;
	  sprintf(&text[1],"Flydeo parachutte device!");
	  text[33] = JETI_TEXT_STOP;
	  uint8_t text_l = MAX_MSG_LENGHT;

	  ex_text_plain._msg_lenght = text_l;
	  generate_seq(text,ex_text_plain._seq,text_l);

	  esemble_seq_data(&ex_data);
	  esemble_seq_text(&ex_text1);
	  esemble_seq_text(&ex_text2);
	  esemble_seq_text(&ex_text3);


}

void send_state(STATE state, uint8_t errors[], ERROR_STATUS err_status){
	static uint32_t every_xms = 0;
	static uint32_t every_xs = 0;
	uint8_t err_data =0;
	if(timetick_ms >= every_xms+INTERVAL_DATA){

		ex_data.data_1 = 0x1F & state;


		if(err_status == FAIL){
			for(int i = 0; i < 6;++i){
				err_data |= errors[i];
				if(i != 0) err_data <<= 1;
			}
		}else{
			err_data = 0;
		}

		ex_data.data_2 = err_data;

		esemble_seq_data(&ex_data);

		send_jeti_data(&ex_data, &ex_text_plain);

		every_xms = timetick_ms;
	}


	if(timetick_ms >= every_xs+INTERVAL_TEXT){

		send_jeti_text(&ex_text1, &ex_text_plain);
		send_jeti_text(&ex_text2, &ex_text_plain);
		send_jeti_text(&ex_text3,  &ex_text_plain);

		every_xs =timetick_ms;
	}


	
}





