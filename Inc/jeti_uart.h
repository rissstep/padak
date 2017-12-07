/*
 * jeti_uart.h
 *
 *  Created on: Nov 2, 2017
 *      Author: stepan
 */

#ifndef JETI_UART_H_
#define JETI_UART_H_

#include "stm32f3xx_hal.h"
#include "defines.h"

#define MSG_BUFF_SIZE 64
#define POLY 0x07
#define EX_SEPARATOR 0x7E
#define EX_ID 0x6F
#define EX_NULL 0x00
#define JETI_TEXT_START 0xFE
#define JETI_TEXT_STOP 0xFF
#define JETI_PROTOCOL_LENGHT 12 // 1 start + 9 data + 1 parity + 2 stop = 13 // 12 for 8bit-data frame
#define SW_TX_GPIO_Port JETI_TX_GPIO_Port
#define SW_TX_Pin JETI_TX_Pin
#define MAX_MSG_LENGHT 34


#define INTERVAL_DATA 100 //ms
#define INTERVAL_TEXT 2000 //s


//!! STATICKY NA JEDEN BYTE DATA
typedef struct{
	uint16_t man_ID;
	uint16_t dev_ID;
	uint8_t identifier_1;
	uint8_t data_type_1; // 0 = 1B
	uint16_t data_1; // -> 1|01|00000 = sign(zero = positive)|position of decimal point|value, 2 = 2B dtto...
	uint8_t identifier_2;
	uint8_t data_type_2;
	uint16_t data_2;
	//I know you wont listen, but don't ever touch this members.
	uint8_t _msg_lenght;
	uint16_t _seq[MAX_MSG_LENGHT];

}JETI_EX_DATA;

typedef struct{
	uint16_t man_ID;
	uint16_t dev_ID;
	uint8_t identifier;
	uint8_t * label_value;
	uint8_t * label_unit;
	//I know you wont listen, but don't ever touch this members.
	uint8_t _msg_lenght;
	uint16_t _seq[MAX_MSG_LENGHT];
}JETI_EX_TEXT;

unsigned char update_crc (unsigned char crc, unsigned char crc_seed);
unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght);
int has_odd_parity(uint16_t x);
void print_bite(uint8_t b);
void print_binary(uint16_t msg);
void generate_seq(uint8_t * msg, uint16_t * seq, uint8_t full_length);
int send_jeti_data(JETI_EX_DATA *msg, JETI_EX_TEXT * text);
int send_jeti_text(JETI_EX_TEXT *msg, JETI_EX_TEXT * text);
void jeti_uart();

void esemble_seq_data(JETI_EX_DATA * msg);
void esemble_seq_text(JETI_EX_TEXT * msg);

void init_jeti_msgs();
void send_state(STATE state);
#endif /* JETI_UART_H_ */
