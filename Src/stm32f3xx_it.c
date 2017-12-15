/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */

#include "jeti_uart.h"

extern uint16_t jeti_uart_count;
extern uint8_t jeti_uart_start;
extern uint32_t timetick_ms;
extern uint32_t timetick_cus;

extern uint32_t pwm1_interval_us;
uint8_t pwm1_counting = 0;
extern uint32_t pwm2_interval_us;
uint8_t pwm2_counting = 0;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)

{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET){
		if(HAL_GPIO_ReadPin(PWM1_GPIO_Port,PWM1_Pin)){

			__HAL_TIM_SET_COUNTER(&htim6, 0);
			pwm1_counting = 1;
		} else {
			if (pwm1_counting) {
				pwm1_interval_us = __HAL_TIM_GET_COUNTER(&htim6);

				pwm1_counting = 0;
				__HAL_TIM_SET_COUNTER(&htim6, 0);
			}
		}
	}
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) {
		if (HAL_GPIO_ReadPin(PWM2_GPIO_Port, PWM2_Pin)) {
			__HAL_TIM_SET_COUNTER(&htim7, 0);
			pwm2_counting = 1;
		} else {
			if (pwm2_counting) {
				pwm2_interval_us = (float) (__HAL_TIM_GET_COUNTER(&htim7));

				pwm2_counting = 0;
				__HAL_TIM_SET_COUNTER(&htim7, 0);
			}

		}
	}

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM1 update and TIM16 interrupts.
*/
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
* @brief This function handles TIM1 trigger and commutation and TIM17 interrupts.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */
	static uint8_t dec =0;
	timetick_cus++;

	dec++;
	if(dec >= 10){
		timetick_ms++;
		dec = 0;
	}

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	extern uint16_t stack_seq[MSG_BUFF_SIZE][2 * MAX_MSG_LENGHT];
	extern uint8_t stack_lenght[MSG_BUFF_SIZE];

	extern uint16_t p_stack_low;
	extern uint16_t p_stack_high;

	static uint8_t seq_p = 0;
	static uint8_t seq_shift = 13;

	static uint32_t time_between = 0;
	static uint8_t interval_gone = 1;

	static uint8_t msg_on_sending = 0;

	static uint16_t * seq;

	if(timetick_ms < time_between && interval_gone == 0 ){
		HAL_TIM_IRQHandler(&htim3);
			return;
		}else{
			interval_gone = 1;
		}
	if (p_stack_low != p_stack_high && msg_on_sending == 0) {
		msg_on_sending = 1;
		jeti_uart_start = 1;
		seq = &stack_seq[p_stack_low][0];

	}

	if (msg_on_sending) {
		//HAL_GPIO_TogglePin(LD8_GPIO_Port,LD8_Pin);

		seq_shift--;

		HAL_GPIO_WritePin(SW_TX_GPIO_Port, SW_TX_Pin,
				(seq[seq_p] >> seq_shift) & 1);
		//print_bite((seq[seq_p] >> seq_shift) & 1);

		if (seq_shift == 0 && seq_p == stack_lenght[p_stack_low] - 1) {
			jeti_uart_start = 0;
			seq_p = 0;
			seq_shift = JETI_PROTOCOL_LENGHT;

			jeti_uart_count = 0;
			msg_on_sending = 0;
			p_stack_low++;

			if (p_stack_low >= MSG_BUFF_SIZE)
				p_stack_low = 0;

			HAL_GPIO_WritePin(SW_TX_GPIO_Port, SW_TX_Pin, 1);

			time_between = timetick_ms + 20;// tady mel byt jeste zajisten 20ms interval
			//interval_gone = 0;

			return;
		}

		if (seq_shift == 0) {
			seq_p++;
			seq_shift = JETI_PROTOCOL_LENGHT;
			//HAL_UART_Transmit(&huart2, "\n",1,1);
		}

	}
	//HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global and DAC1 underrun error interrupts.
*/
void TIM6_DAC1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC1_IRQn 0 */
	pwm1_interval_us = 0;
  /* USER CODE END TIM6_DAC1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC1_IRQn 1 */

  /* USER CODE END TIM6_DAC1_IRQn 1 */
}

/**
* @brief This function handles TIM7 global and DAC2 underrun error interrupts.
*/
void TIM7_DAC2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC2_IRQn 0 */
	pwm2_interval_us = 0;
  /* USER CODE END TIM7_DAC2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_DAC2_IRQn 1 */

  /* USER CODE END TIM7_DAC2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
