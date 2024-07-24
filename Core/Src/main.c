/*
 * 	File: main.c
 *  Created on: 24-June-2024
 *  Author: oakyol
 */


#include "stm32f407xx.h"
#include "uart.h"

#define SIZE			4

usart_handle usart_base_obj;


const uint8_t TxBuffer[SIZE] = { 'T', 'E', 'S', 'T'};
uint8_t RxBuffer[SIZE];

int main(void) {
	/*Test code for USART2 */
	usart_base_obj.usart = USART2;
	usart_base_obj.TxBuffPtr = TxBuffer;
	usart_base_obj.TxCounter = SIZE;
	usart_base_obj.RxBuffPtr = RxBuffer;
	usart_base_obj.RxCounter = SIZE;

	usart_periph_clock_init();

	usart_gpio_init(&usart_base_obj, GPIOA);

	usart_config_function(&usart_base_obj);

	usart_interrupt_config(&usart_base_obj);

	DMAx_interrupt_config(DMA1_Stream6);

	DMAx_interrupt_config(DMA1_Stream5);

	global_interrupt_enable(USART2_IRQn, 0);

	global_interrupt_enable(DMA1_Stream6_IRQn, 0);

	global_interrupt_enable(DMA1_Stream5_IRQn, 0);

	stream_config_for_dma_transmit(DMA1_Stream6, TxBuffer, 5);


	while (1) {	
	}

}
