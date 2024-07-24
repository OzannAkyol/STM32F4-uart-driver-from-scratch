/******************************************************************************
 * File Name.......: uart                           						  *
 * Application.....: Self Development									      *
 * Author..........: Ozan Akyol												  *
 * Created on......: 24-June-2024 00:54                                       *
 *				   : uart driver  functions          	  			          *
 -----------------------------------------------------------------------------*/

#include "uart.h"

/*
 * Date............: 11-July-2024 19:27
 * Function........: _DMA1_Stream6_IRQHandler
 * .................
 */

void _DMA1_Stream6_IRQHandler(usart_handle* usart_p, DMA_TypeDef* DMAx){
	 
	/* A transfer complete event occurred on*/
	if(DMAx->HISR & DMA_TCIF6)
	{
		/*clear transfer complete flag*/
		DMAx->HIFCR |= DMA_CTCIF6;

		if(USART_IS_TC(usart_p->usart))
		{
			usart_p->usart->SR &= ~SR_TC;
			usart_p->state = TX_READY;
		}
	}
	/* Stream 6 transfer error interrupt flag*/
	else if(DMAx->HISR & DMA_TEIF6)
	{
		DMAx->HIFCR |= (DMA_TEIF6);
	}
	/* Stream 6 direct mode error interrupt flag*/
	else if(DMAx->HISR & DMA_DMEIF6)
	{
		/* Clear the flag*/
		DMAx->HIFCR |= (DMA_DMEIF6);
	}

}

/*
 * Date............: 12-July-2024 15:37
 * Function........: _DMA1_Stream5_IRQHandler
 * ................. 
 */

void _DMA1_Stream5_IRQHandler(usart_handle* usart_p, DMA_TypeDef* DMAx){
	 /*Stream 5 transfer complete interrupt flag*/
	if(DMAx-> HISR & DMA_TCIF5)
	{
		DMAx->HIFCR |= DMA_CTCIF5; /*clear flag*/

	}
	/* Stream 5 transfer error interrupt flag*/
	else if(DMAx->HISR & DMA_TEIF5)
	{
		DMAx->HIFCR |= (DMA_TEIF5);
	}
	/* Stream 6 direct mode error interrupt flag*/
	else if(DMAx->HISR & DMA_DMEIF5)
	{
		/* Clear the flag*/
		DMAx->HIFCR |= (DMA_DMEIF5);
	}
		
}


/*
 * Date............: 10-July-2024 17:20
 * Function........: disable_dma_stream
 * ................. 
 */

usart_states disable_dma_stream(DMA_Stream_TypeDef* DMA1_StreamX, stream_names stream){
	if(TX_STREAM6_CHANNEL4 == stream)
	{
	/*To configure stream, first you have to disable.So, reset stream EN bits*/
		DMA1_StreamX->CR &= ~DMA_CR_EN;
		/*check the stream status bit is really disabled if it is enable, get error*/
		while(DMA1_StreamX->CR & DMA_CR_EN )
						;
	return USART_OK;
	}
	
	else if(RX_STREAM5_CHANNEL4 == stream)
	{
	/*To configure stream, first you have to disable.So, reset stream EN bits*/
		DMA1_StreamX->CR &= ~DMA_CR_EN;
		/*check the stream status bit is really disabled if it is enable, get error*/
		while(DMA1_StreamX->CR & DMA_CR_EN )
						;
	return USART_OK;
	}

	else
	{
		return USART_ERROR;
	}
}

/*
 * Date............: 10-July-2024 17:32
 * Function........: enable_dma_stream
 * ................. 
 */

void enable_dma_stream(DMA_Stream_TypeDef* DMA1_StreamX, stream_names stream){
	if(TX_STREAM6_CHANNEL4 == stream)
	{
		/*Activate the stream by setting EN bit*/
		DMA1_StreamX->CR |= DMA_CR_EN;
	}
	else if(RX_STREAM5_CHANNEL4 == stream)
	{
		/*Activate the stream by setting EN bit*/
		DMA1_StreamX->CR |= DMA_CR_EN;
	}
	else
	{
		return;
	}

}

/*
 * Date............: 10-July-2024 17:26
 * Function........: stream_config_for_dma_to_usart
 * .................
 */

void stream_config_for_dma_receive(DMA_Stream_TypeDef* DMA1_StreamX, uint8_t* RcvBufferPtr, uint32_t length){
	
	/* To configure the stream, first, close the stream channel */	
	usart_states ret_state;
	ret_state = disable_dma_stream(DMA1_StreamX,RX_STREAM5_CHANNEL4);
	if(USART_OK != ret_state)
		return;
	
	/* Select Direct Mode */
	DMA1_StreamX->CR &= ~DMA_FCR_DMDIS;

	DMA1_StreamX->CR &= ~SxCR_DIR_MASK;
	/*Set Data direction to Memory to Peripheral */
	DMA1_StreamX->CR |= SxCR_MEMORY_TO_PERIPH;

	/* Set the peripheral port register address(USART2_DR) */	
	DMA1_StreamX-> PAR = USART2_DATA_REG_ADDRESS;
	
	/* Set the memory port register address */	
	DMA1_StreamX-> M0AR = (uint32_t)RcvBufferPtr;

	/* Set number of data items to transfer & this value is decremented by authomatically.*/
	DMA1_StreamX->NDTR = length;

	/* Select the Channel for (Usart2_Rx) */
	DMA1_StreamX->CR |= DMA_CHANNEL_4;

	/* Select Configure the stream priority */
	DMA1_StreamX->CR |= DMA_CHANNEL_4_MEDIUM_PRIORITY;

	/*Stream5 Transfer complete interrupt enable*/
	DMA1_StreamX->CR |= DMA_SxCR_TCIE;

	/* Enable the stream again */
	enable_dma_stream(DMA1_StreamX, RX_STREAM5_CHANNEL4);
}

/*
 * Date............: 09-July-2024 16:20
 * Function........: stream_config_for_usart_to_dma
 * ................. 
 */

void stream_config_for_dma_transmit(DMA_Stream_TypeDef* DMA1_StreamX , const uint8_t* BufferPtr, uint32_t length){

	/* To configure the stream, first, close the stream channel */	
	usart_states ret_state;
	ret_state = disable_dma_stream(DMA1_StreamX,TX_STREAM6_CHANNEL4);
	if(USART_OK != ret_state)
		return;

	/* Select Direct Mode */
	DMA1_StreamX->CR &= ~DMA_FCR_DMDIS;

	/*Set Data direction to Peripheral to memory*/
	DMA1_StreamX->CR &= ~SxCR_DIR_MASK;
	/*Set Data direction to Peripheral to memory !!request diir*/
	DMA1_StreamX->CR |= SxCR_PERIPH_TO_MEMORY;

	/* Set the peripheral port register address(USART2_DR) */	
	DMA1_StreamX-> PAR = USART2_DATA_REG_ADDRESS;
	
	/* Set the memory port register address */	
	DMA1_StreamX-> M0AR = (uint32_t)BufferPtr;

	/* Set number of data items to transfer & this value is decremented by authomatically.*/
	DMA1_StreamX->NDTR = length;

	/* Select the Channel for (Usart2_Tx) */
	DMA1_StreamX->CR |= DMA_CHANNEL_4;

	/* Select Configure the stream priority */
	DMA1_StreamX->CR |= DMA_CHANNEL_4_MEDIUM_PRIORITY;

	/*Stream6 Transfer complete interrupt enable*/
	DMA1_StreamX->CR |= DMA_SxCR_TCIE;
	
	/* Enable the stream again */
	enable_dma_stream(DMA1_StreamX,TX_STREAM6_CHANNEL4);
	
}

/*
 * Date............: 09-July-2024 16:12
 * Function........: usart_periph_clock_init
 * ................. 
 */

void usart_periph_clock_init(){
	/* Set RCC Clock for GPIOA */
	RCC->AHB1ENR |= GPIOAEN;

	/* Set RCC Clock for USART2 */
	RCC->APB1ENR |= USART2EN;
	
	/*Set RCC clock for DMA*/
	RCC->AHB1ENR |= DMA1EN;

}

/*
 * Date............: 05-July-2024 11:36
 * Function........: _usart_IRQHandler
 * ................. 
 */

void _usart_IRQHandler(usart_handle* usart_p) {
	if(usart_p == NULL)
		return;	
	/*Check  Transmit Data Register Empty and TXEIE Interrupt Enable*/
	if ((USART_IS_TXE(usart_p->usart)) && USART_IS_TXEIE(usart_p->usart))
	{
		usart_p->state = TX_READY;
		usart_transmit_it(usart_p);
	}
	else if (USART_IS_RXNE(usart_p->usart) && USART_IS_RXNEIE(usart_p->usart))
	{
		usart_p->state = RX_READY;
		usart_receive_it(usart_p);
	}
	else if(USART_IS_TC(usart_p->usart))
	{
		if(DMA1->HISR & DMA_TCIF6)
		{
			DMA1->HIFCR |= DMA_CTCIF6; /*clear flag*/
		}
		usart_p->usart->SR &= ~SR_TC;
		usart_p->state = TX_READY; 
	}
	else if(USART_IS_RXNE(usart_p->usart))
	{
		if(DMA1->HIFCR & DMA_TCIF5)
		{
			DMA1->HIFCR |= DMA_CTCIF5; /*clear flag*/
		}
		usart_p->usart->CR3 &= ~(DMAR); /*Clear reset bit*/
		usart_p->state = RX_READY;
	}
	else
	{
		/*....*/;
	}
}

/*
 * Date............: 24-June-2024 01:15
 * Function........: usart_gpio_init
 * ................. 
 */

void usart_gpio_init(usart_handle* usart_base_p, GPIO_TypeDef * GPIOx){
	if(usart_base_p == NULL)
		return;
	/* Set Rx pin's (PA3) as Alternate Function */
	GPIOx->MODER |= (1U << 7);
	GPIOx->MODER &= ~(1U << 6);

	/* Set Tx pin's (PA2) as Alternate Function */
	GPIOx->MODER |= (1U << 5);
	GPIOx->MODER &= ~(1U << 4);

	/* Set Tx pin's (PA2) as Alternate Function Low Register. AF7 0 1 1 1  */
	GPIOx->AFR[0] |= (1U << 8);
	GPIOx->AFR[0] |= (1U << 9);
	GPIOx->AFR[0] |= (1U << 10);
	GPIOx->AFR[0] &= ~(1U << 11);

	/* Set Rx pin's (PA3) as Alternate Function Low Register. AF7 0 1 1 1  */
	GPIOx->AFR[0] |= (1U << 12);
	GPIOx->AFR[0] |= (1U << 13);
	GPIOx->AFR[0] |= (1U << 14);
	GPIOx->AFR[0] &= ~(1U << 15);

	/* Enable the USART Module */
	usart_base_p->usart->CR1 |= CR1_UE;
}

/*
 * Date............: 08-July-2024 13:12
 * Function........: usart_config_function
 * ................. 
 */

void usart_config_function(usart_handle* usart_base_p){
	if(usart_base_p == NULL)
		return;	
	usart_base_p->usart->CR1 &= ~CR1_MBIT;

	usart_base_p->usart->CR2 &= ~CR2_STOP_BIT;

	/*OVER SAMPLE*/
	usart_base_p->usart->CR1 &= ~CR1_OVER8;

	/* Set USART Baud rate 115200 MHz */
	usart_set_baudrate(usart_base_p->usart, APB1_CLK, UART_BAUDRATE_115200);

	/* Configure The Transfer Direction TE: Transmitter Enable */
	usart_base_p->usart->CR1 |= CR1_TE;

	/* Bit 2 RE: Receiver enable */
	usart_base_p->usart->CR1 |= CR1_RE;

	usart_base_p->state = TX_READY;
	usart_base_p->state = RX_READY;

	/*Enable DMA Transmit Mode*/
	usart_base_p->usart->CR3 |= DMAT;

	/*DMA enable receiver*/
	usart_base_p->usart->CR3 |= DMAR;
}

/*
 * Date............: 24-July-2024 19:23
 * Function........: DMAx_interrupt_config
 * .................
 */
void DMAx_interrupt_config(DMA_Stream_TypeDef* DMA1_StreamX){

	/* Set DMAx Transfer complete interrupt enable */
	DMA1_StreamX->CR |= (DMA_TCIE);

	/* Set DMAx Direct mode error interrupt enable */
	DMA1_StreamX->CR |= (DMA_DMEIE);

	/* Set DMA Transfer error interrupt enable */
	DMA1_StreamX->CR |= (DMA_TEIE);
}

/*
 * Date............: 08-July-2024 11:05
 * Function........: usart_interrupt_config
 * ................. 
 */

void usart_interrupt_config(usart_handle* usart_base_p){
	if(usart_base_p == NULL)
		return;		
	/*CTSIE: CTS interrupt enable*/
	usart_base_p->usart->CR3 |= CR3_CTSIE;

	/* TCIE: Transmission complete interrupt enable */
	usart_base_p->usart->CR1 |= CR1_TCIE;

	/* Set TXE interrupt flag */
	usart_base_p->usart->CR1 |= CR1_TXEIE;

	/*Set RXNE interrupt flag */
	usart_base_p->usart->CR1 |= CR1_RXNEIE;

}

/*
 * Date............: 08-July-2024 09:12
 * Function........: global_interrupt_enable
 * ................. 
 */

void global_interrupt_enable(IRQn_Type irq_num, uint32_t priority){
#if EXTI_EVENT_MODE_WAKE_UP
	/*Tx and Rx pins interrupts*/
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;

	/*Enable event request line for PA2 (Rx)*/
	//EXTI->IMR |= EXTI_IMR_MR2;
	EXTI->EMR |= EXTI_IMR_MR2;

	/*Enable event request line for PA3 (Tx)*/
	//EXTI->IMR |= EXTI_IMR_MR3;
	EXTI->EMR |= EXTI_IMR_MR3;

	/*Enable software event register*/
	EXTI->SWIER |= EXTI_IMR_MR2;
	EXTI->SWIER |= EXTI_IMR_MR3;
#endif
//#if  NVIC_PERIHP_INTERRUPT
	NVIC_SetPriority(irq_num, priority);
	NVIC_EnableIRQ(irq_num);
//#endif
}

/*
 * Date............: 04-July-2024 12:22
 * Function........: usart_transmit_it
 * ................. 
 */

void usart_transmit_it(usart_handle* usart_p) {
	if(usart_p == NULL)
		return;		

	if(usart_p->state != TX_READY) 
		return ;

	const uint8_t *tmp = usart_p->TxBuffPtr;		//" read only " access to read  
	usart_p->usart->DR = (*tmp & 0xFF);
	usart_p->TxBuffPtr++;

	usart_p->TxCounter--;

	/* Check Whether The Transmission End Or Not */
	if (usart_p->TxCounter == 0U)
	{
		/* Set Transmission Complete Interrupt bit And Clear TXE Interupt*/
		usart_p->usart->CR1 |= CR1_TCIE;
		usart_p->usart->CR1 &= ~CR1_TXEIE;
	}

	usart_p->state = RX_READY;

}

/*
 * Date............: 04-July-20204 13:52
 * Function........: usart_receive_it
 * ................. 
 */

void usart_receive_it(usart_handle* usart_p) {
	if(usart_p == NULL)
		return;

	if(usart_p->state != RX_READY) 
		return;

	uint8_t *receive_tmp = usart_p->RxBuffPtr;

	*(uint8_t*) receive_tmp = (usart_p->usart->DR & (0xFF));
	++usart_p->RxBuffPtr;

	/* Clear the RXNE bits.*/
	if (--usart_p->RxCounter == 0U)
		usart_p->usart->SR &= ~SR_RXNE;

	usart_p->state = TX_READY;
}

/*
 * Date............: 26-June-2024 16:12
 * Function........: usart_transmit
 * ................. 
 */

usart_states usart_transmit(usart_handle* usart_base_ptr, const uint8_t* BufferPtr, size_t length){
	if(usart_base_ptr == NULL)
		return;	
	
	if(usart_base_ptr->state != TX_READY)
		return USART_ERROR;

	usart_base_ptr->state = TX_BUSY;

	for (size_t i = 0; i < length; ++i) {
		// Make Sure about the transmit data register is empty.
		while (!(USART_IS_TXE(usart_base_ptr->usart)))
						;
		/* Write to transmit data register. */
		usart_base_ptr->usart->DR = (*(BufferPtr + i) & 0xFF);
	}

	while (!(USART_IS_TC(usart_base_ptr->usart)))
						;
	usart_base_ptr->state = TX_READY;
	return USART_OK;
}

/*
 * Date............: 26-June-2024 16:52
 * Function........: usart_receive
 * ................. 
 */

usart_states usart_receive(usart_handle* usart_base_ptr, uint8_t* RxBuffer, size_t length) {
	if(usart_base_ptr == NULL)
		return;		
	
	if(usart_base_ptr->state != RX_READY)
		return USART_ERROR;
	
	usart_base_ptr->state = RX_BUSY;
	/* Received data is  ready to be read.*/
	for (int i = 0; i < length; ++i)
	{
		while (!(USART_IS_RXNE(usart_base_ptr->usart)))
						;

		*(RxBuffer + i) = usart_base_ptr->usart->DR;
	}
	usart_base_ptr->state = RX_READY;
	return USART_OK;

}

/*
 * Date............: 24-June-2024 01:42
 * Function........: uart_set_baudrate
 * ................. 
 */

void usart_set_baudrate(USART_TypeDef* usart_p, uint32_t PeriphClk, uint32_t BaudRate) {
	if(usart_p == NULL)
		return;	

	usart_p->BRR =
			(uint16_t) ((((((uint32_t) ((((uint64_t) (((PeriphClk)))) * 25)
					/ (4 * ((uint64_t) (((BaudRate))))))) / 100) << 4)
					+ (((((((uint32_t) ((((uint64_t) (((PeriphClk)))) * 25)
							/ (4 * ((uint64_t) (((BaudRate)))))))
							- ((((uint32_t) ((((uint64_t) ((((PeriphClk)))))
									* 25) / (4 * ((uint64_t) ((((BaudRate))))))))
									/ 100) * 100)) * 16)\
 + 50) / 100) & 0xF0))
					+ (((((((uint32_t) ((((uint64_t) (((PeriphClk)))) * 25)
							/ (4 * ((uint64_t) (((BaudRate)))))))
							- ((((uint32_t) ((((uint64_t) ((((PeriphClk)))))
									* 25) / (4 * ((uint64_t) ((((BaudRate))))))))
									/ 100) * 100)) * 16)\
 + 50) / 100) & 0x0F));

}



