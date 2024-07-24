/******************************************************************************
 * File Name.......: uart                           						  *
 * Application.....: Self Development									      *
 * Author..........: Ozan Akyol												  *
 * Created on......: 22-June-2024 00:32                                         *
 *				   : uart driver  functions          	  			          *
 -----------------------------------------------------------------------------*/

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "stm32f407xx.h"
#include <stddef.h>

typedef enum usart_transmission_state_enum{
	TX_BUSY,
	RX_BUSY,
	TX_READY,
	RX_READY
}usart_transmission_states;

typedef struct usart_handle_s {
	USART_TypeDef *usart;
	const uint8_t *TxBuffPtr;
	volatile uint8_t TxCounter;
	uint8_t TxBuffSize;
	uint8_t *RxBuffPtr;
	volatile uint8_t RxCounter;
	uint8_t RxBuffSize;
	usart_transmission_states state;
}usart_handle;

typedef enum usart_states_enum{
	USART_OK,
	USART_ERROR
}usart_states;

typedef enum {
	TX_STREAM6_CHANNEL4,	/*USART2_Tx REQ*/
	RX_STREAM5_CHANNEL4		/*USART2_Rx REQ*/
}stream_names;

#define USART_IS_TXE(__USART__)		(__USART__-> SR  & SR_TXE) == SR_TXE
#define USART_IS_TC(__USART__)		(__USART__-> SR  & SR_TC) == SR_TC	
#define USART_IS_RXNE(__USART__)	(__USART__-> SR  & SR_RXNE) == SR_RXNE
#define USART_IS_TXEIE(__USART__) 	(__USART__-> CR1  & CR1_TXEIE) == CR1_TXEIE
#define USART_IS_RXNEIE(__USART__)	(__USART__-> CR1  & CR1_RXNEIE) == CR1_RXNEIE

/* RCC Clock bits*/
#define GPIOAEN			(1U << 0)
#define USART2EN		(1U << 17)
#define DMA1EN			(1U << 21)

/*Clock Settings*/
#define SYS_FREQ				16000000
#define	APB1_CLK				SYS_FREQ
#define UART_BAUDRATE_115200	115200
#define UART_BAUDRATE_460800	460800

/* Usart2 pin. Tx = PA2, Rx = PA3 */
#define PIN_2			(1U << 2)
#define PIN_3			(1U << 3)
#define TX_PIN			PIN_2
#define RX_PIN			PIN_3

/* Usart Control & Status Register Bits*/
#define CR1_UE 		 	(1U << 13U)
#define CR1_PCE      	(1U << 10)
#define CR2_CLKEN	 	(1U << 11)
#define CR1_M		 	(1U << 12)
#define CR3_DMAT	 	(1U << 7)
#define CR1_MBIT		(1U << 12)
#define CR2_STOP_BIT	(0x03 << 12)
#define CR1_OVER8		(1U << 15)
#define CR1_TE			(1U << 3)
#define SR_TXE			(1U << 7)
#define CR_RXNEIE		(1U << 5)
#define SR_RXNE			(1U << 5)
#define CR1_RE			(1U << 2)

#define USART2_DATA_REG_OFFSET	 		(0x04UL)
#define USART2_DATA_REG_ADDRESS	 		(USART2_BASE + USART2_DATA_REG_OFFSET)

/*DMA Configuration*/
#define DMAT							(1U << 7)
#define DMAR							(1U << 6)
#define SxCR_PERIPH_TO_MEMORY			(0 << 6)
#define SxCR_MEMORY_TO_PERIPH			(1 << 6)
#define DMA_CHANNEL_4			 		(1U << 27)
#define DMA_CHANNEL_4_MEDIUM_PRIORITY	(1 << 16)
#define DMA_FCR_DMDIS					(1 << 2) 		//Direct mode
#define SxCR_DIR_MASK					(0x03 << 6)

/* Interrupt Bits. */
#define CR1_TXEIE			(1U << 7)
#define CR1_TCIE			(1U << 6)
#define SR_TC				(1U << 6)
#define CR1_RXNEIE			(1U << 5)
#define CR3_CTSIE			(1U << 10)
#define ERROR_FLAG_DMEIF6	(1U << 18)
#define ERROR_FLAG_DMEIF5	(1U << 8)

/*DMA Bits.*/
#define DMA_CR_EN			(1U << 0)	

/*Interrupt event control bits*/
#define DMA_TCIE			(1U << 4)
#define DMA_TEIE			(1U << 2)
#define DMA_DMEIE			(1U << 0)

/*Interrupt event flag bits*/
#define DMA_TCIF5			(1U << 11)
#define DMA_TCIF6			(1U << 21)
#define	DMA_TEIF6			(1U << 19)
#define	DMA_TEIF5			(1U << 9)
#define DMA_DMEIF6			(1U << 18)
#define DMA_DMEIF5			(1U << 8)

/*Interrupt flag clear bits*/
#define DMA_CTCIF5			(1U << 11)
#define DMA_CTCIF6			(1U << 21)

/*Usart Configuration & initialization Functions*/
void usart_periph_clock_init();
void usart_set_baudrate(USART_TypeDef* usart_p, uint32_t PeriphClk, uint32_t BaudRate);
void usart_gpio_init(usart_handle* usart_base_p, GPIO_TypeDef * GPIOx);
void usart_config_function(usart_handle* usart_base_p);

/*Usart Transmit & Receive Functions*/
usart_states usart_transmit(usart_handle* usart_base_ptr ,const uint8_t* BufferPtr, size_t length);
usart_states usart_receive(usart_handle* usart_base_ptr,uint8_t* RxBuffer, size_t length);
void usart_transmit_it(usart_handle* usart_p);
void usart_receive_it(usart_handle* usart_p);
void stream_config_for_dma_receive(DMA_Stream_TypeDef* DMA1_StreamX, uint8_t* RcvBufferPtr, uint32_t length);
void stream_config_for_dma_transmit(DMA_Stream_TypeDef* DMA1_StreamX, const uint8_t* BufferPtr, uint32_t length);

/*Interrupt configuration*/
void usart_interrupt_config(usart_handle* usart_base_p);
void _usart_IRQHandler(usart_handle* usart_p);
void _DMA1_Stream5_IRQHandler(usart_handle* usart_p, DMA_TypeDef* DMAx);
void _DMA1_Stream6_IRQHandler(usart_handle* usart_p, DMA_TypeDef* DMAx);
void global_interrupt_enable(IRQn_Type irq_num, uint32_t priority);

/* DMA Configuration Functions*/
usart_states disable_dma_stream(DMA_Stream_TypeDef* DMA1_StreamX, stream_names stream);
void enable_dma_stream(DMA_Stream_TypeDef* DMA1_StreamX, stream_names stream);
void DMAx_interrupt_config(DMA_Stream_TypeDef* DMA1_StreamX);

#endif /* SRC_UART_H_ */
