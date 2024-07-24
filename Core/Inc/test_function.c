/*
 * test_function.c
 *
 *  Created on: Jul 22, 2024
 *      Author: ozana
 */

/*
 * Date............: 22-July-2024 23:19
 * Function........: usart_transmit_test


	usart_base_obj.usart = USART2;

	usart_periph_clock_init();

	usart_init(&usart_base_obj);
	usart_config_function(&usart_base_obj);

	usart_base_obj.TxBuffPtr = TxBuffer;
	usart_base_obj.TxCounter = SIZE;

	usart_base_obj.RxBuffPtr = RxBuffer;
	usart_base_obj.RxCounter = SIZE;

	while (1) {
		usart_transmit(&usart_base_obj,TxBuffer, SIZE);
		//usart_receive(&usart_base_obj,RxBuffer, SIZE);
	}

 */


