#include "stm32f411xx_usart_driver.h"


/*
 * Peripheral Clock setup
 */
void usart_periClockControl(USART_RegDef_t *usart, uint8_t isEnable) {
	if (isEnable == ENABLE) {
		if (usart == USART1) {
			USART1_PCLK_EN();
		} else if (usart == USART2) {
			USART2_PCLK_EN();
		} else if (usart == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if (usart == USART1) {
			USART1_PCLK_DI();
		} else if (usart == USART2) {
			USART2_PCLK_DI();
		} else if (usart == USART6) {
			USART6_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void usart_init(USART_Handle_t *usartHandle) {

	//Temporary variable
	uint32_t tempreg = 0;

	/******************************** Configuration of CR1******************************************/

	// Enable the Clock for given USART peripheral
	usart_periClockControl(usartHandle->usart, ENABLE);

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (usartHandle->usartCfg.mode == USART_MODE_ONLY_RX) {
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (usartHandle->usartCfg.mode == USART_MODE_ONLY_TX) {
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);
	} else if (usartHandle->usartCfg.mode == USART_MODE_TXRX) {
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= usartHandle->usartCfg.wordLength << USART_CR1_M;

	//Configuration of parity control bit fields
	if (usartHandle->usartCfg.parityControl == USART_PARITY_EN_EVEN) {
		//Implement the code to enaBle the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control
	} else if (usartHandle->usartCfg.parityControl == USART_PARITY_EN_ODD) {
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	//Program the CR1 register
	usartHandle->usart->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= usartHandle->usartCfg.noOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	usartHandle->usart->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if (usartHandle->usartCfg.hWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	} else if (usartHandle->usartCfg.hWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	} else if (usartHandle->usartCfg.hWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	usartHandle->usart->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
}


void usart_deInit(USART_RegDef_t *usart) {
	if (usart == USART1) {
		USART1_REG_RESET();
	} else if (usart == USART2) {
		USART2_REG_RESET();
	} else if (usart == USART6) {
		USART6_REG_RESET();
	}
}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void usart_sendData(USART_Handle_t *usartHandle, uint8_t *txBuffer, uint32_t len) {
	uint16_t *pdata;
	//Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < len; i++) {
		//Implement the code to wait until TXE flag is set in the SR
		WAIT_UNTIL_SET(usartHandle->usart->SR, USART_SR_TXE);

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (usartHandle->usartCfg.wordLength == USART_WORDLEN_9BITS) {
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) txBuffer;
			usartHandle->usart->DR = (*pdata & (uint16_t) 0x01FF);

			//check for USART_ParityControl
			if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				txBuffer++;
				txBuffer++;
			} else {
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				txBuffer++;
			}
		} else {
			//This is 8bit data transfer
			usartHandle->usart->DR = (*txBuffer & (uint8_t) 0xFF);

			//Implement the code to increment the buffer address
			txBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	WAIT_UNTIL_SET(usartHandle->usart->SR, USART_SR_TC);
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void usart_receiveData(USART_Handle_t *usartHandle, uint8_t *rxBuffer, uint32_t len) {
	//Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < len; i++) {
		//Implement the code to wait until RXNE flag is set in the SR
		WAIT_UNTIL_SET(usartHandle->usart->SR, USART_SR_RXNE);

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (usartHandle->usartCfg.wordLength == USART_WORDLEN_9BITS) {
			//We are going to receive 9bit data in a frame
			//check are we using USART_ParityControl control or not
			if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
				//No parity is used. so, all 9bits will be of user data
				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) rxBuffer) = (usartHandle->usart->DR & (uint16_t) 0x1FF);
				//Now increment the rxBuffer two times
				rxBuffer++;
				rxBuffer++;
			} else {
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*rxBuffer = (usartHandle->usart->DR & (uint8_t) 0xFF);
				//Increment the pRxBuffer
				rxBuffer++;
			}
		} else {
			//We are going to receive 8bit data in a frame
			//check are we using USART_ParityControl control or not
			if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
				//No parity is used, so all 8 bits will be of user data
				//read 8 bits from DR
				*rxBuffer = (usartHandle->usart->DR & (uint8_t) 0xFF);
			} else {
				//Parity is used, so, 7 bits will be of user data and 1 bit is parity
				//read only 7 bits, hence mask the DR with 0X7F
				*rxBuffer = (uint8_t) (usartHandle->usart->DR & (uint8_t) 0x7F);
			}
			//increment the pRxBuffer
			rxBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t usart_sendDataIt(USART_Handle_t *usartHandle, uint8_t *txBuffer, uint32_t len) {
	uint8_t txstate = usartHandle->txBusyState;

	if (txstate != USART_STATE_BUSY_IN_TX) {
		usartHandle->txLen = len;
		usartHandle->txBuffer = txBuffer;
		usartHandle->txBusyState = USART_STATE_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		usartHandle->usart->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC
		usartHandle->usart->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *usartHandle, uint8_t *rxBuffer, uint32_t len) {
	uint8_t rxstate = usartHandle->rxBusyState;

	if (rxstate != USART_STATE_BUSY_IN_RX) {
		usartHandle->rxLen = len;
		usartHandle->rxBuffer = rxBuffer;
		usartHandle->rxBusyState = USART_STATE_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		usartHandle->usart->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}



/*
 * IRQ Configuration and ISR handling
 */
void usart_irqInterruptConfig(uint8_t irqNumber, uint8_t isEnable) {
	if (isEnable == ENABLE) {
		if (irqNumber <= 31) {
			// program ISER0 register
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber >= 32 && irqNumber <= 63) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber <= 95) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (irqNumber % 32));
		}
	} else {
		if (irqNumber <= 31) {
			// program ICER0 register
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber >= 32 && irqNumber <= 63) {
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber >= 64 && irqNumber <= 95) {
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (irqNumber % 32));
		}
	}
}


void usart_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (irqPriority << shiftAmount);
}


void usart_irqHandling(USART_Handle_t *usartHandle) {

}
















