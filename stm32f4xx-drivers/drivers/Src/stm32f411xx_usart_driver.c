#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_rcc_driver.h"

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

void usart_peripheralControl(USART_RegDef_t *usart, uint8_t isEnable) {
	if (isEnable == ENABLE) {
		// set SPE bit, i.e., enable I2C peripheral
		usart->CR1 |= (1 << USART_CR1_UE);
	} else {
		// clear SPE bit i.e., disable I2C peripheral
		usart->CR1 &= ~(1 << USART_CR1_UE);
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
	usart_setBaudRate(usartHandle->usart, usartHandle->usartCfg.baud);
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
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void usart_setBaudRate(USART_RegDef_t *usart, uint32_t baudRate) {

	//Variable to hold the APB clock
	uint32_t periClkx;
	uint32_t usartdiv;
	//variables to hold Mantissa and Fraction values
	uint32_t mantissa, fraction;

	uint32_t tempreg = 0;
	//Get the value of APB bus clock in to the variable periClkx
	if (usart == USART1 || usart == USART6) {
		//USART1 and USART6 are hanging on APB2 bus
		periClkx = rcc_getPclk2Value();
	} else {
		periClkx = rcc_getPclk1Value();
	}

	//Check for OVER8 configuration bit
	if (IS_BIT_SET(usart->CR1, USART_CR1_OVER8)) {
		//OVER8 = 1, over sampling by 8
		// usartdiv = ((100 * periClk) / (2 * (2 - OVER8) * baudRate)).
		// Here multiplication by 100 is done to make it an integer. Real formula doesn't have 100.
		usartdiv = ((25 * periClkx) / (2 * baudRate));
	} else {
		//OVER8 = 0, over sampling by 16
		usartdiv = ((25 * periClkx) / (4 * baudRate));
	}

	//Calculate the Mantissa part
	mantissa = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position. Refer USART_BRR
	tempreg |= mantissa << USART_BRR_DIV_MANTISSA;

	//Extract the fraction part
	fraction = (usartdiv - (mantissa * 100));

	//Calculate the final fractional
	if (IS_BIT_SET(usart->CR1, USART_CR1_OVER8)) {
		//OVER8 = 1, over sampling by 8
		fraction = (((fraction * 8) + 50) / 100) & ((uint8_t) 0x07);

	} else {
		//over sampling by 16
		fraction = (((fraction * 16) + 50) / 100) & ((uint8_t) 0x0F);

	}

	//Place the fractional part in appropriate bit position. refer USART_BRR
	tempreg |= fraction;

	//copy the value of tempreg in to BRR register
	usart->BRR = tempreg;
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
uint8_t usart_receiveDataIt(USART_Handle_t *usartHandle, uint8_t *rxBuffer, uint32_t len) {
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

/*
 * IRQ Handling
 */
void usart_irqHandling(USART_Handle_t *usartHandle) {
	uint32_t temp1, temp2, temp3;

	// 1. Handle interrupt generated by TC bit set.
	// Implement the code to check the state of TC bit in the SR
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_TC);
	// Implement the code to check the state of TCEIE bit
	temp2 = IS_BIT_SET(usartHandle->usart->CR1, USART_CR1_TCIE);
	if (temp1 && temp2) {
		// This interrupt is because of TC.
		// Close transmission and call application callback if txLen is zero.
		if (usartHandle->txBusyState == USART_STATE_BUSY_IN_TX) {
			// Check the txLen. If it is zero then close the data transmission.
			if (usartHandle->txLen == 0) {
				// Implement the code to clear the TC flag
				usartHandle->usart->SR &= ~(1 << USART_SR_TC);
				// Implement the code to clear the TCIE control bit
				usartHandle->usart->CR1 &= ~(1 << USART_CR1_TCIE);
				// Reset the application state
				usartHandle->txBusyState = USART_STATE_READY;
				// Reset Buffer address to NULL
				usartHandle->txBuffer = NULL;
				// Reset the length to zero
				usartHandle->txLen = 0;
				// Call the application call back with event USART_EVENT_TX_CMPLT
				usart_appEventCallback(usartHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	// 2. Handle interrupt generated by TXE bit set.
	// Implement the code to check the state of TXE bit in the SR
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_TXE);
	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = IS_BIT_SET(usartHandle->usart->CR1, USART_CR1_TXEIE);

	if (temp1 && temp2) {
		// This interrupt is because of TXE.
		if (usartHandle->txBusyState == USART_STATE_BUSY_IN_TX) {
			// Keep sending data until txlen reaches to zero
			if (usartHandle->txLen > 0) {
				// Check the word length item for 9BIT or 8BIT in a frame
				if (usartHandle->usartCfg.wordLength == USART_WORDLEN_9BITS) {
					// If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
					uint16_t *pdata = (uint16_t*) usartHandle->txBuffer;
					// Loading only first 9 bits , so we have to mask with the value 0x01FF
					usartHandle->usart->DR = (*pdata & (uint16_t) 0x01FF);
					// Check for parity control.
					if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
						// No parity is used in this transfer, so, 9bits of user data will be sent
						// Implement the code to increment pTxBuffer twice
						usartHandle->txBuffer++;
						usartHandle->txBuffer++;
						//Implement the code to decrement the length
						usartHandle->txLen -= 2;
					} else {
						// Parity bit is used in this transfer. so , 8bits of user data will be sent
						// The 9th bit will be replaced by parity bit by the hardware
						usartHandle->txBuffer++;
						// Implement the code to decrement the length
						usartHandle->txLen--;
					}
				} else {
					//T his is 8bit data transfer
					usartHandle->usart->DR = (*usartHandle->txBuffer & (uint8_t) 0xFF);
					// Implement the code to increment the buffer address
					usartHandle->txBuffer++;
					// Implement the code to decrement the length
					usartHandle->txLen--;
				}

			}
			if (usartHandle->txLen == 0) {
				// TxLen is zero
				// Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				usartHandle->usart->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	// 3. Handle interrupt generated by RXNE bit set.
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_RXNE);
	temp2 = IS_BIT_SET(usartHandle->usart->CR1, USART_CR1_RXNEIE);
	if (temp1 && temp2) {
		// This interrupt is because of rxne
		// This interrupt is because of txe
		if (usartHandle->rxBusyState == USART_STATE_BUSY_IN_RX) {
			// TXE is set so send data
			if (usartHandle->rxLen > 0) {
				// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (usartHandle->usartCfg.wordLength == USART_WORDLEN_9BITS) {
					// We are going to receive 9bit data in a frame
					// Now, check are we using parity control control or not
					if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
						// No parity is used. so, all 9bits will be of user data
						// read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) usartHandle->rxBuffer) = (usartHandle->usart->DR & (uint16_t) 0x01FF);
						// Now increment the pRxBuffer two times
						usartHandle->rxBuffer += 2;
						// Implement the code to decrement the length
						usartHandle->rxLen -= 2;
					} else {
						// Parity is used. so, 8bits will be of user data and 1 bit is parity
						*usartHandle->rxBuffer = (usartHandle->usart->DR & (uint8_t) 0xFF);
						//Now increment the pRxBuffer
						usartHandle->rxBuffer++;
						//Implement the code to decrement the length
						usartHandle->rxLen--;
					}
				} else {
					// We are going to receive 8bit data in a frame
					// Now, check are we using parity control control or not
					if (usartHandle->usartCfg.parityControl == USART_PARITY_DISABLE) {
						// No parity is used , so all 8bits will be of user data
						// read 8 bits from DR
						*usartHandle->rxBuffer = (uint8_t) (usartHandle->usart->DR & (uint8_t) 0xFF);
					}

					else {
						// Parity is used, so, 7 bits will be of user data and 1 bit is parity
						// read only 7 bits , hence mask the DR with 0X7F
						*usartHandle->rxBuffer = (uint8_t) (usartHandle->usart->DR & (uint8_t) 0x7F);
					}
					// Now, increment the pRxBuffer
					usartHandle->rxBuffer++;
					// Implement the code to decrement the length
					usartHandle->rxLen--;
				}
			}
			if (usartHandle->rxLen == 0) {
				// Disable the RXNE
				usartHandle->usart->CR1 &= ~(1 << USART_CR1_RXNEIE);
				usartHandle->rxBusyState = USART_STATE_READY;
				usart_appEventCallback(usartHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	// 4. Handle interrupt generated by CTS bit set.
	// Note : CTS feature is not applicable for UART4 and UART5
	// Implement the code to check the status of CTS bit in the SR
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_CTS);
	// Implement the code to check the state of CTSE bit in CR1
	temp2 = IS_BIT_SET(usartHandle->usart->CR3, USART_CR3_CTSE);
	// Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = IS_BIT_SET(usartHandle->usart->CR3, USART_CR3_CTSIE);
	(void) temp3;

	if (temp1 && temp2) {
		// Implement the code to clear the CTS flag in SR
		usartHandle->usart->SR &= ~(1 << USART_SR_CTS);
		// This interrupt is because of CTS.
		usart_appEventCallback(usartHandle, USART_EVENT_CTS);
	}

	// 5. Handle interrupt generated by IDLE bit set.
	// Implement the code to check the status of IDLE flag bit in the SR
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_IDLE);

	// Implement the code to check the state of IDLEIE bit in CR1
	temp2 = IS_BIT_SET(usartHandle->usart->CR3, USART_CR3_CTSE);

	if (temp1 && temp2) {
		// Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		usart_clearORE(usartHandle->usart);
		// This interrupt is because of idle
		usart_appEventCallback(usartHandle, USART_EVENT_IDLE);
	}

	// 6. Handle interrupt generated by ORE (overrun error) bit set.
	// Implement the code to check the status of ORE flag in the SR
	temp1 = IS_BIT_SET(usartHandle->usart->SR, USART_SR_ORE);
	// Implement the code to check the status of RXNEIE bit in the CR1
	temp2 = IS_BIT_SET(usartHandle->usart->CR1, USART_CR1_RXNEIE);
	if (temp1 && temp2) {
		// Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag.
		// This interrupt is because of Overrun error
		usart_appEventCallback(usartHandle, USART_EVENT_ORE);
	}

	/*************************Check for Error Flag ********************************************/
	// Noise Flag, Overrun error and Framing Error in multibuffer communication
	// We don't discuss multibuffer communication in this course. please refer to the RM
	// The below code will get executed in only if multibuffer mode is used.
	temp2 = usartHandle->usart->CR3 & (1 << USART_CR3_EIE);

	if (temp2) {
		temp1 = usartHandle->usart->SR;
		if (temp1 & (1 << USART_SR_FE)) {
			/*
			 This bit is set by hardware when a de-synchronization, excessive noise or a break character
			 is detected. It is cleared by a software sequence (an read to the USART_SR register
			 followed by a read to the USART_DR register).
			 */
			usart_appEventCallback(usartHandle, USART_ERREVENT_FE);
		}

		if (temp1 & (1 << USART_SR_NE)) {
			/*
			 This bit is set by hardware when noise is detected on a received frame. It is cleared by a
			 software sequence (an read to the USART_SR register followed by a read to the
			 USART_DR register).
			 */
			usart_appEventCallback(usartHandle, USART_ERREVENT_NE);
		}

		if (temp1 & (1 << USART_SR_ORE)) {
			usart_appEventCallback(usartHandle, USART_ERREVENT_ORE);
		}
	}

}

void usart_clearORE(USART_RegDef_t *usart) {
	// Clearing sequence:
	//		read SR register then read DR register.
	uint32_t dummyReadSR = usart->SR;
	uint32_t dummyReadDR = usart->DR;
	// To avoid compiler warning. It does nothing as far as functionality is concerned.
	(void) dummyReadSR;
	(void) dummyReadDR;
}

