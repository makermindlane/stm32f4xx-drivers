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
void USART_Init(USART_Handle_t *usartHandle) {

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
















