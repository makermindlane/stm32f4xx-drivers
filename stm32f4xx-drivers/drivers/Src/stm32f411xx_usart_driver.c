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


/*
 * Init and De-init
 */
void usart_init(USART_Handle_t *usartHandle) {
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
















