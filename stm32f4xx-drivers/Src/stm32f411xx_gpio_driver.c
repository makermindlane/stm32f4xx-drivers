/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: May 7, 2021
 *      Author: pawan
 */

#include "stm32f411xx_gpio_driver.h"


/*
 * @fn				- gpio_periClockControl
 *
 * @brief			- Enables or disables peripheral clock for a given gpio port
 *
 * @param *pGPIOx	- Base address of gpio peripheral
 * @param enOrDi	- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */
void gpio_periClockControl(GPIO_RegDef_t *gpioPort, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		if (gpioPort == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (gpioPort == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (gpioPort == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (gpioPort == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (gpioPort == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (gpioPort == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (gpioPort == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (gpioPort == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (gpioPort == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (gpioPort == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (gpioPort == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (gpioPort == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}

}


/*
 * @fn						- gpio_init
 *
 * @brief					- Initializes the gpio pin with configurations given in the GPIO_Handle_t
 *
 * @param *pGPIOHandle		- Handle of gpio peripheral
 *
 * @return					- none
 *
 * @note					- none
 */
void gpio_init(GPIO_Handle_t *gpioHandle) {

	uint32_t temp = 0;

	// configure mode of the gpio pin
	if (gpioHandle->gpioPinCfg->gpioPinMode <= GPIO_MODE_ANALOG) {
		temp = (gpioHandle->gpioPinCfg->gpioPinMode << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
		gpioHandle->gpioPort->MODER &= ~(0x3 << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
		gpioHandle->gpioPort->MODER |= temp;
	} else {
		//TODO interrupt mode configuration
	}
	temp = 0;

	// configure speed of gpio
	temp = (gpioHandle->gpioPinCfg->gpioPinSpeed << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
	gpioHandle->gpioPort->OSPEEDR &= ~(0x3 << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
	gpioHandle->gpioPort->OSPEEDR |= temp;
	temp = 0;

	// configure pupd settings
	temp = (gpioHandle->gpioPinCfg->gpioPinPuPdCtrl << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
	gpioHandle->gpioPort->PUPDR &= ~(0x3 << (2 * gpioHandle->gpioPinCfg->gpioPinNumber));
	gpioHandle->gpioPort->PUPDR |= temp;
	temp = 0;

	// configure output type
	temp = (gpioHandle->gpioPinCfg->gpioPinOPType << gpioHandle->gpioPinCfg->gpioPinNumber);
	gpioHandle->gpioPort->OTYPER &= ~(0x1 << gpioHandle->gpioPinCfg->gpioPinNumber);
	gpioHandle->gpioPort->OTYPER |= temp;
	temp = 0;

	// configure alternate functionality
	if (gpioHandle->gpioPinCfg->gpioPinMode == GPIO_MODE_ALTFN) {

		uint8_t idx = gpioHandle->gpioPinCfg->gpioPinNumber / 8;
		uint8_t bitIdx = gpioHandle->gpioPinCfg->gpioPinNumber % 8;

		temp = (gpioHandle->gpioPinCfg->gpioPinAltFunMode << (4 * bitIdx));
		gpioHandle->gpioPort->AFR[idx] &= ~(0xF << (4 * bitIdx));
		gpioHandle->gpioPort->AFR[idx] |= temp;
		temp = 0;

	}
}


/*
 * @fn						- gpio_deInit
 *
 * @brief					- Enables or disables peripheral clock for a given gpio port
 *
 * @param *pGPIOHandle		- Handle of gpio peripheral
 *
 * @return					- none
 *
 * @note					- none
 */
void gpio_deInit(GPIO_RegDef_t *gpioPort) {

	if (gpioPort == GPIOA) {
		GPIOA_REG_RESET();
	} else if (gpioPort == GPIOB) {
		GPIOB_REG_RESET();
	} else if (gpioPort == GPIOC) {
		GPIOC_REG_RESET();
	} else if (gpioPort == GPIOD) {
		GPIOD_REG_RESET();
	} else if (gpioPort == GPIOE) {
		GPIOE_REG_RESET();
	} else if (gpioPort == GPIOH) {
		GPIOH_REG_RESET();
	}

}


/*
 * Data read and write
 */
uint8_t gpio_readFromInputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber) {

	uint8_t val;
	val = (uint8_t)((gpioPort->IDR >> pinNumber) & 0x00000001);
	return val;
}


uint16_t gpio_readFromInputPort(GPIO_RegDef_t *gpioPort) {
	uint16_t val;
	val = (uint16_t)(gpioPort->IDR);
	return val;
}


void gpio_writeToOutputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber, uint8_t value) {

	if (value == GPIO_PIN_SET) {
		gpioPort->ODR |= (1 << pinNumber);
	} else {
		gpioPort->ODR &= ~(1 << pinNumber);
	}

}


void gpio_writeToOutputPort(GPIO_RegDef_t *gpioPort, uint16_t value) {
	gpioPort->ODR = value;
}


void gpio_toggleOutputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber) {
	gpioPort->ODR ^= (1 << pinNumber);
}


/*
 * IRQ configuration and ISR handling
 */
void gpio_irqConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enOrDi) {

}


void gpio_irqHandling(uint8_t pinNumber) {

}
















