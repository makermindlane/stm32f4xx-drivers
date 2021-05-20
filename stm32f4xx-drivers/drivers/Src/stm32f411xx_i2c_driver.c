#include "stm32f411xx_i2c_driver.h"


/*
 * Peripheral clock setup
 */
void i2c_periClockControl(I2C_RegDef_t *i2c, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		if (i2c == I2C1) {
			I2C1_PCLK_EN();
		} else if (i2c == I2C2) {
			I2C2_PCLK_EN();
		} else if (i2c == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (i2c == I2C1) {
			SPI1_PCLK_DI();
		} else if (i2c == I2C2) {
			SPI2_PCLK_DI();
		} else if (i2c == I2C3) {
			SPI3_PCLK_DI();
		}
	}

}


/*
 * I2C Init
 */
void i2c_init(I2C_Handle_t *i2cHandle) {

}


/*
 * I2C DeInit
 */
void i2c_deInit(I2C_RegDef_t *i2c) {

	if (i2c == I2C1) {
		I2C1_REG_RESET();
	} else if (i2c == I2C2) {
		I2C2_REG_RESET();
	} else if (i2c == I2C3) {
		I2C3_REG_RESET();
	}

}


/*
 * I2C peripheral enable/disable
 */
void i2c_peripheralControl(I2C_RegDef_t *i2c, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		// set SPE bit, i.e., enable SPI peripheral
		i2c->CR1 |= (1 << I2C_CR1_PE);
	} else {
		// clear SPE bit i.e., disable SPI peripheral
		i2c->CR1 &= ~(1 << I2C_CR1_PE);
	}

}



/*
 * I2C irq interrupt config
 */
void i2c_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {

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


/*
 * I2C irq priority config
 */
void i2c_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {

	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (irqPriority << shiftAmount);

}



















