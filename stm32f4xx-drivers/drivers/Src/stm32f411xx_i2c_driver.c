#include "stm32f411xx_i2c_driver.h"



/**********************************************************************************************************************
 * 												Functions prototypes
 *********************************************************************************************************************/

static uint32_t rccGetPclk1Value();

/*
 * AHB and APB1 prescaler values
 */
int16_t ahbPrescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t apb1Prescaler[4] = { 2, 4 , 8, 16};


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

	uint32_t tempReg = 0;

	// set ACK bit
	tempReg |= i2cHandle->i2cCfg.ackCtrl << I2C_CR1_ACK;
	i2cHandle->i2c->CR1 = tempReg;

	// configure the FREQ bits
	tempReg = 0;
	tempReg = rccGetPclk1Value() / 1000000;
	i2cHandle->i2c->CR2 = (tempReg & 0x3F);

	// set device's own address
	tempReg = i2cHandle->i2cCfg.deviceAddr << 1;
	tempReg |= 1 << 14; 	// Don't know why but reference manual suggests to do it.
	i2cHandle->i2c->OAR1 = tempReg;

	// CCR calculations
	uint16_t ccr = 0;
	tempReg = 0;
	if (i2cHandle->i2cCfg.sclSpeed <= I2C_SCL_SPEED_SM) {
		// mode is standard mode
		ccr = (rccGetPclk1Value() / (2 * i2cHandle->i2cCfg.sclSpeed));
		tempReg |= (ccr & 0xFFF);
	} else {
		// mode is fast mode
		tempReg |= (1 << I2C_CCR_FS);
		tempReg |= (i2cHandle->i2cCfg.fmDutyCycle << I2C_CCR_DUTY);

		if (i2cHandle->i2cCfg.fmDutyCycle == I2C_FM_DUTY_CYCLE_2) {
			ccr = (rccGetPclk1Value() / (3 * i2cHandle->i2cCfg.sclSpeed));
		} else {
			ccr = (rccGetPclk1Value() / (25 * i2cHandle->i2cCfg.sclSpeed));
		}
		tempReg |= (ccr & 0xFFF);
	}
	i2cHandle->i2c->CCR = tempReg;

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



/**********************************************************************************************************************
 * 												Helper functions
 *********************************************************************************************************************/

/*
 * Get the PLL clock value (as of now it does nothing)
 */
static uint32_t rccGetPllOutputClk() {
	return 16000000;
}


/*
 * Get the peripheral clock 1 value
 */
static uint32_t rccGetPclk1Value() {

	uint32_t sysClk, pClk1;
	uint8_t clkSrc, temp, ahbPres, apb1Pres;
	// get the clock source
	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0) {
		// HSI is selected as system clock source
		sysClk = 16000000;
	} else if (clkSrc == 1) {
		// HSE is selected as system clock source
		sysClk = 8000000;
	} else if (clkSrc == 2) {
		// PLL is selected as system clock source
		sysClk = rccGetPllOutputClk();
	}

	// for ahb prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbPres = 1;
	} else {
		ahbPres = ahbPrescaler[temp - 8];
	}

	// for apb1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4) {
		apb1Pres = apb1Prescaler[temp - 4];
	}

	pClk1 = (sysClk / ahbPres) / apb1Pres;

	return pClk1;
}














