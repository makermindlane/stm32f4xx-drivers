#include "stm32f411xx_i2c_driver.h"



/**********************************************************************************************************************
 * 												START: Functions prototypes
 *********************************************************************************************************************/

static uint32_t rccGetPclk1Value();
static void i2cGenerateStartCondition(I2C_RegDef_t * i2c);
static void i2cGenerateStopCondition(I2C_RegDef_t * i2c);
static void i2cExecuteAddressPhase(I2C_RegDef_t *i2c, uint8_t slaveAddr, uint8_t isRead);
static void i2cClearAddrFlag(I2C_RegDef_t *i2c);
static void i2cManageAck(I2C_RegDef_t *i2c, uint8_t isEnable);

/**********************************************************************************************************************
 * 												END: Functions prototypes
 *********************************************************************************************************************/

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

	// TRISE configuration
	if (i2cHandle->i2cCfg.sclSpeed <= I2C_SCL_SPEED_SM) {
		// mode is standard mode
		tempReg = (rccGetPclk1Value() / 1000000U) + 1;
	} else {
		// mode is fast mode
		tempReg = ((rccGetPclk1Value() * 300) / 1000000000U) + 1;
	}
	i2cHandle->i2c->TRISE = (tempReg & 0x3F);
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
 * Master send api
 */
void i2c_masterSendData(I2C_Handle_t *i2cHandle, uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr) {

	// generate start condition
	i2cGenerateStartCondition(i2cHandle->i2c);

	// wait until SB bit in SR1 register is set. This indicates that start condition has been generated
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_SB);

	// send the address of the slave with read/write bit
	i2cExecuteAddressPhase(i2cHandle->i2c, slaveAddr, I2C_MASTER_WRITE);

	// wait until ADDR bit is set in SR1 register, indicating end of address transmission.
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_ADDR);

	// clear the ADDR bit
	i2cClearAddrFlag(i2cHandle->i2c);

	// send data until len becomes 0
	while (len > 0) {
		// wait until TxE bit in SR1 register is set, indicating data register is empty and can be written
		WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_TxE);
		i2cHandle->i2c->DR = *txBuffer;
		txBuffer++;
		len--;
	}

	// wait until TxE and BTF bits in SR1 register are set, indicating data register is empty and data byte transfer
	// succeeded. After which only, stop condition can be generated.
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_TxE);
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_BTF);

	// generate the stop condition
	i2cGenerateStopCondition(i2cHandle->i2c);
}


/*
 * Master receive data
 */
void i2c_masterReceiveData(I2C_Handle_t *i2cHandle, uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr) {
	// generate start condition
	i2cGenerateStartCondition(i2cHandle->i2c);

	// wait until SB bit in SR1 register is set. This indicates that start condition has been generated
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_SB);

	// send the address of the slave with read/write bit
	i2cExecuteAddressPhase(i2cHandle->i2c, slaveAddr, I2C_MASTER_READ);

	// wait until ADDR bit is set in SR1 register, indicating end of address transmission.
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_ADDR);

	// for 1 byte of data reception
	if (len == 1) {
		// disable the ACK
		i2cManageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
		// generate stop condition
		i2cGenerateStopCondition(i2cHandle->i2c);
		// clear the ADDR flag
		i2cClearAddrFlag(i2cHandle->i2c);
		// wait until RxNE bit is set in SR1 register, indicating data register is not empty
		WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_RxNE);
		// read data into rx buffer
		*rxBuffer = i2cHandle->i2c->DR;

		return;
	} else if (len > 1) {
		// clear the ADDR flag
		i2cClearAddrFlag(i2cHandle->i2c);
		for (uint32_t i = len; i > 0; --i) {
			// wait until RxNE bit is set in SR1 register, indicating data register is not empty
			WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_RxNE);
			if (i == 2) {
				// disable ACK
				i2cManageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
				// generate stop condition
				i2cGenerateStopCondition(i2cHandle->i2c);
			}
			// read data into rx buffer
			*rxBuffer = i2cHandle->i2c->DR;
			// increment the rx buffer address
			rxBuffer++;
		}
	}
	// re-enable ACK
	if (i2cHandle->i2cCfg.ackCtrl == I2C_ACK_CTRL_ENABLE) {
		i2cManageAck(i2cHandle->i2c, I2C_ACK_CTRL_ENABLE);
	}
}


/*
 * Master send data interrupt api
 */
uint8_t i2c_masterSendDataIt(I2C_Handle_t *i2cHandle, uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart) {
	uint8_t busystate = i2cHandle->txRxState;

	if((busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX)) {
		i2cHandle->txBuffer = txBuffer;
		i2cHandle->txLen = len;
		i2cHandle->txRxState = I2C_STATE_BUSY_IN_TX;
		i2cHandle->devAddr = slaveAddr;
		i2cHandle->repeatedStart = repeatedStart;

		//Implement code to Generate START Condition
		i2cGenerateStartCondition(i2cHandle->i2c);

		//Implement the code to enable ITBUFEN Control Bit
		i2cHandle->i2c->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		i2cHandle->i2c->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		i2cHandle->i2c->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/*
 * Master receive data interrupt api
 */
uint8_t i2c_masterReceiveDataIt(I2C_Handle_t *i2cHandle, uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart) {
	uint8_t busystate = i2cHandle->txRxState;

	if((busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX)) {
		i2cHandle->rxBuffer = rxBuffer;
		i2cHandle->rxLen = len;
		i2cHandle->txRxState = I2C_STATE_BUSY_IN_RX;
		i2cHandle->rxSize = len; //Rxsize is used in the ISR code to manage the data reception
		i2cHandle->devAddr = slaveAddr;
		i2cHandle->repeatedStart = repeatedStart;

		//Implement code to Generate START Condition
		i2cGenerateStartCondition(i2cHandle->i2c);

		//Implement the code to enable ITBUFEN Control Bit
		i2cHandle->i2c->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		i2cHandle->i2c->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		i2cHandle->i2c->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/*
 * I2C peripheral enable/disable
 */
void i2c_peripheralControl(I2C_RegDef_t *i2c, uint8_t enOrDi) {
	if (enOrDi == ENABLE) {
		// set SPE bit, i.e., enable I2C peripheral
		i2c->CR1 |= (1 << I2C_CR1_PE);
	} else {
		// clear SPE bit i.e., disable I2C peripheral
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
 * 												START: Helper functions
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


static void i2cGenerateStartCondition(I2C_RegDef_t * i2c) {
	i2c->CR1 |= (1 << I2C_CR1_START);
}


static void i2cGenerateStopCondition(I2C_RegDef_t * i2c) {
	i2c->CR1 |= (1 << I2C_CR1_STOP);
}


static void i2cExecuteAddressPhase(I2C_RegDef_t *i2c, uint8_t slaveAddr, uint8_t isRead) {
	// make room for read/write bit (zeroth bit)
	slaveAddr = slaveAddr << 1;
	if (isRead) {
		// set the zeroth bit to indicate read operation
		slaveAddr |= 1;
	} else {
		// clear the zeroth bit to indicate write operation
		slaveAddr &= ~(1);
	}
	// put the slave address + read/write bit into data register (DR)
	i2c->DR = slaveAddr;
}


static void i2cClearAddrFlag(I2C_RegDef_t *i2c) {
	// reading SR1 and SR2 registers clears the ADDR bit
	uint32_t dummyRead = i2c->SR1;
	dummyRead = i2c->SR2;
	(void) dummyRead;
}


static void i2cManageAck(I2C_RegDef_t *i2c, uint8_t isEnable) {
	if (isEnable == I2C_ACK_CTRL_ENABLE) {
		i2c->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		i2c->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/**********************************************************************************************************************
 * 												END: Helper functions
 *********************************************************************************************************************/









