#include "stm32f411xx_i2c_driver.h"



/**********************************************************************************************************************
 * 												START: Private functions prototypes
 *********************************************************************************************************************/

static uint32_t rccGetPclk1Value();
static void i2cGenerateStartCondition(I2C_RegDef_t * i2c);
static void i2cGenerateStopCondition(I2C_RegDef_t * i2c);
static void i2cExecuteAddressPhase(I2C_RegDef_t *i2c, uint8_t slaveAddr, uint8_t isRead);
static void i2cClearAddrFlag(I2C_Handle_t *i2cHandle);
static void i2cCloseSendData(I2C_Handle_t *i2cHandle);
static void i2cCloseReceiveData(I2C_Handle_t *i2cHandle);
static void i2cMasterHandleRxneIt(I2C_Handle_t *i2cHandle);
static void i2cMasterHandleTxeIt(I2C_Handle_t *i2cHandle);

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
 * Enable or disable the ACK bit
 */
void i2c_manageAck(I2C_RegDef_t *i2c, uint8_t isEnable) {
	if (isEnable == I2C_ACK_CTRL_ENABLE) {
		i2c->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		i2c->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


/*
 * I2C Init
 */
void i2c_init(I2C_Handle_t *i2cHandle) {

	uint32_t tempReg = 0;

	// enable i2c peripheral clock control
	i2c_periClockControl(i2cHandle->i2c, ENABLE);

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
void i2c_masterSendData(I2C_Handle_t *i2cHandle, uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart) {

	// generate start condition
	i2cGenerateStartCondition(i2cHandle->i2c);

	// wait until SB bit in SR1 register is set. This indicates that start condition has been generated
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_SB);

	// send the address of the slave with read/write bit
	i2cExecuteAddressPhase(i2cHandle->i2c, slaveAddr, I2C_MASTER_WRITE);

	// wait until ADDR bit is set in SR1 register, indicating end of address transmission.
	WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_ADDR);

	// clear the ADDR bit
	i2cClearAddrFlag(i2cHandle);

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

	// Generate the stop condition only if repeated start is disabled.
	if (repeatedStart == DISABLE) {
		i2cGenerateStopCondition(i2cHandle->i2c);
	}
}


/*
 * Master receive data
 */
void i2c_masterReceiveData(I2C_Handle_t *i2cHandle, uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart) {
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
		i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
		// clear the ADDR flag
		i2cClearAddrFlag(i2cHandle);
		// wait until RxNE bit is set in SR1 register, indicating data register is not empty
		WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_RxNE);
		// Generate stop condition only if repeated start is disabled.
		if (repeatedStart == DISABLE) {
			i2cGenerateStopCondition(i2cHandle->i2c);
		}
		// read data into rx buffer
		*rxBuffer = i2cHandle->i2c->DR;
	} else if (len > 1) {
		// clear the ADDR flag
		i2cClearAddrFlag(i2cHandle);
		for (uint32_t i = len; i > 0; --i) {
			// wait until RxNE bit is set in SR1 register, indicating data register is not empty
			WAIT_UNTIL_SET(i2cHandle->i2c->SR1, I2C_SR1_RxNE);
			if (i == 2) {
				// disable ACK
				i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
				// Generate stop condition only if repeated start is disabled.
				if (repeatedStart == DISABLE) {
					i2cGenerateStopCondition(i2cHandle->i2c);
				}
			}
			// read data into rx buffer
			*rxBuffer = i2cHandle->i2c->DR;
			// increment the rx buffer address
			rxBuffer++;
		}
	}
	// re-enable ACK
	if (i2cHandle->i2cCfg.ackCtrl == I2C_ACK_CTRL_ENABLE) {
		i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_ENABLE);
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
 * Slave send data
 */
void i2c_slaveSendData(I2C_RegDef_t *i2c, uint8_t data) {
	i2c->DR = data;
}


/*
 * Slave receive data
 */
uint8_t i2c_slaveReceiveData(I2C_RegDef_t *i2c) {
	return (uint8_t) i2c->DR;
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


/*
 * I2C event interrupt handler
 */
void i2c_evIrqHandling(I2C_Handle_t *i2cHandle) {
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = IS_BIT_SET(i2cHandle->i2c->CR2, I2C_CR2_ITEVTEN);
	temp2 = IS_BIT_SET(i2cHandle->i2c->CR2, I2C_CR2_ITBUFEN);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_SB);
	if (temp1 && temp3) {
		// handle SB event.
		// NOTE: SB event is generated only in master mode

		if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_TX) {
			i2cExecuteAddressPhase(i2cHandle->i2c, i2cHandle->devAddr, I2C_MASTER_WRITE);
		} else if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_RX) {
			i2cExecuteAddressPhase(i2cHandle->i2c, i2cHandle->devAddr, I2C_MASTER_READ);
		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_ADDR);
	if (temp1 && temp3) {
		i2cClearAddrFlag(i2cHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_BTF);
	if (temp1 && temp3) {
		if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_TX) {
			// i2c is in transmission state.
			if (IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_TxE)) {
				// Data register is empty. Check further if this is the last byte of transmission.
				if (i2cHandle->txLen == 0) {
					// This is the last byte of transmission.
					// If control comes here, then it means both BTF and TXE bits are set in transmission
					// mode. This further means that last byte has been transferred and we can initiate
					// i2c stopping condition.
					if (i2cHandle->repeatedStart == I2C_RS_DISABLE) {
						// Repeated start is disabled, hence can generate stop condition.
						i2cGenerateStopCondition(i2cHandle->i2c);
						// Reset all the member variables of i2c handle struct.
						i2cCloseSendData(i2cHandle);
						// Notify application about transmission complete.
						i2c_appEventCallback(i2cHandle, I2C_EVENT_TX_CMPLT);
					}
				}
			}
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_STOPF);
	if (temp1 && temp3) {
		// Clear the STOPF bit. This is done by reading the SR1 (which has already been done when temp3 variable
		// was read above) and writing to CR1 register.
		// Not actually writing anything but performing the write operation.
		i2cHandle->i2c->CR1 |= 0x0000;
		// Notify the application that stop is detected.
		i2c_appEventCallback(i2cHandle, I2C_EVENT_STOP);
	}

	//5. Handle For interrupt generated by TXE event
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_TxE);
	if (temp1 && temp2 && temp3) {
		if (IS_BIT_SET(i2cHandle->i2c->SR2, I2C_SR2_MSL)) {
			// Device is in master mode.
			if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_TX) {
				// I2C is in transmission state
				i2cMasterHandleTxeIt(i2cHandle);
			}
		} else {
			// Device is in slave mode.
			// Make sure we are in the transmission mode.
			if (IS_BIT_SET(i2cHandle->i2c->SR2, I2C_SR2_TRA)) {
				i2c_appEventCallback(i2cHandle, I2C_EVENT_DATA_REQ);
			}
		}
	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = IS_BIT_SET(i2cHandle->i2c->SR1, I2C_SR1_RxNE);
	if (temp1 && temp2 && temp3) {
		// Check if device is in master mode.
		if (IS_BIT_SET(i2cHandle->i2c->SR2, I2C_SR2_MSL)) {
			// Device is in master mode.
			// Check if i2c state is receiver state.
			if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_RX) {
				// I2c state is receiver state.
				// Handle receiver not empty interrupt.
				i2cMasterHandleRxneIt(i2cHandle);
			}
		} else {
			// Device is in slave mode.
			// Make sure we are in the receiver mode.
			if (IS_BIT_RESET(i2cHandle->i2c->SR2, I2C_SR2_TRA)) {
				i2c_appEventCallback(i2cHandle, I2C_EVENT_DATA_RCV);
			}
		}
	}
}


/*
 * I2C error interrupt handler
 */
void i2c_erIrqHandling(I2C_Handle_t *i2cHandle) {

	uint32_t temp1, temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (i2cHandle->i2c->CR2) & (1 << I2C_CR2_ITERREN);

	// Check for Bus error
	temp1 = (i2cHandle->i2c->SR1) & (1 << I2C_SR1_BERR);
	if (temp1 && temp2) {
		//This is Bus error
		//Implement the code to clear the buss error flag
		i2cHandle->i2c->SR1 &= ~(1 << I2C_SR1_BERR);
		//Implement the code to notify the application about the error
		i2c_appEventCallback(i2cHandle, I2C_ERROR_BERR);
	}

	// Check for arbitration lost error
	temp1 = (i2cHandle->i2c->SR1) & (1 << I2C_SR1_ARLO);
	if (temp1 && temp2) {
		//This is arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		i2cHandle->i2c->SR1 &= ~(1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		i2c_appEventCallback(i2cHandle, I2C_ERROR_ARLO);
	}

	// Check for ACK failure error
	temp1 = (i2cHandle->i2c->SR1) & (1 << I2C_SR1_AF);
	if (temp1 && temp2) {
		//This is ACK failure error
		//Implement the code to clear the ACK failure error flag
		i2cHandle->i2c->SR1 &= ~(1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		i2c_appEventCallback(i2cHandle, I2C_ERROR_AF);
	}

	// Check for Overrun/underrun error
	temp1 = (i2cHandle->i2c->SR1) & (1 << I2C_SR1_OVR);
	if (temp1 && temp2) {
		//This is Overrun/underrun
		//Implement the code to clear the Overrun/underrun error flag
		i2cHandle->i2c->SR1 &= ~(1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		i2c_appEventCallback(i2cHandle, I2C_ERROR_OVR);
	}

	// Check for Time out error
	temp1 = (i2cHandle->i2c->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (temp1 && temp2) {
		//This is Time out error
		//Implement the code to clear the Time out error flag
		i2cHandle->i2c->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		i2c_appEventCallback(i2cHandle, I2C_ERROR_TIMEOUT);
	}
}


/*
 * Application event callback
 */
void i2c_appEventCallback(I2C_Handle_t *i2cHandle, uint8_t appEvent) {

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
		apb1Pres = 1;
	} else {
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


static void i2cClearAddrFlag(I2C_Handle_t *i2cHandle) {
	uint32_t dummyRead;

	// Check if the mode is master or slave
	if (IS_BIT_SET(i2cHandle->i2c->SR2, I2C_SR2_MSL)) {
		// I2C is in master mode.
		// Check if i2c is in transmission or reception mode.
		if (i2cHandle->txRxState == I2C_STATE_BUSY_IN_RX) {
			// I2C is in reception mode.
			// Check if size is 1 byte.
			if (i2cHandle->rxSize == 1) {
				// Size is of 1 byte.
				// Disable ack.
				i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
				// Clear the ADDR bit by reading both SR1 and SR2.
				dummyRead = i2cHandle->i2c->SR1;
				dummyRead = i2cHandle->i2c->SR2;
				// This typecasting is to avoid unused variable compiler warning.
				(void) dummyRead;
			}
		} else {
			// I2C is in transmission mode.
			dummyRead = i2cHandle->i2c->SR1;
			dummyRead = i2cHandle->i2c->SR2;
			// This typecasting is to avoid unused variable compiler warning.
			(void) dummyRead;
		}
	} else {
		// I2C is in slave mode.
		dummyRead = i2cHandle->i2c->SR1;
		dummyRead = i2cHandle->i2c->SR2;
		// This typecasting is to avoid unused variable compiler warning.
		(void) dummyRead;
	}
}


static void i2cCloseSendData(I2C_Handle_t *i2cHandle) {
	//Implement the code to disable ITBUFEN Control Bit
	i2cHandle->i2c->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	i2cHandle->i2c->CR2 &= ~(1 << I2C_CR2_ITEVTEN);


	i2cHandle->txRxState = I2C_STATE_READY;
	i2cHandle->txBuffer = NULL;
	i2cHandle->txLen = 0;
}


static void i2cCloseReceiveData(I2C_Handle_t *i2cHandle) {
	// Code to disable ITBUFEN Control Bit
	i2cHandle->i2c->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Code to disable ITEVFEN Control Bit
	i2cHandle->i2c->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	i2cHandle->txRxState = I2C_STATE_READY;
	i2cHandle->rxBuffer = NULL;
	i2cHandle->rxLen = 0;
	i2cHandle->rxSize = 0;

	if(i2cHandle->i2cCfg.ackCtrl == I2C_ACK_CTRL_ENABLE)
	{
		i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_ENABLE);
	}
}


static void i2cMasterHandleRxneIt(I2C_Handle_t *i2cHandle) {
	// Check if size is 1 byte.
	if (i2cHandle->rxSize == 1) {
		// Size is 1 byte.
		// Read data register into receiver buffer.
		*i2cHandle->rxBuffer = i2cHandle->i2c->DR;
		// Decrease receive byte count.
		i2cHandle->rxLen--;
	}

	if (i2cHandle->rxSize > 1) {
		// Size is greater than 1 byte.
		// Check if length of receiving byte is 2 byte.
		if (i2cHandle->rxLen == 2) {
			// Receiving byte is of 2 byte.
			// Disable ack.
			i2c_manageAck(i2cHandle->i2c, I2C_ACK_CTRL_DISABLE);
		}
		// Read data register into receiver buffer.
		*i2cHandle->rxBuffer = i2cHandle->i2c->DR;
		// Increase receiver buffer address.
		i2cHandle->rxBuffer++;
		// Decrease receive byte count.
		i2cHandle->rxLen--;
	}

	if (i2cHandle->rxLen == 0) {
		// Length is 0 byte.
		// Check if repeated start is disabled.
		if (i2cHandle->repeatedStart == I2C_RS_DISABLE) {
			// Repeated start is disabled.
			// Generate stop condition.
			i2cGenerateStopCondition(i2cHandle->i2c);
		}
		// Close the receive data.
		i2cCloseReceiveData(i2cHandle);
		// Inform application about data reception complete.
		i2c_appEventCallback(i2cHandle, I2C_EVENT_RX_CMPLT);
	}
}


static void i2cMasterHandleTxeIt(I2C_Handle_t *i2cHandle) {
	if (i2cHandle->txLen > 0) {
		// Load data into DR
		i2cHandle->i2c->DR = *(i2cHandle->txBuffer);
		// Increment buffer address
		i2cHandle->txBuffer++;
		// Decrement txLen
		i2cHandle->txLen--;
	}
}

/**********************************************************************************************************************
 * 												END: Helper functions
 *********************************************************************************************************************/









