#include "stm32f411xx_spi_driver.h"

/**********************************************************************************************************************
 *						 						Function prototypes
 **********************************************************************************************************************/
static void spiTxeItHandle(SPI_Handle_t *spiHandle);
static void spiRxneItHandle(SPI_Handle_t *spiHandle);
static void spiOvrErrItHandle(SPI_Handle_t *spiHandle);

/*
 * Peripheral clock setup
 */
void spi_periClockControl(SPI_RegDef_t *spiReg, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		if (spiReg == SPI1) {
			SPI1_PCLK_EN();
		} else if (spiReg == SPI2) {
			SPI2_PCLK_EN();
		} else if (spiReg == SPI3) {
			SPI3_PCLK_EN();
		} else if (spiReg == SPI4) {
			SPI4_PCLK_EN();
		} else if (spiReg == SPI5) {
			SPI5_PCLK_EN();
		}
	} else {
		if (spiReg == SPI1) {
			SPI1_PCLK_DI();
		} else if (spiReg == SPI2) {
			SPI2_PCLK_DI();
		} else if (spiReg == SPI3) {
			SPI3_PCLK_DI();
		} else if (spiReg == SPI4) {
			SPI4_PCLK_DI();
		} else if (spiReg == SPI5) {
			SPI5_PCLK_DI();
		}
	}

}

/*
 * Init and De Init
 */
void spi_init(SPI_Handle_t *spiHandle) {

	uint32_t temp = 0;

	// enable spi peripheral clock
	spi_periClockControl(spiHandle->spi, ENABLE);

	// configure the device mode
	temp |= ((spiHandle->spiCfg.deviceMode) << SPI_CR1_MSTR);

	// configure the bus
	if (spiHandle->spiCfg.busCfg == SPI_BUS_CFG_FD) {

		// clear BIDIMODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (spiHandle->spiCfg.busCfg == SPI_BUS_CFG_HD) {

		// set BIDIMODE
		temp |= (1 << SPI_CR1_BIDIMODE);

	} else if (spiHandle->spiCfg.busCfg == SPI_BUS_CFG_SIMPLEX_RXONLY) {

		// clear BIDIMODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//set RXONLY
		temp |= (1 << SPI_CR1_RXONLY);

	}

	// configure the serial clock speed
	temp |= ((spiHandle->spiCfg.sClkSpeed) << SPI_CR1_BR);

	// configure data frame format
	temp |= ((spiHandle->spiCfg.dff) << SPI_CR1_DFF);

	// configure clock polarity
	temp |= ((spiHandle->spiCfg.cpol) << SPI_CR1_CPOL);

	// configure clock phase
	temp |= ((spiHandle->spiCfg.cpha) << SPI_CR1_CPHA);

	// configure software slave management
	temp |= ((spiHandle->spiCfg.ssm) << SPI_CR1_SSM);

	spiHandle->spi->CR1 = temp;

}

void spi_deInit(SPI_RegDef_t *spiReg) {

	if (spiReg == SPI1) {
		SPI1_REG_RESET();
	} else if (spiReg == SPI2) {
		SPI2_REG_RESET();
	} else if (spiReg == SPI3) {
		SPI3_REG_RESET();
	} else if (spiReg == SPI4) {
		SPI4_REG_RESET();
	} else if (spiReg == SPI5) {
		SPI5_REG_RESET();
	}

}

/*
 * Data send and receive
 */
void spi_sendData(SPI_RegDef_t *spiReg, uint8_t *txBuffer, uint32_t len) {

	while (len > 0) {
		while (IS_BIT_RESET(spiReg->SR, SPI_SR_TXE))
			;
		if (IS_BIT_SET(spiReg->CR1, SPI_CR1_DFF)) {
			// Data is 16 bit wide
			spiReg->DR = *((uint16_t*) txBuffer);
			(uint16_t*) txBuffer++;
			len -= 2;
		} else {
			// Data is 8 bit wide
			spiReg->DR = *txBuffer;
			txBuffer++;
			len--;
		}
	}

}

/*
 * Receive data
 */
void spi_receiveData(SPI_RegDef_t *spiReg, uint8_t *rxBuffer, uint32_t len) {

	while (len > 0) {

		// wait until data becomes available in hardware rx buffer
		while (IS_BIT_RESET(spiReg->SR, SPI_SR_RXNE))
			;
		if (IS_BIT_SET(spiReg->CR1, SPI_CR1_DFF)) {
			// data is 16 bit wide
			*((uint16_t*) rxBuffer) = spiReg->DR;
			(uint16_t*) rxBuffer++;
			len -= 2;
		} else {
			// data is 8 bit wide
			*rxBuffer = spiReg->DR;
			rxBuffer++;
			len--;
		}
	}

}

/*
 * Send data interrupt api
 */
uint8_t spi_sendDataIt(SPI_Handle_t *spiHandle, uint8_t *txBuffer, uint32_t len) {
	uint8_t state = spiHandle->txState;
	if (state != SPI_STATE_BUSY_IN_TX) {
		spiHandle->txBuffer = txBuffer;
		spiHandle->txLen = len;
		// change state to busy in transmission
		spiHandle->txState = SPI_STATE_BUSY_IN_TX;
		// set TXEIE bit, to enable the tx buffer empty interrupt. So, whenever TXE flag is set,
		// an interrupt is generated
		spiHandle->spi->CR2 = (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/*
 * Receive data interrupt api
 */
uint8_t spi_receiveDataIt(SPI_Handle_t *spiHandle, uint8_t *rxBuffer,
		uint32_t len) {
	uint8_t state = spiHandle->rxState;
	if (state != SPI_STATE_BUSY_IN_RX) {
		spiHandle->rxBuffer = rxBuffer;
		spiHandle->rxLen = len;
		// change state to busy in transmission
		spiHandle->rxState = SPI_STATE_BUSY_IN_RX;
		// set RXNEIE bit, to enable the rx buffer not empty interrupt. So, whenever RXNE flag is set,
		// an interrupt is generated
		spiHandle->spi->CR2 = (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/*
 * SPI peripheral enable/disable
 */
void spi_peripheralControl(SPI_RegDef_t *spiReg, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		// set SPE bit, i.e., enable SPI peripheral
		spiReg->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		// clear SPE bit i.e., disable SPI peripheral
		spiReg->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/*
 * SPI SSI config
 */
void spi_ssiConfig(SPI_RegDef_t *spiReg, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		// set SSI bit, i.e., internal slave select
		spiReg->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		// clear SSI bit i.e., internal slave select
		spiReg->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}

/*
 * SPI SSI config
 */
void spi_ssoeConfig(SPI_RegDef_t *spiReg, uint8_t enOrDi) {

	if (enOrDi == ENABLE) {
		// set SSI bit, i.e., internal slave select
		spiReg->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		// clear SSI bit i.e., internal slave select
		spiReg->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/*
 * Clear overrun error flag
 */
void spi_clearOvrFlag(SPI_RegDef_t *spiReg) {
	uint8_t temp;
	// clear the overrun error. This is done by first reading the data register (DR)
	// and then status register (SR)
	temp = spiReg->DR;
	temp = spiReg->SR;

	// it does nothing, its just used to avoid the unused variable warning
	(void) temp;
}

/*
 * Close spi transmission
 */
void spi_closeTransmission(SPI_Handle_t *spiHandle) {
	spiHandle->spi->CR2 &= ~(1 << SPI_CR2_TXEIE);
	spiHandle->txBuffer = NULL;
	spiHandle->txLen = 0;
	spiHandle->txState = SPI_STATE_READY;
}

/*
 * Close spi reception
 */
void spi_closeReception(SPI_Handle_t *spiHandle) {
	spiHandle->spi->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	spiHandle->rxBuffer = NULL;
	spiHandle->rxLen = 0;
	spiHandle->rxState = SPI_STATE_READY;
}

/*
 * Application event callback
 */
__weak void spi_appEventCallback(SPI_Handle_t *spiHandle, uint8_t appEvent) {

}

/*
 * IRQ configuration and ISR handling
 */
void spi_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi) {

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

void spi_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {

	uint8_t iprx = irqNumber / 4;
	uint8_t iprxSection = irqNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (irqPriority << shiftAmount);

}

void spi_irqHandling(SPI_Handle_t *spiHandle) {

	uint8_t temp1, temp2;

	// check for TXE
	temp1 = IS_BIT_SET(spiHandle->spiReg->SR, SPI_SR_TXE);
	temp2 = IS_BIT_SET(spiHandle->spiReg->CR2, SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// handle TXE
		spiTxeItHandle(spiHandle);
	}

	// check for RXNE
	temp1 = IS_BIT_SET(spiHandle->spiReg->SR, SPI_SR_RXNE);
	temp2 = IS_BIT_SET(spiHandle->spiReg->CR2, SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// handle RXNE
		spiRxneItHandle(spiHandle);
	}

	// check for OVR
	temp1 = IS_BIT_SET(spiHandle->spiReg->SR, SPI_SR_OVR);
	temp2 = IS_BIT_SET(spiHandle->spiReg->CR2, SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		// handle overrun error
		spiOvrErrItHandle(spiHandle);
	}

}

/**********************************************************************************************************************
 *						 					Helper Function Implementations
 **********************************************************************************************************************/

static void spiTxeItHandle(SPI_Handle_t *spiHandle) {

	if (IS_BIT_SET(spiHandle->spiReg->CR1, SPI_CR1_DFF)) {
		// Data is 16 bit wide
		spiHandle->spi->DR = *((uint16_t*) spiHandle->txBuffer);
		(uint16_t*) spiHandle->txBuffer++;
		spiHandle->txLen -= 2;
	} else {
		// Data is 8 bit wide
		spiHandle->spi->DR = *spiHandle->txBuffer;
		spiHandle->txBuffer++;
		spiHandle->txLen--;
	}

	if (spiHandle->txLen == 0) {
		//txLen is 0, so close the spi transmission and inform the application that tx is over
		// This prevents the interrupt from setting up of TXE flag
		spi_closeTransmission(spiHandle);
		spi_appEventCallback(spiHandle, SPI_EVENT_TX_CMPLT);
	}

}


static void spiRxneIntHandle(SPI_Handle_t *spiHandle) {

	if (IS_BIT_SET(spiHandle->spiReg->CR1, SPI_CR1_DFF)) {
		// data is 16 bit wide
		*((uint16_t*) spiHandle->rxBuffer) = spiHandle->spi->DR;
		(uint16_t*) spiHandle->rxBuffer++;
		spiHandle->rxLen -= 2;
	} else {
		// data is 8 bit wide
		*(spiHandle->rxBuffer) = spiHandle->spi->DR;
		spiHandle->rxBuffer++;
		spiHandle->rxLen--;
	}

	if (spiHandle->txLen == 0) {
		// reception is complete, turn off the RXNEIE interrupt
		spi_closeReception(spiHandle);
		spi_appEventCallback(spiHandle, SPI_EVENT_RX_CMPLT);
	}

}


static void spiOvrErrIntHandle(SPI_Handle_t *spiHandle) {
	uint8_t temp;
	// if the spi peripheral is not busy in transmission
	if (spiHandle->txState != SPI_STATE_BUSY_IN_TX) {
		spi_clearOvrFlag(spiHandle->spi);
	}
	// it does nothing, its just used to avoid the unused variable warning
	(void) temp;
	// inform the application
	spi_appEventCallback(spiHandle, SPI_EVENT_OVR_ERR);

}

