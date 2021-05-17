#include "stm32f411xx_spi_driver.h"


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
	}

}


/*
 * Init and De Init
 */
void spi_init(SPI_Handle_t *spiHandle) {

	uint32_t temp = 0;

	// enable spi peripheral clock
	spi_periClockControl(spiHandle->spiReg, ENABLE);

	// configure the device mode
	temp |= ((spiHandle->spiPinCfg.deviceMode) << SPI_CR1_MSTR);

	// configure the bus
	if (spiHandle->spiPinCfg.busCfg == SPI_BUS_CFG_FD) {

		// clear BIDIMODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (spiHandle->spiPinCfg.busCfg == SPI_BUS_CFG_HD) {

		// set BIDIMODE
		temp |= (1 << SPI_CR1_BIDIMODE);

	} else if (spiHandle->spiPinCfg.busCfg == SPI_BUS_CFG_SIMPLEX_RXONLY) {

		// clear BIDIMODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//set RXONLY
		temp |= (1 << SPI_CR1_RXONLY);

	}

	// configure the serial clock speed
	temp |= ((spiHandle->spiPinCfg.sClkSpeed) << SPI_CR1_BR);

	// configure data frame format
	temp |= ((spiHandle->spiPinCfg.dff) << SPI_CR1_DFF);

	// configure clock polarity
	temp |= ((spiHandle->spiPinCfg.cpol) << SPI_CR1_CPOL);

	// configure clock phase
	temp |= ((spiHandle->spiPinCfg.cpha) << SPI_CR1_CPHA);

	// configure software slave management
	temp |= ((spiHandle->spiPinCfg.ssm) << SPI_CR1_SSM);

	spiHandle->spiReg->CR1 = temp;

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

		while (!(CHECK_BIT_FOR_SET(spiReg->SR, SPI_SR_TXE)));

		if (spiReg->CR1 & (1 << SPI_CR1_DFF)) {
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
		while (!(CHECK_BIT_FOR_SET(spiReg->SR, SPI_SR_RXNE)));
		if (spiReg->CR1 & (1 << SPI_CR1_DFF)) {
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
 * IRQ configuration and ISR handling
 */
void spi_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi) {

}


void spi_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {

}


void spi_irqHandling(SPI_Handle_t *spiHandle) {

}


