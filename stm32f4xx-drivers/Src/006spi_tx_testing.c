#include <string.h>

#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"


/*
 * SPI2_MOSI --> PB15
 * SPI2_MISO --> PB14
 * SPI2_SCLK --> PB13
 * SPI2_NSS  --> PB12
 */


void delay() {
	for (uint32_t i = 250000; i > 0; --i)
		;
}


void spi2GpioInit() {

	GPIO_Handle_t spiGpioHandle;

	spiGpioHandle.gpio = GPIOB;

//	memset(&spiGpioHandle.pinCfg, 0, sizeof(spiGpioHandle.pinCfg));

	spiGpioHandle.pinCfg.pinAF = GPIO_PIN_AF_5;
	spiGpioHandle.pinCfg.pinMode = GPIO_PIN_MODE_AF;
	spiGpioHandle.pinCfg.pinOPType = GPIO_PIN_OPTYPE_PP;
	spiGpioHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_NO;
	spiGpioHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	// set MOSI pin
	spiGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_15;
	gpio_init(&spiGpioHandle);

	// set MISO pin
	spiGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_14;
	gpio_init(&spiGpioHandle);

	// set SCLK pin
	spiGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_13;
	gpio_init(&spiGpioHandle);

	// set NSS pin
	spiGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_12;
	gpio_init(&spiGpioHandle);

}


void spi2Init() {
	SPI_Handle_t spiHandle;

	spiHandle.spi = SPI2;

//	memset(&spiHandle.spiCfg, 0, sizeof(spiHandle.spiCfg));

	spiHandle.spiCfg.busCfg = SPI_BUS_CFG_FD;
	spiHandle.spiCfg.cpha = SPI_CPHA_LOW;
	spiHandle.spiCfg.cpol = SPI_CPOL_LOW;
	spiHandle.spiCfg.deviceMode = SPI_DEVICE_MODE_MASTER;
	spiHandle.spiCfg.dff = SPI_DFF_8_BIT;
	spiHandle.spiCfg.sClkSpeed = SPI_SERIAL_CLK_SPEED_DIV_8;
	spiHandle.spiCfg.ssm = SPI_SSM_DI;

	spi_init(&spiHandle);
}


void buttonInit() {
	GPIO_Handle_t buttonHandle;

	buttonHandle.gpio = GPIOB;

	buttonHandle.pinCfg.pinMode = GPIO_PIN_MODE_IN;
	buttonHandle.pinCfg.pinNumber = GPIO_PIN_NO_4;
	buttonHandle.pinCfg.pinOPType = GPIO_PIN_OPTYPE_PP;
	buttonHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_PD;
	buttonHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_init(&buttonHandle);
}


int main() {

	char data[] = "Hello world";

	spi2GpioInit();
	spi2Init();

	buttonInit();

	while (1) {

		// wait here until button goes down
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_4)));
		delay();
		/*
		 * SSOE = 1:
		 * 		SPE = 1, hardware makes NSS = 0 and selects the slave.
		 * 		SPE = 0, hardware makes NSS = 1 and doesn't selects the slave.
		 * SSOE = 0:
		 * 		This is a multi-master mode and we are not concerned about it right now.
		 */
		spi_ssoeConfig(SPI2, ENABLE);
		spi_peripheralControl(SPI2, ENABLE);

		uint8_t dataLen = strlen(data);
		// first send length of data
		spi_sendData(SPI2, &dataLen, 1);
		// send the actual data;
		spi_sendData(SPI2, (uint8_t*) data, dataLen);
		// before closing the spi check if spi is busy in any communication
		WAIT_UNTIL_RESET(SPI2->SR, SPI_SR_BSY);
		// close the spi communication
		spi_peripheralControl(SPI2, DISABLE);
	}


	return 0;

}
