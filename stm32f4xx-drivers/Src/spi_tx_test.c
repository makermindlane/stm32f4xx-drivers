#include <string.h>

#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"


/*
 * SPI2_MOSI --> PB15
 * SPI2_MISO --> PB14
 * SPI2_SCLK --> PB13
 * SPI2_NSS  --> PB12
 */

void spi2GpioInit() {

	GPIO_Handle_t spiGpioHandle;

	spiGpioHandle.gpioPort = GPIOB;

//	memset(&spiGpioHandle.gpioPinCfg, 0, sizeof(spiGpioHandle.gpioPinCfg));

	spiGpioHandle.gpioPinCfg.gpioPinAF = GPIO_PIN_AF_5;
	spiGpioHandle.gpioPinCfg.gpioPinMode = GPIO_PIN_MODE_AF;
	spiGpioHandle.gpioPinCfg.gpioPinOPType = GPIO_PIN_OPTYPE_PP;
	spiGpioHandle.gpioPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
	spiGpioHandle.gpioPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	// set MOSI pin
	spiGpioHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_15;
	gpio_init(&spiGpioHandle);

	// set MISO pin
//	spiGpioHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_14;
//	gpio_init(&spiGpioHandle);

	// set SCLK pin
	spiGpioHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_13;
	gpio_init(&spiGpioHandle);

	// set NSS pin
	spiGpioHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_12;
	gpio_init(&spiGpioHandle);

}


void spi2Init() {
	SPI_Handle_t spiHandle;

	spiHandle.spiReg = SPI2;

//	memset(&spiHandle.spiPinCfg, 0, sizeof(spiHandle.spiPinCfg));

	spiHandle.spiPinCfg.busCfg = SPI_BUS_CFG_FD;
	spiHandle.spiPinCfg.cpha = SPI_CPHA_LOW;
	spiHandle.spiPinCfg.cpol = SPI_CPOL_LOW;
	spiHandle.spiPinCfg.deviceMode = SPI_DEVICE_MODE_MASTER;
	spiHandle.spiPinCfg.dff = SPI_DFF_8_BIT;
	spiHandle.spiPinCfg.sClkSpeed = SPI_SERIAL_CLK_SPEED_DIV_8;
	spiHandle.spiPinCfg.ssm = SPI_SSM_DI;

	spi_init(&spiHandle);
}


int main() {

	char data[] = "Hello world";

	spi2GpioInit();
	spi2Init();

	spi_peripheralControl(SPI2, ENABLE);
	spi_sendData(SPI2, (uint8_t*) data, strlen(data));
	spi_peripheralControl(SPI2, DISABLE);

	while (1);

	return 0;

}
