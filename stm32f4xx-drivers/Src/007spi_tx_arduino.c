#include <string.h>
#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"

/*
 * PB15 --> SPI2_MOSI
 * PB14 --> SPI2_MISO
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void spi2GpioInits(void) {
	GPIO_Handle_t spiPins;

	spiPins.gpio = GPIOB;
	spiPins.pinCfg.pinMode = GPIO_PIN_MODE_AF;
	spiPins.pinCfg.pinAF = GPIO_PIN_AF_5;
	spiPins.pinCfg.pinOPType = GPIO_PIN_OPTYPE_PP;
	spiPins.pinCfg.pinPuPd = GPIO_PIN_PUPD_NO;
	spiPins.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	//SCLK
	spiPins.pinCfg.pinNumber = GPIO_PIN_NO_13;
	gpio_init(&spiPins);

	//MOSI
	spiPins.pinCfg.pinNumber = GPIO_PIN_NO_15;
	gpio_init(&spiPins);

	//MISO
	//spiPins.pinCfg.pinNumber = GPIO_PIN_NO_14;
	//gpio_init(&spiPins);

	//NSS
	spiPins.pinCfg.pinNumber = GPIO_PIN_NO_12;
	gpio_init(&spiPins);
}

void spi2Init(void) {

	SPI_Handle_t spiHandle;

	spiHandle.spi = SPI2;
	spiHandle.spiCfg.busCfg = SPI_BUS_CFG_FD;
	spiHandle.spiCfg.deviceMode = SPI_DEVICE_MODE_MASTER;
	spiHandle.spiCfg.sClkSpeed = SPI_SERIAL_CLK_SPEED_DIV_8; //generates sclk of 2MHz
	spiHandle.spiCfg.dff = SPI_DFF_8_BIT;
	spiHandle.spiCfg.cpol = SPI_CPOL_LOW;
	spiHandle.spiCfg.cpha = SPI_CPHA_LOW;
	spiHandle.spiCfg.ssm = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	spi_init(&spiHandle);
}

void delay() {
	for (int i = 0; i < 500000 / 2; ++i)
		;
}

void gpioButtonInit() {
	GPIO_Handle_t gpioButtonHandle;

	gpioButtonHandle.gpio = GPIOB;
	gpioButtonHandle.pinCfg.pinMode = GPIO_PIN_MODE_IN;
	gpioButtonHandle.pinCfg.pinNumber = GPIO_PIN_NO_2;
	gpioButtonHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_PD;
	gpioButtonHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_init(&gpioButtonHandle);
}

int main(void) {

	char user_data[] = "Hi this is dummy text to test that more a large text can be sent successfully via spi.";


	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	spi2GpioInits();
	gpioButtonInit();

	//This function is used to initialize the SPI2 peripheral parameters
	spi2Init();

	spi_ssoeConfig(SPI2, ENABLE);

	while (1) {
		// Wait for button press to initiate the communication.
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_2)))
			;
		// Debouncing
		delay();
		//enable the SPI2 peripheral
		spi_peripheralControl(SPI2, ENABLE);
		// Send data length
		uint8_t dataLen = strlen(user_data);
		spi_sendData(SPI2, &dataLen, 1);
		// Send data
		spi_sendData(SPI2, (uint8_t*) user_data, dataLen);
		// Lets confirm SPI is not busy
		while (IS_BIT_SET(SPI2->SR, SPI_SR_BSY))
			;
		//Disable the SPI2 peripheral
		spi_peripheralControl(SPI2, DISABLE);
	}

	return 0;
}
