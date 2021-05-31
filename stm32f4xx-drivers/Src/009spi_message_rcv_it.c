/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */
#include <stdio.h>
#include <string.h>

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"

SPI_Handle_t spiHandle;

#define MAX_LEN 500

char rcvBuff[MAX_LEN];

volatile char readByte;

volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void) {
	for (uint32_t i = 500000 / 2; i > 0; i++)
		;
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void spi2GpioInit() {

	GPIO_Handle_t spiGpioHandle;

	spiGpioHandle.gpio = GPIOB;
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


void spi2Init(void) {
	spiHandle.spi = SPI2;
	spiHandle.spiCfg.busCfg = SPI_BUS_CFG_FD;
	spiHandle.spiCfg.cpha = SPI_CPHA_LOW;
	spiHandle.spiCfg.cpol = SPI_CPOL_LOW;
	spiHandle.spiCfg.deviceMode = SPI_DEVICE_MODE_MASTER;
	spiHandle.spiCfg.dff = SPI_DFF_8_BIT;
	spiHandle.spiCfg.sClkSpeed = SPI_SERIAL_CLK_SPEED_DIV_32;
	spiHandle.spiCfg.ssm = SPI_SSM_DI;

	spi_init(&spiHandle);
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void slaveGpioInterruptPinInit(void) {
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.gpio = GPIOB;
	spiIntPin.pinCfg.pinNumber = GPIO_PIN_NO_2;
	spiIntPin.pinCfg.pinMode = GPIO_PIN_MODE_IT_FT;
	spiIntPin.pinCfg.pinSpeed = GPIO_PIN_SPEED_LOW;
	spiIntPin.pinCfg.pinPuPd = GPIO_PIN_PUPD_PU;

	gpio_init(&spiIntPin);

	gpio_irqPriorityConfig(IRQ_NO_EXTI2, NVIC_PRIORITY_15);
	gpio_irqInterruptConfig(IRQ_NO_EXTI2, ENABLE);

}


void toggleLedInit() {
	GPIO_Handle_t gpioHandle;

	memset(&gpioHandle, 0, sizeof(gpioHandle));

	gpioHandle.gpio = GPIOB;
	gpioHandle.pinCfg.pinMode = GPIO_PIN_MODE_OUT;
	gpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_4;
	gpioHandle.pinCfg.pinOPType = GPIO_PIN_OPTYPE_OD;
	gpioHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_NO;
	gpioHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_init(&gpioHandle);
}


int main(void) {

	uint8_t dummy = 0xff;

	slaveGpioInterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	spi2GpioInit();

	//This function is used to initialize the SPI2 peripheral parameters
	spi2Init();

	toggleLedInit();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	spi_ssoeConfig(SPI2, ENABLE);

	spi_irqInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while (1) {

		rcvStop = 0;

		while (!dataAvailable)
			; //wait till data available interrupt from transmitter device(slave)

		gpio_irqInterruptConfig(IRQ_NO_EXTI2, DISABLE);

		//enable the SPI2 peripheral
		spi_peripheralControl(SPI2, ENABLE);

		while (!rcvStop) {
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while (spi_sendDataIt(&spiHandle, &dummy, 1) == SPI_STATE_BUSY_IN_TX)
				;
			while (spi_receiveDataIt(&spiHandle, &readByte, 1) == SPI_STATE_BUSY_IN_RX)
				;
		}

		// confirm SPI is not busy
		WAIT_UNTIL_RESET(SPI2->SR, SPI_SR_BSY);

		//Disable the SPI2 peripheral
		spi_peripheralControl(SPI2, DISABLE);

//		printf("Rcvd data = %s\n", RcvBuff);

		dataAvailable = 0;

		gpio_irqInterruptConfig(IRQ_NO_EXTI2, ENABLE);

	}

	return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void) {

	spi_irqHandling(&spiHandle);
}

void spi_appEventCallback(SPI_Handle_t *spiHandle, uint8_t appEvent) {
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if (appEvent == SPI_EVENT_RX_CMPLT) {
		rcvBuff[i++] = readByte;
		if (readByte == '\0' || (i == MAX_LEN)) {
			rcvStop = 1;
			rcvBuff[i - 1] = '\0';
			i = 0;
		}
		gpio_toggleOutputPin(GPIOB, GPIO_PIN_NO_4);
	}

}

/* Slave data available interrupt handler */
void EXTI2_IRQHandler(void) {
	gpio_irqHandling(GPIO_PIN_NO_2);
	dataAvailable = 1;
}
