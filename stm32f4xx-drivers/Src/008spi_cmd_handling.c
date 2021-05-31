//#include <string.h>
//#include <stdio.h>

#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"


/*
 * SPI2_MOSI --> PB15
 * SPI2_MISO --> PB14
 * SPI2_SCLK --> PB13
 * SPI2_NSS  --> PB12
// */


//command codes
#define CMD_LED_CTRL      		0x50
#define CMD_SENSOR_READ      	0x51
#define CMD_LED_READ      		0x52
#define CMD_PRINT      			0x53
#define CMD_ID_READ      		0x54

#define LED_ON   				1
#define LED_OFF  				0

//arduino analog pins
#define ANALOG_PIN0 			0
#define ANALOG_PIN1 			1
#define ANALOG_PIN2 			2
#define ANALOG_PIN3			 	3
#define ANALOG_PIN4			 	4

//arduino led
#define LED_PIN  				9

#define SUCCESS					1
#define FAILURE					0


//extern void initialise_monitor_handles();


/**********************************************************************************************************************
 * 												Functions prototypes
 *********************************************************************************************************************/
void spi2GpioInit();
void spi2Init();
void buttonInit();

void delay();

uint8_t spiVerifyResponse(uint8_t ackByte);
uint8_t spiSendCmd(SPI_RegDef_t *spi, uint8_t cmdCode);
uint8_t spiSendReadCmd(SPI_RegDef_t *spi, uint8_t cmdCode, uint8_t args[], uint8_t nargs, uint8_t *rxBuffer);



int main() {

//	initialise_monitor_handles();

	uint8_t cmdCode;
	uint8_t args[2];

	uint8_t dummyRead;
	uint8_t dummyWrite = 0xFF;

	spi2GpioInit();
	spi2Init();

	buttonInit();


	/*
	 * SSOE = 1:
	 * 		SPE = 1, hardware makes NSS = 0 and selects the slave.
	 * 		SPE = 0, hardware makes NSS = 1 and doesn't selects the slave.
	 * SSOE = 0:
	 * 		This is a multi-master mode and we are not concerned about it right now.
	 */
	spi_ssoeConfig(SPI2, ENABLE);


	while (1) {

		// wait here until button goes down
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_2)));
		// for debouncing purpose
		delay();
		// enables the spi peripheral
		spi_peripheralControl(SPI2, ENABLE);

		// 1. Led control command
		cmdCode = CMD_LED_CTRL;
		args[0] = LED_PIN;
		args[1] = LED_ON;
		if (spiSendCmd(SPI2, cmdCode) == SUCCESS) {
			spi_sendData(SPI2, args, 2);
			spi_receiveData(SPI2, &dummyRead, 2);
		} else {
			// handle error code (right now its just a simple infinite while loop to help in debugging
			while (1);
		}

		// 2. Analog read command
		cmdCode = CMD_SENSOR_READ;
		args[0] = ANALOG_PIN0;
		uint8_t analogRead;
		if (spiSendCmd(SPI2, cmdCode) == SUCCESS) {

			spi_sendData(SPI2, args, 1);
			spi_receiveData(SPI2, &dummyRead, 1);
			delay();

			spi_sendData(SPI2, &dummyWrite, 1);
			spi_receiveData(SPI2, &analogRead, 1);
//			printf("COMMAND_SENSOR_READ %d\n", analogRead);

		} else {
			// handle error code (right now its just a simple infinite while loop to help in debugging
			while (1);
		}

		// 3. Led read command
		cmdCode = CMD_LED_READ;
		args[0] = LED_PIN;
		uint8_t ledStatus;
		if (spiSendCmd(SPI2, cmdCode) == SUCCESS) {

			spi_sendData(SPI2, args, 1);
			spi_receiveData(SPI2, &dummyRead, 1);
			delay();
			spi_sendData(SPI2, &dummyWrite, 1);
			spi_receiveData(SPI2, &ledStatus, 1);
//			printf("COMMAND_READ_LED %d\n",ledStatus);

		} else {
			// handle error code (right now its just a simple infinite while loop to help in debugging
			while (1);
		}

		// 4. Print command
		cmdCode = CMD_PRINT;
		uint8_t msg[] = "Hello from master";
		if (spiSendCmd(SPI2, cmdCode) == SUCCESS) {
			args[0] = strlen((char*) msg);
			spi_sendData(SPI2, args, 1);
			spi_receiveData(SPI2, &dummyRead, 1);
			delay();

			for (int i = 0; i < args[0]; ++i) {
				spi_sendData(SPI2, &msg[i], 1);
				spi_receiveData(SPI2, &dummyRead, 1);
			}
//			printf("COMMAND_PRINT Executed \n");
		} else {
			// handle error code (right now its just a simple infinite while loop to help in debugging
			while (1);
		}

		// 5. Device ID read command
		cmdCode = CMD_ID_READ;
		uint8_t id[10];
		if (spiSendCmd(SPI2, cmdCode) == SUCCESS) {
			for (int i = 0; i < 10; ++i) {
				spi_sendData(SPI2, &dummyWrite, 1);
				spi_receiveData(SPI2, &id[i], 1);
			}
		} else {
			// handle error code (right now its just a simple infinite while loop to help in debugging
			while (1);
		}

		// before closing the spi check if spi is busy in any communication
		while (IS_BIT_SET(SPI2->SR, SPI_SR_BSY));
		// close the spi communication
		spi_peripheralControl(SPI2, DISABLE);
	}


	return 0;

}

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


void delay() {
	for (int i = 500000 / 2; i > 0; --i);
}


void spi2Init() {
	SPI_Handle_t spiHandle;

	spiHandle.spi = SPI2;
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
	buttonHandle.pinCfg.pinNumber = GPIO_PIN_NO_2;
	buttonHandle.pinCfg.pinOPType = GPIO_PIN_OPTYPE_PP;
	buttonHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_PD;
	buttonHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_init(&buttonHandle);
}


uint8_t spiVerifyResponse(uint8_t ackByte) {
	if (ackByte == (uint8_t) 0xF5) {
		// ack byte received
		return 1;
	}
	//nack byte received
	return 0;
}


uint8_t spiSendCmd(SPI_RegDef_t* spi, uint8_t cmdCode) {

	uint8_t dummyRead;
	uint8_t dummyWrite = 0xFF;
	uint8_t ackByte;

	// wait here until button goes down
	while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_2)));
	// for debouncing purpose
	delay();

	// send command
	spi_sendData(spi, &cmdCode, 1);
	// read the value to clear the RXNE bit
	spi_receiveData(spi, &dummyRead, 1);

	// send dummy byte to receive the ack/nack byte
	spi_sendData(spi, &dummyWrite, 1);
	spi_receiveData(spi, &ackByte, 1);

	// check if ack byte is received
	if (spiVerifyResponse(ackByte)) {
		// ack byte is received, return success status
		return SUCCESS;
	}
	// nack byte is received, return with failure status
	return FAILURE;
}


