/*
 * uart_tx.c
 *
 *  Created on: Jan 22, 2019
 *      Author: admin
 */

#include <stdio.h>
#include <string.h>

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_usart_driver.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void eventCallback(USART_Handle_t *usartHandle, uint8_t appEvent);

void USART2_Init(void) {
	usart2_handle.usart = USART2;
	usart2_handle.usartCfg.baud = USART_STD_BAUD_115200;
	usart2_handle.usartCfg.hWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.usartCfg.mode = USART_MODE_ONLY_TX;
	usart2_handle.usartCfg.noOfStopBits = USART_STOPBITS_1;
	usart2_handle.usartCfg.wordLength = USART_WORDLEN_8BITS;
	usart2_handle.usartCfg.parityControl = USART_PARITY_DISABLE;

	usart2_handle.appEventCallback = eventCallback;

	usart_init(&usart2_handle);
}

void USART2_GPIOInit(void) {
	GPIO_Handle_t usart_gpios;

	usart_gpios.gpio = GPIOA;
	usart_gpios.pinCfg.pinMode = GPIO_PIN_MODE_AF;
	usart_gpios.pinCfg.pinOPType = GPIO_PIN_OPTYPE_PP;
	usart_gpios.pinCfg.pinPuPd = GPIO_PIN_PUPD_PU;
	usart_gpios.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;
	usart_gpios.pinCfg.pinAF = GPIO_PIN_AF_7;

	//USART2 TX
	usart_gpios.pinCfg.pinNumber = GPIO_PIN_NO_2;
	gpio_init(&usart_gpios);

	//USART2 RX
	usart_gpios.pinCfg.pinNumber = GPIO_PIN_NO_3;
	gpio_init(&usart_gpios);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.gpio = GPIOB;
	GPIOBtn.pinCfg.pinNumber = GPIO_PIN_NO_4;
	GPIOBtn.pinCfg.pinMode = GPIO_PIN_MODE_IN;
	GPIOBtn.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;
	GPIOBtn.pinCfg.pinPuPd = GPIO_PIN_PUPD_PD;

	gpio_init(&GPIOBtn);

	//this is led gpio configuration
	//	GpioLed.pGPIOx = GPIOD;
	//	GpioLed.pinCfg.pinNumber = GPIO_PIN_NO_12;
	//	GpioLed.pinCfg.pinMode = GPIO_MODE_OUT;
	//	GpioLed.pinCfg.pinSpeed = GPIO_SPEED_FAST;
	//	GpioLed.pinCfg.pinOPType = GPIO_OP_TYPE_OD;
	//	GpioLed.pinCfg.pinPuPd = GPIO_NO_PUPD;
	//
	//	GPIO_PeriClockControl(GPIOD, ENABLE);
	//
	//	GPIO_Init(&GpioLed);

}

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

int main(void) {

	GPIO_ButtonInit();

	USART2_GPIOInit();

	USART2_Init();

	usart_peripheralControl(USART2, ENABLE);

	while (1) {
		//wait till button is pressed
		while (!gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_4))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		usart_sendData(&usart2_handle, (uint8_t*) msg, strlen(msg));

	}

	return 0;
}


void eventCallback(USART_Handle_t *usartHandle, uint8_t appEvent) {

}
