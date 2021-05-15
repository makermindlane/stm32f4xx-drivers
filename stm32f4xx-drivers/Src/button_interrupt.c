#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include <string.h>

void delay() {
	for (int i = 500000 / 2; i > 0; --i);
}


void initButton() {

//	GPIO_PinConfig_t gpioButtonPinCfg;
//	memset(&gpioButtonPinCfg, 0, sizeof(gpioButtonPinCfg));
//	gpioButtonPinCfg.gpioPinMode = GPIO_PIN_MODE_IT_FT;
//	gpioButtonPinCfg.gpioPinMode = GPIO_PIN_MODE_IN;
//	gpioButtonPinCfg.gpioPinNumber = GPIO_PIN_NO_12;
//	gpioButtonPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_PD;
//	gpioButtonPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	// TODO check whether code works without memset(). If not find a way to use it properly.

	GPIO_Handle_t gpioButtonHandle;

	gpioButtonHandle.gpioPort = GPIOB;
	gpioButtonHandle.gpioPinCfg.gpioPinMode = GPIO_PIN_MODE_IT_FT;
	gpioButtonHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_12;
	gpioButtonHandle.gpioPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_PD;
	gpioButtonHandle.gpioPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_periClockControl(GPIOB, ENABLE);
	gpio_init(&gpioButtonHandle);

}


void initLed() {

//	GPIO_PinConfig_t gpioLedPinCfg;
//	memset(&gpioLedPinCfg, 0, sizeof(gpioLedPinCfg));
//	gpioLedPinCfg.gpioPinMode = GPIO_PIN_MODE_OUT;
//	gpioLedPinCfg.gpioPinNumber = GPIO_PIN_NO_13;
//	gpioLedPinCfg.gpioPinOPType = GPIO_PIN_OPTYPE_PP;
//	gpioLedPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
//	gpioLedPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	GPIO_Handle_t gpioLedHandle;

	gpioLedHandle.gpioPort = GPIOB;
	gpioLedHandle.gpioPinCfg.gpioPinMode = GPIO_PIN_MODE_OUT;
	gpioLedHandle.gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_13;
	gpioLedHandle.gpioPinCfg.gpioPinOPType = GPIO_PIN_OPTYPE_PP;
	gpioLedHandle.gpioPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
	gpioLedHandle.gpioPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	gpio_periClockControl(GPIOB, ENABLE);
	gpio_init(&gpioLedHandle);

}


int main() {

	initButton();
	initLed();

	// configure irq
	gpio_irqInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	gpio_irqPriorityConfig(IRQ_NO_EXTI15_10, NVIC_PRIORITY_15);

	while (1) {

//		if (gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_12)) {
//			delay();
//			gpio_toggleOutputPin(GPIOB, GPIO_PIN_NO_13);
//		}

	}

	return 0;

}


void EXTI15_10_IRQHandler() {

	delay();
	gpio_irqHandling(GPIO_PIN_NO_12);
	gpio_toggleOutputPin(GPIOB, GPIO_PIN_NO_13);

}

