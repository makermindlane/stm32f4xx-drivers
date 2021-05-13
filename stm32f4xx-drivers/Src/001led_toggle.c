#include "stm32f411xx.h"


void initButton() {

	GPIO_PinConfig_t gpioButtonPinCfg;
	gpioButtonPinCfg.gpioPinMode = GPIO_PIN_MODE_IN;
	gpioButtonPinCfg.gpioPinNumber = GPIO_PIN_NO_12;
	gpioButtonPinCfg.gpioPinOPType= GPIO_PIN_OPTYPE_PP;
	gpioButtonPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
	gpioButtonPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	GPIO_Handle_t gpioButtonHandle;
	gpioButtonHandle.gpioPinCfg = &gpioButtonPinCfg;
	gpioButtonHandle.gpioPort = GPIOB;

	gpio_periClockControl(GPIOB, ENABLE);
	gpio_init(&gpioButtonHandle);

}


void initLed() {

	GPIO_PinConfig_t gpioLedPinCfg;
	gpioLedPinCfg.gpioPinMode = GPIO_PIN_MODE_OUT;
	gpioLedPinCfg.gpioPinNumber = GPIO_PIN_NO_13;
	gpioLedPinCfg.gpioPinOPType = GPIO_PIN_OPTYPE_PP;
	gpioLedPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
	gpioLedPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;

	GPIO_Handle_t gpioLedHandle;
	gpioLedHandle.gpioPort = GPIOB;
	gpioLedHandle.gpioPinCfg = &gpioLedPinCfg;

	gpio_periClockControl(GPIOB, ENABLE);
	gpio_init(&gpioLedHandle);

}


void delay() {
	for (int i = 500000 / 2; i > 0; --i);
}


int main() {

//	GPIO_PinConfig_t gpioPinCfg;
//	gpioPinCfg.gpioPinNumber = GPIO_PIN_NO_12;
//	gpioPinCfg.gpioPinMode = GPIO_PIN_MODE_OUT;
//	gpioPinCfg.gpioPinSpeed = GPIO_PIN_SPEED_FAST;
//	gpioPinCfg.gpioPinOPType = GPIO_PIN_OPTYPE_OD;
//	gpioPinCfg.gpioPinPuPd = GPIO_PIN_PUPD_NO;
//
//	GPIO_Handle_t gpioLed;
//	gpioLed.gpioPort = GPIOB;
//	gpioLed.gpioPinCfg = &gpioPinCfg;
//
//	gpio_periClockControl(GPIOB, ENABLE);
//	gpio_init(&gpioLed);

	initButton();
	initLed();



	while (1) {

//		gpio_toggleOutputPin(GPIOB, GPIO_PIN_NO_12);
//		delay();

		if (gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_12)) {

			delay();
			gpio_toggleOutputPin(GPIOB, GPIO_PIN_NO_13);

		}

	}

	return 0;
}


void EXTI0_IRQHandler() {

	gpio_irqHandling(0);

}

