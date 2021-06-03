#include <string.h>

#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_gpio_driver.h"


#define MY_ADDR								0x61
#define SLAVE_ADDR							0x68

I2C_Handle_t i2c1Handle;

uint8_t msg[] = "Hello from master\n";


void delay() {
	for (uint32_t i = 250000; i > 0; --i)
		;
}




/**
 * PB6 -> SCL
 * PB9 -> SDA
 */
void i2c1GpioInit() {

	GPIO_Handle_t i2cGpioHandle;

	i2cGpioHandle.gpio = GPIOB;
	i2cGpioHandle.pinCfg.pinAF = GPIO_PIN_AF_4;
	i2cGpioHandle.pinCfg.pinMode = GPIO_PIN_MODE_AF;
	i2cGpioHandle.pinCfg.pinOPType = GPIO_PIN_OPTYPE_OD;
	i2cGpioHandle.pinCfg.pinPuPd = GPIO_PIN_PUPD_PU;
	i2cGpioHandle.pinCfg.pinSpeed = GPIO_PIN_SPEED_FAST;

	// SCL pin
	i2cGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_6;
	gpio_init(&i2cGpioHandle);
	// SDA pin
	i2cGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_9;
	gpio_init(&i2cGpioHandle);

}


void i2c1Init() {
	i2c1Handle.i2c = I2C1;

	i2c1Handle.i2cCfg.ackCtrl = I2C_ACK_CTRL_ENABLE;
	i2c1Handle.i2cCfg.deviceAddr = MY_ADDR;
	i2c1Handle.i2cCfg.fmDutyCycle = I2C_FM_DUTY_CYCLE_2;
	i2c1Handle.i2cCfg.sclSpeed = I2C_SCL_SPEED_SM;

	i2c_init(&i2c1Handle);
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
	// I2C pin init
	i2c1GpioInit();

	// button init
	buttonInit();

	// I2C peripheral config
	i2c1Init();

	// Enable i2c peripheral
	i2c_peripheralControl(i2c1Handle.i2c, ENABLE);

	while (1) {
		// wait here until button goes down
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_4)));
		delay();

		i2c_masterSendData(&i2c1Handle, msg, strlen((char *) msg), SLAVE_ADDR);
	}

	return 0;
}
