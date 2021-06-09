#include <string.h>

#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_gpio_driver.h"


#define SLAVE_ADDR							0x69
#define MY_ADDR								SLAVE_ADDR

#define CMD_GET_DATA_LEN					0x51
#define CMD_GET_DATA						0x52

I2C_Handle_t i2c1Handle;

uint8_t txBuff[32] = "STM32 Slave mode testing..";

// Flag variable
uint8_t rxComplete = RESET;

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
	i2cGpioHandle.pinCfg.pinNumber = GPIO_PIN_NO_7;
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

	// Configure i2c interrupts
	i2c_irqInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	i2c_irqInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	i2c_slaveCallbackEvents(I2C1, ENABLE);

	// Enable i2c peripheral
	i2c_peripheralControl(I2C1, ENABLE);

	// Set the ACK bit here also. Why? Because without setting the PE bit (which is done in above line)
	// ACK bit cannot be set, it will stay reset.
	i2c_manageAck(I2C1, ENABLE);

	while (1)
		;

	return 0;
}


void I2C1_EV_IRQHandler() {
	i2c_evIrqHandling(&i2c1Handle);
}


void I2C1_ER_IRQHandler() {
	i2c_erIrqHandling(&i2c1Handle);
}


void i2c_appEventCallback(I2C_Handle_t *i2cHandle, uint8_t appEvent) {

	static uint8_t cmdCode = 0;
	static uint8_t cnt = 0;

	if (appEvent == I2C_EVENT_DATA_REQ) {
		//Master wants some data. slave has to send it
		if (cmdCode == 0x51) {
			//send the length information to the master
			i2c_slaveSendData(i2cHandle->i2c, strlen((char*) txBuff));
		} else if (cmdCode == 0x52) {
			//Send the contents of Tx_buf
			i2c_slaveSendData(i2cHandle->i2c, txBuff[cnt++]);

		}
	} else if (appEvent == I2C_EVENT_DATA_RCV) {
		//Data is waiting for the slave to read . slave has to read it
		cmdCode = i2c_slaveReceiveData(i2cHandle->i2c);

	} else if (appEvent == I2C_ERROR_AF) {
		//This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		cmdCode = 0xff;
		cnt = 0;
	} else if (appEvent == I2C_EVENT_STOP) {
		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
	}
}















