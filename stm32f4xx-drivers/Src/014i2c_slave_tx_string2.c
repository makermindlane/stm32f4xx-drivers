#include <string.h>

#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_gpio_driver.h"


#define SLAVE_ADDR							0x68
#define MY_ADDR								SLAVE_ADDR

#define CMD_GET_DATA_LEN					0x51
#define CMD_GET_DATA						0x52

I2C_Handle_t i2c1Handle;

uint8_t txBuff[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123";

uint8_t rxBuff[32];
uint32_t data_len = 0;

uint8_t cmdCode;


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

	data_len = strlen((char*) txBuff);
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

	static uint32_t cnt = 0;
	static uint32_t w_ptr = 0;

	if (appEvent == I2C_ERROR_AF) {
		//This will happen during slave transmitting data to master .
		// slave should understand master needs no more data
		//slave concludes end of Tx

		//if the current active code is 0x52 then dont invalidate
		if (!(cmdCode == 0x52))
			cmdCode = 0XFF;

		//reset the cnt variable because its end of transmission
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if (w_ptr >= (data_len)) {
			w_ptr = 0;
			cmdCode = 0xff;
		}

	} else if (appEvent == I2C_EVENT_STOP) {
		//This will happen during end slave reception
		//slave concludes end of Rx

		cnt = 0;

	} else if (appEvent == I2C_EVENT_DATA_REQ) {
		//Master is requesting for the data . send data
		if (cmdCode == 0x51) {
			//Here we are sending 4 bytes of length information
			i2c_slaveSendData(I2C1, ((data_len >> ((cnt % 4) * 8)) & 0xFF));
			cnt++;
		} else if (cmdCode == 0x52) {
			//sending Tx_buf contents indexed by w_ptr variable
			i2c_slaveSendData(I2C1, txBuff[w_ptr++]);
		}
	} else if (appEvent == I2C_EVENT_DATA_RCV) {
		//Master has sent command code, read it
		cmdCode = i2c_slaveReceiveData(I2C1);

	}
}















