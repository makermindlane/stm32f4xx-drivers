#include <string.h>

#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_gpio_driver.h"


#define MY_ADDR								0x61
#define SLAVE_ADDR							0x68

#define CMD_GET_DATA_LEN					0x51
#define CMD_GET_DATA						0x52

I2C_Handle_t i2c1Handle;


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


//static void masterGetDataLen(uint8_t *dataLen) {
//	uint8_t cmd = CMD_GET_DATA_LEN;
//	i2c_masterSendData(&i2c1Handle, &cmd, 1, SLAVE_ADDR);
//	i2c_masterReceiveData(&i2c1Handle, dataLen, 1, SLAVE_ADDR);
//}
//
//
//static void masterGetData(uint8_t *dataBuff, uint8_t dataLen) {
//	uint8_t cmd = CMD_GET_DATA;
//	i2c_masterSendData(&i2c1Handle, &cmd, 1, SLAVE_ADDR);
//	i2c_masterReceiveData(&i2c1Handle, dataBuff, dataLen, SLAVE_ADDR);
//}


int main() {

	uint8_t dataLen;
	uint8_t dataIn[255];

	uint8_t cmd;

	// I2C pin init
	i2c1GpioInit();

	// button init
	buttonInit();

	// I2C peripheral config
	i2c1Init();

	// Enable i2c peripheral
	i2c_peripheralControl(i2c1Handle.i2c, ENABLE);

	// Set the ACK bit here also. Why? Because without setting the PE bit (which is done in above line)
	// ACK bit cannot be set, it will stay reset.
	i2c_manageAck(i2c1Handle.i2c, ENABLE);

	while (1) {
		// wait here until button goes down
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_4)));
		delay();

//		masterGetDataLen(&dataLen);
//		masterGetData(dataIn, dataLen);

		// Get the data length
		cmd = CMD_GET_DATA_LEN;
		i2c_masterSendData(&i2c1Handle, &cmd, 1, SLAVE_ADDR);
		i2c_masterReceiveData(&i2c1Handle, &dataLen, 1, SLAVE_ADDR);

		// Get the data
		cmd = CMD_GET_DATA;
		i2c_masterSendData(&i2c1Handle, &cmd, 1, SLAVE_ADDR);
		i2c_masterReceiveData(&i2c1Handle, dataIn, dataLen, SLAVE_ADDR);

	}

	return 0;
}
