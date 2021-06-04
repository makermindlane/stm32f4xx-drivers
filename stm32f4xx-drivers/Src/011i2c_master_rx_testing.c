#include <string.h>

#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_gpio_driver.h"


#define MY_ADDR								0x61
#define SLAVE_ADDR							0x68

#define CMD_GET_DATA_LEN					0x51
#define CMD_GET_DATA						0x52

I2C_Handle_t i2c1Handle;

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

	uint8_t dataLen;
	uint8_t dataIn[255];

	uint8_t cmd;

	// I2C pin init
	i2c1GpioInit();

	// button init
	buttonInit();

	// I2C peripheral config
	i2c1Init();

	// Configure i2c interrupts
	i2c_irqInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	i2c_irqInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	// Enable i2c peripheral
	i2c_peripheralControl(i2c1Handle.i2c, ENABLE);

	// Set the ACK bit here also. Why? Because without setting the PE bit (which is done in above line)
	// ACK bit cannot be set, it will stay reset.
	i2c_manageAck(i2c1Handle.i2c, ENABLE);

	while (1) {
		// wait here until button goes down
		while (!(gpio_readFromInputPin(GPIOB, GPIO_PIN_NO_4)));
		delay();

		// Get the data length
		cmd = CMD_GET_DATA_LEN;
		// Wait until i2c state becomes ready.
		while (i2c_masterSendDataIt(&i2c1Handle, &cmd, 1, SLAVE_ADDR, ENABLE) != I2C_STATE_READY);
		// Wait until i2c state becomes ready.
		while (i2c_masterReceiveDataIt(&i2c1Handle, &dataLen, 1, SLAVE_ADDR, ENABLE) != I2C_STATE_READY);
		rxComplete = RESET;
		// Get the data
		cmd = CMD_GET_DATA;
		// Wait until i2c state becomes ready.
		while (i2c_masterSendData(&i2c1Handle, &cmd, 1, SLAVE_ADDR, ENABLE) != I2C_STATE_READY);
		// Wait until i2c state becomes ready.
		while (i2c_masterReceiveData(&i2c1Handle, dataIn, dataLen, SLAVE_ADDR, DISABLE) != I2C_STATE_READY);
		while (rxComplete != SET);
		rxComplete = RESET;
	}

	return 0;
}


void I2C1_EV_IRQHandler() {
	i2c_evIrqHandling(&i2c1Handle);
}


void I2C1_ER_IRQHandler() {
	i2c_erIrqHandling(&i2c1Handle);
}


void i2c_appEventCallback(I2C_Handle_t *i2cHandle, uint8_t appEvent) {
	if (appEvent == I2C_EVENT_TX_CMPLT) {
		// printf("Tx is completed\n");
	} else if (appEvent == I2C_EVENT_RX_CMPLT) {
		// printf("Rx is completed\n");
		rxComplete = SET;
	} else if (appEvent == I2C_ERROR_AF) {
		// Why ack failure happens?
		// In master, ack failure happens when slave fails to send ack for the byte sent from master.
		// printf("Error: Ack failure\n");
		i2c_closeSendData(i2cHandle);
		// Generate the stop condition to release the bus.
		i2c_generateStopCondition(i2cHandle->i2c);
		// Hang in infinite loop
		while (1);
	}
}















