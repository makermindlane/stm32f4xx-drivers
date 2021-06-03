#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"


/*
 * I2C peripheral configuration structure
 */
typedef struct {

	uint32_t sclSpeed;									/* Possible values from @I2C_SCL_SPEED */
	uint8_t deviceAddr;
	uint8_t ackCtrl;									/* Possible values from @I2C_ACK_CTRL */
	uint8_t fmDutyCycle;								/* Possible values from @I2C_FM_DUTY_CYCLE */

} I2C_Config_t;


/*
 * I2C peripheral handle structure
 */
typedef struct {

	I2C_RegDef_t *i2c;
	I2C_Config_t i2cCfg;
	uint8_t *txBuffer;
	uint8_t *rxBuffer;
	uint32_t txLen;
	uint32_t rxLen;
	uint8_t txRxState;									/* Possible values from @I2C_STATE */
	uint8_t devAddr;
	uint32_t rxSize;
	uint8_t repeatedStart;								/* Possible values from @I2C_RS */

} I2C_Handle_t;


/*
 * @I2C_SCL_SPEED
 */
#define I2C_SCL_SPEED_SM								100000
#define I2C_SCL_SPEED_FM_200K							200000
#define I2C_SCL_SPEED_SM_400K							400000


/*
 * @I2C_ACK_CTRL
 */
#define I2C_ACK_CTRL_DISABLE							0
#define I2C_ACK_CTRL_ENABLE								1


/*
 * @I2C_FM_DUTY_CYCLE
 */
#define I2C_FM_DUTY_CYCLE_2								0
#define I2C_FM_DUTY_CYCLE_16_9							1


/*
 * Read and write indication macros
 */
#define I2C_MASTER_READ									1
#define I2C_MASTER_WRITE								0


/*
 * @I2C_STATE
 */
#define I2C_STATE_READY									0
#define I2C_STATE_BUSY_IN_RX							1
#define I2C_STATE_BUSY_IN_TX							2


/*
 * @I2C_EVENT
 */
#define I2C_EVENT_TX_CMPLT								1
#define I2C_EVENT_RX_CMPLT								2
#define I2C_EVENT_STOP									3
#define I2C_EVENT_DATA_REQ								4
#define I2C_EVENT_DATA_RCV								5


/*
 *
 */
#define I2C_ERROR_BERR  								3
#define I2C_ERROR_ARLO  								4
#define I2C_ERROR_AF									5
#define I2C_ERROR_OVR   								6
#define I2C_ERROR_TIMEOUT 								7


/*
 * @I2C_RS
 * I2C repeated start possible values
 */
#define I2C_RS_ENABLE									1
#define I2C_RS_DISABLE									0

/**********************************************************************************************************************
 * 													Driver APIs
 *********************************************************************************************************************/

/*
 * Peripheral clock setup
 */
void i2c_periClockControl(I2C_RegDef_t *i2c, uint8_t enOrDi);


/*
 * Enable or disable the ACK bit
 */
void i2c_manageAck(I2C_RegDef_t *i2c, uint8_t isEnable);


/*
 * I2C Init
 */
void i2c_init(I2C_Handle_t *i2cHandle);


/*
 * I2C DeInit
 */
void i2c_deInit(I2C_RegDef_t *i2c);


/*
 * Master send data
 */
void i2c_masterSendData(I2C_Handle_t *i2cHandle, uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr);


/*
 * Master receive data
 */
void i2c_masterReceiveData(I2C_Handle_t *i2cHandle, uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr);


/*
 * Master send data interrupt api
 */
uint8_t i2c_masterSendDataIt(I2C_Handle_t *i2cHandle, uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);


/*
 * Master receive data interrupt api
 */
uint8_t i2c_masterReceiveDataIt(I2C_Handle_t *i2cHandle, uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);


/*
 * Slave send data
 */
void i2c_slaveSendData(I2C_RegDef_t *i2c, uint8_t data);


/*
 * Slave receive data
 */
uint8_t i2c_slaveReceiveData(I2C_RegDef_t *i2c);

/*
 * I2C peripheral enable/disable
 */
void i2c_peripheralControl(I2C_RegDef_t *i2c, uint8_t enOrDi);


/*
 * I2C irq interrupt config
 */
void i2c_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);


/*
 * I2C irq priority config
 */
void i2c_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);


/*
 * I2C event interrupt handler
 */
void i2c_evIrqHandling(I2C_Handle_t *i2cHandle);


/*
 * I2C error interrupt handler
 */
void i2c_erIrqHandling(I2C_Handle_t *i2cHandle);


/*
 * Application event callback
 */
void i2c_appEventCallback(I2C_Handle_t *i2cHandle, uint8_t appEvent);

#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */



















