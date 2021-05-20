#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"


/*
 * I2C peripheral configuration structure
 */
typedef struct {

	uint32_t sclSpeed;									/* Possible values from @I2C_SCL_SPEED*/
	uint8_t deviceAddr;
	uint8_t ackCtrl;									/* Possible values from @I2C_ACK_CTRL*/
	uint8_t fmDutyCycle;								/* Possible values from @I2C_FM_DUTY_CYCLE*/

} I2C_Config_t;


/*
 * I2C peripheral handle structure
 */
typedef struct {

	I2C_RegDef_t *i2c;
	I2C_Config_t i2cCfg;

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


/**********************************************************************************************************************
 * 													Driver APIs
 *********************************************************************************************************************/

/*
 * Peripheral clock setup
 */
void i2c_periClockControl(I2C_RegDef_t *i2c, uint8_t enOrDi);


/*
 * I2C Init
 */
void i2c_init(I2C_Handle_t *i2cHandle);


/*
 * I2C DeInit
 */
void i2c_deInit(I2C_RegDef_t *i2c);


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



#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */



















