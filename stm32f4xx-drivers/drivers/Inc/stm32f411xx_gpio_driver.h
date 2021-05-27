/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: May 7, 2021
 *      Author: pawan
 */

#ifndef STM32F411XX_GPIO_DRIVER_H_
#define STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"


/*
 * Configuration structure for a GPIOx pin
 */
typedef struct {

	uint8_t pinNumber;						/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t pinMode;						/* Possible values from @GPIO_PIN_MODES */
	uint8_t pinOPType;						/* Possible values from @GPIO_PIN_OPTYPE */
	uint8_t pinSpeed;						/* Possible values from @GPIO_PIN_SPEED */
	uint8_t pinPuPd;						/* Possible values from @GPIO_PIN_PUPD */
	uint8_t pinAF;							/* Possible values from @GPIO_PIN_AF */

} GPIO_PinConfig_t;


/*
 * Handle structure for GPIOx pin
 */
typedef struct {

	GPIO_RegDef_t *gpio;
	GPIO_PinConfig_t pinCfg;

} GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_PIN_MODE_IN			0
#define GPIO_PIN_MODE_OUT			1
#define GPIO_PIN_MODE_AF			2
#define GPIO_PIN_MODE_ANALOG		3

#define GPIO_PIN_MODE_IT_FT			4
#define GPIO_PIN_MODE_IT_RT			5
#define GPIO_PIN_MODE_IT_RFT		6


/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_PIN_OPTYPE_PP				0
#define GPIO_PIN_OPTYPE_OD				1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_PIN_SPEED_LOW			0
#define GPIO_PIN_SPEED_MEDIUM		1
#define GPIO_PIN_SPEED_FAST			2
#define GPIO_PIN_SPEED_HIGH			3


/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull and pull down configuration macros
 */
#define GPIO_PIN_PUPD_NO			0
#define GPIO_PIN_PUPD_PU			1
#define GPIO_PIN_PUPD_PD			2


/*
 * @GPIO_PIN_AF
 * GPIO alternate function mode possible values
 */
#define GPIO_PIN_AF_0				0
#define GPIO_PIN_AF_1				1
#define GPIO_PIN_AF_2				2
#define GPIO_PIN_AF_3				3
#define GPIO_PIN_AF_4				4
#define GPIO_PIN_AF_5				5
#define GPIO_PIN_AF_6				6
#define GPIO_PIN_AF_7				7
#define GPIO_PIN_AF_8				8
#define GPIO_PIN_AF_9				9
#define GPIO_PIN_AF_10				10
#define GPIO_PIN_AF_11				11
#define GPIO_PIN_AF_12				12
#define GPIO_PIN_AF_13				13
#define GPIO_PIN_AF_14				14
#define GPIO_PIN_AF_15				15


/*
 ************************************ Driver APIs **************************************
 */

/*
 * Peripheral clock setup
 */
void gpio_periClockControl(GPIO_RegDef_t *gpioPort, uint8_t enOrDi);


/*
 * Init and De Init
 */
void gpio_init(GPIO_Handle_t *gpioHandle);
void gpio_deInit(GPIO_RegDef_t *gpioPort);


/*
 * Data read and write
 */
uint8_t gpio_readFromInputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber);
uint16_t gpio_readFromInputPort(GPIO_RegDef_t *gpioPort);
void gpio_writeToOutputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber, uint8_t value);
void gpio_writeToOutputPort(GPIO_RegDef_t *gpioPort, uint16_t value);
void gpio_toggleOutputPin(GPIO_RegDef_t *gpioPort, uint8_t pinNumber);


/*
 * IRQ configuration and ISR handling
 */
void gpio_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);
void gpio_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void gpio_irqHandling(uint8_t pinNumber);




















#endif /* STM32F411XX_GPIO_DRIVER_H_ */
