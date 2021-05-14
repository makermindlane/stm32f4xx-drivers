#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_


#include "stm32f411xx.h"


/*
 * Configuration structure for a SPIx pin
 */
typedef struct {

	uint8_t gpioPinNumber;						/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t gpioPinMode;						/* Possible values from @GPIO_PIN_MODES */
	uint8_t gpioPinOPType;						/* Possible values from @GPIO_PIN_OPTYPE */
	uint8_t gpioPinSpeed;						/* Possible values from @GPIO_PIN_SPEED */
	uint8_t gpioPinPuPd;						/* Possible values from @GPIO_PIN_PUPD */
	uint8_t gpioPinAF;							/* Possible values from @GPIO_PIN_AF */

} SPI_PinConfig_t;


/*
 * Handle structure for SPIx pin
 */
typedef struct {

	SPI_RegDef_t *spiReg;
	SPI_PinConfig_t spiPinCfg;

} SPI_Handle_t;


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
