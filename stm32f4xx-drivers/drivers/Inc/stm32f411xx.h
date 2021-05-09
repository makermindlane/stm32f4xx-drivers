/*
 * stm32f411xx.h
 *
 *  Created on: May 7, 2021
 *      Author: pawan
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>


/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR 				0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0xC000)

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)


/********************************* Peripheral register definition structures *********************************/

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	uint32_t RESERVED8;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;


/**
 * Peripheral definition (peripheral base address typecasted to xxx_RegDef_t)
 */
#define GPIOA								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))


/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 7))


/*
 * Macros to disable GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while (0)
#define GPIOB_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while (0)
#define GPIOC_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while (0)
#define GPIOH_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while (0)


/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1 << 23))


/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()						(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()						(RCC->APB2ENR |= (1 << 20))


/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 20))


/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1 << 5))


/*
 * Clock disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock enable macros for SYSCFG peripherals
 */



/*
 * Some generic macros
 */
#define ENABLE								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET


#endif /* INC_STM32F411XX_H_ */

















