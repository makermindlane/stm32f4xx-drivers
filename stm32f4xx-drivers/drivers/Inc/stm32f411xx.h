/*
 * stm32f411xx.h
 *
 *  Created on: May 7, 2021
 *      Author: pawan
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#include <stddef.h>



/**********************************************************************************************************************
 *					START: Some defines (could be used in future for platform independent purposes
 *********************************************************************************************************************/

#define __weak 								__attribute((weak))

/**********************************************************************************************************************
 *					START: Some defines (could be used in future for platform independent purposes
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *										 START: Processor specific details
 *********************************************************************************************************************/

/*
 * ARM cortex M4 processor NVIC ISERx register addresses
 */
#define NVIC_ISER0							((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1							((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2							((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3							((volatile uint32_t*) 0xE000E10C)


/*
 * ARM cortex M4 processor NVIC ICERx register addresses
 */
#define NVIC_ICER0							((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1							((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2							((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3							((volatile uint32_t*) 0xE000E18C)


/*
 * ARM cortex M4 processor NVIC IPRx register addresses
 */
#define NVIC_IPR_BASEADDR					((volatile uint32_t*) 0xE000E400)


#define NO_PR_BITS_IMPLEMENTED				(4)


/*
 * NVIC interrupt priority possible values
 */
#define NVIC_PRIORITY_0						(0)
#define NVIC_PRIORITY_1						(1)
#define NVIC_PRIORITY_2						(2)
#define NVIC_PRIORITY_3						(3)
#define NVIC_PRIORITY_4						(4)
#define NVIC_PRIORITY_5						(5)
#define NVIC_PRIORITY_6						(6)
#define NVIC_PRIORITY_7						(7)
#define NVIC_PRIORITY_8						(8)
#define NVIC_PRIORITY_9						(9)
#define NVIC_PRIORITY_10					(10)
#define NVIC_PRIORITY_11					(11)
#define NVIC_PRIORITY_12					(12)
#define NVIC_PRIORITY_13					(13)
#define NVIC_PRIORITY_14					(14)
#define NVIC_PRIORITY_15					(15)

/**********************************************************************************************************************
 *										END: Processor specific details
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *										 START: Register Base addresses
 *********************************************************************************************************************/

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

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

/**********************************************************************************************************************
 *										END: Register Base addresses
 *********************************************************************************************************************/



/**********************************************************************************************************************
 * 								START: Peripheral register definition structures
 *********************************************************************************************************************/

/*
 * GPIO peripheral register definition
 */
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


/*
 * RCC peripheral register definition
 */
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


/*
 * EXTI peripheral register definition
 */
typedef struct {

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

} EXTI_RegDef_t;


/*
 * SYSCFG register definition
 */
typedef struct {

	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;

} SYSCFG_RegDef_t;


/*
 * SPI register definition
 */
typedef struct {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

} SPI_RegDef_t;


/*
 * I2C register definition
 */
typedef struct {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;

} I2C_RegDef_t;

/**********************************************************************************************************************
 *								END: Peripheral register definition structures
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *					START: Peripheral definitions typecasted to corresponding struct defs
 *********************************************************************************************************************/

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

#define EXTI								((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG								((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1								((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4								((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5								((SPI_RegDef_t*) SPI5_BASEADDR)

#define I2C1								((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2								((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3								((I2C_RegDef_t*) I2C3_BASEADDR)

/**********************************************************************************************************************
 *					END: Peripheral definitions typecasted to corresponding struct defs
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *							 START: Peripheral clock enable and disable macros
 *********************************************************************************************************************/

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
#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 14))

/**********************************************************************************************************************
 *									END: Peripheral clock enable and disable macros
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *										START: Peripheral register reset macros
 *********************************************************************************************************************/

/*
 * GPIOx peripherals reset macros
 */
#define GPIOA_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while (0)
#define GPIOB_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while (0)
#define GPIOC_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while (0)
#define GPIOH_REG_RESET()					do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while (0)


/*
 * SPIx peripherals reset macros
 */
#define SPI1_REG_RESET()					do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while (0)
#define SPI2_REG_RESET()					do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while (0)
#define SPI3_REG_RESET()					do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while (0)
#define SPI4_REG_RESET()					do { (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); } while (0)
#define SPI5_REG_RESET()					do { (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); } while (0)


/*
 * I2Cx peripheral reset macros
 */
#define I2C1_REG_RESET()					do { (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); } while (0)
#define I2C2_REG_RESET()					do { (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while (0)
#define I2C3_REG_RESET()					do { (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); } while (0)

/**********************************************************************************************************************
 *										END: Peripheral register reset macros
 *********************************************************************************************************************/




/**********************************************************************************************************************
 *								START: Peripheral register bit position definitions
 *********************************************************************************************************************/

/*
 * SPI_CR1 register bit position defines
 */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSBFIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15


/*
 * SPI_CR2 register bit position defines
 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7


/*
 * SPI_SR register bit position defines
 */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8


/*
 * I2C_CR1 register bit position defines
 */
#define I2C_CR1_PE							0
#define I2C_CR1_SMBUS						1
#define I2C_CR1_SMBTYPE						3
#define I2C_CR1_ENARP						4
#define I2C_CR1_ENPEC  						5
#define I2C_CR1_ENGC   						6
#define I2C_CR1_NOSTRETCH					7
#define I2C_CR1_START						8
#define I2C_CR1_STOP    					9
#define I2C_CR1_ACK							10
#define I2C_CR1_POS							11
#define I2C_CR1_PEC							12
#define I2C_CR1_ALERT						13
#define I2C_CR1_SWRST						15


/*
 * I2C_CR2 register bit position defines
 */
#define I2C_CR2_FREQ						0
#define I2C_CR2_ITERREN						8
#define I2C_CR2_ITEVTEN  					9
#define I2C_CR2_ITBUFEN  					10
#define I2C_CR2_DMAEN						11
#define I2C_CR2_LAST						12


/*
 * I2C_OAR1 register bit position defines
 */
#define I2C_OAR1_ADD0						0
#define I2C_OAR1_ADD1						1
#define I2C_OAR1_ADD8  						8
#define I2C_OAR1_ADDMODE  					15


/*
 * I2C_OAR2 register bit position defines
 */
#define I2C_OAR2_ENDUAL						0
#define I2C_OAR2_ADD2						1


/*
 * I2C_DR register bit position defines
 */



/*
 * I2C_SR1 register bit position defines
 */
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2
#define I2C_SR1_ADD10						3
#define I2C_SR1_STOPF						4
#define I2C_SR1_RxNE						6
#define I2C_SR1_TxE							7
#define I2C_SR1_BERR						8
#define I2C_SR1_ARLO						9
#define I2C_SR1_AF							10
#define I2C_SR1_OVR							11
#define I2C_SR1_PECERR						12
#define I2C_SR1_TIMEOUT						14
#define I2C_SR1_SMBALERT					15


/*
 * I2C_SR2 register bit position defines
 */
#define I2C_SR2_MSL							0
#define I2C_SR2_BUSY						1
#define I2C_SR2_TRA							2
#define I2C_SR2_GENCALL						4
#define I2C_SR2_SMBDEFALT					5
#define I2C_SR2_SMBHOST						6
#define I2C_SR2_DUALF						7
#define I2C_SR2_PEC							8


/*
 * I2C_CCR register bit position defines
 */
#define I2C_CCR_CCR							0
#define I2C_CCR_DUTY						14
#define I2C_CCR_FS							15


/*
 * I2C_TRISE register bit position defines
 */
#define I2C_TRISE_TRISE						0


/*
 * I2C_FLTR register bit position defines
 */
#define I2C_FLTR_DNF						0
#define I2C_FLTR_ANOFF						4

/**********************************************************************************************************************
 *								END: Peripheral register bit position definitions
 *********************************************************************************************************************/



/**********************************************************************************************************************
 *								START: Miscellaneous macros
 *********************************************************************************************************************/

/*
 * Returns the port code for the given GPIOx base address
 */
#define GPIO_BASEADDR_TO_PORTCODE(p)		((p == GPIOA)?0:\
											 (p == GPIOB)?1:\
											 (p == GPIOC)?2:\
											 (p == GPIOD)?3:\
											 (p == GPIOE)?4:\
											 (p == GPIOH)?7:0)


/*
 * IRQ numbers of stm32f411x MCU
 */
#define IRQ_NO_EXTI0						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

#define IRQ_NO_SPI1							35
#define IRQ_NO_SPI2							36
#define IRQ_NO_SPI3							51
#define IRQ_NO_SPI4							84
#define IRQ_NO_SPI5							85

#define IRQ_NO_I2C1_EV						31
#define IRQ_NO_I2C1_ER						32
#define IRQ_NO_I2C2_EV						33
#define IRQ_NO_I2C2_ER						34
#define IRQ_NO_I2C3_EV						72
#define IRQ_NO_I2C3_ER						73


/*
 * Some generic macros
 */
#define ENABLE								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET

#define CHECK_BIT_FOR_SET(reg, bit)					((reg) & (1 << (bit)))
#define WAIT_UNTIL_SET(reg, bit)					while (!CHECK_BIT_FOR_SET(reg, bit))
#define WAIT_UNTIL_RESET(reg, bit)					while (CHECK_BIT_FOR_SET(reg, bit))

/**********************************************************************************************************************
 *								END: Miscellaneous macros
 *********************************************************************************************************************/


#endif /* INC_STM32F411XX_H_ */

















