#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_


#include "stm32f411xx.h"


/*
 * Configuration structure for a SPIx pin
 */
typedef struct {

	uint8_t deviceMode;						/* Possible values from @SPI_DEVICE_MODE */
	uint8_t busCfg;							/* Possible values from @SPI_BUS_CFG */
	uint8_t sClkSpeed;						/* Possible values from @SPI_SERIAL_CLK_SPEED */
	uint8_t dff;							/* Possible values from @SPI_DFF */
	uint8_t cpol;							/* Possible values from @SPI_CPOL */
	uint8_t cpha;							/* Possible values from @SPI_CPHA */
	uint8_t ssm;							/* Possible values from @SPI_SSM */

} SPI_Config_t;


/*
 * Handle structure for SPIx pin
 */
typedef struct {

	SPI_RegDef_t *spi;
	SPI_Config_t spiCfg;
	uint8_t *txBuffer;
	uint8_t *rxBuffer;
	uint32_t txLen;
	uint32_t rxLen;
	uint8_t txState;
	uint8_t rxState;

} SPI_Handle_t;


/*
 * @SPI_DEVICE_MODE
 * Possible device modes
 */
#define SPI_DEVICE_MODE_SLAVE					0
#define SPI_DEVICE_MODE_MASTER					1


/*
 * @SPI_BUS_CFG
 */
#define SPI_BUS_CFG_FD							1
#define SPI_BUS_CFG_HD							2
#define SPI_BUS_CFG_SIMPLEX_RXONLY				3


/*
 * @SPI_SERIAL_CLK_SPEED
 * Possible serial clock speeds
 */
#define SPI_SERIAL_CLK_SPEED_DIV_2				0
#define SPI_SERIAL_CLK_SPEED_DIV_4				1
#define SPI_SERIAL_CLK_SPEED_DIV_8				2
#define SPI_SERIAL_CLK_SPEED_DIV_16				3
#define SPI_SERIAL_CLK_SPEED_DIV_32				4
#define SPI_SERIAL_CLK_SPEED_DIV_64				5
#define SPI_SERIAL_CLK_SPEED_DIV_128			6
#define SPI_SERIAL_CLK_SPEED_DIV_256			7


/*
 * @SPI_DFF
 * Possible spi data frame formats
 */
#define SPI_DFF_8_BIT							0
#define SPI_DFF_16_BIT							1


/*
 * @SPI_CPOL
 * Clock polarity values
 */
#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1


/*
 * @SPI_CPHA
 * Clock phase values
 */
#define SPI_CPHA_LOW							0
#define SPI_CPHA_HIGH							1


/*
 * @SPI_SSM
 * Software slave management
 */
#define SPI_SSM_DI								0
#define SPI_SSM_EN								1


/*
 * SPI application states
 */
#define SPI_STATE_READY							0
#define SPI_STATE_BUSY_IN_RX					1
#define SPI_STATE_BUSY_IN_TX					2


/*
 * SPI application events
 */
#define SPI_EVENT_TX_CMPLT						1
#define SPI_EVENT_RX_CMPLT						2
#define SPI_EVENT_OVR_ERR						3
#define SPI_EVENT_CRC_ERR						4


/*
 ************************************ Driver APIs **************************************
 */

/*
 * Peripheral clock setup
 */
void spi_periClockControl(SPI_RegDef_t *spi, uint8_t enOrDi);


/*
 * Init and De Init
 */
void spi_init(SPI_Handle_t *spiHandle);
void spi_deInit(SPI_RegDef_t *spi);


/*
 * Send data
 */
void spi_sendData(SPI_RegDef_t *spi, uint8_t *txBuffer, uint32_t len);


/*
 * Receive data
 */
void spi_receiveData(SPI_RegDef_t *spi, uint8_t *rxBuffer, uint32_t len);


/*
 * Send data interrupt api
 */
uint8_t spi_sendDataIt(SPI_Handle_t *spiHandle, uint8_t *txBuffer, uint32_t len);


/*
 * Receive data interrupt api
 */
uint8_t spi_receiveDataIt(SPI_Handle_t *spiHandle, uint8_t *rxBuffer, uint32_t len);

/*
 * SPI peripheral enable/disable
 */
void spi_peripheralControl(SPI_RegDef_t *spi, uint8_t enOrDi);


/*
 * SPI SSI config
 */
void spi_ssiConfig(SPI_RegDef_t *spi, uint8_t enOrDi);


/*
 * SPI SSI config
 */
void spi_ssoeConfig(SPI_RegDef_t *spi, uint8_t enOrDi);


/*
 * Clear overrun error flag
 */
void spi_clearOvrFlag(SPI_RegDef_t *spi);


/*
 * Close spi transmission
 */
void spi_closeTransmission(SPI_Handle_t *spiHandle);


/*
 * Close spi reception
 */
void spi_closeReception(SPI_Handle_t *spiHandle);


/*
 * Application event callback
 */
void spi_appEventCallback(SPI_Handle_t *spiHandle, uint8_t appEvent);


/*
 * IRQ configuration and ISR handling
 */
void spi_irqInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);
void spi_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void spi_irqHandling(SPI_Handle_t *spiHandle);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */





























