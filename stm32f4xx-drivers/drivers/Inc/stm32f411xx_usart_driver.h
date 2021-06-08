#ifndef INC_STM32F411XX_USART_DRIVER_H_
#define INC_STM32F411XX_USART_DRIVER_H_

#include "stm32f411xx.h"

/*
 * USART peripheral configuration structure
 */
typedef struct {

	uint8_t mode;
	uint32_t baud;
	uint8_t noOfStopBits;
	uint8_t wordLength;
	uint8_t parityControl;
	uint8_t hWFlowControl;

} USART_Config_t;


/*
 * USART peripheral handle structure
 */
typedef struct {

	USART_RegDef_t *usart;
	USART_Config_t usartCfg;

	void (*appEventCallback)(USART_Handle_t *, uint8_t);

} USART_Handle_t;


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200								1200
#define USART_STD_BAUD_2400								400
#define USART_STD_BAUD_9600								9600
#define USART_STD_BAUD_19200 							19200
#define USART_STD_BAUD_38400 							38400
#define USART_STD_BAUD_57600 							57600
#define USART_STD_BAUD_115200 							115200
#define USART_STD_BAUD_230400 							230400
#define USART_STD_BAUD_460800 							460800
#define USART_STD_BAUD_921600 							921600
#define USART_STD_BAUD_2M 								2000000
#define SUART_STD_BAUD_3M 								3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   							2
#define USART_PARITY_EN_EVEN  							1
#define USART_PARITY_DISABLE   							0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  							0
#define USART_WORDLEN_9BITS  							1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     							0
#define USART_STOPBITS_0_5   							1
#define USART_STOPBITS_2     							2
#define USART_STOPBITS_1_5   							3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    						0
#define USART_HW_FLOW_CTRL_CTS    						1
#define USART_HW_FLOW_CTRL_RTS    						2
#define USART_HW_FLOW_CTRL_CTS_RTS						3


/******************************************************************************************
 *								APIs supported by this driver
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void usart_periClockControl(USART_RegDef_t *usart, uint8_t isEnable);

/*
 * Init and De-init
 */
void usart_init(USART_Handle_t *usartHandle);
void usart_deInit(USART_RegDef_t *usart);


/*
 * Data Send and Receive
 */
void usart_sendData(USART_RegDef_t *usart,uint8_t *txBuffer, uint32_t len);
void usart_receiveData(USART_RegDef_t *usart, uint8_t *rxBuffer, uint32_t len);
uint8_t usart_sendDataIT(USART_Handle_t *usartHandle,uint8_t *txBuffer, uint32_t len);
uint8_t usart_receiveDataIT(usart_Handle_t *usartHandle, uint8_t *rxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */
void usart_irqInterruptConfig(uint8_t irqNumber, uint8_t isEnable);
void usart_irqPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void usart_irqHandling(USART_Handle_t *usartHandle);

/*
 * Other Peripheral Control APIs
 */
void usart_peripheralControl(USART_RegDef_t *usart, uint8_t isEnable);



#endif /* INC_STM32F411XX_USART_DRIVER_H_ */
