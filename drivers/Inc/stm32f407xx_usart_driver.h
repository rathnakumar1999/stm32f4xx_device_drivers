/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 09-Sep-2021
 *      Author: RathnakumarKannan
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t	 USART_Mode;
	uint32_t USART_BaudRate;
	uint8_t  USART_Parity;
	uint8_t	 USART_HWFlowControl;
	uint8_t  USART_DataBits;
	uint8_t  USART_StopBits;
	uint8_t  USART_SamplingRate;
}USART_Config_t;


typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t UsartConfig;
	uint8_t 	   TxState;
	uint8_t		   RxState;
	uint8_t		   *pTxBuffer;
	uint8_t		   *pRxBuffer;
	uint32_t	   TxLen;
	uint32_t	   RxLen;
}USART_Handle_t;


/*
 * @USART_Mode
 */
#define USART_MODE_TX_ONLY			0
#define USART_MODE_RX_ONLY			1
#define USART_MODE_RXTX_BOTH		2

/*
 * @USART_BaudRate
 */
#define USART_BAUD_1200			1200
#define USART_BAUD_2400			2400
#define USART_BAUD_9600			9600
#define USART_BAUD_19200		19200
#define USART_BAUD_38400		38400
#define USART_BAUD_57600		57600
#define USART_BAUD_115200		115200
#define USART_BAUD_230400		230400
#define USART_BAUD_460800		460800
#define USART_BAUD_921600		921600
#define USART_BAUD_2M			200000
#define USART_BAUD_3M			300000


/*
 * @USART_Parity
 */
#define USART_PARITY_DI			0
#define USART_PARITY_ODD		1
#define USART_PARITY_EVEN		2

/*
 * @USART_HWFlowControl
 */
#define USART_HW_FLOWCNTRL_DI				0
#define USART_HW_FLOWCNTRL_RTS_ONLY			1
#define USART_HW_FLOWCNTRL_CTS_ONLY			2
#define USART_HW_FLOWCNTRL_CTS_RTS_BOTH		3

/*
 * @USART_DataBits
 */
#define USART_8BITS		0
#define USART_9BITS		1

/*
 * @USART_StopBits
 */
#define USART_STOPBIT_1			0
#define USART_STOPBIT_0_5		1
#define USART_STOPBIT_2 		2
#define USART_STOPBIT_1_5		3

/*
 * @USART_SamplingRate
 */
#define USART_SAMPLEBY_16		0
#define USART_SAMPLEBY_8		1

/*
 * Possible USART States
 */
#define USART_READY				0
#define USART_BSY_IN_TX			1
#define USART_BSY_IN_RX			2

/*
 * Possible USART Events
 */
#define USART_EVT_TX_READY		0
#define USART_EVT_TX_CMPLT		1
#define USART_EVT_RX_CMPLT		2
#define USART_EVT_CTS_ASSERTED  4



/*
 * USART FlagNames
 */
#define USART_FLAG_PE			( 1 << USART_SR_PE)
#define USART_FLAG_FE			( 1 << USART_SR_FE)
#define USART_FLAG_NF			( 1 << USART_SR_NF)
#define USART_FLAG_ORE			( 1 << USART_SR_ORE)
#define USART_FLAG_IDLE			( 1 << USART_SR_IDLE)
#define USART_FLAG_RXNE			( 1 << USART_SR_RXNE)
#define USART_FLAG_TC			( 1 << USART_SR_TC)
#define USART_FLAG_TXE			( 1 << USART_SR_TXE)
#define USART_FLAG_LBD			( 1 << USART_SR_LBD)
#define USART_FLAG_CTS			( 1 << USART_SR_CTS)



/*************************************************************************
 * 							API Supported by This Driver
 *************************************************************************/

/*
 * USART Init & De-Init API
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Send & Receive Data API's
 */
void USART_SendData(USART_Handle_t *pUSARTHandlex, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);


/*
 * Send & Receive Data using Interrupt
 */
void USART_SendDataIT(USART_Handle_t *pUSARTHandlex, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveDataIT(USART_Handle_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);

/*
 * Other API's
 */
void USART_PclkControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);


/*
 * I2C IRQ Configuration & Handle
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_EVTHandling(USART_Handle_t *pI2CHandle);

/*
 * Application Callback functions
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
