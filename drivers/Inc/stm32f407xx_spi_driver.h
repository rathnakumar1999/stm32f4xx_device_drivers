/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 13-Jun-2021
 *      Author: RathnakumarKannan
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;							/*!< Possible values from @SPI_DeviceMode>*/
	uint8_t SPI_BusConfig;							/*!< Possible values from @SPI_BusConfig>*/
	uint8_t SPI_SclkSpeed;							/*!< Possible values from @SPI_SclkSpeed>*/
	uint8_t SPI_DFF;								/*!< Possible values from @SPI_DFF>*/
	uint8_t SPI_CPOL;								/*!< Possible values from @SPI_CPOL>*/
	uint8_t SPI_CPHA;								/*!< Possible values from @SPI_CPHA>*/
	uint8_t SPI_SSM;								/*!< Possible values from @SPI_SSM>*/
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx;							/*!< Base Address of SPI peripheral through which SPI communication going to happen>*/
	SPI_Config_t SPI_PeripheralConfig;				/*!< Configuration Settings for the SPI Peripheral>*/
	uint8_t *pTxBuffer;								/*!< Pointer to store base address of TxBuffer>*/
	uint8_t *pRxBuffer;								/*!< Pointer to store base address of RxBuffer>*/
	uint8_t TxLen;									/*!< Length of data to be transmitted>*/
	uint8_t RxLen;									/*!< Length of data to be Received>*/
	uint8_t TxState;								/*!< To store Tx state>*/
	uint8_t RxState;								/*!< To store Rx state>*/
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 * SPI - Possible Device Modes
 */
#define SPI_MODE_MASTER		1					/*!< SPI Peripheral will act as Master>*/
#define SPI_MODE_SLAVE		0					/*!< SPI Peripheral will act as Slave>*/

/*
 * @SPI_BusConfig
 * SPI - Possible Bus Configurations
 */
#define SPI_BUS_CONFIG_FD				1			/*!< FullDuplex, transmit & receive simultaneously using MISO & MOSI pins>*/
#define SPI_BUS_CONFIG_HD				2			/*!< HalfDuplex, can either transmit or receive at a time, MOSI Used for master, MISO used for slave>*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3			/*!< Simplex & Receive Only, can only receive the data through MISO if master & through MOSI if slave>*/

/*
 * @SPI_SclkSpeed
 * SPI - Possible values of clock frequency
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 * SPI - Data Frame Format
 */
#define SPI_DFF_8BITS		0						/*!< Size of shift register will be 8 bits, single transfer can receive 1 Byte of data>*/
#define SPI_DFF_16BITS		1						/*!< Size of shift register will be 16 bits, single transfer can receive 2 Bytes of data>*/

/*
 * @SPI_CPOL
 * SPI - Clock Polarity Setting
 */
#define SPI_CPOL_LOW		0						/*!< Ideal state of clock will be "LOW", first edge of clk will be rising>*/
#define SPI_CPOL_HIGH		1						/*!< Ideal state of clock will be "HIGH", first edge of clk will be falling>*/

/*
 * @SPI_CPHA
 * SPI - Clock Phase Setting
 */
#define SPI_CPHA_LOW		0						/*!< Samples the data at the 1st edge of Clock & transist the MOSI & MISO lines at the 2nd edge of clock>*/
#define SPI_CPHA_HIGH		1						/*!< Samples the data at the 2nd edge of Clock & transist the MOSI & MISO lines at the 1st edge of clock>*/

/*
 * @SPI_SSM
 * SPI - Software Slave Management Setting
 */
#define SPI_SSM_DI			0						/*!<Disable the Software Slave Management, NSS pin will be driven by the Hardware of Master>*/
#define SPI_SSM_EN			1						/*!<Enable the Software Slave Management, NSS pin will be driven by the MCU using SSI bit>*/

/*
 * SPI Related status flag definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)

/*
 * Possible SPI Application states
 */
#define SPI_STATUS_READY		0
#define SPI_STATUS_BSY_IN_TX    1
#define SPI_STATUS_BSY_IN_RX    2

/*
 * Possible SPI Application Events
 */
#define SPI_EVT_TX_CMPLT		0
#define SPI_EVT_RX_CMPLT		1
#define SPI_EVT_OVRERR			2



/*************************************************************************
 * 						API Supported by This Driver
 *************************************************************************/
/*
 * SPI Peripheral Clock control API
 */
void SPI_PclkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init API's of SPI
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Flag Status Indicating API
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);

/*
 * Send Data & Receive Data API's (blocking)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * Send Data & Receive Data API's (interrupt mode)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * other API's
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlagClear(SPI_RegDef_t *pSPIx);

/*
 * Application Callback functions
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

/*
 * SPI IRQ Configuration & Handle
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
