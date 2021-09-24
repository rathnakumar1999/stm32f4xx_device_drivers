/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 24-Jul-2021
 *      Author: RathnakumarKannan
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure of I2C peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle structure for I2C peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer;
	uint8_t  		*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint32_t		RxSize;
	uint8_t			TxRxState;
	uint8_t			SlaveAddr;
	uint8_t			RepeatStart;
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K  200000
#define I2C_SCL_SPEED_FM4K  400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C Peripheral Flag Names
 */
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)				//Start Condition generation
#define I2C_FLAG_ADDR		( 1 << I2C_SR1_ADDR)			//Address match
#define I2C_FLAG_BTF		( 1 << I2C_SR1_BTF)				//Byte Transfer Finish
#define I2C_FLAG_STOPF		( 1 << I2C_SR1_STOPF)			//Stop Condition detection
#define I2C_FLAG_RxnE		( 1 << I2C_SR1_RxNE)			//Data Register not Empty (receiver)
#define I2C_FLAG_TxE		( 1 << I2C_SR1_TxE)				//Data Register Empty (transmitter)
#define I2C_FLAG_BERR		( 1 << I2C_SR1_BERR)			//Bus Error
#define I2C_FLAG_ARLO		( 1 << I2C_SR1_ARLO)			//Arbitration Loss
#define I2C_FLAG_AF			( 1 << I2C_SR1_AF)				//Acknowledge Failure
#define I2C_FLAG_OVR		( 1 << I2C_SR1_OVR)				//Overrun
#define I2C_FLAG_TIMEOUT	( 1 << I2C_SR1_TIMEOUT)			//Timeout
#define I2C_FLAG_MSL		( 1 << I2C_SR2_MSL)				//Master indication flag


/*
 * I2C Repeated START macros
 */
#define I2C_REPEATSTART_DI		RESET									//Used to generate stop condition
#define I2C_REPEATSTART_EN		SET										//Used to generate repeated start


/*
 * I2C Application States
 */
#define I2C_READY			0
#define I2C_BUSY_IN_TX		1
#define I2C_BUSY_IN_RX		2


/*
 * I2C Events
 */
#define	I2C_EVT_TX_CMPLT				1
#define I2C_EVT_RX_CMPLT				2
#define I2C_EVT_STOP_DETECT				3
#define I2C_MASTER_REQUEST_DATA			4
#define I2C_MASTER_SENT_DATA			5



/*************************************************************************
 * 							API Supported by This Driver
 *************************************************************************/

/*
 * I2C Peripheral Clock control API
 */
void I2C_PclkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-Init API's of I2C
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Send Data & Receive Data API's for Master (blocking)
 */
void I2C_MasterSendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t repeatStart);
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t repeatStart);

/*
 * Send Data & Receive Data API's for Master (interrupt mode)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeat);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeat);

/*
 * Send Data & Received Data API's for Slave
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * other API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback functions
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

/*
 * I2C IRQ Configuration & Handle
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EVT_Handling(I2C_Handle_t *pI2CHandle);



#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
