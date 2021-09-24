/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 25-Jul-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx_i2c_driver.h"

//definition of helper functions
static uint32_t get_apb1_pclk();
static uint32_t get_pll_clk();
static void 	i2c_generate_start(I2C_RegDef_t *pI2Cx);
static void 	i2c_generate_stop(I2C_RegDef_t *pI2Cx);
static void 	i2c_clear_addr_flag(I2C_RegDef_t *pI2Cx);
static uint8_t 	i2c_formAddr_withWrite(uint8_t addr);
static uint8_t 	i2c_formAddr_withRead(uint8_t addr);
static void 	i2c_close_sendData(I2C_Handle_t *pI2CHandle);
static void 	i2c_close_receiveData(I2C_Handle_t *pI2CHandle);
static void 	i2c_master_txeinterrupt_handle(I2C_Handle_t *pI2CHandle);
static void 	i2c_master_rxeinterrupt_handle(I2C_Handle_t *pI2CHandle);


static uint16_t clkPrescalers[] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint32_t get_apb1_pclk()
{
	uint32_t systemclock;
	uint8_t clksrc,ahbpre,apb1pre;
	uint16_t temp;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0){
		systemclock = 16000000;
	}
	else if(clksrc == 1){
		systemclock = 8000000;
	}
	else if(clksrc == 2){
		systemclock = get_pll_clk();
	}

	//ahb prescaler
	temp = ( (RCC->CFGR >> 4) & 0x0F);

	if(temp < 8) {
		ahbpre = 1;
	}
	else {
		ahbpre = clkPrescalers[temp - 8];
	}

	systemclock = systemclock / ahbpre;

	//abp1 prescaler
	temp = ( (RCC->CFGR >> 10) & 0x07);
	if(temp < 4) {
		apb1pre = 1;
	}
	else {
		apb1pre = clkPrescalers[temp - 4];
	}

	systemclock = systemclock / apb1pre;

	return systemclock;

}

static uint32_t get_pll_clk()
{
	uint8_t clksrc;
	uint32_t pllclk;
	uint8_t pllm,pllp;
	uint16_t plln;

	clksrc = ( (RCC->PLLCFGR >> 22) & 0x01 );

	if(clksrc == 0) {
		pllclk = 16000000;
	}
	else if(clksrc == 1) {
		pllclk = 8000000;
	}

	pllm = ( RCC->PLLCFGR & 0x3F);
	pllclk = pllclk / pllm;

	plln = ( (RCC->PLLCFGR >> 6) & 0x1FF);
	pllclk = pllclk * plln;

	pllp = ( (RCC->PLLCFGR >> 16) & 0x03);
	pllclk = pllclk / pllp;

	return pllclk;
}

static void i2c_generate_start(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void i2c_generate_stop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static uint8_t i2c_formAddr_withWrite(uint8_t addr)
{
	addr = addr << 1;
	addr &= ~(1);
	return addr;
}

static uint8_t i2c_formAddr_withRead(uint8_t addr)
{
	addr = addr << 1;
	addr |= 1;
	return addr;
}

static void i2c_clear_addr_flag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyread;
	dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;
	(void)dummyread;
}

static void i2c_close_sendData(I2C_Handle_t *pI2CHandle)
{
	//generate STOP condition if repeated start is not enabled
	if(pI2CHandle->RepeatStart == I2C_REPEATSTART_DI)
		i2c_generate_stop(pI2CHandle->pI2Cx);

	//disable interrupt control bits
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
}

static void i2c_close_receiveData(I2C_Handle_t *pI2CHandle)
{
	//disable interrupt control bits
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxLen = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
}

static void i2c_master_txeinterrupt_handle(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer++;
		pI2CHandle->TxLen--;

		if(pI2CHandle->TxLen == 0)
			i2c_close_sendData(pI2CHandle);
	}
}

static void i2c_master_rxeinterrupt_handle(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxLen == 1)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen > 1)
	{
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		if(pI2CHandle->RxLen == 1)
		{
			if(pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ACK))
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		}
	}
}

/*
 * I2C Peripheral Clock Control API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral Clock Control
 *
 * @brief		- Enable or Disable the peripheral clock of I2C
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral to which
 * 				  clock should be enabled/disabled
 *
 * @param[in]	- (uint8_t) to specify enabling or diabling the clock
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_PclkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * I2C Peripheral Initialize API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral Initialize
 *
 * @brief		- Initialize the specified I2C peripheral with specfied configuration
 *
 * @param[in]	- (I2C_Handle_t) to specify I2C peripheral and required configurations
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t apb1clk;
	uint16_t tempreg = 0;

	//Enable the Peripheral
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

	//Configure the ACK process of the peripheral
	tempreg = (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//Configure the SCLK of the peripheral
	apb1clk = get_apb1_pclk();
	apb1clk = apb1clk / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (apb1clk & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg = ( pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 );
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//calculate CCR and program the CCR field
	uint16_t ccrvalue = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standard Mode
		ccrvalue = get_apb1_pclk() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ( ccrvalue & 0xFFF);
	}else
	{
		//Fast Mode
		tempreg	|= ( 1 << I2C_CCR_FS);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccrvalue =  get_apb1_pclk() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			tempreg	|= ( 1 << I2C_CCR_DUTY);
			ccrvalue =  get_apb1_pclk() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ( ccrvalue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM)
	{
		//as per the I2C standard Traise for standard mode is 1000ns
		tempreg = (get_apb1_pclk() / 1000000U) + 1;
	}
	else
	{
		//as per the I2C standard Traise for standard mode is 300ns
		tempreg = ((get_apb1_pclk() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}



/*
 * I2C Peripheral GetFlagStatus API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral GetFlagStatus
 *
 * @brief		- retrieve and return the specified status flag of SR1
 * 				  register in the I2C peripheral
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t*) to specify the flag name

 *
 * @return		- None
 *
 * @note		- None
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * I2C Peripheral Master SendData API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral MasterSendData
 *
 * @brief		- Act as master and Send data to slave devices
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t*) to store the base address of TxBuffer
 * @param[in]	- (uint32_t) total number of bytes to send
 * @param[in]	- (uint8_t) specifies slave address
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_MasterSendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t repeatStart)
{
	//1. Generate Start Condition
	i2c_generate_start(pI2Cx);

	//2. confirm that start generation is complete by checking SB in SR1
	while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_SB) );

	//3. Send the address of slave with R/nW bit as 0
	uint8_t writeAddr = i2c_formAddr_withWrite(slaveAddress);
	pI2Cx->DR = writeAddr;


	//4. Wait until address completely sent
	while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR) );

	//5. Clear the ADDR flag
	i2c_clear_addr_flag(pI2Cx);

	//6. Send data until length becomes 0
	while(len > 0)
	{
		while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_TxE) );

		pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. Wait for TxE and BTF bits to set
	while(!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_TxE));
	while(!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_BTF));

	//8. Generate i2c stop condition
	if(repeatStart == I2C_REPEATSTART_DI)
		i2c_generate_stop(pI2Cx);
}


/*
 * I2C Master Send data using Interrupt
 */
/****************************************************************
 *
 * @fn			- I2C Master Send Data using Interrupt
 *
 * @brief		- Configure I2C as master and send data to slave using Interrupt
 *
 * @param[in]	- (I2C_Handle_t*) to specify I2C peripheral & its configurations
 * @param[in]	- (uint8_t*) to specify base address of TxBuffer
 * @param[in]	- (uint32_t) to specify length of data to transmit
 * @param[in]	- (uint8_t) to specify slave address
 * @param[in]	- (uint8_t) to specify repeated start
 *
 * @return		- None
 *
 * @note		- None
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeat)
{
	uint8_t state = pI2CHandle->TxRxState;

	if(state == I2C_READY)
	{
		pI2CHandle->pTxBuffer 	= pTxBuffer;
		pI2CHandle->TxLen	  	= len;
		pI2CHandle->SlaveAddr 	= slaveAddr;
		pI2CHandle->RepeatStart = repeat;
		pI2CHandle->TxRxState	= I2C_BUSY_IN_TX;

		//generate START condition
		i2c_generate_start(pI2CHandle->pI2Cx);

		//enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return state;
}


/*
 * I2C Peripheral Master ReceiveData API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral MasterReceiveData
 *
 * @brief		- Act as master and Receive data from slave devices
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t*) to store the base address of RxBuffer
 * @param[in]	- (uint32_t) total number of bytes to Receive
 * @param[in]	- (uint8_t) specifies slave address
 * @param[in]	- (uint8_t) specifies enabling repeated start or not
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress, uint8_t repeatStart)
{
	//1. Generate Start Condition
	i2c_generate_start(pI2Cx);

	//2. confirm that start generation is complete by checking SB in SR1
	while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_SB) );

	//3. Send the address of slave with R/nW bit as 0
	uint8_t readAddr = i2c_formAddr_withRead(slaveAddress);
	pI2Cx->DR = readAddr;

	if(len == 1) {
		//clear the ACK bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		//Wait until address completely sent
		while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR) );

		//Clear the ADDR flag
		i2c_clear_addr_flag(pI2Cx);

		//wait until RxNE is set
		while(!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_RxnE));

		//generate i2c stop condition
		if(repeatStart == I2C_REPEATSTART_DI)
			i2c_generate_stop(pI2Cx);

		//read the Data Register
		*pRxBuffer  = pI2Cx->DR;

		//again Enable the ACK bit
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

		return;
	}

	//Wait until address completely sent
	while( !I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR) );

	//Clear the ADDR flag
	i2c_clear_addr_flag(pI2Cx);

	while(len > 0)
	{
		//wait until RxnE flag is set
		while(!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_RxnE));

		//read the Data Register
		*pRxBuffer = pI2Cx->DR;

		//increment the RxBuffer
		pRxBuffer++;

		//decrease the length
		len--;

		//generate NACK if the second last byte is received
		if(len == 1) {
			//set ACK bit as 0
			pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

			//generate i2c stop condition
			if(repeatStart == I2C_REPEATSTART_DI)
				i2c_generate_stop(pI2Cx);
		}

	}
	pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

}


/*
 * I2C Master Receive data using Interrupt
 */
/****************************************************************
 *
 * @fn			- I2C Master Receive Data using Interrupt
 *
 * @brief		- Configure I2C as master and receive data from slave using Interrupt
 *
 * @param[in]	- (I2C_Handle_t*) to specify I2C peripheral & its configurations
 * @param[in]	- (uint8_t*) to specify base address of TxBuffer
 * @param[in]	- (uint32_t) to specify length of data to receive
 * @param[in]	- (uint8_t) to specify slave address
 * @param[in]	- (uint8_t) to specify repeated start
 *
 * @return		- None
 *
 * @note		- None
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t repeat)
{
	uint8_t state = pI2CHandle->TxRxState;

	if(state == I2C_READY)
	{
		pI2CHandle->pRxBuffer 	= pRxBuffer;
		pI2CHandle->RxLen	  	= len;
		pI2CHandle->SlaveAddr 	= slaveAddr;
		pI2CHandle->RepeatStart = repeat;
		pI2CHandle->TxRxState	= I2C_BUSY_IN_RX;

		//generate START condition
		i2c_generate_start(pI2CHandle->pI2Cx);

		//enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return state;
}


/*
 * I2C Peripheral Slave SendData API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral SlaveSendData
 *
 * @brief		- Act as slave and Send data to master devices
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t) data to be transmit
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}


/*
 * I2C Peripheral Slave ReceiveData API
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral SlaveSendData
 *
 * @brief		- Act as slave and Send data to master devices
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t) data to be transmit
 *
 * @return		- None
 *
 * @note		- None
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return pI2Cx->DR;
}


/*
 * I2C Peripheral Control
 */
/****************************************************************
 *
 * @fn			- I2C Peripheral Control
 *
 * @brief		- Enable/Disable the specified I2C peripheral
 *
 * @param[in]	- (I2C_RegDef_t) to specify I2C peripheral
 * @param[in]	- (uint8_t*) to specify Enable or Disable
 *
 * @return		- None
 *
 * @note		- None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_PE);
}


/*
 * I2C IRQ Configuration
 */
/********************************************************************
 * @fn					- I2C IRQ Configuration
 *
 * @brief				- Enable/Disable the specified IRQNumber in the NVIC
 *
 * @param[in]			- (uint8_t)IRQNumber, the Interrupt that want to be enabled/disabled
 * @param[in]			- (uint8_t)EnorDi, to specify Enable/Disable the Interrupt
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi)
{
	//1. find the offset address & section of NVIC_ISER & NVIC_ICER registers
	uint8_t temp = IRQNumber / 32;
	uint8_t temp1 = IRQNumber % 32;
	if(IRQEnorDi == ENABLE)
	{
		// if IRQ is want to enable then make the corresponding ISER bit to 1 & ICER bit to 0
		*( NVIC_ISER_BASEADDR + (temp * 4) ) |= ( 1 << temp1 );
		*( NVIC_ICER_BASEADDR + (temp * 4) ) &= ~( 1 << temp1 );
	}
	else
	{
		// if IRQ is want to disable then make the corresponding ICER bit to 1 & ISER bit to 0
		*( NVIC_ICER_BASEADDR + (temp * 4) ) |= ( 1 << temp1 );
		*( NVIC_ISER_BASEADDR + (temp * 4) ) &= ~( 1 << temp1 );
	}
}


/*
 * I2C IRQ Priority Configuration
 */
/********************************************************************
 * @fn					- SPI IRQ Priority Configuration
 *
 * @brief				- Configure the specified priority to the specified IRQNumber
 *
 * @param[in]			- (uint8_t)IRQNumber, the Interrupt to which priority want to be configured
 * @param[in]			- (uint8_t)IRQPriority, priority to be set
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find the offset address & section of NVIC_IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//2. find out the No.of.times data to be shifted to load that in the proper section & proper bit field of register
	uint8_t shiftAmount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//3. load the IRQPriority in to the register
	*( NVIC_IPR_BASEADDR + iprx ) |= (IRQPriority << shiftAmount);
}


/*
 * I2C Event Handling
 */
/********************************************************************
 * @fn					- I2C EVTHandling
 *
 * @brief				- Handle the interrupt events triggered in the USART peripheral
 *
 * @param[in]			- (I2C_Handle_t*) Handle structure to handle the event
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void I2C_EVT_Handling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1;

	//check for SB (start bit sent) event
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_SB);
	//handle the start bit sent event
	if(temp1)
	{
		//send slave address after the SB bit is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			uint8_t writeAddr = i2c_formAddr_withWrite(pI2CHandle->SlaveAddr);
			pI2CHandle->pI2Cx->DR = writeAddr;
		}
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			uint8_t readAddr = i2c_formAddr_withRead(pI2CHandle->SlaveAddr);
			pI2CHandle->pI2Cx->DR = readAddr;
		}
	}

	//check ADDR flag
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_ADDR);
	//handle the ADDR flag event
	if(temp1)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxLen == 1)
			{
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
			}
		}
		i2c_clear_addr_flag(pI2CHandle->pI2Cx);
	}

	//check BTF flag
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_BTF);
	//handle BTF flag
	if(temp1)
	{
		//if the mode was transmission and user likes to disable repeat start
		//without sending further data, then generate stop
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->TxLen == 0)
			{
				//reset the handle variables
				i2c_close_sendData(pI2CHandle);
				//call application callback
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVT_TX_CMPLT);
			}
		}
	}

	//check the STOPF flag
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_STOPF);
	if(temp1)
	{
		//clear the STOPF flag
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//Notify the application about occurrence of stop condition
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVT_STOP_DETECT);
	}

	//check for TxE flag
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_TxE);
	if(temp1)
	{
		if(pI2CHandle->pI2Cx->SR2 & I2C_FLAG_MSL)
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				i2c_master_txeinterrupt_handle(pI2CHandle);

			if(pI2CHandle->TxLen == 0)
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVT_TX_CMPLT);
		}
		else
		{
			//device acting as slave
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_MASTER_REQUEST_DATA);
			}
		}
	}

	//check for RxNE flag
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_FLAG_RxnE);
	if(temp1)
	{
		if(pI2CHandle->pI2Cx->SR2 & I2C_FLAG_MSL)
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				i2c_master_rxeinterrupt_handle(pI2CHandle);

			if(pI2CHandle->RxLen == 0){
				i2c_close_receiveData(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVT_RX_CMPLT);
			}

		}
		else
		{
			//device acting as slave
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_MASTER_SENT_DATA);
			}
		}
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{

}
