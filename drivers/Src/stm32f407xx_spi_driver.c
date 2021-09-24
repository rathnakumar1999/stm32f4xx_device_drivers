/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 13-Jun-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx_spi_driver.h"

//definition of helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_ovrerr_interrupt_handle(SPI_Handle_t* pSPIHandle);

/*
 * SPI Peripheral Clock Control API
 */
/********************************************************************
 * @fn					- SPI Peripheral Clock Control
 *
 * @brief				- Enables or Disables Peripheral Clock of the given SPI peripheral
 *
 * @param[in]			- Base Address of the SPI peripheral
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 *
 */
void SPI_PclkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*
 * SPI Peripheral Initialize API
 */
/********************************************************************
 * @fn					- SPI Peripheral Initialize
 *
 * @brief				- Initialize the specified SPI peripheral with the given configuration
 *
 * @param[in]			- Handle structure of SPI peripheral
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable the Peripheral clock for SPI
	SPI_PclkControl(pSPIHandle->pSPIx, ENABLE);

	//first Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPI_PeripheralConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Do the Bus Configuration
	if(pSPIHandle->SPI_PeripheralConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI bit field must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_PeripheralConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set the BIDI bit field
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_PeripheralConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear the BIDI bit field
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//set the RXOnly Bit field
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI Peripheral Clock Speed using Baud Rate bits
	tempreg |= (pSPIHandle->SPI_PeripheralConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the Data Frame Format bit field
	tempreg |= (pSPIHandle->SPI_PeripheralConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the Clock polarity using CPOL bit field
	tempreg |= (pSPIHandle->SPI_PeripheralConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the Clock Phase using CPHA bit field
	tempreg |= (pSPIHandle->SPI_PeripheralConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. Configure the Software Slave Management bit field
	tempreg |= (pSPIHandle->SPI_PeripheralConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*
 * SPI Peripheral De-Initialize API
 */
/********************************************************************
 * @fn					- SPI Peripheral De-Initialize
 *
 * @brief				- De-Initialize the specified SPI peripheral
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral to De-Initialize
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName)
{
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * SPI Peripheral Send Data API
 */
/********************************************************************
 * @fn					- SPI SendData
 *
 * @brief				- used to send data from the MCU
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral
 * @param[in]			- (uint8_t*)pTxBuffer, pointer to the buffer which holds data to send
 * @param[in]			- (uint32_t)len, Length of data to send
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. wait until TXE bit is set
		while((SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET) );

		//2. check the DFF bit
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			//16 bit DFF
			//Load 2 Bytes of data to DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);

			//reduce the length by two bytes
			len--;
			len--;

			//increment the pTxBuffer address by two
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			//Load 1 Byte of data to DR register
			pSPIx->DR = *pTxBuffer;

			//reduce the length by 1 byte
			len--;

			//increment the pTxBuffer address by one
			pTxBuffer++;
		}

	}
}



/*
 * SPI Peripheral Receive Data API
 */
/********************************************************************
 * @fn					- SPI ReceiveData
 *
 * @brief				- used to received data from the communicating device
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral
 * @param[in]			- (uint8_t*)pRxBuffer, pointer to the buffer which holds received data
 * @param[in]			- (uint32_t)len, Length of data to receive
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. wait until RXNE bit is set
		while((SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE) == FLAG_RESET) );

		//2. check the DFF bit
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			//16 bit DFF
			//Load 2 Bytes of data to RxBuffer from DR register
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;

			//reduce the length by two bytes
			len--;
			len--;

			//increment the pTxBuffer address by two
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			//Load 1 Byte of data to RxBuffer from DR register
			*pRxBuffer = pSPIx->DR;

			//reduce the length by 1 byte
			len--;

			//increment the pTxBuffer address by one
			pRxBuffer++;
		}
	}

}



/*
* SPI Peripheral Send Data using Interrupt API
*/
/********************************************************************
* @fn					- SPI SendDataInterrupt
*
* @brief				- used to send data from the communicating device using Interrupt
*
* @param[in]			- (SPI_Handle_t*)pSPIHandle, to specify the SPI Peripheral & confiugre it
* @param[in]			- (uint8_t*)pTxBuffer, pointer to the buffer which holds received data
* @param[in]			- (uint32_t)len, Length of data to receive
*
* @return				- NONE
*
* @note					- NONE
*
*/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_STATUS_BSY_IN_TX)
	{
		//1. store the based address of TxBuffer and its length in Handle structure
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen 	  = len;

		//2. make application status of the particular SPI to SPI_BUSY_IN_TX
		pSPIHandle->TxState = SPI_STATUS_BSY_IN_TX;

		//3. enable the TXIE bit to enable interrupt while TXE bit in SR register is set
		pSPIHandle->pSPIx->CR2 = ( 1 << SPI_CR2_TXEIE);
	}
	return state;
}


/*
 * SPI Peripheral Receive Data API using Interrupt
 */
/********************************************************************
 * @fn					- SPI ReceiveDataInterrupt
 *
 * @brief				- used to received data from the communicating device using interrupt
 *
 * @param[in]			- (SPI_Handle_t*)pSpIHandle, to specify the SPI Peripheral & its configuration
 * @param[in]			- (uint8_t*)pRxBuffer, pointer to the buffer which holds received data
 * @param[in]			- (uint32_t)len, Length of data to receive
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_STATUS_BSY_IN_RX)
	{
		//1. store the based address of RxBuffer and its length in Handle structure
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen 	  = len;

		//2. make application status of the particular SPI to SPI_BUSY_IN_RX
		pSPIHandle->RxState = SPI_STATUS_BSY_IN_RX;

		//3. enable the TXIE bit to enable interrupt while TXE bit in SR register is set
		pSPIHandle->pSPIx->CR2 = ( 1 << SPI_CR2_RXNEIE);
	}
	return state;
}


/*
 * SPI Peripheral Control API
 */
/********************************************************************
 * @fn					- SPI PeripheralControl
 *
 * @brief				- Enable/Disable the specified SPI peripheral
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral
 * @param[in]			- (uint8_t*)EnorDi, to specify enable/disable
 *
 * @return				- NONE
 *
 * @note				- to enable the SPI peripheral the SPE bit in CR1 register must be enabled
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * SPI Peripheral SSI Configuration
 */
/********************************************************************
 * @fn					- SPI SSIConfig
 *
 * @brief				- SET/RESET the SSI bit to control slave select
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral
 * @param[in]			- (uint8_t*)EnorDi, to specify enable/disable
 *
 * @return				- NONE
 *
 * @note				- to enable the slave make SSI to 0, for master make SSI to 1
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*
 * SPI Peripheral SSOE Configuration
 */
/********************************************************************
 * @fn					- SPI SSOEConfig
 *
 * @brief				- SET/RESET the SSOE bit to control slave select output enable
 *
 * @param[in]			- (SPI_RegDef_t*)pSPIx, to specify the SPI Peripheral
 * @param[in]			- (uint8_t*)EnorDi, to specify enable/disable
 *
 * @return				- NONE
 *
 * @note				- to enable the NSS pin make SSOE to 1
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * SPI IRQ Configuration
 */
/********************************************************************
 * @fn					- SPI IRQ Configuration
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
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi)
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
 * SPI IRQ Priority Configuration
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * SPI IRQ Handling
 */
/********************************************************************
 * @fn					- SPI IRQ Handling
 *
 * @brief				-
 *
 * @param[in]			- (SPI_Handle_t*)pSPIHandle
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//check for Tx related interrupt
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	//if interrupt is because of Tx related flags handle transmission
	if(temp1 && temp2)
	{
		//handle TXE interrupt
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for Rx related interrupt
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	//if interrupt is because of Rx related flags handle reception
	if(temp1 && temp2)
	{
		//handle RXE interrupt
		spi_rxe_interrupt_handle(pSPIHandle);
	}

	//check for Overrun related interrupt
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	//if interrupt is because of overrun error handle the error
	if(temp1 && temp2)
	{
		spi_ovrerr_interrupt_handle(pSPIHandle);
	}

}


/*
 * SPI Close Transmission
 */
/********************************************************************
 * @fn					- SPI_CloseTransmission
 *
 * @brief				- used to close SPI transmission if interrupt mode transmission
 * 						  is enabled
 *
 * @param[in]			- (SPI_Handle_t*)pSPIHandle
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_STATUS_READY;
}


/*
 * SPI Close Reception
 */
/********************************************************************
 * @fn					- SPI_CloseReception
 *
 * @brief				- used to close SPI Reception if interrupt mode reception
 * 						  is enabled
 *
 * @param[in]			- (SPI_Handle_t*)pSPIHandle
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_STATUS_READY;
}


/*
 * SPI Overrun Flag Clear
 */
/********************************************************************
 * @fn					- SPI_OVRFlagClear
 *
 * @brief				- used to clear the overrun flag if OVR flag set
 * 						  during communication
 *
 * @param[in]			- (SPI_Regdef_t*)pSPIx, spi peripheral to which ovr flag to be cleared
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void SPI_ClearOVRFlagClear(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


//helper function implementation
static void spi_txe_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	if(pSPIHandle->TxLen != 0)
	{
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			//load 16 bit data to data register
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);

			//decrease the length of 2 bytes
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;

			//increase the Tx buffer pointer
			(uint16_t*)pSPIHandle->pTxBuffer++;

		}
		else
		{
			//load 8 bit data to data register
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

			//decrease the length
			pSPIHandle->TxLen--;

			//increase the Tx buffer pointer
			pSPIHandle->pTxBuffer++;
		}

		if(pSPIHandle->TxLen == 0)
		{
			/*
			 * when Tx length becomes 0, then close spi transmission and inform the application that
			 * Transmission is over
			 */
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVT_TX_CMPLT);

		}
	}
}

static void spi_rxe_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	if(pSPIHandle->RxLen != 0)
	{
		if( pSPIHandle->pSPIx->CR1 & SPI_CR1_DFF )
		{
			//load data from DR register to the RxBuffer
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;

			//decrease the RxLen
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;

			//increase the Rx buffer pointer
			(uint16_t*)pSPIHandle->pRxBuffer++;

		}
		else
		{
			//load data from DR register to the RxBuffer
			*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;

			//decrease the RxLen
			pSPIHandle->RxLen--;

			//increase the Rx buffer pointer
			pSPIHandle->pRxBuffer++;
		}

		if(pSPIHandle->RxLen == 0)
		{

			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVT_RX_CMPLT);
		}
	}
}

static void spi_ovrerr_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	uint8_t temp;
	//1. clear the overrun flag if SPI is not busy
	if(pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_BSY ) )
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//2. inform the application about ovrrun error
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVT_OVRERR);
	(void)temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	/*This is the weak implementation application can override this function*/
}
