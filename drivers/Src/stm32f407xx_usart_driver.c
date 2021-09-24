/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 14-Sep-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_usart_driver.h"

static void set_baud_rate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


static void set_baud_rate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t apbClk;						//variable to store the APB bus clock
	uint32_t usartDiv;						//variable to store divison factor
	uint32_t mantissaPart, exponentPart;	//components of usart divison factor

	uint32_t tempreg = 0;

	//Get Peripheral Clock information
	if(pUSARTx == USART1 || pUSARTx == USART6)
		apbClk = Get_APB2_Pclk();
	else
		apbClk = Get_APB1_Pclk();

	//Get the over-sampling rate
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
		usartDiv = (25 * apbClk) / (2 * BaudRate);
	else
		usartDiv = (25 * apbClk) / (4 * BaudRate);

	//calculate the mantissa part
	mantissaPart = usartDiv / 100;
	tempreg |= (mantissaPart << 4);

	//calculate the exponent
	exponentPart = (usartDiv - (mantissaPart * 100));

	//Calculate the fraction part
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
		exponentPart = ( ((exponentPart * 8) + 50) / 100) & ((uint8_t)0x07);
	else
		exponentPart = ( ((exponentPart * 16) + 50) / 100) & ((uint8_t)0x0F);

	tempreg |= exponentPart;

	//load calculated value to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 * USART Peripheral Clock Control
 */
/*********************************************************************************
 *
 * @fn			- USART_PclkControl
 *
 * @brief		- Enable/Disable Clock for specified USART peripheral
 *
 * @param[in]	- (USART_RegDef_t) to specify USART peripheral
 * @param[in]	- (uint8_t) to specify Enable/Disable
 *
 * @return		- None
 *
 * @note		- None
 */
void USART_PclkControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}



/*
 * USART Initialize
 */
/*********************************************************************************
 *
 * @fn			- USART_Init
 *
 * @brief		- Initialize the specified USART peripheral with the configuration
 * 				  specified in the handle_t
 *
 * @param[in]	- (USART_Handle_t) to specify USART peripheral & its configuration
 *
 * @return		- None
 *
 * @note		- None
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;
	uint8_t check = 0;

	//Enable the Peripheral clock of the USART peripheral
	USART_PclkControl(pUSARTHandle->pUSARTx, ENABLE);

	//1. Enable the USART Peripheral
	USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);

	//2. Configure the mode of communication
	check = pUSARTHandle->UsartConfig.USART_Mode;
	if(check == USART_MODE_TX_ONLY || check == USART_MODE_RXTX_BOTH)
		tempreg |= (1 << USART_CR1_TE);
	if(check == USART_MODE_RX_ONLY || check == USART_MODE_RXTX_BOTH)
		tempreg |= (1 << USART_CR1_RE);

	//3. Configure parity setting
	if(pUSARTHandle->UsartConfig.USART_Parity != USART_PARITY_DI)
	{
		//enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
		//configure the type of parity
		if(pUSARTHandle->UsartConfig.USART_Parity == USART_PARITY_ODD)
			tempreg |= (1 << USART_CR1_PS);
		else if(pUSARTHandle->UsartConfig.USART_Parity == USART_PARITY_EVEN)
			tempreg &= ~(1 << USART_CR1_PS);
	}

	//4. Configure the word length
	tempreg |= (pUSARTHandle->UsartConfig.USART_DataBits << USART_CR1_M);

	//5. Configure the over-sampling rate
	if(pUSARTHandle->UsartConfig.USART_SamplingRate == USART_SAMPLEBY_8)
		tempreg |= ( 1 << USART_CR1_OVER8);
	else
		tempreg &= ~( 1 << USART_CR1_OVER8);

	//6. Load the CR1 register with proper configuration settings
	pUSARTHandle->pUSARTx->CR1 |= tempreg;

	//7. Configure Number of Stop bits
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->UsartConfig.USART_StopBits << USART_CR2_STOP);

	//8. Configure the Hardware Flow Control
	check = pUSARTHandle->UsartConfig.USART_HWFlowControl;
	if(check != USART_HW_FLOWCNTRL_DI)
	{
		if(check == USART_HW_FLOWCNTRL_CTS_ONLY || check == USART_HW_FLOWCNTRL_CTS_RTS_BOTH)
			pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_CTSE);
		if(check == USART_HW_FLOWCNTRL_RTS_ONLY || check == USART_HW_FLOWCNTRL_CTS_RTS_BOTH)
			pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_RTSE);
	}


	//9. Set Baud Rate
	set_baud_rate(pUSARTHandle->pUSARTx, pUSARTHandle->UsartConfig.USART_BaudRate);
}


/*
 * USART Peripheral Control
 */
/*********************************************************************************
 *
 * @fn			- USART_PeripheralControl
 *
 * @brief		- Enable/Disable the specified USART peripheral
 *
 * @param[in]	- (USART_RegDef_t) to specify USART peripheral
 * @param[in]	- (uint8_t) to specify Enable/Disable
 *
 * @return		- None
 *
 * @note		- None
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE);
	}
	else if(EnorDi == DISABLE)
	{
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE);
	}
}


/*
 * UART Get Flag Status
 */
/*********************************************************************************
 *
 * @fn			- USART_GetFlagStatus
 *
 * @brief		- return the specified flag status of the specified peripheral
 *
 * @param[in]	- (USART_RegDef_t) to specify USART peripheral
 * @param[in]	- (uint8_t) to specify the flag status to be returned
 *
 * @return		- status of the flag
 *
 * @note		- None
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * UART SendData
 */
/*********************************************************************************
 *
 * @fn			- USART_SendData
 *
 * @brief		- send data via the specified USART peripheral
 *
 * @param[in]	- (USART_Handle_t) to specify USART peripheral & its configuration
 * @param[in]	- (uint8_t*) to specify the base address of TxBuffer
 * @param[in]	- (uint32_t) to specify the length of data to send
 *
 * @return		- status of the flag
 *
 * @note		- Blocking API
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;

	while(len > 0)
	{
		//wait until TXE Flag reset
		while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) == FLAG_RESET);

		if(pUSARTHandle->UsartConfig.USART_DataBits == USART_9BITS)
		{
			pData = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = ( (*pData) & (0x1FF) );

			if(pUSARTHandle->UsartConfig.USART_Parity == USART_PARITY_DI)
			{
				pTxBuffer++;
				pTxBuffer++;
				len--;
				len--;
			}
			else
			{
				pTxBuffer++;
				//decrease the length
				len--;
			}

		}
		else
		{
			//load data to send in DR register
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer) & 0xFF;

			//increase the TxBuffer
			pTxBuffer++;

			//decrease the length
			len--;
		}


		//check for the last byte transmission
		if(len == 0)
		{
			while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) == FLAG_RESET);

			//clear the TC bit
			pUSARTHandle->pUSARTx->SR &= ~(USART_FLAG_TC);
		}
	}
}


/*
 * UART SendData using Interrupt
 */
/*********************************************************************************
 *
 * @fn			- USART_SendDataIT
 *
 * @brief		- send data via the specified USART peripheral using interrupts
 *
 * @param[in]	- (USART_Handle_t*) to specify USART peripheral & its configuration
 * @param[in]	- (uint8_t*) to specify the base address of TxBuffer
 * @param[in]	- (uint32_t) to specify the length of data to send
 *
 * @return		- status of the flag
 *
 * @note		- Blocking API
 */
void USART_SendDataIT(USART_Handle_t *pUSARTHandlex, uint8_t *pTxBuffer, uint32_t len)
{
	uint32_t tempreg = 0;

	pUSARTHandlex->pTxBuffer = pTxBuffer;
	pUSARTHandlex->TxState   = USART_BSY_IN_TX;
	pUSARTHandlex->TxLen	 = len;

	//Enable the Tx related interrupt flags
	tempreg |= ( 1 << USART_CR1_TXEIE);
	tempreg |= ( 1 << USART_CR1_TCIE);

	//If parity is used enable parity related interrupt flag
	if(pUSARTHandlex->UsartConfig.USART_Parity != USART_PARITY_DI)
	{
		if(USART_GetFlagStatus(pUSARTHandlex->pUSARTx, USART_CR1_PEIE) == FLAG_RESET)
			tempreg |= ( 1 << USART_CR1_PEIE);
	}

	pUSARTHandlex->pUSARTx->CR1 |= tempreg;

	uint8_t check = pUSARTHandlex->UsartConfig.USART_HWFlowControl;
	if(check == USART_HW_FLOWCNTRL_CTS_ONLY || check == USART_HW_FLOWCNTRL_CTS_RTS_BOTH)
	{
		if(USART_GetFlagStatus(pUSARTHandlex->pUSARTx, USART_CR3_CTSIE) == FLAG_RESET)
			pUSARTHandlex->pUSARTx->CR3 |= ( 1 << USART_CR3_CTSIE);
	}
}


/*
 * UART ReceiveData
 */
/*********************************************************************************
 *
 * @fn			- USART_ReceiveData
 *
 * @brief		- receive data via the specified USART peripheral
 *
 * @param[in]	- (USART_RegDef_t) to specify USART peripheral
 * @param[in]	- (uint8_t*) to specify the base address of RxBuffer
 * @param[in]	- (uint32_t) to specify the length of data to receive
 *
 * @return		- status of the flag
 *
 * @note		- Blocking API
 */
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len)
{
	//Enable the Receive Block of USART Peripheral
	pUSARTx->CR1 |= ( 1 << USART_CR1_RE);

	while(len > 0)
	{
		//wait until RXNE bit is set
		while(USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE) == FLAG_RESET);

		//read the data from DR register
		*pRxBuffer = pUSARTx->DR;

		//increase the RxBuffer
		pRxBuffer++;

		//decrease the length
		len--;
	}

	//Disable the Receive Block of USART peripheral
	pUSARTx->CR1 &= ~( 1 << USART_CR1_RE);
}



/*
 * UART ReceiveData using Interrupt
 */
/*********************************************************************************
 *
 * @fn			- USART_ReceiveDataIT
 *
 * @brief		- receive data via the specified USART peripheral using interrupts
 *
 * @param[in]	- (USART_Handle_t*) to specify USART peripheral & its configurations
 * @param[in]	- (uint8_t*) to specify the base address of TxBuffer
 * @param[in]	- (uint32_t) to specify the length of data to send
 *
 * @return		- status of the flag
 *
 * @note		- Blocking API
 */
void USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	pUSARTHandle->pRxBuffer = pRxBuffer;
	pUSARTHandle->RxState	= USART_BSY_IN_RX;
	pUSARTHandle->RxLen		= len;

	uint32_t tempreg = 0;

	tempreg |= ( 1 << USART_CR1_RXNEIE);

	//If parity is used enable parity related interrupt flag
	if(pUSARTHandle->UsartConfig.USART_Parity != USART_PARITY_DI)
	{
		if(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_CR1_PEIE) == FLAG_RESET)
			tempreg |= ( 1 << USART_CR1_PEIE);
	}

	pUSARTHandle->pUSARTx->CR1 |= tempreg;
}


/*
 * UART DeInit
 */
/*********************************************************************************
 *
 * @fn			- USART_DeInit
 *
 * @brief		- de-initialize the specified USART peripheral
 *
 * @param[in]	- (USART_RegDef_t) to specify USART peripheral
 *
 * @return		- None
 *
 * @note		- None
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}


/*
 * USART IRQ Configuration
 */
/********************************************************************
 * @fn					- USART IRQ Configuration
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
void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi)
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
 * USART IRQ Priority Configuration
 */
/********************************************************************
 * @fn					- USART Priority Configuration
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * USART Event Handling
 */
/********************************************************************
 * @fn					- USART EVTHandling
 *
 * @brief				- Handle the interrupt events triggered in the USART peripheral
 *
 * @param[in]			- (USART_Handle_t*) Handle structure to handle the event
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void USART_EVTHandling(USART_Handle_t *pUSARTHandle)
{
	uint8_t temp = 0;

	//check for Transmit Empty (TxE) flag
	temp = (pUSARTHandle->pUSARTx->SR) & (USART_FLAG_TXE);
	if(temp)
	{
		if(pUSARTHandle->TxLen > 0)
		{
			//Send Data
			pUSARTHandle->pUSARTx->DR = *pUSARTHandle->pTxBuffer;
			pUSARTHandle->pTxBuffer++;
			pUSARTHandle->TxLen--;
		}
	}

	//check for Transmit Complete
	temp = (pUSARTHandle->pUSARTx->SR) & (USART_FLAG_TC);
	if(temp)
	{
		//clear the TC bit
		pUSARTHandle->pUSARTx->SR &= ~(USART_FLAG_TC);

		//clear the transmit interrupt enable bits
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

		//reset variables
		pUSARTHandle->pRxBuffer = NULL;
		pUSARTHandle->TxLen = 0;
		pUSARTHandle->TxState = USART_READY;

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVT_TX_CMPLT);
	}

	//check for Receive Not Empty (RxNE) flag
	temp = (pUSARTHandle->pUSARTx->SR) & (USART_FLAG_RXNE);
	if(temp)
	{
		//read the received Data
		pUSARTHandle->pTxBuffer = (uint8_t*)pUSARTHandle->pUSARTx->DR;
		pUSARTHandle->pRxBuffer++;
		pUSARTHandle->RxLen--;

		if(pUSARTHandle->RxLen == 0)
		{
			pUSARTHandle->pRxBuffer = NULL;
			pUSARTHandle->RxLen = 0;
			pUSARTHandle->RxState = USART_READY;

			//clear the receive interrupt enable bits
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);

			USART_ApplicationEventCallback(pUSARTHandle, USART_EVT_RX_CMPLT);
		}
	}

	//check for CTS interrupt
	temp = (pUSARTHandle->pUSARTx->SR) & (USART_FLAG_CTS);
	if(temp)
	{
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVT_CTS_ASSERTED);
	}
}


__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{

}
