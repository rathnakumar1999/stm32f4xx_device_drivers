/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 16-May-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx_gpio_driver.h"


/*
 * GPIO Peripheral Clock Control API
 */
/********************************************************************
 * @fn					- GPIO Peripheral Clock Control
 *
 * @brief				- This Function Enables or Disables Peripheral Clock of the given GPIO port
 *
 * @param[in]			- Base Address of the GPIO peripheral
 * @param[in]			- Enable or Disable Macro
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 *
 */
void GPIO_PclkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}


/*
 * GPIO Initialize API
 */
/********************************************************************
 * @fn					- GPIO Initialize
 *
 * @brief				- This Function Initialize the specified GPIO port for the specified pin number
 * 						  with the configurations made by the application code
 *
 * @param[in]			- GPIO Handle Structure
 *
 * @return				- NONE
 *
 * @note				- Using configuration structure the configuration must be done in the application code
 * 						  by the user and it should be passed to this API as GPIO_Handle_t with GPIO_Reg_def_t,
 * 						  so that this API will configure the specified GPIO pin in the
 * 						  specified GPIO port as per the configurations defined in the GPIO_Config_t
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//enable the Peripheral clock
	GPIO_PclkControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the Mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IRQ_FT)
		{
			//1. Configure the Corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the Corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IRQ_RT)
		{
			//1. Configure the Corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the Corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IRQ_RFT)
		{
			//1. Configure the Corresponding FTSR & RTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the pull-up/pull-dn register
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the Output Type Register
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure Alternate Function Register
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}

}


/*
 * GPIO DeInitialize API
*/
/********************************************************************
 * @fn					- GPIO DeInitialize
 *
 * @brief				- This Function DeInitialize the specified GPIO port
 *
 * @param[in]			- GPIO port to DeInitialize
 *
 * @return				- NONE
 *
 * @note				- This function resets the all of the registers belongs to the specified GPIO port
 *
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


/*
 * GPIO Read from Input pin
 */
/********************************************************************
 * @fn					- GPIO Input Pin Read
 *
 * @brief				- Reads and returns the value of the specified input pin
 *
 * @param[in]			- GPIO_RegDef_t to specify the GPIO port
 * @param[in]			- (unit8_t)PinNumber, specify the pin to read
 *
 * @return				- (uint8_t)current value of the specified Input pin
 *
 * @note				- NONE
 *
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & (0x00000001);

	return value;
}


/*
 * GPIO Read from Input port
 */
/********************************************************************
 * @fn					- GPIO Port to Read
 *
 * @brief				- Reads and returns the current value of all the pins which belongs
 * 						  to the specified GPIO port
 *
 * @param[in]			- GPIO_RegDef_t to specify the GPIO port
 *
 * @return				- (uint16_t)current value of pins belong to specified GPIO port
 *
 * @note				- NONE
 *
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value =(uint16_t)pGPIOx->IDR;

	return value;
}


/*
 * GPIO Write to Output pin
 */
/********************************************************************
 * @fn					- GPIO Output Pin Write
 *
 * @brief				- Writes the specified value to the specified pin of GPIO port
 *
 * @param[in]			- GPIO_RegDef_t to specify the GPIO port
 * @param[in]			- (unit8_t)PinNumber, specify the pin to write
 * @param[in]			- (unit8_t)value, specify the value to write
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write value 1 to the corresponding bit field of the specified pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write value 0 to the corresponding bit field of the specified pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}


/*
 * GPIO Write to Output port
 */
/********************************************************************
 * @fn					- GPIO Output Port Write
 *
 * @brief				- Writes the specified value to the specified GPIO port
 *
 * @param[in]			- GPIO_RegDef_t to specify the GPIO port
 * @param[in]			- (unit8_t)value, specify the value to write
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}


/*
 * GPIO pin Toggle
 */
/********************************************************************
 * @fn					- GPIO Output Pin Toggle
 *
 * @brief				- Toggles the specified pin of specified GPIO port
 *
 * @param[in]			- GPIO_RegDef_t to specify the GPIO port
 * @param[in]			- (unit8_t)PinNumber, specify the pin to toggle
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * GPIO IRQ Configuration
 */
/********************************************************************
 * @fn					- GPIO IRQ Configuration
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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi)
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
 * GPIO IRQ Priority Configuration
 */
/********************************************************************
 * @fn					- GPIO IRQ Priority Configuration
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * GPIO IRQ Handling
 */
/********************************************************************
 * @fn					- GPIO IRQ Handling
 *
 * @brief				- Clear the Pending register of  EXTI block after interrupt occurs
 *
 * @param[in]			- (uint8_t)PinNumber, to specify the bit position in pending register
 *
 * @return				- NONE
 *
 * @note				- NONE
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if( EXTI->PR & ( 1 << PinNumber) ){
		//Clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}

