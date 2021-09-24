/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 16-May-2021
 *      Author: RathnakumarKannan
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;						/*!< Possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;						/*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;						/*!< Possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;				/*!< Possible values from @GPIO_PU/PD_CONFIG >*/
	uint8_t GPIO_PinOPType;						/*!< Possible values from @GPIO_PIN_OUTPUT_TYPES >*/
	uint8_t GPIO_PinAltFunMode;					/*!< Possible values from @GPIO_PIN_ALTERNATE_FUNCTIONS >*/
}GPIO_PinConfig_t;


/*
 * Handle Structure of GPIO Peripheral
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!< Base Address of GPIO port to which the specified pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig; 	/*!< GPIO pin configuration settings >*/
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO - Possible Pin Numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO - Possible Pin Modes
 */
#define GPIO_MODE_IN			0		/*!< GPIO pin Mode Input >*/
#define GPIO_MODE_OUT			1		/*!< GPIO pin Mode Output >*/
#define GPIO_MODE_ALTFN 		2		/*!< GPIO pin Mode Alternate Function >*/
#define GPIO_MODE_ANALOG 		3		/*!< GPIO pin Mode Analog >*/
#define GPIO_MODE_IRQ_FT		4		/*!< GPIO pin Mode Interrupt Falling Edge Trigger >*/
#define GPIO_MODE_IRQ_RT		5		/*!< GPIO pin Mode Interrupt Raising Edge Trigger >*/
#define GPIO_MODE_IRQ_RFT		6		/*!< GPIO pin Mode Interrupt Raising & Falling Edge Trigger >*/

/*
 * @GPIO_PIN_SPEED
 * GPIO - Possible Pin Speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERYHIGH		3

/*
 * @GPIO_PU/PD_CONFIG
 * GPIO - Possible Pin Configuration (pull_up/pull_down)
 */
#define GPIO_PIN_NOPUPD				0		/*!< GPIO pin No Pull Up/Pull Down >*/
#define GPIO_PIN_PU					1		/*!< GPIO pin with Pull Up>*/
#define GPIO_PIN_PD					2		/*!< GPIO pin with Pull Down >*/

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO - Possible Output Types
 */
#define GPIO_OUTPUT_PP		0				/*!< GPIO pin Output Type Push Pull >*/
#define GPIO_OUTPUT_OD		1				/*!< GPIO pin Output Type Open Drain >*/

/*************************************************************************
 * 						API Supported by This Driver
 *************************************************************************/
/*
 * GPIO Peripheral Clock Control
 */
void GPIO_PclkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Initialize & DeInitialize
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read & Write to Pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO IRQ Configuration & Handle
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQEnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
