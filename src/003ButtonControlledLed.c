/*
 * 003ButtonControlledLed.c
 *
 *  Created on: 23-May-2021
 *      Author: RathnakumarKannan
 */
#include "stm32f407xx.h"

void delay()
{
	for(int i = 0; i < 50000/2; i++);
}

int main(void){

	uint8_t PushButtonValue;
	bool flag = false;

	GPIO_Handle_t GpioPushButton;

	GpioPushButton.pGPIOx = GPIOA;

	GpioPushButton.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_IN;
	GpioPushButton.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
	GpioPushButton.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_NOPUPD;
	GpioPushButton.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_HIGH;

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber				= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OUTPUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PIN_NOPUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_SPEED_HIGH;

	GPIOD_PCLK_EN();

	GPIOA_PCLK_EN();

	GPIO_Init(&GpioLed);

	GPIO_Init(&GpioPushButton);

	while(1)
	{
		PushButtonValue = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0);
		delay();
		if(PushButtonValue == 1 && !flag)
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			flag = true;
		}
		else if(PushButtonValue == 0){
			flag = false;
		}
	}
}

