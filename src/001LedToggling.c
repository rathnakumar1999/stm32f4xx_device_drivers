/*
 * 001LedToggling.c
 *
 *  Created on: 23-May-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx.h"

void delay(uint16_t delayTime)
{
	for(uint16_t i = 0; i < delayTime; i++){
		for(uint16_t j = 0; j < delayTime; j++);
	}
}

int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   		= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OUTPUT_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_NOPUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_HIGH;

	GPIOD_PCLK_EN();

	GPIO_Init(&GpioLed);

	while(1){

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay(400);

	}
	return 0;

}
