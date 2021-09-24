/*
 * 004ButtonInterrupt.c
 *
 *  Created on: 06-Jun-2021
 *      Author: RathnakumarKannan
 */
#include "stm32f407xx.h"

void delay(){
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void){

	//1. Create Handle Structures for LED & User Button connected in the GPIO pin
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	//2. Initialize the Handle structure members to '0'
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioButton));

	GPIOD_PCLK_EN();

	//3. Assign the GPIO port to which LED is connected
	GpioLed.pGPIOx = GPIOD;

	//4. Do the configurations required for pin to which LED is connected
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   		= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OUTPUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_NOPUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_HIGH;

	//5. Initialize the GPIO pin with configurations done in the previous steps
	GPIO_Init(&GpioLed);

	//6. Assign the GPIO port to which Button is connected
	GpioButton.pGPIOx = GPIOD;

	//7. Do the configurations required for pin to which Button is connected
	GpioButton.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_6;
	GpioButton.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IRQ_RT;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_HIGH;

	//8. Initialize the GPIO pin with configurations done in the previous steps
	GPIO_Init(&GpioButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}


