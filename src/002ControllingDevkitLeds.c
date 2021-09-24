/*
 * 002ControllingDevkitLeds.c
 *
 *  Created on: 23-May-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx.h"

#define ORANGE_LED	GPIO_PIN_NO_13
#define GREEN_LED	GPIO_PIN_NO_12
#define RED_LED		GPIO_PIN_NO_14
#define BLUE_LED	GPIO_PIN_NO_15

void delay(uint16_t delayTime)
{
	for(uint16_t i = 0; i < delayTime; i++){
		for(uint16_t j = 0; j < delayTime; j++);
	}
}

int main(void){

	GPIOD_PCLK_EN();								//Enable PCLK for GPIO_D peripheral

	GPIO_Handle_t GpioLed;							//Initialize the handle structure of GPIO

	GpioLed.pGPIOx = GPIOD;							//Initialize the GPIO port to which LEDs are Connected

	//Command GPIO Configuration used to glow LEDs
	GpioLed.GPIO_PinConfig.GPIO_PinMode   		= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OUTPUT_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_NOPUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_HIGH;

	//Initialize Orange LED LD3
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= ORANGE_LED;
	GPIO_Init(&GpioLed);

	//Initialize Green LED LD4
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GREEN_LED;
	GPIO_Init(&GpioLed);

	//Initialize Red LED LD4
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= RED_LED;
	GPIO_Init(&GpioLed);

	//Initialize Blue LED LD4
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= BLUE_LED;
	GPIO_Init(&GpioLed);


	while(1){

		//Switch ON all the LEDs one by one
		GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 1);
		delay(450);
		GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 1);
		delay(500);
		GPIO_WriteToOutputPin(GPIOD, RED_LED, 1);
		delay(550);
		GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 1);
		delay(600);

		//Switch OFF all the LEDs one by one
		GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 0);
		delay(450);
		GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 0);
		delay(500);
		GPIO_WriteToOutputPin(GPIOD, RED_LED, 0);
		delay(550);
		GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 0);
		delay(600);

		//Switch ON Orane and Green LEDs
		GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 1);
		GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 1);
		delay(800);


		for(uint8_t i = 0; i < 4; i++)
		{
			GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, RED_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 1);
			delay(800);
			GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, RED_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 0);
			delay(800);
		}

		for(uint8_t i = 0; i < 4; i++)
		{
			GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, RED_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 0);
			delay(800);
			GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 1);
			GPIO_WriteToOutputPin(GPIOD, RED_LED, 0);
			GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 1);
			delay(800);
		}

		GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 1);
		GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 1);
		GPIO_WriteToOutputPin(GPIOD, RED_LED, 1);
		GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 1);
		delay(600);

		GPIO_WriteToOutputPin(GPIOD, RED_LED, 0);
		GPIO_WriteToOutputPin(GPIOD, BLUE_LED, 0);
		GPIO_WriteToOutputPin(GPIOD, ORANGE_LED, 0);
		GPIO_WriteToOutputPin(GPIOD, GREEN_LED, 0);
		delay(600);

		GPIO_WriteToOutputPort(GPIOD, 0X1000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X2000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X1000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X2000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X3000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X4000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X5000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X6000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X7000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X8000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0X9000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XA000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XB000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XC000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XD000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XE000);
		delay(650);
		GPIO_WriteToOutputPort(GPIOD, 0XF000);
		delay(650);



	}
	return 0;

}


