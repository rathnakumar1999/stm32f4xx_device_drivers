/*
 * Usart_Tx_test.c
 *
 *  Created on: 18-Sep-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx.h"

char data[] = "I am from STM32\n";

void Button_ControlInit(void)
{
	GPIO_Handle_t Button;

	memset(&Button, 0, sizeof(Button));

	Button.pGPIOx = GPIOA;

	//configure the User Button in the development board
	//user button connected in the development board
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(&Button);
}

void USART2_Init(USART_Handle_t *stmUSART)
{

	stmUSART->pUSARTx = USART1;

	stmUSART->UsartConfig.USART_BaudRate = USART_BAUD_115200;
	stmUSART->UsartConfig.USART_DataBits = USART_8BITS;
	stmUSART->UsartConfig.USART_HWFlowControl = USART_HW_FLOWCNTRL_DI;
	stmUSART->UsartConfig.USART_Mode = USART_MODE_TX_ONLY;
	stmUSART->UsartConfig.USART_Parity  = USART_PARITY_DI;
	stmUSART->UsartConfig.USART_SamplingRate = USART_SAMPLEBY_16;
	stmUSART->UsartConfig.USART_StopBits = USART_STOPBIT_1;

	USART_Init(stmUSART);
}

void USART2_Gpio_Init()
{
	GPIO_Handle_t usartGpio;

	usartGpio.pGPIOx = GPIOB;

	usartGpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usartGpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usartGpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	usartGpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usartGpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//Configure Tx pin
	usartGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&usartGpio);

	//Configure Rx pin
	usartGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&usartGpio);
}

void Wait(int delay)
{
	for(int i = 0; i < delay; i++) {
		for(int j = delay; j >= 0; j--);
	}
}


int main(void)
{
	USART_Handle_t stmUSART;

	Button_ControlInit();

	USART2_Gpio_Init();

	USART2_Init(&stmUSART);

	while(1)
	{
		while(!(GPIO_ReadFromInputPin(GPIOA, 0)));

		Wait(1000);

		//USART_PeripheralControl(USART1, ENABLE);
		USART_SendData(&stmUSART, (uint8_t*)data, sizeof(data));
	}

	return 0;
}
