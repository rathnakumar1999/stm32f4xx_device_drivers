/*
 * 006Spi_Txonly_arduino.c
 *
 *  Created on: 27-Jun-2021
 *      Author: RathnakumarKannan
 */
#include "stm32f407xx.h"
#include <string.h>

/*
 * PB12 -->	NSS
 * PB13 -->	SCK
 * PB14 --> MISO
 * PB15 --> MOSI
 */
void Stm_SPI_GPIOInit(void)
{
	GPIO_Handle_t SPIGpio;

	SPIGpio.pGPIOx = GPIOB;

	SPIGpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIGpio.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIGpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	SPIGpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	SPIGpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//configure the	SCK pin
	SPIGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIGpio);

	//configure the MOSI pin
	SPIGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIGpio);

	//configure the NSS pin
	SPIGpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIGpio);
}

void Stm_SPI_Config()
{
	SPI_Handle_t STMSpi;

	STMSpi.pSPIx = SPI2;

	STMSpi.SPI_PeripheralConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	STMSpi.SPI_PeripheralConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	STMSpi.SPI_PeripheralConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	STMSpi.SPI_PeripheralConfig.SPI_DFF = SPI_DFF_8BITS;
	STMSpi.SPI_PeripheralConfig.SPI_CPHA = SPI_CPHA_LOW;
	STMSpi.SPI_PeripheralConfig.SPI_CPOL = SPI_CPOL_LOW;
	STMSpi.SPI_PeripheralConfig.SPI_SSM = SPI_SSM_DI;

	//initialize the SPI2
	SPI_Init(&STMSpi);
}

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

void Wait(int delay)
{
	for(int i = 0; i < delay; i++) {
		for(int j = delay; j >= 0; j--);
	}
}

int main(void)
{
	char to_send[] = "I am from STM32";

	//initialize GPIO for SPI2
	Stm_SPI_GPIOInit();
	//configure & initialize the button to control data transfer
	Button_ControlInit();

	//configure the SPI2 peripheral
	Stm_SPI_Config();

	//enable the slave select output
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		//wait until input button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		Wait(100);

		//enable the peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send the lenght of data to be sent
		uint8_t dataLen = strlen(to_send);
		SPI_SendData(SPI2, &dataLen, 1);

		//send the data
		SPI_SendData(SPI2, (uint8_t*)to_send, dataLen);

		//wait until last byte of data is transmitted
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//disable the SPI peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
