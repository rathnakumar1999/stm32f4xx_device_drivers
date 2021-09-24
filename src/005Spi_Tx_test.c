/*
 * 005Spi_Tx_test.c
 *
 *  Created on: 26-Jun-2021
 *      Author: RathnakumarKannan
 */


/*
 * PB12 -> SPI2_NSS pin
 * PB13 -> SPI2_SCK pin
 * PB14 -> SPI2_MISO pin
 * PB15 -> SPI2_MOSI pin
 * ALT Function Mode: 5
 */
#include "stm32f407xx.h"
#include <string.h>

void SPI_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	//configure SPI2 pins in Port-B
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//configure the SCK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//configure the MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Config()
{
	SPI_Handle_t  SPI2Handle;

	//configure the SPI2 peripherals as per the requirement
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_PeripheralConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_PeripheralConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_PeripheralConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_PeripheralConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_PeripheralConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handle.SPI_PeripheralConfig.SPI_SSM = SPI_SSM_EN;
	SPI2Handle.SPI_PeripheralConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	//initialize the SPI2 peripheral
	SPI_Init(&SPI2Handle);

}

int main(void)
{
	//declare the data to send
	char user_data[] = "Hello World";

	//configure the GPIO pins for SPI2 and initialize
	SPI_GPIOInit();

	//configure the SPI2 as per the requirement and initialize
	SPI2_Config();

	//set the SSI bit for Master mode of operation
	SPI_SSIConfig(SPI2, SET);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//send the user data via SPI2
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//disable the SPI peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);

	return 0;
}


