/*
 * I2C_ReceiveAPI_Test.c
 *
 *  Created on: 21-Aug-2021
 *      Author: RathnakumarKannan
 */

#include"stm32f407xx.h"

#define slaveAddress	0x34

char SampleData[] = "Hello World\r\n";
char receiveData[512];

void Stm_I2C_GPIOInit()
{
	GPIO_Handle_t I2CGpio;

	memset(&I2CGpio, 0, sizeof(I2CGpio));

	I2CGpio.pGPIOx = GPIOB;

	I2CGpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CGpio.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CGpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_OD;
	I2CGpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	I2CGpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//configure the SCL pin
	I2CGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CGpio);

	//configure the SDA pin
	I2CGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CGpio);
}

void Stm_I2C_Config()
{
	I2C_Handle_t StmI2C;

	memset(&StmI2C, 0 , sizeof(StmI2C));

	StmI2C.pI2Cx = I2C1;

	StmI2C.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	StmI2C.I2C_Config.I2C_DeviceAddress = 0x32;
	StmI2C.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PclkControl(I2C1, ENABLE);

	I2C_Init(&StmI2C);
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
	//configure & initialize the button to control data transfer
	Button_ControlInit();

	//configure GPIO pins for I2C peripheral
	Stm_I2C_GPIOInit();

	//configure the I2C peripheral
	Stm_I2C_Config();


	while(1)
	{
		//wait until input button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		Wait(1000);
		//I2C_MasterSendData(I2C1, (uint8_t*)&SampleData, sizeof(SampleData), slaveAddress);
		I2C_MasterReceiveData(I2C1, (uint8_t*)&receiveData, 6, 0x08);
	}

	return 0;
}
