/*
 * 008I2c_interrupt_test.c
 *
 *  Created on: 04-Sep-2021
 *      Author: RathnakumarKannan
 */
/*
 * I2C_Master_TxRx.c
 *
 *  Created on: 22-Aug-2021
 *      Author: RathnakumarKannan
 */

#include"stm32f407xx.h"
I2C_Handle_t StmI2C;

#define slaveAddress	0x68

char RequestLength = 0x51;
char RequestData = 0x52;
char SampleData[256];
uint8_t len = 0;

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

void Stm_I2C_Config(I2C_Handle_t *pI2CHandle)
{

	memset(pI2CHandle, 0 , sizeof(*pI2CHandle));

	pI2CHandle->pI2Cx = I2C1;

	pI2CHandle->I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	pI2CHandle->I2C_Config.I2C_DeviceAddress = 0x32;
	pI2CHandle->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PclkControl(pI2CHandle->pI2Cx, ENABLE);

	I2C_Init(pI2CHandle);
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
	Stm_I2C_Config(&StmI2C);

	//Configure I2C Interrupt
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PRIO10);

	while(1) {
		//wait until input button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		Wait(1000);
		//send command to request the length of data to receive
		//I2C_MasterSendData(I2C1, (uint8_t*)&RequestLength, 1, slaveAddress, I2C_REPEATSTART_EN);
		while(I2C_MasterSendDataIT(&StmI2C, (uint8_t*)&RequestLength, 1, slaveAddress, I2C_REPEATSTART_EN) != I2C_BUSY_IN_TX);
		//receive the length information
		//I2C_MasterReceiveData(I2C1, &len, 1, slaveAddress, I2C_REPEATSTART_EN);
		while(I2C_MasterReceiveDataIT(&StmI2C, &len, 1, slaveAddress, I2C_REPEATSTART_EN) != I2C_BUSY_IN_RX);
		//send command to request data
		//I2C_MasterSendData(I2C1, (uint8_t*)&RequestData, 1, slaveAddress, I2C_REPEATSTART_EN);
		while(I2C_MasterSendDataIT(&StmI2C, (uint8_t*)&RequestData, 1, slaveAddress, I2C_REPEATSTART_EN) != I2C_BUSY_IN_TX);
		//receive the data
		while(I2C_MasterReceiveDataIT(&StmI2C, (uint8_t*)&SampleData, len, slaveAddress, I2C_REPEATSTART_DI) != I2C_BUSY_IN_RX);
	}

}

void I2C1_EV_IRQHandler()
{
	I2C_EVT_Handling(&StmI2C);
}

