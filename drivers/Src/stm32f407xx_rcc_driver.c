/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 18-Sep-2021
 *      Author: RathnakumarKannan
 */

#include "stm32f407xx_rcc_driver.h"

static uint16_t Prescalers[] = {2, 4, 8, 16, 64, 128, 256, 512};

static uint32_t get_pll_clk()
{
	uint8_t clksrc;
	uint32_t pllclk;
	uint8_t pllm,pllp;
	uint16_t plln;

	clksrc = ( (RCC->PLLCFGR >> 22) & 0x01 );

	if(clksrc == 0) {
		pllclk = 16000000;
	}
	else if(clksrc == 1) {
		pllclk = 8000000;
	}

	pllm = ( RCC->PLLCFGR & 0x3F);
	pllclk = pllclk / pllm;

	plln = ( (RCC->PLLCFGR >> 6) & 0x1FF);
	pllclk = pllclk * plln;

	pllp = ( (RCC->PLLCFGR >> 16) & 0x03);
	pllclk = pllclk / pllp;

	return pllclk;
}


uint32_t Get_APB1_Pclk()
{
	uint32_t systemclock;
	uint8_t clksrc,ahbpre,apb1pre;
	uint16_t temp;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0){
		systemclock = 16000000;
	}
	else if(clksrc == 1){
		systemclock = 8000000;
	}
	else if(clksrc == 2){
		systemclock = get_pll_clk();
	}

	//ahb prescaler
	temp = ( (RCC->CFGR >> 4) & 0x0F);

	if(temp < 8) {
		ahbpre = 1;
	}
	else {
		ahbpre = Prescalers[temp - 8];
	}

	systemclock = systemclock / ahbpre;

	//abp1 prescaler
	temp = ( (RCC->CFGR >> 10) & 0x07);
	if(temp < 4) {
		apb1pre = 1;
	}
	else {
		apb1pre = Prescalers[temp - 4];
	}

	systemclock = systemclock / apb1pre;

	return systemclock;

}


uint32_t Get_APB2_Pclk()
{
	uint32_t systemclock;
	uint8_t clksrc,ahbpre,apb2pre;
	uint16_t temp;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0){
		systemclock = 16000000;
	}
	else if(clksrc == 1){
		systemclock = 8000000;
	}
	else if(clksrc == 2){
		systemclock = get_pll_clk();
	}

	//ahb prescaler
	temp = ( (RCC->CFGR >> 4) & 0x0F);

	if(temp < 8) {
		ahbpre = 1;
	}
	else {
		ahbpre = Prescalers[temp - 8];
	}

	systemclock = systemclock / ahbpre;

	//abp1 prescaler
	temp = ( (RCC->CFGR >> 10) & 0x07);
	if(temp < 4) {
		apb2pre = 1;
	}
	else {
		apb2pre = Prescalers[temp - 4];
	}

	systemclock = systemclock / apb2pre;

	return systemclock;

}
