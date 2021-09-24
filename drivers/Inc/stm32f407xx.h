/*
 * stm32f407xx.h
 *
 *  Created on: May 1, 2021
 *      Author: RathnakumarKannan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*
 * base Address of FLASH and SRAM memories
 */
#define FLASH_BASEADDR 				0x08000000U							/*<!Base Address of Flash Memory*/
#define SRAM1_BASEADDR				0x20000000U							/*<!Base Address of SRAM1 Memory*/
#define SRAM2_BASEADDR				0x2001C000U							/*<!Base Address of SRAM2 Memory*/
#define ROM_BASEADDR				0x1FFF0000U							/*<!Base Address of SYSTEM(ROM) Memory*/


/*
 * base Address of AHB1,AHB2,APB1,APB2 buses
 */
#define PERIPH_BASEADDR				0x40000000U							/*<!Base Address of Peripheral Registers*/
#define AHB1PERIPH_BASEADDR			0x40020000U	 						/*<!Base Address of AHB1 Bus */
#define AHB2PERIPH_BASEADDR			0x50000000U							/*<!Base Address of AHB2 Bus */
#define	APB1PERIPH_BASEADDR			PERIPH_BASEADDR						/*<!Base Address of APB1 Bus */
#define	APB2PERIPH_BASEADDR			0x40010000U							/*<!Base Address of APB2 Bus */


/*
 * Arm Cortex M4 NVIC Base Addresses
 */
#define NVIC_ISER_BASEADDR			((__vo uint32_t*) 0xE000E100)		/*<!Base Address of NVIC Interrupt Set Enable Register*/
#define NVIC_ICER_BASEADDR			((__vo uint32_t*) 0XE000E180)		/*<!Base Address of NVIC Interrupt Clear Enable Register*/
#define NVIC_IPR_BASEADDR			((__vo uint32_t*) 0xE000E400)		/*<!Base Address of NVIC Interrupt Priority Register*/

/*
 * definition of No.of.Priority bits implemented for stm32f407x
 */
#define	NO_PR_BITS_IMPLEMENTED		4									/*<!In the NVIC of ARM priority levels can be implemented as per the manufacturer wish*/

/*
 * base Address of AHB1 Peripherals
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0000)		/*<!Base Address of GPIOA Peripheral*/
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0400)		/*<!Base Address of GPIOB Peripheral*/
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0800)		/*<!Base Address of GPIOC Peripheral*/
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0C00)		/*<!Base Address of GPIOD Peripheral*/
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1000)		/*<!Base Address of GPIOE Peripheral*/
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1400)		/*<!Base Address of GPIOF Peripheral*/
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1800)		/*<!Base Address of GPIOG Peripheral*/
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1C00)		/*<!Base Address of GPIOH Peripheral*/
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0X2000)		/*<!Base Address of GPIOI Peripheral*/
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)		/*<!Base Address of RCC Peripheral*/


/*
 * base Address of APB1 Peripherals
 */
#define	I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0X5400)		/*<!Base Address of I2C1 Peripheral*/
#define	I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0X5800)		/*<!Base Address of I2C1 Peripheral*/
#define	I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0X5C00)		/*<!Base Address of I2C1 Peripheral*/
#define	SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0X3800)		/*<!Base Address of SPI2 Peripheral*/
#define	SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0X3C00)		/*<!Base Address of SPI3 Peripheral*/
#define	USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0X4400)		/*<!Base Address of USART2 Peripheral*/
#define	USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0X4800)		/*<!Base Address of USART2 Peripheral*/
#define	UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0X4C00)		/*<!Base Address of UART4 Peripheral*/
#define	UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0X5000)		/*<!Base Address of UART5 Peripheral*/


/*
 * base Address of APB2 Peripherals
 */
#define	USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0X1000)		/*<!Base Address of USART1 Peripheral*/
#define	USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0X1400)		/*<!Base Address of USART6 Peripheral*/
#define	SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0X3000)		/*<!Base Address of SPI1 Peripheral*/
#define	EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0X3C00)		/*<!Base Address of EXTI Peripheral*/
#define	SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0X3800)		/*<!Base Address of SYSCFG Peripheral*/




/******************************************************************************************************
 * Definition of Peripheral RegDef_t
 ******************************************************************************************************/
/*
 * Register Definition Structure of RCC
 */
typedef struct
{
	__vo uint32_t CR;							/*<!RCC Clock Control Register					Address Offset: 0x00*/
	__vo uint32_t PLLCFGR;						/*<!RCC PLL Configuration Register				Address Offset: 0x04*/
	__vo uint32_t CFGR;							/*<!RCC Configuration Register					Address Offset: 0x08*/
	__vo uint32_t CIR;							/*<!RCC Clock Interrupt Register				Address Offset: 0x0C*/
	__vo uint32_t AHB1RSTR;						/*<!RCC AHB1 Peripheral Reset Register			Address Offset: 0x10*/
	__vo uint32_t AHB2RSTR;						/*<!RCC AHB2 Peripheral Reset Register			Address Offset: 0x14*/
	__vo uint32_t AHB3RSTR;						/*<!RCC AHB3 Peripheral Reset Register			Address Offset: 0x18*/
	__vo uint32_t RESERVED0;					/*<!RCC RESERVED0								Address Offset: 0x1C*/
	__vo uint32_t APB1RSTR;						/*<!RCC APB1 Peripheral Reset Register			Address Offset: 0x20*/
	__vo uint32_t APB2RSTR;						/*<!RCC APB2 Peripheral Reset Register			Address Offset: 0x24*/
	__vo uint32_t RESRVED1[2];					/*<!RCC RESERVED1								Address Offset: 0x28*/
	__vo uint32_t AHB1ENR;						/*<!RCC AHB1 pCLK EN Register					Address Offset: 0x30*/
	__vo uint32_t AHB2ENR;						/*<!RCC AHB2 pCLK EN Register					Address Offset: 0x34*/
	__vo uint32_t AHB3ENR;						/*<!RCC AHB3 pCLK EN Register					Address Offset: 0x38*/
	__vo uint32_t RESRVED2;						/*<!RCC RESERVED2								Address Offset: 0x3C*/
	__vo uint32_t APB1ENR;						/*<!RCC ABP1 pCLK EN Register					Address Offset: 0x40*/
	__vo uint32_t APB2ENR;						/*<!RCC ABP1 pCLK EN Register					Address Offset: 0x44*/
	__vo uint32_t RESERVED3[2];					/*<!RCC RESERVED3								Address Offset: 0x48*/
	__vo uint32_t AHB1LPENR;					/*<!RCC AHB1 pCLK EN in Low PWR Mode Register	Address Offset: 0x50*/
	__vo uint32_t AHB2LPENR;					/*<!RCC AHB2 pCLK EN in Low PWR Mode Register	Address Offset: 0x54*/
	__vo uint32_t AHB3LPENR;					/*<!RCC AHB3 pCLK EN in Low PWR Mode Register	Address Offset: 0x58*/
	__vo uint32_t RESERVED4;					/*<!RCC RESERVED4								Address Offset: 0x5C*/
	__vo uint32_t APB1LPENR;					/*<!RCC APB1 pCLK EN in Low PWR Mode Register	Address Offset: 0x60*/
	__vo uint32_t APB2LPENR;					/*<!RCC APB2 pCLK EN in Low PWR Mode Register	Address Offset: 0x64*/
	__vo uint32_t RESERVED5[2];					/*<!RCC RESERVED5								Address Offset: 0x68*/
	__vo uint32_t BDCR;							/*<!Backup Domain Control Register				Address Offset: 0x70*/
	__vo uint32_t CSR;							/*<!Clock Control & Status Register				Address Offset: 0x74*/
	__vo uint32_t RESERVED6[2];					/*<!RCC RESERVED6								Address Offset: 0x78*/
	__vo uint32_t SSCGR;						/*<!Spread Spectrum Clock Generation Register	Address Offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;					/*<!PLLI2S Configuration Register				Address Offset: 0x84*/
}RCC_RegDef_t;


/*
 * Register definition structure of GPIO
 */
typedef struct
{
	__vo uint32_t MODER;						/*<!GPIO Port Mode Register						Address Offset: 0x00*/
	__vo uint32_t OTYPER;						/*<!GPIO Port Output Type Register				Address Offset: 0x04*/
	__vo uint32_t OSPEEDR;						/*<!GPIO Port Output Speed Register				Address Offset: 0x08*/
	__vo uint32_t PUPDR;						/*<!GPIO Port Pull up/down Register				Address Offset: 0x0C*/
	__vo uint32_t IDR;							/*<!GPIO Port Input data Register				Address Offset: 0x10*/
	__vo uint32_t ODR;							/*<!GPIO Port Output data Register				Address Offset: 0x14*/
	__vo uint32_t BSRR;							/*<!GPIO Port bit set/reset Register			Address Offset: 0x18*/
	__vo uint32_t LCKR;							/*<!GPIO Port Configuration lock Register		Address Offset: 0x1C*/
	__vo uint32_t AFR[2];						/*<!GPIO Port AF[0]: Alternate Function Low AF[1]: Alternate Function High Register					Address Offset: 0x00*/
}GPIO_RegDef_t;


/*
 * Register definition structure of SPI
 */
typedef struct
{
	__vo uint32_t CR1;							/*<!SPI Control Register1						Address Offset: 0x00*/
	__vo uint32_t CR2;							/*<!SPI Control Register2						Address Offset: 0x04*/
	__vo uint32_t SR;							/*<!SPI Status Register							Address Offset: 0x08*/
	__vo uint32_t DR;							/*<!SPI Data Register							Address Offset: 0x0C*/
	__vo uint32_t CRCPR;						/*<!SPI CRC Polynomial Register					Address Offset: 0x10*/
	__vo uint32_t RXCRCR;						/*<!SPI Rx CRC Register							Address Offset: 0x14*/
	__vo uint32_t TXCRCR;						/*<!SPI Tx CRC Register							Address Offset: 0x18*/
	__vo uint32_t I2SCFGR;						/*<!SPI I2S Configuration Register				Address Offset: 0x1C*/
	__vo uint32_t I2SPR;						/*<!SPI I2S Prescaler Register					Address Offset: 0x20*/
}SPI_RegDef_t;

/*
 * Register Definition Structure of I2C
 */
typedef struct
{
	__vo uint32_t CR1;							/*<!I2C Control Register1						Address Offset: 0x00*/
	__vo uint32_t CR2;							/*<!I2C Control Register2						Address Offset: 0x04*/
	__vo uint32_t OAR1;							/*<!I2C Own Address Register1					Address Offset: 0x08*/
	__vo uint32_t OAR2;							/*<!I2C Own Address Register2					Address Offset: 0x0C*/
	__vo uint32_t DR;							/*<!I2C Data Register							Address Offset: 0x10*/
	__vo uint32_t SR1;							/*<!I2C Status Register1						Address Offset: 0x14*/
	__vo uint32_t SR2;							/*<!I2C Status Register2						Address Offset: 0x18*/
	__vo uint32_t CCR;							/*<!I2C Clock Control Register1					Address Offset: 0x1C*/
	__vo uint32_t TRISE;						/*<!I2C TRISE Register							Address Offset: 0x20*/
	__vo uint32_t FLTR;							/*<!I2C FLTR Register							Address Offset: 0x24*/
}I2C_RegDef_t;

/*
 * Register definition structure of USART
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*
 * Register definition structure of EXTI
 */
typedef struct
{
	__vo uint32_t IMR;							/*<!EXTI Interrupt Mask Register				Address Offset: 0x00*/
	__vo uint32_t EMR;							/*<!EXTI Event Mask Register					Address Offset: 0x04*/
	__vo uint32_t RTSR;							/*<!EXTI Rising Trigger Selection Register		Address Offset: 0x08*/
	__vo uint32_t FTSR;							/*<!EXTI Falling Trigger Selection Register		Address Offset: 0x0C*/
	__vo uint32_t SWIER;						/*<!EXTI Software Interrupt Event Register		Address Offset: 0x10*/
	__vo uint32_t PR;							/*<!EXTI Pending Register						Address Offset: 0x14*/
}EXTI_RegDef_t;


/*
 * Register definition structure of SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;					/*<!SYSCFG Memory Remap Register					Address Offset: 0x00*/
	__vo uint32_t PMC;						/*<!SYSCFG Peripheral Mode Config Register			Address Offset: 0x04*/
	__vo uint32_t EXTICR[4];				/*<!SYSCFG External Interrupt Config Registers[0:3]	Address Offset: 0x08,0C,10,14*/
	__vo uint32_t RESERVED[2];				/*<!SYSCFG Reserved[2]								Address Offset: 0x18,1C*/
	__vo uint32_t CMPCR;					/*<!SYSCFG Compensation Cell Control Register		Address Offset: 0x20*/
}SYSCFG_RegDef_t;

/*
 *RCC Peripheral base address for assigning Regdef structure
 */
#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)




/******************************************************************************************************
 * Definition of Peripherals
 ******************************************************************************************************/
/*
 *	GPIO Peripheral base address for assigning RegDef structure
 */
#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 *	SPI Peripheral base address for assigning RegDef structure
 */
#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)


/*
 * I2C Peripheral base address for assigning RegDef structure
 */
#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * USART Peripheral base address for assigning RegDef structure
 */
#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3					((USART_RegDef_t*)USART3_BASEADDR)
#define UART4					((USART_RegDef_t*)UART4_BASEADDR)
#define UART5					((USART_RegDef_t*)UART5_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)


/*
 * EXTI Peripheral base address for assigning RegDef structure
 */
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * Peripheral base address for assigning RegDef structure
 */
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)




/******************************************************************************************************
 * Clock Enable & Disable Macros
 ******************************************************************************************************/

/*
 * Clock Enable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()			( RCC->AHB1ENR |= (1 << 8) )


/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )


/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15) )


/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5)  )


/*
 * Clock Enable Macros for UARTx Peripherals
 */
#define UART4_PCLK_EN()			( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= (1 << 20) )


/*
 * Clock Disable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 8) )


/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 15) )


/*
 * Clock Disable Macros for USARTx Peripherals
 */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 5)  )


/*
 * Clock Disable Macros for UARTx Peripherals
 */
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 20) )




/******************************************************************************************************
 * Register Reset Macros
 ******************************************************************************************************/

/*
 * GPIOx Register Reset Macros
 */

#define GPIOA_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * SPIx Register Reset Macros
 */
#define SPI1_REG_RESET()		do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*
 * I2Cx Register Reset Macros
 */
#define I2C1_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
 * USARTx Register Reset Macros
 */
#define USART1_REG_RESET()		do { (RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4));  }while(0)
#define USART2_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()		do { (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()		do { (RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5));  }while(0)

/*
 * return portCode for given GPIO port base address
 */
#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :0 )

/*
 * IRQ numbers of stm324f407 MCU
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_ER		73


/*
 * Possible Interrupt priorities implemented in NVIC of stm32f407x
 */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15




/******************************************************************************************************
 * Bit Position Definition of SPI Peripheral
 ******************************************************************************************************/
/*
 * Bit position definition of SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definition of SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 * Bit position definition of SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8



/******************************************************************************************************
 * Bit Position Definition of I2C Peripheral
 ******************************************************************************************************/

/*
 * Bit Position of I2C CR1 Register
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * Bit Position of I2C CR2 Register
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit Position of I2C SR1 Register
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * Bit Position of I2C SR2 Register
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7

/*
 * Bit Position of I2C SR2 Register
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		1
#define I2C_CCR_FS			2


/******************************************************************************************************
 * Bit Position Definition of USART Peripheral
 ******************************************************************************************************/

/*
 * Bit Position of USART SR Register
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 * Bit Position of USART CR1 Register
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Bit Position of USART CR2 Register
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 * Bit Position of USART CR3 Register
 */
#define USART_CR3_EIE		0
#define USART_CR3_IRE		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11



/*******************************************************************************************
 * General Definitions
 *******************************************************************************************/
/*
 * Generic Macros
 */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/*
 * the following header files are useful while writing application code
 * in the application code no need to include all the driver header files
 * only the "stm32f407xx.h" (device specific header file) is enough other
 * header files can be added via device specific header file
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
