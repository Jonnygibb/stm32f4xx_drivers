/*
 * stm32f407xx.h
 *
 *  Created on: Aug 20, 2024
 *      Author: Jonathan Gibbons
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile	// Shorthand volatile macro


/*
 * Define the base addresses of Flash and SRAM memory.
 */

#define FLASH_BASEADDR				0x08000000U // Base Address of flash memory.
#define SRAM1_BASEADDR				0x20000000U // Base Address of SRAM1 memory.
#define SRAM2_BASEADDR				0x20001C00U // Base Address of SRAM1 memory.
#define SRAM 						SRAM1_BASEADDR // Macro for SRAM since SRAM1 is main memory.
#define ROM							0x1FFF0000U	// Base Address of ROM/System Memory.

/*
 * Define the base addresses for AHBx and APBx Peripheral busses.
 */

#define PERIPH_BASEADDR				0x40000000U // Peripheral busses Base Address.
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR // APB1 Peripheral bus Base Address.
#define APB2PERIPH_BASEADDR			0x40010000U // APB2 Peripheral bus Base Address.
#define AHB1PERIPH_BASEADDR			0x40020000U // AHB1 Peripheral bus Base Address.
#define AHB2PERIPH_BASEADDR			0x50000000U // AHB2 Peripheral bus Base Address.

/*
 * Base Addresses for all peripherals used in these drivers.
 * NOTE: This list of peripherals is incomplete, only necessary
 * peripherals are included.
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2800)

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

/********************Peripheral Register Definition Structures********************/

/*
 * The Registers listed here are specific to stm32f407xx micro-controller.
 * Registers for peripherals may be more/less for different MCU's.
 * Check your MCU reference manual before using this header file.
 */

typedef struct {
	__vo uint32_t MODER;				// Configures the I/O mode of the GPIO pin. Offset:0x00.
	__vo uint32_t OTYPER;				// Set the output type - push/pull or open-drain. Offset:0x04.
	__vo uint32_t OSPEEDR;				// Set the speed of a GPIO pin e.g Low, Med, High. Offset:0x08.
	__vo uint32_t PUPDR;				// Configures pull-up or pull down registers on pins. Offset:0x0C.
	__vo uint32_t IDR;					// Read only input data register. Offset:0x10.
	__vo uint32_t ODR;					// Read/Write output data register. Offset:0x14.
	__vo uint32_t BSRR;					// Bit set/Reset register. Offset:0x18.
	__vo uint32_t LCKR;					// Lock the configuration of the GPIO. Offset:0x1C.
	__vo uint32_t AFR[2];				// Alternate function registers low&high. AFR[0] - low, AFR[1] - High. Offset:0x20-24.
} GPIO_RegDef_t;


typedef struct {
	__vo uint32_t CR;						// RCC clock control register. Offset:0x00
	__vo uint32_t PLLCFGR;					// RCC PLL configuration register  Offset:0x04
	__vo uint32_t CFGR;						// RCC clock configuration register. Offset:0x08
	__vo uint32_t CIR;						// RCC clock interrupt register. Offset:0x0C
	__vo uint32_t AHB1RSTR;					// RCC AHB1 peripheral reset register. Offset:0x10
	__vo uint32_t AHB2RSTR;					// RCC AHB2 peripheral reset register. Offset:0x14
	__vo uint32_t AHB3RSTR;					// RCC AHB3 peripheral reset register. Offset:0x18
	uint32_t RESERVED_1;				// Reserved/Unused. Offset:0x1C
	__vo uint32_t APB1RSTR;					// RCC APB1 peripheral reset register. Offset:0x20
	__vo uint32_t APB2RSTR;					// RCC APB2 peripheral reset register. Offset:0x24
	uint32_t RESERVED_2[2];				// Reserved/Unused. Offset:0x28-2C
	__vo uint32_t AHB1ENR;					// RCC AHB1 peripheral clock enable register. Offset:0x30
	__vo uint32_t AHB2ENR;					// RCC AHB2 peripheral clock enable register. Offset:0x34
	__vo uint32_t AHB3ENR;					// RCC AHB3 peripheral clock enable register. Offset:0x38
	uint32_t RESERVED_3;				// Reserved/Unused. Offset:0x3C
	__vo uint32_t APB1ENR;					// RCC APB1 peripheral clock enable register. Offset:0x40
	__vo uint32_t APB2ENR;					// RCC APB2 peripheral clock enable register. Offset:0x44
	uint32_t RESERVED_4[2];				// Reserved/Unused. Offset:0x48-4C
	__vo uint32_t AHB1LPENR;					// RCC AHB1 peripheral clock enable in low power mode register. Offset:0x50
	__vo uint32_t AHB2LPENR;					// RCC AHB2 peripheral clock enable in low power mode register. Offset:0x54
	__vo uint32_t AHB3LPENR;					// RCC AHB3 peripheral clock enable in low power mode register. Offset:0x58
	uint32_t RESERVED_5;				// Reserved/Unused. Offset:0x5C
	__vo uint32_t APB1LPENR;					// RCC APB1 peripheral clock enable in low power mode register. Offset:0x60
	__vo uint32_t APB2LPENR;					// RCC APB2 peripheral clock enable in low power mode register. Offset:0x64
	uint32_t RESERVED_6[2];				// Reserved/Unused. Offset:0x68-6C
	__vo uint32_t BDCR;						// RCC Backup domain control register . Offset:0x70
	__vo uint32_t CSR;						// RCC clock control & status register . Offset:0x74
	uint32_t RESERVED_7[2];				// Reserved/Unused. Offset:0x78-7C
	__vo uint32_t SSCGR;						// RCC spread spectrum clock generation register. Offset:0x80
	__vo uint32_t PLLI2SCFGR;				// RCC PLLI2S configuration register. Offset:0x84
} RCC_RegDef_t;

/*
 * Peripheral Definitions (type-casted pointers to address in memory)
 */


#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ						((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK						((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1<<8))

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))

/*
 * Clock Enable Macros for USARTx Peripherals
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
//#define USART4_PCLK_EN()			(RCC->APB1ENR |= (1<<15))
//#define USART5_PCLK_EN()			(RCC->APB1ENR |= (1<<15))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))
//#define USART4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<15))
//#define USART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<15))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */


#endif /* INC_STM32F407XX_H_ */
