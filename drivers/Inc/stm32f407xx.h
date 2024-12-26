/*
 * stm32f407xx.h
 *
 *  Created on: Aug 20, 2024
 *      Author: Jonathan Gibbons
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile	// Shorthand volatile macro
#define __weak __attribute__((weak)) // Shorthand weak macro

/****************************** Processor Specific Details ******************************
 *
 */

/*
 * ARM Cortex Mx Processor NVIC ISERx register address
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register address
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)

/*
 *  ARM Cortex Mx Processor Priority register address
 */
#define NVIC_PR_BASE_ADDR			((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

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

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)


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
	__vo uint32_t CR1;					// SPI control register 1. Offset:0x00.
	__vo uint32_t CR2;					// SPI control register 2. Offset:0x04.
	__vo uint32_t SR;					// SPI status register. Offset:0x08.
	__vo uint32_t DR;					// SPI data register. Offset:0x0C.
	__vo uint32_t CRCPR;				// SPI CRC polynomial register. Offset:0x10.
	__vo uint32_t RXCRCR;				// SPI RX CRC register. Offset:0x14.
	__vo uint32_t TXCRCR;				// SPI TX CRC register. Offset:0x18.
	__vo uint32_t I2SCFGR;				// SPI_I2S configuration register. Offset:0x1C.
	__vo uint32_t I2SPR;				// SPI_I2S pre-scaler register. Offset:0x20.
} SPI_RegDef_t;


typedef struct {
	__vo uint32_t CR;					// RCC clock control register. Offset:0x00
	__vo uint32_t PLLCFGR;				// RCC PLL configuration register  Offset:0x04
	__vo uint32_t CFGR;					// RCC clock configuration register. Offset:0x08
	__vo uint32_t CIR;					// RCC clock interrupt register. Offset:0x0C
	__vo uint32_t AHB1RSTR;				// RCC AHB1 peripheral reset register. Offset:0x10
	__vo uint32_t AHB2RSTR;				// RCC AHB2 peripheral reset register. Offset:0x14
	__vo uint32_t AHB3RSTR;				// RCC AHB3 peripheral reset register. Offset:0x18
	uint32_t RESERVED_1;				// Reserved/Unused. Offset:0x1C
	__vo uint32_t APB1RSTR;				// RCC APB1 peripheral reset register. Offset:0x20
	__vo uint32_t APB2RSTR;				// RCC APB2 peripheral reset register. Offset:0x24
	uint32_t RESERVED_2[2];				// Reserved/Unused. Offset:0x28-2C
	__vo uint32_t AHB1ENR;				// RCC AHB1 peripheral clock enable register. Offset:0x30
	__vo uint32_t AHB2ENR;				// RCC AHB2 peripheral clock enable register. Offset:0x34
	__vo uint32_t AHB3ENR;				// RCC AHB3 peripheral clock enable register. Offset:0x38
	uint32_t RESERVED_3;				// Reserved/Unused. Offset:0x3C
	__vo uint32_t APB1ENR;				// RCC APB1 peripheral clock enable register. Offset:0x40
	__vo uint32_t APB2ENR;				// RCC APB2 peripheral clock enable register. Offset:0x44
	uint32_t RESERVED_4[2];				// Reserved/Unused. Offset:0x48-4C
	__vo uint32_t AHB1LPENR;			// RCC AHB1 peripheral clock enable in low power mode register. Offset:0x50
	__vo uint32_t AHB2LPENR;			// RCC AHB2 peripheral clock enable in low power mode register. Offset:0x54
	__vo uint32_t AHB3LPENR;			// RCC AHB3 peripheral clock enable in low power mode register. Offset:0x58
	uint32_t RESERVED_5;				// Reserved/Unused. Offset:0x5C
	__vo uint32_t APB1LPENR;			// RCC APB1 peripheral clock enable in low power mode register. Offset:0x60
	__vo uint32_t APB2LPENR;			// RCC APB2 peripheral clock enable in low power mode register. Offset:0x64
	uint32_t RESERVED_6[2];				// Reserved/Unused. Offset:0x68-6C
	__vo uint32_t BDCR;					// RCC Backup domain control register . Offset:0x70
	__vo uint32_t CSR;					// RCC clock control & status register . Offset:0x74
	uint32_t RESERVED_7[2];				// Reserved/Unused. Offset:0x78-7C
	__vo uint32_t SSCGR;				// RCC spread spectrum clock generation register. Offset:0x80
	__vo uint32_t PLLI2SCFGR;			// RCC PLLI2S configuration register. Offset:0x84
} RCC_RegDef_t;


typedef struct {
	__vo uint32_t IMR;					// Interrupt mask register. Offset: 0x00
	__vo uint32_t EMR;					// Event mask register. Offset 0x04
	__vo uint32_t RTSR;					// Rising trigger selection register. Offset 0x08
	__vo uint32_t FTSR;					// Falling trigger selection register. Offset 0x0C
	__vo uint32_t SWIER;				// Software interrupt event register. Offset 0x10
	__vo uint32_t PR;					// Pending Register. Offset 0x14
} EXTI_RegDef_t;


typedef struct {
	__vo uint32_t MEMRMP;				// Memory remap register. Offset: 0x00
	__vo uint32_t PMC;					// Peripheral mode configuration register. Offset: 0x04
	__vo uint32_t EXTICR[4];			// External interrupt configuration registers. Offset: 0x08-0x14
	uint32_t RESERVED[2];				// Reserved/Unused. Offset: 0x18-0x1C
	__vo uint32_t CMPCR;				// Compensation cell control register. Offset: 0x20
} SYSCFG_RegDef_t;

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

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

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

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

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

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))

/*
 * GPIO Register Reset Macros
 */

#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) // Set then reset.
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0) // Set then reset.
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0) // Set then reset.
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0) // Set then reset.
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0) // Set then reset.
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0) // Set then reset.
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0) // Set then reset.
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0) // Set then reset.
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0) // Set then reset.

/*
 * Calculate port-code from GPIO_BASEADDR.
 */

#define GPIO_BASEADDR_TO_PORTCODE(x)		 ((x == GPIOA)?0:\
											  (x == GPIOB)?1:\
											  (x == GPIOC)?2:\
											  (x == GPIOD)?3:\
											  (x == GPIOE)?4:\
										      (x == GPIOF)?5:\
											  (x == GPIOG)?6:\
											  (x == GPIOH)?7:\
											  (x == GPIOI)?8:0)


/*
 * IRQ (Interrupt Requests) numbers for stm32407xx.
 * THIS IS NOT A COMPLETE LIST
 * TODO - Complete this list for other peripherals.
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
#define IRQ_NO_SPI4			84

/*
 * Interrupt priority macros.
 */
#define NVIC_IRQ_PRIO_0		0
#define NVIC_IRQ_PRIO_1		1
#define NVIC_IRQ_PRIO_2		2
#define NVIC_IRQ_PRIO_3		3
#define NVIC_IRQ_PRIO_4		4
#define NVIC_IRQ_PRIO_5		5
#define NVIC_IRQ_PRIO_6		6
#define NVIC_IRQ_PRIO_7		7
#define NVIC_IRQ_PRIO_8		8
#define NVIC_IRQ_PRIO_9		9
#define NVIC_IRQ_PRIO_10	10
#define NVIC_IRQ_PRIO_11	11
#define NVIC_IRQ_PRIO_12	12
#define NVIC_IRQ_PRIO_13	13
#define NVIC_IRQ_PRIO_14	14
#define NVIC_IRQ_PRIO_15	15

/*
 * Generic Macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				1
#define LOW					0
#define FLAG_SET			SET
#define FLAG_RESET			RESET

/*
 * Bit positions for SPI peripheral.
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRF			8


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
