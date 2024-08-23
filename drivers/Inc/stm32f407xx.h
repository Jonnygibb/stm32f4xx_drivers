/*
 * stm32f407xx.h
 *
 *  Created on: Aug 20, 2024
 *      Author: jonat
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile


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



#endif /* INC_STM32F407XX_H_ */
