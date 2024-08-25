# Microcontroller Learnings

Concepts and tricks that I learnt whilst writing the drivers for the stm32f47xx microcontroller.

- [Volatile Keyword](#volatile-keyword)
- [Enabling Clocks](#enabling-clocks)
- [Pre-processor Directives and Operator Precedence](#pre-processor-directives-and-operator-precedence)
- [Header file documentation](#header-file-documentation)
- [Do while zero condition macros](#do-while-zero-condition-macros)

## Volatile Keyword

When compiling code with an optimistation level greater than 0, variables that **change** but are not given the volatile keyword will not be checked for changes during runtime. You **must** use the volatile keyword for a variable value that undergoes changes.

The compiler does this since reading values from memory is time consuming, so when set to a higher optimistion value the compiler skips the step of re-reading the value.

```c
#include <stdint.h>

#define EXAMPLE_SRAM_ADDRESS     (0x20000004U)

int main() {
		uint32_t    value = 0;
		uint32_t volatile *pointer = (uint32_t*) EXAMPLE_SRAM_ADDRESS;
		
		while(1) {
				if(value) break;
		}
		
		while(1);
		
		return 0;
}
```

Now that the volatile keyword is in place, the compiler knows that the memory location EXAMPLE_SRAM_ADDRESS is subject to a change in value.

## Enabling Clocks

Enabling clocks is as simple as picking the correct register and then later enabling the bits necessary using an OR/AND operator or a bit mask.

```c
#include<stdint.h>

#define RCC_BASE_ADDR              0x40023800UL

#define RCC_CFGR_REG_OFFSET        0x08UL

#define RCC_CFGR_REG_ADDR          (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )

#define GPIOA_BASE_ADDR            0x40020000UL

int main(void)
{
	uint32_t *pRccCfgrReg =  (uint32_t*) RCC_CFGR_REG_ADDR;

	// Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
	*pRccCfgrReg &= ~(0x3 << 21); //clear 21 and 22 bit positions

	//Configure MCO1 prescaler
	*pRccCfgrReg |= ( 1 << 24); // This would divide the clock by 5.
	*pRccCfgrReg |= ( 1 << 25);
	*pRccCfgrReg |= ( 1 << 26);
}
```

This code configures the HSI clock as the MCU’s clock

## Pre-processor Directives and Operator Precedence

The use of brackets is important in pre-processor directives since in a macro, the parameters are replaced into the macro in place. This has little effect on something like a memory address in the example below but is good practise regardless.

```c
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

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
```

However, in an example like the square root macro, operator precedence is very important.

```c
#define SQR(x) (x*x) // Poorly defined macro..

int main()
{
    int a, b=3;
    a = SQR(b+5);      // This translates to SQR(3+5*3+5) which gives 23, not the expected 64.
    printf("%d\n",a);
    return 0;
}
```

## Header file documentation

Documentation within the function prototype in the header file should tell the function user what the function does and how to use it. In-line code documentation should go in the source file and leave notes to anyone maintaining the codes logic on how it works. Documenting like in the example below from stm32f407xx_gpio_driver also allows for auto generating documentation.

```c
/******************************************************************************
 * Initialisation function that configures a GPIO Pin based on the parameters
 * of the GPIOHandle structure.
 *
 * @param *pGPIOHandle A pointer a GPIO_Handle_t structure that is contains
 * 						the desired configuration for the GPIO Pin.
 ******************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
```

## Do while zero condition macros

If theres ever a need to perform two or more actions in a macro, a do while zero condition macro allows for this. See the example below that sets and then clears a register value.

```c
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) // Set then reset.
```


