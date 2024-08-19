# Microcontroller Learnings

Some things I learnt that are worth writing down

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