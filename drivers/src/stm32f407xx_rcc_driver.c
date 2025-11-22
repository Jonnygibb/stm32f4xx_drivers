/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Nov 22, 2025
 *      Author: jonat
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPLLOutputClock(void) {
	return 0;
}

/*
 * Calculates the frequency of the Peripheral clock. 1.
 */
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		SystemClk = 16000000;
	} else if (clksrc == 1) {
		SystemClk = 8000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 8) {
		apb1p = 1;
	} else {
		apb1p = APB_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		SystemClk = 16000000;
	} else if (clksrc == 1) {
		SystemClk = 8000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 13) & 0x7);

	if (temp < 8) {
		apb2p = 1;
	} else {
		apb2p = APB_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp) / apb2p;

	return pclk2;
}
