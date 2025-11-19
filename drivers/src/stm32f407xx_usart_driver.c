/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Nov 19, 2025
 *      Author: jonat
 */

#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= (1 << USART_SR1_EN);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_SR1_EN);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName) {
	if (pUSARTx->USART_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName) {
	pUSARTx->USART_SR &= ~(FlagName);
}



