/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 24, 2024
 *      Author: Jonathan Gibbons
 */


#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Initialisation and De-Initialisation
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// Configure the mode of the GPIO Pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Generate the mask for the Pin Mode register. Multiply by 2 since the pin register is 2 bits wide.
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear bit positions.
		pGPIOHandle->pGPIOx->MODER |= temp; // Save the value in the GPIO Mode register.
	} else {
		//TODO - Configure settings for interrupt modes
	}

	temp = 0;	// Reset the temp value

	// Configure the Speed of the GPIO pin.
	// Create another bit shift to place the desired speed in the correct pins register position.
	// Multiply by 2 again since bit field is 2 bits wide.
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear bit positions.
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// Configure the pull-up/pull-down state of the GPIO pin.
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear bit positions.
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// Configure the output type of the GPIO pin. No need to multiply by 2 since only 1 bit wide register.
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear bit position.
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// Configure the alternate functionality registers (if necessary).
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		// Used to determine high or low AFR.
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		// Determines mask for register.

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 * temp2));
		// Multiply by 4 here since alternate function register is 4 bits wide.
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}

/*
 * Read from input pin and port.
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	// Shift desired bit to the least significant bit position.
	// Mask all other bits using AND operator.
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x0000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	// cast the entire GPIO output data register to a value.
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*
 * Write date to output pin and port.
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber); // Set the desired output value.
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber); // Clear the desired output value.
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value; // Write the entire value to the output data register.
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}
