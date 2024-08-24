/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Jonathan Gibbons
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle structure for GPIO pin
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;	// Pointer to base address of GPIOx.
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;


/********************************************************************************************
 * 								APIs supported by this driver
 *******************************************************************************************/

/*
 * Peripheral Clock Setup
 */

/******************************************************************************
 * GPIO Driver API that enables or disables the Peripheral Clock for the given
 * GPIO port. ENABLE or DISABLE macro can be used for parameter EnOrDi.
 *
 * @param *pGPIOx A pointer to the base address of the GPIO port using the
 * 					defined register structure.
 * @param EnOrDi Integer value to enable or disable the peripheral clock. Can
 * 					be 1 or 0. Alternatively use macro ENABLE or DISABLE.
 ******************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Initialisation and De-Initialisation
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
