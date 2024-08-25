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
	uint8_t GPIO_PinNumber;				// Possible Pin number from @GPIO_PIN_NOS
	uint8_t GPIO_PinMode;				// Possible Pin modes from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				// Possible Pin Speeds from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;		// Possible Pin pull-up/pull-down config from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;				// Possible Pin output types from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle structure for GPIO pin
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;	// Pointer to base address of GPIOx.
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NOS
 * Macro for each of the available pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin possible modes
 */

#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4	// GPIO interrupt mode falling edge
#define GPIO_MODE_IT_RT			5	// GPIO interrupt mode rising edge
#define GPIO_MODE_IT_RFT		6	// GPIO interrupt mode rising falling edge

/*
 * @GPIO_PIN_SPEEDS
 * GPIO Pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3

/*
 * @GPIO_PIN_PUPD
 * GPIO Pin pull up and pull down states
 */
#define GPIO_NO_PUPD			0	// No pull up or pull down
#define GPIO_PIN_PU				1	// Pull up resistor
#define GPIO_PIN_PD				2	// Pull down resistor

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO Pin possible output types
 */
#define GPIO_OP_TYPE_PP			0	// Push-Pull
#define GPIO_OP_TYPE_OD			1	// Open Drain


/********************************************************************************************
 * 								APIs supported by this driver
 *******************************************************************************************/

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

/******************************************************************************
 * Initialisation function that configures a GPIO Pin based on the parameters
 * of the GPIOHandle structure.
 *
 * @param *pGPIOHandle A pointer a GPIO_Handle_t structure that is contains
 * 						the desired configuration for the GPIO Pin.
 ******************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/******************************************************************************
 * De-initialisation function that sets and clears the reset register for the
 * GPIO port referenced in pGPIOx.
 *
 * @param *pGPIOx A pointer to the base address of the GPIO port using the
 * 					defined register structure.
 ******************************************************************************/
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
