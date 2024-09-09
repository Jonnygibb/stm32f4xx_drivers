/*
 * LED_toggle.c
 *
 *  Created on: Aug 25, 2024
 *      Author: Jonathan Gibbons
 */

#include <string.h>
#include "stm32f407xx.h"

void delay() {
	// Basic delay
	for(uint32_t i = 0; i < 250000; i++);
}

void led_toggle() {
	GPIO_Handle_t GpioLed;

		// Configure the GPIO port and pin for LED PD12
		GpioLed.pGPIOx = GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		// Enable the peripheral clock for GPIOD.
		GPIO_PeriClockControl(GPIOD, ENABLE);
		// Use the configuration to initialise the GPIO port and pin.
		GPIO_Init(&GpioLed);

		// Continue to toggle the LED on port D Pin 12.
		while(1) {
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay();
		}
}


void led_button_toggle() {
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Configure the GPIO port and pin for LED PD12
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Configure input pin for on-board user button
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	// Enable the peripheral clock for GPIOD and GPIOA.
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Use the configuration to initialise the GPIO port and pin.
	// Initialise both the LED and the user button.
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	while(1) {
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == HIGH) {
			delay();	// Add delay to deal with button de-bouncing.
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
}

void led_interrupt() {
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Configure the GPIO port and pin for LED PD12
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Configure input pin for on-board user button.
	// This button will be used to trigger an interrupt.
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Enable the peripheral clock for GPIOD and GPIOA.
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Use the configuration to initialise the GPIO port and pin.
	// Initialise both the LED and the user button.
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	// Set the priority of the interrupt to 15.
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);

	// Enabled the IRQ EXTI0 line since the user button is portA, pin0.
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);
}

void EXTI0_IRQHandler() {
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}


int main(void) {
	led_interrupt();
	return 0;
}
