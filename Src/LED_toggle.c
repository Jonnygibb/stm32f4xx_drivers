/*
 * LED_toggle.c
 *
 *  Created on: Aug 25, 2024
 *      Author: Jonathan Gibbons
 */

#include "stm32f407xx.h"

void delay() {
	// Basic delay
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
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
	return 0;
}
