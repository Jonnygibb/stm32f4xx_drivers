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

void SPI2_GPIOInits() {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);


	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);


	// NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits() {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONF_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}


void SPI_SendTest() {
	/*
	 * SPI2 MOSI == PB15
	 * SPI2 MISO == PB14
	 * SPI2 SCLK == PB13
	 * SPI2 NSS  == PB12
	 * Alternate function 5
	 */

	// Establish the GPIO pins into SPI2 alternate function.
	SPI2_GPIOInits();

	// Set the configuration of the SPI peripheral.
	SPI2_Inits();

	// Sets the internal slave select pin (SSI) to high to
	// avoid MODF error.
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI peripheral.
	SPI_PeripheralControl(SPI2, ENABLE);

	// Create a test buffer to transmit.
	char test_data[] = "Hello World!";

	// Send the data over the desired SPI peripheral.
	SPI_SendData(SPI2, (uint8_t*)test_data, strlen(test_data));
}


int main(void) {
	SPI_SendTest();
	// Loop forever.
	while(1);
}
