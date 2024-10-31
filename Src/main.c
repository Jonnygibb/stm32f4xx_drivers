/*
 * LED_toggle.c
 *
 *  Created on: Aug 25, 2024
 *      Author: Jonathan Gibbons
 */

#include <string.h>
#include "stm32f407xx.h"
#include <stdio.h>

// SPI command codes
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LEF_OFF						0

//arduino analog pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4

// Arduino LED
#define LED_PIN						9

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

void GPIO_BtnInits() {
	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Configure input pin for on-board user button.
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

void SPI2_GPIOInits() {
	/*
	 * SPI2 MOSI == PB15
	 * SPI2 MISO == PB14
	 * SPI2 SCLK == PB13
	 * SPI2 NSS  == PB12
	 * Alternate function 5
	 */
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);


	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);


	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits() {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONF_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}


void SPI_SendTest() {
	// This function expects an arduino slave module running the
	// sketch 001SPISlaveRxString.ino

	// Establish the GPIO pins into SPI2 alternate function.
	SPI2_GPIOInits();

	// Set the configuration of the SPI peripheral.
	SPI2_Inits();

	// Initialise the user button on the microcontroller.
	GPIO_BtnInits();

	// Sets the internal slave select pin (SSI) to high to
	// avoid MODF error.
	//SPI_SSIConfig(SPI2, ENABLE);

	// Setting SSOE will set the NSS line to low and the
	// slave device will be activated.
	// When SSOE is disabled, multiple master nodes could
	// be used and the NNS will be High.
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1) {
		// Wait until the user button is pressed to send on SPI.
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// Avoid problem with the button debouncing.
		delay();

		// Enable the SPI peripheral.
		SPI_PeripheralControl(SPI2, ENABLE);

		// Create a test buffer to transmit.
		char test_data[] = "According to all known laws of aviation, there is no way a bee should be able to fly";

		// Arduino sketch expects the length of data its recieving
		// before any actual data.
		uint8_t data_length = strlen(test_data);
		SPI_SendData(SPI2, &data_length, 1);

		// Send the data over the desired SPI peripheral.
		SPI_SendData(SPI2, (uint8_t*)test_data, strlen(test_data));

		// While the SPI is busy, do not disable the peripheral.
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// Disable the SPI peripheral after the send operation.
		SPI_PeripheralControl(SPI2, DISABLE);
	}
}


uint8_t SPI_VerifyResponse(uint8_t ack_byte) {
	if (ack_byte == (uint8_t)0xF5) {
		return 1;
	} else {
		return 0;
	}
}


void SPI_CommandResponse() {
	// This function expects an arduino slave module running the
	// sketch 002SPISlaveCmdHandling.ino

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_BtnInits();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI Init. done\n");

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2,args,2);
			// dummy read
			SPI_ReceiveData(SPI2,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}

		//end of COMMAND_LED_CTRL




		//2. CMD_SENOSR_READ   <analog pin number(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");
		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("SPI Communication Closed\n");
	}
}


int main() {
	SPI_CommandResponse();
	return 0;
}
