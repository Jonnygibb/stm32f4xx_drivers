/*
 * Application responsible for received a message from a SPI follower module and printing it to the terminal over SWV.
 */
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

// Max length in bytes of messages that can be received.
#define MAX_LEN 500
char RcvBuff[MAX_LEN];

SPI_Handle_t SPI2handle;

uint8_t ReadByte;

volatile uint8_t rcvStop = 0;

// Flag that is set on trigger of GPIO interrupt.
volatile uint8_t dataAvailable = 0;

// Arbitrary delay function.
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}



void SPI2_GPIOInits(void)
{
	/*
	 * PB14 --> SPI2_MISO
	 * PB15 --> SPI2_MOSI
	 * PB13 -> SPI2_SCLK
	 * PB12 --> SPI2_NSS
	 * ALT function mode : 5
	 */

	//Configuration of the GPIO Pins to act as the SPI2 interface.

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}


void SPI2_Inits(void)
{
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONF_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}


// Configures the GPIO pin for interrupt to receive data.
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t InterruptPin;
	memset(&InterruptPin,0,sizeof(InterruptPin));

	InterruptPin.pGPIOx = GPIOD;
	InterruptPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	InterruptPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	InterruptPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	InterruptPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&InterruptPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRIO_15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5,ENABLE);

}


int main(void)
{
	// Placeholder to receive
	uint8_t placeholder_byte = 0xff;

	Slave_GPIO_InterruptPinInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 pins.
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters.
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	// Enable the SPI interrupts for the SPI2 peripheral.
	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	while(1){

		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		GPIO_IRQConfig(IRQ_NO_EXTI9_5,DISABLE);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_SendDataIT(&SPI2handle,&placeholder_byte,1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveDataIT(&SPI2handle,&ReadByte,1) == SPI_BUSY_IN_RX );
		}


		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQConfig(IRQ_NO_EXTI9_5,ENABLE);


	}

	return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}
