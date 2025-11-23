#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

#define MAX_LEN 500 //Max len SPI
#define MY_ADDR 0x61 // Device addr I2C
#define SLAVE_ADDR 0x68 // Slave device addr I2C

// I2C Globals
I2C_Handle_t I2C1handle;
uint8_t rcv_buffer[32];
uint8_t tx_buffer[32] = "STM32 Slave Mode Test Msg";

// SPI Globals
SPI_Handle_t SPI2handle;
uint8_t ReadByte;
char RcvBuff[MAX_LEN];
volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0; // Flag that is set on trigger of GPIO interrupt.

// UART Globals
USART_Handle_t USART2Handle;
char *msg[3] = {"Example Sentence with Different Cases", "Hello How are you ?" , "Jonny is sometimes a reasonably good coder"};
char rx_buf[1024] ; //reply from arduino will be stored here
uint8_t rxCmplt = RESET;  //This flag indicates reception completion
uint8_t g_data = 0;

// Arbitrary delay function.
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

void I2C1_GPIOInits(void)
{
	/*
	 * PB6 --> I2C1_SCL
	 * PB7 --> I2C1_SDA
	 */

	//Configuration of the GPIO Pins to act as the I2C1 interface.
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
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

void USART2_GPIOInits(void) {
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	GPIO_Init(&USARTPins);

	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USARTPins);
}


void I2C1_Inits(void)
{
	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = SLAVE_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1handle);
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

void USART2_Init(void) {
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART2Handle);
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

	int32_t cnt = 0;


	initialise_monitor_handles();

	USART2_GPIOInits();
	USART2_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);

	printf("Application is running\n");

	//do forever
	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&USART2Handle, (uint8_t*)rx_buf, strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
		USART_SendData(&USART2Handle,(uint8_t*)msg[cnt],strlen(msg[cnt]));

		printf("Transmitted : %s\n",msg[cnt]);


		//Now lets wait until all the bytes are received from the arduino .
		//When all the bytes are received rxCmplt will be SET in application callback
		while(rxCmplt != SET);

		//just make sure that last byte should be null otherwise %s fails while printing
		rx_buf[strlen(msg[cnt])+ 1] = '\0';

		//Print what we received from the arduino
		printf("Received    : %s\n",rx_buf);

		//invalidate the flag
		rxCmplt = RESET;

		//move on to next message indexed in msg[]
		cnt ++;
	}


	return 0;
}




void SPI_send_recieve_example() {
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

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1handle);
}

void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	static uint8_t commandCode = 0;
	static uint8_t cnt = 0;

	if (AppEv == I2C_EV_DATA_REQ)
	{
		// Master device has requested data.
		// Slave device has to send data.
		if (commandCode == 0x51)
		{
			// Send the length of of the tx buffer.
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buffer));
		}
		else if (commandCode == 0x52)
		{
			// Send the contents of the tx buffer.
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buffer[cnt++]);
		}

	}
	else if (AppEv == I2C_EV_DATA_RCV) {
		// Slave has to recieve data.
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}
	else if (AppEv == I2C_ERROR_AF) {
		// This can only happen during slave transmission.
		// Master device replied with a NACK and doesnt need more data.
		commandCode = 0xFF;
		cnt = 0;

	}
	else if (AppEv == I2C_EV_STOP) {
		// This can only happen during slave reception.
		// Master device has ended communication with the slave.
	}


}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&USART2Handle);
}





void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}
