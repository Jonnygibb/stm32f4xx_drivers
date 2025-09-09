/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Jonathan Gibbons
 */

#include "stm32f407xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * SPI initialisation function
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	uint32_t tempreg = 0;

	// Enable the peripheral clock.
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Set the device mode.
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Set the bus config to full duplex, half or simplex.
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_FD) {
		// Clear the bidirectional bit. Enables full duplex 2 wire
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_HD) {
		// Set the bidirectional bit. Enables on half duplex 1 wire
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_SIMP_RXONLY) {
		// Clear the bidirectional bit and set the rxonly bit.
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// Set the SPI speed scaling factor.
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// Configure the Data Frame Format.
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Configure the clock polarity CPOL.
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Configure the clock phase CPHA.
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// Configure the use of software slave management.
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Set the register based on the SPI config.
	pSPIHandle->pSPIx->CR1 = tempreg;

}
/*
 * SPI De-initialisation function.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	// TODO
}

/*
 * SPI get flag status function.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * SPI Send data function.
 * This API can be considered a blocking or polling setting since
 * it uses while loops and continuous polling of the status register.
 *
 * An interrupt based approach is better and will be implemented later.
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while(len > 0){

		// Wait until the TX buffer is empty.
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)  == (uint8_t)FLAG_RESET);

		// Check the data frame format, 16bit or 8bit.
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16bit DFF
			// Typecast here since the pointer is of uint8_t but
			// two bytes of data are being sent in 16bit DFF.
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			// Increment the pointer to the next buffer item.
			(uint16_t*)pTxBuffer++;
		} else {
			// 8bit DFF
			pSPIx->DR = *pTxBuffer;
			len--;
			// Increment the pointer to the next buffer item.
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while(len > 0){

		// Wait until the RX buffer is full.
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET);

		// Check the data frame format, 16bit or 8bit.
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16bit DFF
			// Typecast here since the pointer is of uint8_t but
			// two bytes of data are being recieved in 16bit DFF.
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			// Increment the pointer to the next buffer item.
			(uint16_t*)pRxBuffer++;
		} else {
			// 8bit DFF
			*pRxBuffer = pSPIx->DR;
			len--;
			// Increment the pointer to the next buffer item.
			pRxBuffer++;
		}
	}
}

/*
 * API function for enabling SPI peripheral.
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * API function to toggle the internal slave select pin when
 * using software slave selection mode.
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*
 * API function to toggle the slave select output.
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->TxState;

	// Only attempt to transmit if the SPI peripheral is not busy transmitting.
	if(state != SPI_BUSY_IN_TX){
		// Save the Tx buffer & length in a global variable. In this case the
		// SPI handle was modified.
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// Mark the SPI peripheral as busy during this transmission.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE to cause an interrupt to be raised when TXE flag is set.
		// I.E when the transmit buffer is empty, interrupt.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->RxState;

	// Only attempt to receive if the SPI peripheral is not busy receiving.
	if(state != SPI_BUSY_IN_RX){
		// Save the Rx buffer & length in a global variable. In this case the
		// SPI handle was modified to accomodate the new variables.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// Mark the SPI peripheral as busy during this reception of data.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RNNEIE to cause an interrupt to be raised when RXNE flag is set.
		// I.E when the receive buffer is NOT empty, interrupt.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint8_t temp1, temp2;

	// First, check whether transmit buffer is empty.
	// TXEIE = Transmit buffer empty interrupt enable.
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2) {
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Next, check whether receive buffer is not empty.
	// RXNEIE = Receive buffer not empty interrupt enable.
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2) {
		// Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// Finally, check whether an overrun error has occured.
	// Overrun = next frame received while the previous frame
	// has not yet been completed.
	// ERRIE = Error interrupt enable.
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2) {
		// Handle OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Check the data frame format, 16bit or 8bit.
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		// 16bit DFF
		// Typecast here since the pointer is of uint8_t but
		// two bytes of data are being sent in 16bit DFF.
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		// Increment the pointer to the next buffer item.
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		// 8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		// Increment the pointer to the next buffer item.
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen) {
		// No more data left to transmit if TxLen is zero.
		// Close SPI transmission and inform application.

		// Clear the transmit buffer empty interrupt.
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);


	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Check the data frame format, 16bit or 8bit.
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		// 16bit DFF
		// Typecast here since the pointer is of uint8_t but
		// two bytes of data are being received in 16bit DFF.
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;	// Reduce the receive length by 2.
		pSPIHandle->pRxBuffer++; // Move the pointer forward by 2 bytes.
		pSPIHandle->pRxBuffer++;
	} else {
		// 8bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++; // Move the pointer forward 1 byte.
	}

	if(!pSPIHandle->RxLen) {
		// No more data left to recieve if RxLen is zero.
		// Close SPI transmission and inform application.

		// Clear the receive buffer not empty interrupt.
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Clear the overrun flag.
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// Inform the application.
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event) {
	// Weak Declaration. Application should override this.
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	// Clear the transmit buffer empty interrupt.
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	// Clear the receive buffer not empty interrupt.
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
