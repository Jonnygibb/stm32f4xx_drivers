/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Jonathan Gibbons
 */

#include "stm32f407xx_spi_driver.h"

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
