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
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// Set the bus config to full duplex, half or simplex.
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_FD) {
		// Clear the Bidi bit.
		tempreg &= ~(1 << 15);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_HD) {
		// Set the Bidi bit.
		tempreg |= (1 << 15);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONF_SIMP_RXONLY) {
		// Clear the Bidi bit and set the rxonly bit.
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
	}
}
