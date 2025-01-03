/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Sep 29, 2024
 *      Author: Jonathan Gibbons
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * Configuration Structure for SPI peripheral.
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPI peripheral.
 */

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t		 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	 TxLen;
	uint32_t	 RxLen;
	uint8_t		 TxState;
	uint8_t		 RxState;
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONF_FD				0
#define SPI_BUS_CONF_HD				1
#define SPI_BUS_CONF_SIMP_RXONLY	2

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1


/*
 * @SPI_SSM
 */
#define SPI_SSM_DI					0
#define SPI_SSM_EN					1

/*
 * @FlagName
 */
#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG				(1 << SPI_SR_CHSIDE)
#define SPI_UDS_FLAG				(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG				(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG				(1 << SPI_SR_MODF)
#define SPI_OCR_FLAG				(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG				(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG				(1 << SPI_SR_FRE)

/*
 * SPI Application States
 */
#define SPI_READY					0
#define SPI_BUSY_IN_RX				0
#define SPI_BUSY_IN_TX				0

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERROR			3
#define SPI_EVENT_CRC_ERR			4

/********************************************************************************************
 * 								APIs supported by this driver
 ********************************************************************************************/

/******************************************************************************
 * SPI Driver API that enables or disables the Peripheral Clock for the given
 * SPI interface. ENABLE or DISABLE macro can be used for parameter EnOrDi.
 *
 * @param *pSPIx A pointer to the base address of the SPI interface using the
 * 					defined register structure.
 * @param EnOrDi Integer value to enable or disable the peripheral clock. Can
 * 					be 1 or 0. Alternatively use macro ENABLE or DISABLE.
 ******************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/******************************************************************************
 * Initialisation function that configures a SPI interface based on the parameters
 * of the SPIHandle structure.
 *
 * @param *pSPIHandle A pointer to a SPI_Handle_t structure that is contains
 * 						the desired configuration for the SPI interface.
 ******************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle);

/******************************************************************************
 * De-initialisation function that sets and clears the reset register for the
 * SPI interface referenced in pSPIx.
 *
 * @param *pSPIx A pointer to the base address of the SPI interface using the
 * 					defined register structure.
 ******************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/******************************************************************************
 * Checks the status of a user defined flag. List of flags can be found in
 * @FlagName.
 *
 * @param *pSPIx A pointer to the base address of the SPI interface using the
 * 					defined register structure.
 * @param FlagName A user defined flag to check the status of.
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*****************************************************************************
 * Function to write data to the SPI tx data register. This is performed
 * through the SPI->DR register depending on whether a read or write action
 * is requested. Function will iterate until the entire message has been sent
 * over the requested peripheral.
 *
 * @param *pSPIx A pointer to the base address of the SPI interface using the
 * 					defined register structure.
 * @param *pTxBuffer A pointer to a buffer that will be writted to the SPI Tx
 * 					 buffer.
 * @param len User defined length to determine how much data to write.
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/*****************************************************************************
 * Function to read the data recieved by the SPI rx data register. This
 * action clears the SPI rx buffer and allows the stm32f407xx to receive more
 * data. The function reads a length given by the len param.
 *
 * @param *pSPIx A pointer to the base address of the SPI interface using the
 * 					defined register structure.
 * @param *pRxBuffer A pointer to a buffer to store information read from
 * 		   			 the SPI rx buffer.
 * @param len User defined length to determine how much data to read.
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*****************************************************************************
 * Function to write data to the SPI tx data register. This is performed
 * through the SPI->DR register depending on whether a read or write action
 * is requested. Function will iterate until the entire message has been sent
 * over the requested peripheral. This version of the API uses interrupts
 * rather than blocking calls.
 *
 * @param *pSPIHandle A pointer to a SPI_Handle_t structure that is contains
 * 						the desired configuration for the SPI interface.
 * @param *pTxBuffer A pointer to a buffer that will be writted to the SPI Tx
 * 					 buffer.
 * @param len User defined length to determine how much data to write.
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);

/*****************************************************************************
 * Function to read the data recieved by the SPI rx data register. This
 * action clears the SPI rx buffer and allows the stm32f407xx to receive more
 * data. The function reads a length given by the len param. This version of
 * the API uses interrupts rather than blocking calls.
 *
 * @param *pSPIHandle A pointer to a SPI_Handle_t structure that is contains
 * 						the desired configuration for the SPI interface.
 * @param *pRxBuffer A pointer to a buffer to store information read from
 * 		   			 the SPI rx buffer.
 * @param len User defined length to determine how much data to read.
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/******************************************************************************
 * Enables or disables the desired interrupt in the micro-controllers nested
 * vectored interrupt controller (NVIC).
 *
 * @param IRQNumber The desired interrupt request number to be enabled or
 * 					disabled.
 * @param EnOrDI	Integer value to enable or disable the peripheral clock. Can
 * 					be 1 or 0. Alternatively use macro ENABLE or DISABLE.
 ******************************************************************************/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/******************************************************************************
 * Sets the interrupt priority in the NVIC priority register.
 *
 * @param IRQNumber The desired interrupt request number for which the priority
 * 						should be changed.
 * @param IRQPriority The priority of the specified IRQNumber.
 ******************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/******************************************************************************
 * Clears the interrupt from the NVIC interrupt pending register.
 *
 * @param IRQNumber The desired interrupt request number for which the interrupt
 * 						should be handled.
 ******************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/******************************************************************************
 * Controls whether the SPI peripheral is on or off.
 *
 * @param pSPIx A pointer to the base address of the SPI interface using the
 * 				defined register structure.
 * @param EnOrDi Value to represent whether SPI peripheral will be enabled
 * 				 or disabled.
 ******************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/******************************************************************************
 * Controls the value of the Internal Slave Select bit which is necessary when
 * using Software Slave Management (SSM).
 *
 * @param pSPIx A pointer to the base address of the SPI interface using the
 * 				defined register structure.
 * @param EnOrDi Value to represent whether SPI peripheral will be enabled
 * 				 or disabled.
 ******************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*****************************************************************************
 * Configures the setting of the Slave select output enable bit in the
 * SPI_CR2 register.
 *
 * ENABLE == Slave select output is enabled. Stm32f407xx is the master.
 * 			 Mulitmaster not available.
 * DISABLE == Slave select output is disabled. Multimaster is available.
 *
 * @param pSPIx A pointer to the base address of the SPI interface using the
 * 				defined register structure.
 * @param EnOrDi Value to represent whether SPI peripheral will be enabled
 * 				 or disabled.
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
