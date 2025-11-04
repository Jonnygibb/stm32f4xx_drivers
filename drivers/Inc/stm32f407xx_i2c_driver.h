/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jul 5, 2025
 *      Author: Jonathan Gibbons
 */

 #ifndef INC_STM32F407XX_I2C_DRIVER_H_
 #define INC_STM32F407XX_I2C_DRIVER_H_
 
 #include "stm32f407xx.h"

/*
 * Configuration Strucuture for I2C Peripheral
 */
typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * Handle Structure for I2X Peripheral
 */
typedef struct {
    I2C_RegDef_t 	*pI2Cx;
    I2C_Config_t 	I2C_Config;
    uint8_t			*pTxBuffer;
    uint8_t			*pRxBuffer;
    uint32_t		TxLen;
    uint32_t		RxLen;
    uint8_t			TxRxState;
    uint8_t			DevAddr;
    uint32_t		RxSize;
    uint8_t			Sr;
} I2C_Handle_t;

/*
 * @I2C_SCLSpeed 
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM2K      200000
#define I2C_SCL_SPEED_FM4K      400000


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1


/*
 * I2C related status flag definitions.
 */
#define I2C_FLAG_TXE				(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE				(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)

// Repeated start parameters.
#define I2C_DISABLE_SR					DISABLE
#define I2C_ENABLE_SR					ENABLE

/*
 * I2C Application Events Macros
 */
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR			3
#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF			5
#define I2C_ERROR_OVR			6
#define I2C_ERROR_TIMEOUT		7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9

/*
 * I2C Application States
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2


#define I2C_READ				1
#define I2C_WRITE				0


/********************************************************************************************
 * 								APIs supported by this driver
 ********************************************************************************************/

/******************************************************************************
 * I2C Driver API that enables or disables the Peripheral Clock for the given
 * I2C interface. ENABLE or DISABLE macro can be used for parameter EnOrDi.
 *
 * @param *pI2Cx A pointer to the base address of the I2C interface using the
 * 					defined register structure.
 * @param EnOrDi Integer value to enable or disable the peripheral clock. Can
 * 					be 1 or 0. Alternatively use macro ENABLE or DISABLE.
 ******************************************************************************/
 void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

 /******************************************************************************
 * Controls whether the I2C peripheral is on or off.
 *
 * @param pI2Cx A pointer to the base address of the I2C interface using the
 * 				defined register structure.
 * @param EnOrDi Value to represent whether I2C peripheral will be enabled
 * 				 or disabled.
 ******************************************************************************/
 void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

 /******************************************************************************
  * Initialisation function that configures an I2C interface based on the parameters
  * of the I2CHandle structure.
  *
  * @param *pI2CHandle A pointer to a I2C_Handle_t structure that is contains
  * 						the desired configuration for the I2C interface.
  ******************************************************************************/
 void I2C_Init(I2C_Handle_t *pI2CHandle);
 
 /******************************************************************************
  * De-initialisation function that sets and clears the reset register for the
  * I2C interface referenced in pI2Cx.
  *
  * @param *pI2Cx A pointer to the base address of the I2C interface using the
  * 					defined register structure.
  ******************************************************************************/
 void I2C_DeInit(I2C_RegDef_t *pI2Cx);
 
 /******************************************************************************
  * Sends data over I2C to desired slave device.
  *
  * @param *pI2CHandle 	A pointer to the handle structure for an I2C Interface.
  * @param *pTxBuffer 	A pointer to the buffer containing the data to be sent.
  * @param len			The length of data to be sent.
  * @param SlaveAddr	The address of the slave device that should listen for
  * 					the data.
  ******************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SalveAddr, uint8_t Sr);

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

 // Interrupt based versions of I2C Master send/receive.
 uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SalveAddr, uint8_t Sr);

 uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

 /******************************************************************************
  * Checks the status of a user defined flag. List of flags can be found in
  * @FlagName.
  *
  * @param *pI2Cx A pointer to the base address of the I2C interface using the
  * 					defined register structure.
  * @param FlagName A user defined flag to check the status of.
  *
  */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

#endif /* INC_STM32F407XX_I2C`_DRIVER_H_ */
