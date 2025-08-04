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
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
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
  * Checks the status of a user defined flag. List of flags can be found in
  * @FlagName.
  *
  * @param *pI2Cx A pointer to the base address of the I2C interface using the
  * 					defined register structure.
  * @param FlagName A user defined flag to check the status of.
  *
  */
 uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

#endif /* INC_STM32F407XX_I2C`_DRIVER_H_ */
