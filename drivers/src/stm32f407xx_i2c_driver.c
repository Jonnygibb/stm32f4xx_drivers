/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: Jul 07, 2025
 *      Author: Jonathan Gibbons
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t rw);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t rw) {
	// Shift left to make room for r/w bit.
	SlaveAddr = SlaveAddr << 1;
	if (rw == I2C_READ) {
		SlaveAddr |= 1;
	} else {
		SlaveAddr &= ~(1);
	}
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummyRead = pI2Cx->I2C_SR1;
	dummyRead = pI2Cx->I2C_SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
    if(EnOrDi == ENABLE) {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    } else {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == I2C_ACK_ENABLE) {
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


/*
 * I2C Peripheral Control function
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->I2C_CR1 &= ~(1 << 0);
    }
}

uint32_t RCC_GetPLLOutputClock() {
	return 0;
}

/*
 * Calculates the frequency of the Peripheral clock.
 */
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;
	
	if (clksrc == 0) {
		SystemClk = 16000000;
	} else if (clksrc == 1) {
		SystemClk = 8000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 8) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}


/*
 * I2C initialisation function
 */
 void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	// Enable the clock for the I2Cx peripheral.
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Ack Control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

	// Configure the FREQ fields
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	// Configure the devices own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard Mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		// Mask bits since CCR is only 12 bits
		tempreg |= (ccr_value & 0xFFF);
	} else {
		// Fast Mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempreg;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard Mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		// Fast Mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}


 /*
  * I2C get flag status function.
  */
 uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
 	if (pI2Cx->I2C_SR1 & FlagName) {
 		return FLAG_SET;
 	}
 	return FLAG_RESET;
 }


/*
 * Send data as a master node via I2C.
 */
 void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr) {
	 // Generate the Start condition.
	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	 // Confirm that the start generation has completed by checking whether SB has been cleared.
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	 // Send the address of the desired slave with r/nw bit set.
	 I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_WRITE);

	 // Confirm that the ADDR flag in SR1 is
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	 // Clear the ADDR flag by reading SR1 and then SR2.
	 I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	 // Send data until the length becomes zero.
	 while(len > 0){
		 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // Wait until the transmit buffer is empty.
		 pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		 pTxBuffer++;
		 len--;
	 }

	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
 }


 /*
  * Recieve data as a master node via I2C.
  */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
 {

 	//1. Generate the START condition
	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

 	//2. confirm that start generation is completed by checking the SB flag in the SR1
 	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

 	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	 I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_READ);

 	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


 	//procedure to read only 1 byte from slave
 	if(Len == 1)
 	{
 		//Disable Acking
 		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

 		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

 		//clear the ADDR flag
 		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

 		//wait until  RXNE becomes 1
 		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

 		//read data in to buffer
 		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

 	}


     //procedure to read data from slave when Len > 1
 	if(Len > 1)
 	{
 		//clear the ADDR flag
 		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

 		//read the data until Len becomes zero
 		for ( uint32_t i = Len ; i > 0 ; i--)
 		{
 			//wait until RXNE becomes 1
 			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

 			if(i == 2) //if last 2 bytes are remaining
 			{
 				//Disable Acking
 				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

 				//generate STOP condition
 				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

 			}

 			//read the data from data register in to buffer
 			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

 			//increment the buffer address
 			*pRxBuffer++;

 		}

 	}
 	//re-enable ACKing
 	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
 		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
 	}

 }



