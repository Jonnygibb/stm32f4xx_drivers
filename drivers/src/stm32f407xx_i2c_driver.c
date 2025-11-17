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
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

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

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	//	uint32_t dummyRead = pI2Cx->I2C_SR1;
	//	dummyRead = pI2Cx->I2C_SR2;
	//	(void)dummyRead;

	uint32_t dummy_read;

	// Check what mode the device is in
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
		// Device is Master
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;
			}
		} else {
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
	} else {
		// Device is in Slave mode
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
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
	tempreg = 0;
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
 void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr) {
	 // Generate the Start condition.
	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	 // Confirm that the start generation has completed by checking whether SB has been cleared.
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	 // Send the address of the desired slave with r/nw bit set.
	 I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_WRITE);

	 // Confirm that the ADDR flag in SR1 is
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	 // Clear the ADDR flag by reading SR1 and then SR2.
	 I2C_ClearADDRFlag(pI2CHandle);

	 // Send data until the length becomes zero.
	 while(len > 0){
		 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // Wait until the transmit buffer is empty.
		 pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		 pTxBuffer++;
		 len--;
	 }

	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	 if (Sr == I2C_DISABLE_SR) {
		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	 }
 }


 /*
  * Recieve data as a master node via I2C.
  */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
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

 		//clear the ADDR flag
 		I2C_ClearADDRFlag(pI2CHandle);

 		//wait until  RXNE becomes 1
 		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

 		if (Sr == I2C_DISABLE_SR){
			//generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

 		//read data in to buffer
 		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

 	}


     //procedure to read data from slave when Len > 1
 	if(Len > 1)
 	{
 		//clear the ADDR flag
 		I2C_ClearADDRFlag(pI2CHandle);

 		//read the data until Len becomes zero
 		for ( uint32_t i = Len ; i > 0 ; i--)
 		{
 			//wait until RXNE becomes 1
 			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

 			if(i == 2) //if last 2 bytes are remaining
 			{
 				//Disable Acking
 				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

 				if (Sr == I2C_DISABLE_SR){
 					//generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
 				}

 			}

 			//read the data from data register in to buffer
 			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

 			//increment the buffer address
 			pRxBuffer++;

 		}

 	}
 	//re-enable ACKing
 	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
 		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
 	}

 }


uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);


	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
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
	} else {
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	// Disable the ITBUFEN bit.
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable the ITEVTEN bit.
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	// Disable the ITBUFEN bit.
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable the ITEVTEN bit.
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->TxLen > 0) {
		// Load the data into the DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
		// Decrement the TxLen
		pI2CHandle->TxLen--;
		// Increment the buffer
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

		pI2CHandle->RxLen--;

	}

	if(pI2CHandle->RxSize > 1) {
		if(pI2CHandle->RxLen == 2) {
			// Clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// Read from the data register
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0) {
		// Close the I2C data reception and notify the application

		if(pI2CHandle->Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t)pI2Cx->I2C_DR;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	// Handle when interrupt is generated by SB event.
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3) {
		// SB Flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3) {
		// ADDR Flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3) {
		// BTF Flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			// Make sure that TXE is also set.
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE)) {
				// BTF & TXE = 1
				if(pI2CHandle->TxLen == 0) {
					// Generate STOP condition.
					if(pI2CHandle->Sr == I2C_DISABLE_SR) {
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// Reset the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					// Notify the application about a complete transmission.
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);

	if(temp1 && temp3) {
		// STOP Flag is set
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;
		// Notify the application that the STOP condition has been detected.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3) {
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
			// TXE Flag is set and device is Master
			// Data Transmission is needed.
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else {
			// Slave mode data handling.
			// Decide whether the device is in transmitter mode.
			if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3) {
		// Ensure device mode is Master
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
			// RXNE Flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				// Data Reception is needed.
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			// Slave mode data handling.
			// Make sure the slave is really in reciever mode.
			if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}

	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//This is Bus error

		//Implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

	/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}





