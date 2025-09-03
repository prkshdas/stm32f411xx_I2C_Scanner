/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: victus
 */

#include "stm32f411xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/* Peripheral Clock setup */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/* Init and De-init */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	// Enable peripheral clock
	I2C_PeriClockCtrl(pI2CHandle->pI2Cx, ENABLE);

	// ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// Configure the FREQ of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// Device address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else {
		// Fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		do { (RCC->APB1RSTR |= (1 << RCC_APB1ENR_I2C1EN)); (RCC->APB1RSTR &= ~(1 << RCC_APB1ENR_I2C1EN)); } while (0);
	} else if (pI2Cx == I2C2) {
		do { (RCC->APB1RSTR |= (1 << RCC_APB1ENR_I2C2EN)); (RCC->APB1RSTR &= ~(1 << RCC_APB1ENR_I2C2EN)); } while (0);
	} else if (pI2Cx == I2C3) {
		do { (RCC->APB1RSTR |= (1 << RCC_APB1ENR_I2C3EN)); (RCC->APB1RSTR &= ~(1 << RCC_APB1ENR_I2C3EN)); } while (0);
	}
}

/* IRQ Configuration and ISR handling */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* Other peripheral control APIs */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/* I2C send data */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/* I2C Receive Data */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if (Len == 1) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	if (Len > 1) {
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		for (uint32_t i = Len; i > 0; i--) {
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if (i == 2) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
		}
	}

	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}
