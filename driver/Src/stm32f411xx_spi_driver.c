/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 19, 2025
 *      Author: victus
 */

#include "stm32f411xx_spi_driver.h"

/* Peripheral Clock setup*/
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {

		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}

	} else {
//		Disable clock
	}
}

/* Init and De-init */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

//	configure the SPI_CR1 register
	uint32_t tempreg = 0;

//	SPI peripheral clock enable
	SPI_PeriClockCtrl(pSPIHandle->pSPIx, ENABLE);

//	congigure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

//	configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
//		bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
//		bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
//		bidi mode should be cleared
//		RXONLY bit must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// Configure the spi serial clock speed( baud rate)

	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// Configure the DFF

	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// Configure the CPOL

	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// Configure the CPHA

	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* Data send and Receive */
/***************************** Blocking Call **********************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		// wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16 but DFF
			// load the data in to the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			pTxBuffer += 2;
			(uint16_t*) pTxBuffer++;
		} else {
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		// wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			// 16 but DFF
			// load the data from DR to Rxbuffer
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;
		} else {
			// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/* IRQ Configuration and ISR handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}
void SPI_IRQHandling(SPI_Handle_t *pHandle) {

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

