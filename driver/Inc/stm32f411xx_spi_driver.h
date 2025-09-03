/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 19, 2025
 *      Author: victus
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/* Configuration structure for SPIx peripheral	*/

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/* Handle structure for SPIx peripheral */

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

//@SPI_DeviceMode

#define SPI_DEVICE_MODE_MASTER 				1
#define SPI_DEVICE_MODE_SLAVE 				0

//@SPI_BusConfig

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

//@SPI_SclkSpeed

#define SPI_SCLK_SPEED_VIV2					0
#define SPI_SCLK_SPEED_VIV4					1
#define SPI_SCLK_SPEED_VIV8					2
#define SPI_SCLK_SPEED_VIV16				3
#define SPI_SCLK_SPEED_VIV32				4
#define SPI_SCLK_SPEED_VIV64				5
#define SPI_SCLK_SPEED_VIV128				6
#define SPI_SCLK_SPEED_VIV256				7

//@SPI_DFF

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

//@SPI_CPOL

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

//@SPI_CPHA

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

//@SPI_SSM

#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

// SPI related status flag definition

#define SPI_TXE_FLAG 						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG 						(1 << SPI_SR_BSY)

/* Peripheral Clock setup*/
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Init and De-init */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data send and Receive */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration and ISR handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**** Other peripheral control apis****/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
