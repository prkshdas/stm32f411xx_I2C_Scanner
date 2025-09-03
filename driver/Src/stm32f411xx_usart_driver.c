/*
 * stm32f411xx_usart_driver.c
 *
 *  Created on: Aug 31, 2025
 *      Author: victus
 */

#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_rcc_driver.h"
#include <stddef.h>

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Configures the baud rate for the USART peripheral
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral
 * @param[in]         - BaudRate: Desired baud rate
 *
 * @return            - None
 *
 * @Note              - Supports oversampling by 8 or 16
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	// Variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	// Get the value of APB bus clock into the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		// USART1 and USART6 are on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		// OVER8 = 1, over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		// Over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate the Mantissa part
	M_part = usartdiv / 100;

	// Place the Mantissa part in appropriate bit position
	tempreg |= M_part << 4;

	// Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	// Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		// OVER8 = 1, over sampling by 8
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	} else {
		// Over sampling by 16
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	// Place the fractional part in appropriate bit position
	tempreg |= F_part;

	// Copy the value of tempreg into BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes the USART peripheral
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 *
 * @return            - None
 *
 * @Note              - Configures mode, word length, parity, stop bits, and hardware flow control
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	// Temporary variable
	uint32_t tempreg = 0;

	// Enable the clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		// Enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		// Enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		// Enable both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	// Configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// Configuration of parity control bit fields
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
	} else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
		// Enable ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	// Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	// Configure CR2
	tempreg = 0;

	// Configure the number of stop bits
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	// Configure CR3
	tempreg = 0;

	// Configuration of USART hardware flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		// Enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		// Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		// Enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	// Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - Resets the USART peripheral
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 *
 * @return            - None
 *
 * @Note              - Resets the peripheral via RCC
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle) {
	if (pUSARTHandle->pUSARTx == USART1) {
		do { (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); } while (0);
	} else if (pUSARTHandle->pUSARTx == USART2) {
		do { (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); } while (0);
	} else if (pUSARTHandle->pUSARTx == USART6) {
		do { (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); } while (0);
	}
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables the USART peripheral
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral
 * @param[in]         - Cmd: ENABLE or DISABLE
 *
 * @return            - None
 *
 * @Note              - Controls the UE bit in CR1
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd) {
	if (Cmd == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Enables or disables the clock for the USART peripheral
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral
 * @param[in]         - EnOrDi: ENABLE or DISABLE
 *
 * @return            - None
 *
 * @Note              - Controls the clock via RCC
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Checks the status of a flag in the SR register
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral
 * @param[in]         - StatusFlagName: Flag to check
 *
 * @return            - SET or RESET
 *
 * @Note              - None
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName) {
	if (pUSARTx->SR & StatusFlagName) {
		return SET;
	}
	return RESET;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Sends data in blocking mode
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - pTxBuffer: Pointer to transmit buffer
 * @param[in]         - Len: Length of data to send
 *
 * @return            - None
 *
 * @Note              - Supports 8-bit and 9-bit data with/without parity
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint16_t *pdata;

	// Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++) {
		// Wait until TXE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			// If 9BIT, load the DR with 2 bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// Check for USART_ParityControl
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity, 9 bits of user data will be sent
				pTxBuffer += 2;
			} else {
				// Parity bit is used, 8 bits of user data will be sent
				pTxBuffer++;
			}
		} else {
			// 8-bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	// Wait until TC flag is set in the SR
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receives data in blocking mode
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - pRxBuffer: Pointer to receive buffer
 * @param[in]         - Len: Length of data to receive
 *
 * @return            - None
 *
 * @Note              - Supports 8-bit and 9-bit data with/without parity
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	// Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++) {
		// Wait until RXNE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check the USART_WordLength to decide whether we are going to receive 9-bit or 8-bit data
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			// 9-bit data in a frame
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity, all 9 bits are user data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer += 2;
			} else {
				// Parity is used, 8 bits of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		} else {
			// 8-bit data in a frame
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity, all 8 bits are user data
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			} else {
				// Parity is used, 7 bits of user data and 1 bit is parity
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Initiates interrupt-based data transmission
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - pTxBuffer: Pointer to transmit buffer
 * @param[in]         - Len: Length of data to send
 *
 * @return            - Current Tx state
 *
 * @Note              - Enables TXE and TC interrupts
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Initiates interrupt-based data reception
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - pRxBuffer: Pointer to receive buffer
 * @param[in]         - Len: Length of data to receive
 *
 * @return            - Current Rx state
 *
 * @Note              - Enables RXNE interrupt
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		// Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears a specified flag in the SR register
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral
 * @param[in]         - StatusFlagName: Flag to clear
 *
 * @return            - None
 *
 * @Note              - Applicable to CTS, LBD, TC flags
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName) {
	pUSARTx->SR &= ~(StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Configures NVIC interrupt for USART
 *
 * @param[in]         - IRQNumber: Interrupt number
 * @param[in]         - EnorDi: ENABLE or DISABLE
 *
 * @return            - None
 *
 * @Note              - Configures ISER/ICER registers
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Configures NVIC priority for USART interrupt
 *
 * @param[in]         - IRQNumber: Interrupt number
 * @param[in]         - IRQPriority: Priority level
 *
 * @return            - None
 *
 * @Note              - Configures priority in NVIC PR registers
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - Handles USART interrupts
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 *
 * @return            - None
 *
 * @Note              - Handles TC, TXE, RXNE, CTS, IDLE, and error interrupts
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;

	/************************* Check for TC flag ********************************************/
	// Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	// Check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (temp1 && temp2) {
		// Interrupt is because of TC
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			// Check the TxLen. If it is zero, close the data transmission
			if (!pUSARTHandle->TxLen) {
				// Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
				// Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				// Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				// Reset the length to 0
				pUSARTHandle->TxLen = 0;
				// Disable the TCIE
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
				// Call the application callback
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/************************* Check for TXE flag ********************************************/
	// Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	// Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2) {
		// Interrupt is because of TXE
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			if (pUSARTHandle->TxLen > 0) {
				// Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					// If 9BIT, load the DR with 2 bytes masking the bits other than first 9 bits
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// Check for USART_ParityControl
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						// No parity, 9 bits of user data will be sent
						pUSARTHandle->pTxBuffer += 2;
						pUSARTHandle->TxLen -= 2;
					} else {
						// Parity bit is used, 8 bits of user data will be sent
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				} else {
					// 8-bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}

				// If TxLen is zero, disable TXEIE to stop further TXE interrupts
				if (pUSARTHandle->TxLen == 0) {
					pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
				}
			}
		}
	}

	/************************* Check for RXNE flag ********************************************/
	// Check the state of RXNE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	// Check the state of RXNEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2) {
		// Interrupt is because of RXNE
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->RxLen > 0) {
				// Check the USART_WordLength to decide whether we are going to receive 9-bit or 8-bit data
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					// 9-bit data in a frame
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						// No parity, all 9 bits are user data
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer += 2;
						pUSARTHandle->RxLen -= 2;
					} else {
						// Parity is used, 8 bits of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen--;
					}
				} else {
					// 8-bit data in a frame
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						// No parity, all 8 bits are user data
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					} else {
						// Parity is used, 7 bits of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}

				// If RxLen is zero, close the reception
				if (pUSARTHandle->RxLen == 0) {
					// Disable the RXNEIE
					pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
					pUSARTHandle->RxBusyState = USART_READY;
					pUSARTHandle->pRxBuffer = NULL;
					// Call application callback
					USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
				}
			}
		}
	}

	/************************* Check for CTS flag ********************************************/
	// Note: CTS feature is implemented only for USART1 and USART2
	if (pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART2) {
		// Check the state of CTS bit in the SR
		temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
		// Check the state of CTSIE bit in CR3
		temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

		if (temp1 && temp3) {
			// Clear the CTS flag
			pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
			// Call application callback
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
		}
	}

	/************************* Check for IDLE flag ********************************************/
	// Check the state of IDLE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	// Check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (temp1 && temp2) {
		// Clear the IDLE flag (read SR followed by DR)
		temp1 = pUSARTHandle->pUSARTx->SR;
		(void)pUSARTHandle->pUSARTx->DR;
		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/************************* Check for Overrun Error ********************************************/
	// Check the state of ORE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	// Check the state of RXNEIE bit in CR1 (since ORE triggers with RXNE)
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2) {
		// Clear ORE by reading SR followed by DR
		temp1 = pUSARTHandle->pUSARTx->SR;
		(void)pUSARTHandle->pUSARTx->DR;
		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/************************* Check for Error Flag ********************************************/
	// Noise Flag, Overrun error, and Framing Error in multi-buffer communication
	// Check the state of PE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	// Check the state of PEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE);

	if (temp1 && temp2) {
		// Clear the PE flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_PE);
		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_PE);
	}

	// Check for Framing error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE);
	if (temp1 && temp2) {
		// Clear the FE flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_FE);
		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
	}

	// Check for Noise error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NE);
	if (temp1 && temp2) {
		// Clear the NE flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_NE);
		// Call application callback
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
	}
}

/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - Application callback function (weak implementation)
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - AppEv: Event identifier
 *
 * @return            - None
 *
 * @Note              - Weak implementation, to be overridden by application
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv) {
	// This is a weak implementation. The application may override this function.
}
