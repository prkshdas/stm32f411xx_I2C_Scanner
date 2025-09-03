/*
 * stm32f411xx_rcc_driver.c
 *
 *  Created on: Sep 03, 2025
 *      Author: victus
 */

#include "stm32f411xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClock(void) {
	// Simplified PLL clock calculation based on RM0383
	uint32_t pll_input, pll_output;
	uint8_t pllsrc, pllm, plln, pllp;

	// Get PLL source (PLLSRC bit in RCC_PLLCFGR)
	pllsrc = (RCC->PLLCFGR >> 22) & 0x1;
	// Get PLLM, PLLN, PLLP values
	pllm = RCC->PLLCFGR & 0x3F; // Bits 0:5
	plln = (RCC->PLLCFGR >> 6) & 0x1FF; // Bits 6:14
	pllp = (((RCC->PLLCFGR >> 16) & 0x3) + 1) * 2; // Bits 16:17, PLLP = 2 * (PLLP value + 1)

	// Determine PLL input clock
	if (pllsrc == 0) {
		pll_input = 16000000; // HSI
	} else {
		pll_input = 8000000; // HSE
	}

	// PLL output = (PLL input / PLLM) * PLLN / PLLP
	if (pllm == 0) pllm = 1; // Avoid division by zero
	pll_output = (pll_input / pllm) * plln / pllp;

	return pll_output;
}

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	// Get clock source
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0) {
		SystemClk = 16000000; // HSI
	} else if (clksrc == 1) {
		SystemClk = 8000000; // HSE
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHB prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp - 8];
	}

	// APB1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0) {
		SystemClk = 16000000; // HSI
	} else if (clksrc == 1) {
		SystemClk = 8000000; // HSE
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHB prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp - 8];
	}

	// APB2 prescaler
	temp = ((RCC->CFGR >> 13) & 0x7);

	if (temp < 4) {
		apb2p = 1;
	} else {
		apb2p = APB1_PreScaler[temp - 4];
	}

	pclk2 = (SystemClk / ahbp) / apb2p;

	return pclk2;
}
