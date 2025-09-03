/*
 * stm32f411xx.h
 *
 *  Created on: Aug 11, 2025
 *      Author: victus
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/* Processor specific details */

// NVIC ISERx registers
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10c)

// NVIC ICERx registers
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18c)

// Priority Register Address
#define NVIC_PR_BASE_ADDR			((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

// Base addresses of Flash and SRAM Memories
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM						SRAM1_BASEADDR

// AHBx and APBx Bus Peripheral base addresses
#define PERIP_BASE					0x40000000U
#define AHB2PERIPH_BASEADDR			0x50000000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define APB2PERIPH_BASEADDR			0x40010000U
#define APB1PERIPH_BASEADDR			PERIP_BASE

// Base address of peripherals on AHB1 bus
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

// Base address of peripherals on APB1 bus
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)

// Base address of peripherals on APB2 bus
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

/**************************** Peripheral register definition structures *************************************/

typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2;
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t RESERVED5;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
} I2C_RegDef_t;

typedef struct {
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} USART_RegDef_t;

/***************** Peripheral base addresses type casted to GPIO_RegDef ******************/

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1						((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3						((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1						((USART_RegDef_t*) USART1_BASEADDR)
#define USART2						((USART_RegDef_t*) USART2_BASEADDR)
#define USART6						((USART_RegDef_t*) USART6_BASEADDR)

// Clock Enable Macros for GPIOx peripherals
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOEEN))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOHEN))

// Disable peripheral clock
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOAEN))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOBEN))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOCEN))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIODEN))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOEEN))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOHEN))

// Reset GPIOx peripherals
#define GPIOA_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIOAEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIOAEN)); } while(0)
#define GPIOB_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIOBEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIOBEN)); } while(0)
#define GPIOC_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIOCEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIOCEN)); } while(0)
#define GPIOD_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIODEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIODEN)); } while(0)
#define GPIOE_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIOEEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIOEEN)); } while(0)
#define GPIOH_REG_RESET()			do { (RCC->AHB1RSTR |= (1 << RCC_AHB1ENR_GPIOHEN)); (RCC->AHB1RSTR &= ~(1 << RCC_AHB1ENR_GPIOHEN)); } while(0)

// Clock Enable Macros for I2Cx peripherals
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C2EN))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C3EN))

// Clock Disable Macros for I2Cx peripherals
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C2EN))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C3EN))

// Clock Enable Macros for SPIx peripherals
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))

// Disable peripheral clock
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))

// Clock Enable Macros for USARTx peripherals
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))

// Disable peripheral clock
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOH) ? 7 : 0)

// IRQ numbers
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
#define IRQ_NO_SPI4					84
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_USART6				71

// IRQ priority
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15

// Some generic macros
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

/******* Bit position definition macros of SPI_CR1 peripheral ******/
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/******* Bit position definition macros of SPI_CR2 peripheral ******/
#define SPI_CR2_RXMADEN				0
#define SPI_CR2_TXMADEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/******* Bit position definition macros of SPI_SR peripheral ******/
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/****************** Bit position definition of I2C peripheral **********/
/************* Bit position definitions I2C_CR1 ****************/
#define I2C_CR1_PE					0
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_SWRST				15

/************* Bit position definitions I2C_CR2 ****************/
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10

/************* Bit position definitions I2C_OAR1 ****************/
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

/************* Bit position definitions I2C_SR1 ****************/
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_TIMEOUT				14

/************* Bit position definitions I2C_SR2 ****************/
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_DUALF				7

/************* Bit position definitions I2C_CCR ****************/
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NE					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9

/********************* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) ************/
#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define RCC_AHB1ENR_GPIOCEN			2
#define RCC_AHB1ENR_GPIODEN			3
#define RCC_AHB1ENR_GPIOEEN			4
#define RCC_AHB1ENR_GPIOHEN			7

/********************* RCC APB1 peripheral clock enable register (RCC_APB1ENR) ************/
#define RCC_APB1ENR_I2C1EN			21
#define RCC_APB1ENR_I2C2EN			22
#define RCC_APB1ENR_I2C3EN			23

/* RCC function prototypes */
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_rcc_driver.h"

#endif /* INC_STM32F411XX_H_ */
