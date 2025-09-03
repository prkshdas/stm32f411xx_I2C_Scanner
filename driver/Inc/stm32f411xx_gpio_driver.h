/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Aug 12, 2025
 *      Author: victus
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PiNAltFncMode;

} GPIO_PinConfig_t;

//Handle structure for a GPIO pin

typedef struct {

	GPIO_RegDef_t *pGPIOx; /* This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;

/* GPIO PIN NUMBERS */

#define GPIO_PIN0					0
#define GPIO_PIN1					1
#define GPIO_PIN2					2
#define GPIO_PIN3					3
#define GPIO_PIN4					4
#define GPIO_PIN5					5
#define GPIO_PIN6					6
#define GPIO_PIN7					7
#define GPIO_PIN8					8
#define GPIO_PIN9					9
#define GPIO_PIN10					10
#define GPIO_PIN11					11
#define GPIO_PIN12					12
#define GPIO_PIN13					13
#define GPIO_PIN14					14
#define GPIO_PIN15					15

/* GPIO Pins Modes */

#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFN 			2
#define GPIO_MODE_ANALOG 			3
#define GPIO_MODE_IT_FT 			4
#define GPIO_MODE_IT_RT 			5
#define GPIO_MODE_IT_RFT 			6

/*  GPIO Pin possible output types */

#define GPIO_OP_TYPE_PP				0   /* push pull */
#define GPIO_OP_TYPE_OD				1	/* open drain */

/* GPIO pin possible output speed */

#define GPIO_SPEED_LOW 				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST 			2
#define GPIO_SPEED_HIGH				3

/* GPIO Pins pull up and pull down configuration macros */

#define GPIO_NO_PUPD 				0
#define GPIO_PU						1
#define GPIO_PD						2

/* Peripheral Clock setup*/
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init and De-init */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data read and write */

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Configuration and ISR Handling  */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
