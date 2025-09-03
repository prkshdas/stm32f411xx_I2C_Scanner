#include "stm32f411xx.h"
#include <string.h>
#include <stdio.h>

/* Handles */
I2C_Handle_t I2C1Handle;
USART_Handle_t USART2Handle;

/* ------------ UART Helper ------------ */
void USART2_SendString(char *str) {
    USART_SendData(&USART2Handle, (uint8_t*) str, strlen(str));
}

/* ------------ I2C Init ------------ */
void I2C1_Init(void) {
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM; // 100kHz
    I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;        // dummy own addr
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

    I2C_Init(&I2C1Handle);
}

/* ------------ UART Init ------------ */
void USART2_Init(void) {
    USART2Handle.pUSARTx = USART2;
    USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&USART2Handle);
}

/* ------------ GPIO Init ------------ */
void I2C1_GPIOInits(void) {
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PiNAltFncMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
    GPIO_Init(&I2CPins);

    // SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7;
    GPIO_Init(&I2CPins);
}

void USART2_GPIOInits(void) {
    GPIO_Handle_t USARTPins;

    USARTPins.pGPIOx = GPIOA;
    USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USARTPins.GPIO_PinConfig.GPIO_PiNAltFncMode = 7;
    USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // TX (PA2)
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN2;
    GPIO_Init(&USARTPins);

    // RX (PA3)
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN3;
    GPIO_Init(&USARTPins);
}

/* ------------ I2C Device Check ------------ */
uint8_t I2C_IsDeviceReady(uint8_t addr) {
    uint32_t timeout = 50000;
    uint8_t slaveAddr = addr << 1; // 7-bit shifted

    // Clear AF flag
    I2C1->SR1 &= ~(1 << 10);

    // Generate START
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & (1 << 0)) && --timeout); // wait for SB
    if (!timeout) return 0;

    // Send address
    I2C1->DR = slaveAddr;
    timeout = 50000;

    // Wait for ADDR (ack) or AF (nack)
    while (!(I2C1->SR1 & ((1 << 1) | (1 << 10))) && --timeout);
    if (!timeout) return 0;

    if (I2C1->SR1 & (1 << 1)) {
        // Address matched
        (void)I2C1->SR2; // clear ADDR
        I2C1->CR1 |= (1 << 9); // STOP
        return 1;
    }

    // NACK
    I2C1->SR1 &= ~(1 << 10); // clear AF
    I2C1->CR1 |= (1 << 9);   // STOP
    return 0;
}

/* ------------ Main ------------ */
int main(void) {
    char msg[64];

    I2C1_GPIOInits();
    I2C1_Init();
    USART2_GPIOInits();
    USART2_Init();

    I2C_PeripheralControl(I2C1, ENABLE);
    USART_PeripheralControl(USART2, ENABLE);

    USART2_SendString("Starting I2C Scan...\r\n");

    while (1) {
        uint8_t found = 0;

        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            if (I2C_IsDeviceReady(addr)) {
                sprintf(msg, "I2C device address : 0x%02X\r\n", addr);
                USART2_SendString(msg);
                found = 1;
            }
        }

        if (!found) {
            USART2_SendString("No Device Found\r\n");
        }

        // small delay between scans
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}
