// usart.h

#ifndef __STM32F_USART
#define __STM32F_USART

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

void InitUSART2(int speed);
void InitUSART3(void);

void Uart2SendChar(uint8_t c);
void Uart3SendChar(uint8_t c);

uint8_t IsUart2Ready(void);
uint8_t Uart2ReadChar(void);
uint8_t Uart3ReadChar(void);

void Uart2WriteStr(uint8_t* s);
void Uart3WriteStr(uint8_t* s);

#endif


