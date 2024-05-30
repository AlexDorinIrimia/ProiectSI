#ifndef DRIVERS_H
#define DRIVERS_H

#include "Types.h"
#include "stm32f4xx.h"
#define PERIODIC_TIMER_VALUE_500US_U16 500

void InitPeriodicTimer(U16 timeInMs);
void StopPeriodicTimer(void);
void SPI1_init(void);
void SPI1_write(U8 data);

void USART2_init(void);
int USART2_write(int c);
int USART2_read(void);

#endif /* DRIVERS_H */