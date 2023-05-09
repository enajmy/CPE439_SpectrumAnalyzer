/* ---- gpio.h ---- */
#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include <stdbool.h>

/* -- Defines -- */
#define GPIOA_CLK RCC_AHB2ENR_GPIOAEN
#define GPIOB_CLK RCC_AHB2ENR_GPIOBEN
#define GPIOC_CLK RCC_AHB2ENR_GPIOCEN
#define GPIO_DEF_A GPIOA
#define GPIO_DEF_B GPIOB

// Include function declarations / prototypes
void GPIOInit(void);
void PushButtonInit(bool IRQ_Enable);

#endif /* SRC_GPIO_H_ */
