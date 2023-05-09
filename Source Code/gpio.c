// ----- gpio.c ----- //
#include "main.h"
#include "gpio.h"

// Function to initialize the red on-board LED connected to PB14
void GPIOInit(void) {
	// Enable GPIO Clock(s)
	RCC->AHB2ENR |= (GPIOA_CLK | GPIOB_CLK);

	// SetGP GPIO Pins Mode to Output Mode (2'b01)
	GPIO_DEF_A->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIO_DEF_A->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);
	GPIO_DEF_B->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE14);
	GPIO_DEF_B->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE14_0);


	// Set GPIO Pins Output Type to Push-Pull (1'b0)
	GPIO_DEF_A->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
	GPIO_DEF_B->OTYPER &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT14);

	// Set GPIO Pins Output Speed to Low (1'b0)
	GPIO_DEF_A->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
	GPIO_DEF_B->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED14);

	// Set GPIO Pins Port to No Pull-Up or Pull-Down (1'b0);
	GPIO_DEF_A->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIO_DEF_B->PUPDR &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD14);
}

void PushButtonInit(bool IRQ_Enable){
	RCC->AHB2ENR |= (GPIOB_CLK | GPIOC_CLK);

	// Set GPIO Pin PC13 (User Button) Mode to Input Mode (2'b00)
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	// Set Blue LED PB7 to Output
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0);

	// Set GPIO Pin PC13 Port to Pull-Down (2'b10);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD13_1);
	// Set Blue LED PB7 to No Pull-Up/Pull-Down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);

	// Enable EXTI interrupt on GPIO PC13 (User Button)
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13_PC);
	SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PC);

	// Setup Interrupt Mask & Rising Trigger for Push Button
	EXTI->IMR1 &=  ~(EXTI_IMR1_IM13);
	EXTI->IMR1 |=  (EXTI_IMR1_IM13);
	EXTI->RTSR1 &= ~(EXTI_RTSR1_RT13);
	EXTI->RTSR1 |= (EXTI_RTSR1_RT13);

	// Turn on Blue LED
	GPIOB->ODR |= GPIO_ODR_OD7;
	// Clear any EXTI interrupts on Pin 13
	EXTI->PR1 |= (EXTI_PR1_PIF13);

	if (IRQ_Enable){
		//Setup Interrupts for EXTI
		NVIC->ISER[1] = (1 << (EXTI15_10_IRQn & 0x1F));
		__enable_irq();
	}
}
