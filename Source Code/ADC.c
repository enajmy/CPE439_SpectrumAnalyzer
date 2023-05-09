/* ---- ADC.c ---- */
#include "main.h"
#include "ADC.h"
#include "delay.h"

// ADC Initialization Function for DMA use
void ADC1Init(bool IRQ_Enable){

	// Turn on ADC Clocks using HCLK (AHB)
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

	// Power up the ADC and voltage regulator
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |= (ADC_CR_ADVREGEN);

	// Wait 20us for the voltage regulator to startup
	delay_us(20);

	// Configure single ended mode for channel 5
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

	// Calibrate ADC - ensure ADEN is 0 and single ended mode
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	ADC1->CR |= ADC_CR_ADCAL;

	// Wait for ADCAL to be 0
	while (ADC1->CR & ADC_CR_ADCAL);

	// Enable ADC
	// Clear ADRDY bit by writing 1
	ADC1->ISR |= (ADC_ISR_ADRDY);
	ADC1->CR |= ADC_CR_ADEN;

	// Wait for ADRDY to be 1
	while(!(ADC1->ISR & ADC_ISR_ADRDY));

	// Clear the ADRDY bit by writing 1
	ADC1->ISR |= ADC_ISR_ADRDY;

	// Set conversion resolution to 12 bits (2'b00)
	ADC1->CFGR &= ~(ADC_CFGR_RES);

	// Set NOT continuous conversion mode
	ADC1->CFGR &= ~ADC_CFGR_CONT;

	// Clear EOC flag?
	ADC1->ISR |= (ADC_ISR_EOC);

	// Right align data
	ADC1->CFGR &= ~(ADC_CFGR_ALIGN);

	// Enable DMA
	ADC1->CFGR |= (ADC_CFGR_DMAEN);
	ADC1->CFGR |= (ADC_CFGR_DMACFG);

	//ADC1->CFGR |= (ADC_CFGR_OVRMOD);

	// Set ADC to convert on timers
	ADC1->CFGR &= ~(ADC_CFGR_EXTEN | ADC_CFGR_EXTSEL);
	ADC1->CFGR |= (ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_0);

	// Configure channel 5 for sampling time (SMP) with 2.5 clocks
	ADC1->SMPR1 = (0b000 << ADC_SMPR1_SMP5_Pos);
	// # determines sampling rate in register reference page
	// SMP5 is GPIO A0 which corresponds with ADCINN5

	// Configure SQR for regular sequence
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	/* ---- GPIO Configure ---- */
	// Configure GPIO for PA0 pin for analog mode
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER |= (GPIO_MODER_MODE0);

	// Low Speed
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);

	/* ---- ADC Interrupt Configure ---- */
	if (IRQ_Enable){
		// Enable and Clear EOC flag
		ADC1->IER |= (ADC_IER_EOCIE);
		ADC1->ISR |= (ADC_ISR_EOC);

		NVIC->ISER[0] |= (1 << (ADC1_2_IRQn & 0x1F));
		__enable_irq();
	}
}
