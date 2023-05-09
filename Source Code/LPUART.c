/* ----------- LPUART.c ------------ */
#include "main.h"
#include "LPUART.h"
#include "delay.h"

// Function to initialize the LPUART
void LPUARTInit(void){

	/* -- GPIOG Enable -- */
	// Turn on GPIOG Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

	// Power on GPIOG/LPUART
	PWR->CR2 |= PWR_CR2_IOSV;

	// Set AF Mode Registers to AF8
	LPUART->AFR[0] &= ~(GPIO_AFRL_AFSEL7);
	LPUART->AFR[0] |= (GPIO_AFRL_AFSEL7_3);

	LPUART->AFR[1] &= ~(GPIO_AFRH_AFSEL8);
	LPUART->AFR[1] |= (GPIO_AFRH_AFSEL8_3);

	// Set PG7 and PG8 to Push Pull
	LPUART->OTYPER &= ~(GPIO_OTYPER_OT7
			| GPIO_OTYPER_OT8);

	// Set PG7 and PG8 to Low Speed
	LPUART->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7
			| GPIO_OSPEEDR_OSPEED8);

	// Set PG7 and PG8 to No PUPD
	LPUART->PUPDR &= ~(GPIO_PUPDR_PUPD7
			| GPIO_PUPDR_PUPD8);

	// Set PG7 (TX) and PG8 (RX) to AF Mode
	LPUART->MODER &= ~(GPIO_MODER_MODE7
			| GPIO_MODER_MODE8);
	LPUART->MODER |= ((2 << GPIO_MODER_MODE7_Pos)
			| (2 << GPIO_MODER_MODE8_Pos));

	/* -- LPUART Enable -- */
	// 8 bit mode, No Parity, 1 Stop Bit
	// All set by default, no configuration needed

	// Baud Rate = 115200, write 177778 (0x2B672) to BRR for 80MHz clock
	LPUART1->BRR = BAUD_RATE_HEX;

	// Interrupts Enable
	LPUART1->CR1 |= USART_CR1_RXNEIE;

	NVIC->ISER[2] = (1<<(LPUART1_IRQn & 0x1F));
	__enable_irq();


	// Enable LPUART
	LPUART1->CR1 |= (USART_CR1_UE);

	/* --- Begin Data Transmission --- */
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
}

void LPUART_write_ESC(void){
	// If the TXE flag is high, meaning transmission over,
	// write to the transmit data register
	while (!(LPUART1->ISR & USART_ISR_TXE)){
	}
	LPUART1->TDR = ESC;
}

void LPUART_print(const char *str){
	// Initialize counting variable
	uint8_t i = 0;

	// While the input str is not terminated, print.
	while (str[i] != '\0'){
		if (LPUART1->ISR & USART_ISR_TXE)
			LPUART1->TDR = (str[i++]);
	}
}

void LPUART_ESC_Code(const char *str){
	LPUART_write_ESC();
	LPUART_print(str);
}

void LPUART_Setup_SpectrumAnalyzer(void){
	LPUART_ESC_Code("[2J");
	LPUART_ESC_Code("[H");
	LPUART_print("Performing FFT... ");
	LPUART_ESC_Code("\n\r ");
	LPUART_ESC_Code("\n\r ");
	LPUART_print("^");
	LPUART_ESC_Code("\n\r ");
	for (uint8_t i = 0; i < 20; i++){
		LPUART_print("|");
		LPUART_ESC_Code("\n\r ");
	}
	LPUART_print("+------+---------+---------+---------+---------+---------+---------+---------+->");
	LPUART_ESC_Code("\n\r ");
	const char *xAxisLabelsPos[8] = {"[5C", "[5C", "[4C", "[4C", "[4C", "[4C", "[4C", "[3C"};
	const char *xAxisLabels[8] = {"500Hz", "1000Hz", "1500Hz", "2000Hz", "2500Hz", "3000Hz", "3500Hz", "4000Hz"};
	for (uint8_t i = 0; i < 8; i++){
		LPUART_ESC_Code(xAxisLabelsPos[i]);
		LPUART_print(xAxisLabels[i]);
	}

	/* Output to LPUART */
	LPUART_ESC_Code("[27;1H");
	LPUART_print("Max Frequency: ");
	LPUART_ESC_Code("\n\r ");
}
