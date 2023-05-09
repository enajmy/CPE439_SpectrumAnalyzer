void USART_init(void){
		//initialize GPIO pins PA2/PA3
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		//alternate function mode
		GPIOA->MODER &= ~(GPIO_MODER_MODE2 |
						GPIO_MODER_MODE3);
		GPIOA->MODER |= ((2<<GPIO_MODER_MODE2_Pos) |
						(2<<GPIO_MODER_MODE3_Pos));
		//alternate function select
		GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_3 
						| GPIO_AFRL_AFSEL3_3);
		GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) 
						| (7 << GPIO_AFRL_AFSEL3_Pos);
		//slow
		GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2 |
						GPIO_OSPEEDR_OSPEED3);
		//no pull up pull down
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);

		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
		//set baud rate to 115.2 kbps @ 80MHz
		USART2->BRR = 80000000/115200;

		USART2->CR1 |= (USART_CR1_TE
						| USART_CR1_RE);
		USART2->CR1 |= USART_CR1_UE;// ue bit in cr1
}

void USART_write(uint8_t data){
	while(!(USART2->ISR & USART_ISR_TXE));
	USART2->TDR = data;
}

uint8_t USART_read(void){
	while((USART2->ISR & USART_CR1_RXNEIE) == 0);
	uint8_t input = USART2->RDR;
	return input;
}
