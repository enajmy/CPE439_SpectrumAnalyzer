/* ---- DMA.c ---- */
#include "main.h"
#include "DMA.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// Function to initialize DMA for MEM-MEM transfer
void DMAInit(volatile uint32_t *peripherialAddress, float32_t (*memoryAddress)[ELEMENT_COUNT], bool IRQ_Enable){

	// Enable DMA clock
	RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);

	// Set the DMA peripheral (source) array address
	DMA1_Channel1->CPAR = (uint32_t)(peripherialAddress);

	// Set the DMA memory (dest.) array address
	DMA1_Channel1->CMAR = (uint32_t)(memoryAddress);

	// Set DMA # of data value
	DMA1_Channel1->CNDTR = ELEMENT_COUNT;

	// Reset DMA1
	DMA1_Channel1->CCR = 0;

	/* Configure DMA1
	 * - Mem2Mem: 						disabled
	 * - Priority: 						very high
	 * - Memory Size: 					32 bits
	 * - Peripheral Size: 				32 bits
	 * - Memory Addr. Increment: 		enabled
	 * - Peripheral Addr. Increment: 	disabled
	 * - Circular Mode: 				DISABLED
	 * - DMA Direction: 				0 (periph = src, mem = dest)
	 * - Error Interrupt: 				enabled
	 * - Half Transfer Interrupt: 		DISABLED
	 * - Full Transfer Interrupt: 		enabled
	 */
	DMA1_Channel1->CCR |= (DMA_CCR_PL | DMA_CCR_MSIZE_1
			| DMA_CCR_PSIZE_1 | DMA_CCR_MINC);

	// Set DMA1 peripheral channel to channel 1 for ADC1 usage
	DMA1_CSELR->CSELR &= ~(DMA_CSELR_C1S);

	/* ---- DMA Interrupts Configure ---- */
	if (IRQ_Enable){
		DMA1_Channel1->CCR |=  (DMA_CCR_TEIE | DMA_CCR_TCIE);

		NVIC->ISER[0] |= (1 << (DMA1_Channel1_IRQn & 0x1F));
		__enable_irq();
	}
}
