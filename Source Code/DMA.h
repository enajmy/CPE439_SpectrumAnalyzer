/* ---- DMA.h ---- */
#ifndef SRC_DMA_H_
#define SRC_DMA_H_

/* -- Includes -- */
// Used to register float32_t type
#include "arm_math.h"
#include <stdbool.h>

#define SAMPLING_RATE 8192

/* Include function declarations / prototypes */
// Function to initialize DMA for MEM-MEM transfer
void DMAInit(volatile uint32_t *peripherialAddress, float32_t (*memoryAddress)[SAMPLING_RATE], bool IRQ_Enable);

#endif /* SRC_DMA_H_ */
