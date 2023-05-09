/* ---- DMA.h ---- */
#ifndef SRC_DMA_H_
#define SRC_DMA_H_

/* -- Includes -- */
// Used to register float32_t type
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdbool.h>

#define ELEMENT_COUNT 8192

/* Include function declarations / prototypes */
// Function to initialize DMA for MEM-MEM transfer
void DMAInit(volatile uint32_t *peripherialAddress, float32_t (*memoryAddress)[ELEMENT_COUNT], bool IRQ_Enable);

#endif /* SRC_DMA_H_ */
