/* ---- LPUART.h ---- */
#ifndef SRC_LPUART_H_
#define SRC_LPUART_H_

// Include #defines
#define LPUART GPIOG
#define BAUD_RATE_HEX 0x2B66B
#define ESC 0x1B


// Include function definitions / prototypes
void LPUARTInit(void);
void LPUART_write_ESC(void);
void LPUART_print(const char *str);
void LPUART_ESC_Code(const char *str);
void LPUART_Setup_SpectrumAnalyzer(void);

#endif /* SRC_LPUART_H_ */
