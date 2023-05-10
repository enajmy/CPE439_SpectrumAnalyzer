#ifndef SRC_USART_H_
#define SRC_USART_H_

void USART_init(void);
void USART_write(uint8_t data);
void USART_string(char *str);
void USART_ESC_code(char *str);
uint8_t USART_read(void);
void USART_print_sample(uint16_t sample);

#endif /* SRC_USART_H_ */
