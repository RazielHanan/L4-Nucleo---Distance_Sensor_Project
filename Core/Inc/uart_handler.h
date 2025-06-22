#ifndef __UART_HANDLER_H
#define __UART_HANDLER_H

typedef struct {
	char rx_temp;           // single received byte
	volatile uint8_t idx;
	char* rx_buffer;
	volatile uint8_t message_ready;
	volatile uint8_t message_just_completed;
} Uart_handler_args;

extern Uart_handler_args* uart;
void init_uart_args(Uart_handler_args* uart);
void send_uart_message(UART_HandleTypeDef huart, char* msg_to_send); // tx uart message
uint8_t send_uart_with_counter(UART_HandleTypeDef huart,char* msg_to_send, uint8_t count); //tx uart with "FROM STM32" ending with msg counting.
void get_rx_uart(UART_HandleTypeDef huart,char* msg); // rx uart msg, blocking!
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); // rx uart msg, non-blocking - using interrupt!

#endif
