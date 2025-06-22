#include "main.h"
#include "uart_handler.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;



void init_uart_args(Uart_handler_args* uart){
	uart->idx = 0;
	uart->message_ready = 0;
	uart->message_just_completed = 0;
}

void get_rx_uart(UART_HandleTypeDef huart,char* msg){
	char error_msg[] = "UART RECEIVE TIMEOUT\r\n";
	char rx_buffer[64];
	uint8_t rx_byte = 0;
	size_t idx = 0;
	while(rx_byte!='\r' && rx_byte!='\n'){
		if(HAL_UART_Receive(&huart, &rx_byte, 1, 500) != HAL_OK){
		    // Timeout or error occurred, handle or break
			HAL_UART_Transmit(&huart, (uint8_t*)error_msg, strlen(error_msg), 200);
		    break; // or break;
		}
		rx_buffer[idx] = rx_byte;
		idx++;
	}

	if (rx_byte=='\r' || rx_byte=='\n'){
	    // Flush until line ends (\n), to clean up
	    while (HAL_UART_Receive(&huart, &rx_byte, 1, 200) == HAL_OK){};
		idx--;
		rx_buffer[idx++]='\r';
		rx_buffer[idx++]='\n';
		rx_buffer[idx]='\0';
		strcpy(msg,rx_buffer);
		HAL_UART_Transmit(&huart, (uint8_t*)msg, strlen(msg), 200);
	}
}
void send_uart_message(UART_HandleTypeDef huart, char* msg_to_send){
	HAL_UART_Transmit(&huart, (uint8_t*)msg_to_send, strlen(msg_to_send), 200);
}

uint8_t send_uart_with_counter(UART_HandleTypeDef huart,char* msg_to_send, uint8_t count){
	char msg[18];
	count++;
	if (count==256){
		count = 0;
	}
//	uint8_t char_count = '0' + count;
//	char char_count_str[2];
//	char_count_str[0] = char_count;
//	char_count_str[1] = '\0';
	send_uart_message(huart,msg_to_send);
	sprintf(msg,"msg_count: %u\r\n",count);
//	strcpy(msg,"msg_count: ");
//	strcat(msg,char_count_str);
//	strcat(msg,"\r\n\0");
	if (HAL_UART_Transmit(&huart, (uint8_t*)msg, strlen(msg), 100) != HAL_OK){
		return 0;
	}

	return count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2)  // adjust if you're using a different UART
    {
        if (uart->message_just_completed && (uart->rx_temp == '\r' || uart->rx_temp == '\n')) {
        	uart->message_just_completed = 0;  // skip repeated newline
        }
        else if (uart->rx_temp == '\r' || uart->rx_temp == '\n') {
            if (uart->rx_buffer != NULL) {
            	char* temp = realloc(uart->rx_buffer, uart->idx + 3);
            	uart->rx_buffer = temp;
            	uart->rx_buffer[uart->idx] = '\r';
            	uart->rx_buffer[uart->idx+1] = '\n';
            	uart->rx_buffer[uart->idx+2] = '\0';  // null-terminate
            	uart->message_ready = 1;
            	uart->idx = 0;
            	uart->message_just_completed = 1;
            }
        }
        else {
            // Resize buffer to hold new char + null terminator
            char* temp = realloc(uart->rx_buffer, uart->idx + 2);
            if (temp != NULL) {
            	uart->rx_buffer = temp;
            	uart->rx_buffer[uart->idx++] = uart->rx_temp;
            	uart->rx_buffer[uart->idx] = '\0';  // keep null-terminated
            } else {
                // allocation failed, reset state
                free(uart->rx_buffer);
                uart->rx_buffer = NULL;
                uart->idx = 0;
            }
        }

        // Schedule next byte reception
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart->rx_temp, 1);
    }
}

