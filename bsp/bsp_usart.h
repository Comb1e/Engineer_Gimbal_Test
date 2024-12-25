//
// Created by Henson on 2023-10-22.
//

#ifndef __BSP_USART_H
#define __BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

void usart_send_str(UART_HandleTypeDef *_huart, uint8_t *_str);
void usart_send_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len);
void usart_idle_receive_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _size);
void usart_send_buf_dma(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len);

#ifdef __cplusplus
}
#endif


#endif //__BSP_USART_H
