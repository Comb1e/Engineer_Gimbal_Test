//
// Created by Henson on 2023-10-22.
//

#include "bsp_usart.h"
#include "string.h"
#include "freertos_inc.h"


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    __NOP();
}

//半双工若连续调用dma发送或者先发送后接收都需要进行延时串口才能正常运行
void usart_send_str(UART_HandleTypeDef *_huart, uint8_t *_str) {
    unsigned int k = 0;
    k = strlen((const char *) _str);
    for (int i = 0; i < 2; i++) {
        HAL_UART_Transmit(_huart, (uint8_t *) _str, k, 10);
    }
    osDelay(1);//可留延时可不留
}

void usart_send_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len) {
    for(int i=0;i<2;i++){
        HAL_UART_Transmit(_huart, _buf, _len, 6);
        osDelay(1);//可留延时可不留
    }
}

void usart_send_buf_dma(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len) {
    HAL_UART_Transmit_DMA(_huart, _buf, _len);
    osDelay(1);
}

void usart_idle_receive_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _size) {
    HAL_UARTEx_ReceiveToIdle_IT(_huart,_buf,_size);//这个地方应该还要关个中断,或者不关也是空的
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);//关闭半满中断
}