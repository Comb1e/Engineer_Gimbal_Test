//
// Created by Henson on 2024-01-14.
//

#include "task_rc.h"

#if RC_ENABLE
extern rc_device_t g_rc;
extern uint8_t dr16_buff[DR16_BUFF_LEN];

uint32_t rc_error_cnt = 0;
uint32_t rc_lost_cnt = 0;
//uint32_t rx_cnt = 0;

void rcReceiveTask(void *argument){
    static osStatus_t stat;
    rc_device_init(&g_rc,&RC_UART,dr16_update_callback);
    HAL_UART_RegisterCallback(g_rc._huart, HAL_UART_ERROR_CB_ID, dr16_unlink_restart);
    osSemaphoreAcquire(dr16UpdateBinarySemHandle, 0);

    for (;;) {
        dr16_start_receive_dma(&g_rc);
        stat = osSemaphoreAcquire(dr16UpdateBinarySemHandle, 15);
        if (stat == osOK) {
            if (rc_update(&g_rc)) {
                set_rc_connect(&g_rc);
            }
        }
        else {
            set_rc_lost(&g_rc);
            rc_lost_cnt++;
        }
    }
}

//相当于HAL_UARTEx_RxEventCallback
void dr16_update_callback(UART_HandleTypeDef *huart, uint16_t Size) {
    osSemaphoreRelease(dr16UpdateBinarySemHandle);
}


void dr16_unlink_restart(UART_HandleTypeDef *huart) {
    //HAL_DMA_Init(huart->hdmarx);
    rc_error_cnt++;
    //set_rc_connect();
}
#endif