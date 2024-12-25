//
// Created by Henson on 2023-10-22.
//

#ifndef __INTERFACE_CAN_H
#define __INTERFACE_CAN_H

#include "can.h"
#include "freertos_inc.h"
#include "user_cpp_lib.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

void can1SendTask(void *argument);
void can2SendTask(void *argument);

#ifdef __cplusplus
}
#endif

#define MAX_CAN_DEV_NUM (14 * 4)//每个can有14个筛选器组,每个筛选器可以用4个ID来接收

class CAN{
public:
    using can_rx_callback = std::function<void(CAN *can_dev,uint8_t *rx_data)>;
    typedef struct {
        bool is_enable;
        uint8_t len;
        uint8_t *buff;
        uint32_t stdid;
        uint32_t cnt;
    } can_tx_t;

    typedef struct {
        can_rx_callback m_callback;
        osSemaphoreId_t semaphore;
        uint32_t stdid;
        uint32_t cnt;
    } can_rx_t;

    can_tx_t tx={.is_enable = true};
    can_rx_t rx={};
    CAN_HandleTypeDef *hcan=nullptr;
    uint32_t index = 0;//0 ~ MAX_CAN_DEV_NUM
public:
    friend void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
    CAN() = default;
    void can_init(CAN_HandleTypeDef *hcan);
    void can_rx_add(uint32_t can_rx_stdid, osSemaphoreId_t rxSem,can_rx_callback rx_callback);
    void can_tx_add(uint32_t can_tx_stdid, uint8_t *tx_buff, uint8_t tx_len);
    void can_rx_tx_add(uint32_t rx_stdid, osSemaphoreId_t rxSem, can_rx_callback rx_callback, uint32_t tx_stdid, uint8_t *tx_buff, uint8_t tx_len);
    void set_can_tx_disable();
    void set_can_tx_enable();
    osStatus_t send_can_msg();

protected :
    static CAN *can_device_list[2][MAX_CAN_DEV_NUM];
    static uint32_t can1_filter[4];
    static uint32_t can1_device_num;
    static uint32_t can2_filter[4];
    static uint32_t can2_device_num;
    static CAN_FilterTypeDef filter_cfg;

};

#endif //__INTERFACE_CAN_H
