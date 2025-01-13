//
// Created by Henson on 2023-02-20.
//

#include "interface_can.h"
#include <utility>

#include "user_lib.h"

CAN* CAN::can_device_list[2][MAX_CAN_DEV_NUM]={};
uint32_t CAN::can1_filter[4]={};
uint32_t CAN::can1_device_num=0;
uint32_t CAN::can2_filter[4]={};
uint32_t CAN::can2_device_num=0;
CAN_FilterTypeDef CAN::filter_cfg={
        .FilterMode = CAN_FILTERMODE_IDLIST,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14  //这里设置从14开始分开can1和can2的过滤器
};

void CAN::can_init(CAN_HandleTypeDef *_hcan){
    this->hcan = _hcan;
    if(hcan==&hcan1){
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_TX_MAILBOX_EMPTY);
    }
    else if(hcan==&hcan2){
        HAL_CAN_Start(&hcan2);
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING|CAN_IT_TX_MAILBOX_EMPTY);
    }
}

//CAN1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    //FilterMatchIndex等价于标准库的FMI,这个返回的不是你是几号筛选器组，而是筛选器编号,是从小到大依次排列的
    uint32_t index = rx_header.FilterMatchIndex;

    CAN *can_dev = CAN::can_device_list[0][index];
    if (can_dev && can_dev->rx.m_callback) {
        can_dev->rx.cnt++;
        can_dev->rx.m_callback(can_dev, rx_data);//更新电机数据
        osSemaphoreRelease(can_dev->rx.semaphore);
    }
}

//CAN2
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    //FilterMatchIndex等价于标准库的FMI,这个返回的不是你是几号筛选器组，而是筛选器编号,是从小到大依次排列的
    uint32_t index = rx_header.FilterMatchIndex;

    CAN *can_dev = CAN::can_device_list[1][index];
    if (can_dev && can_dev->rx.m_callback) {
        can_dev->rx.cnt++;
        can_dev->rx.m_callback(can_dev, rx_data);
        osSemaphoreRelease(can_dev->rx.semaphore);
    }
}

void CAN::can_rx_add(uint32_t can_rx_stdid, osSemaphoreId_t rxSem, can_rx_callback rx_callback) {
    taskENTER_CRITICAL();
    if (rx.m_callback) {
        taskEXIT_CRITICAL();
        return;
    }

    rx.semaphore = rxSem;
    rx.stdid = can_rx_stdid;
    rx.m_callback = std::move(rx_callback);

    if (hcan == &hcan1 && can1_device_num < MAX_CAN_DEV_NUM) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO0;
        index = can1_device_num;//全都用index来操作,与id无关
        can1_device_num++;
        filter_cfg.FilterBank = index / 4; //一个can过滤器有4个stdid可以过滤
        can1_filter[index % 4] =rx.stdid;
        //以下是保留之前的,每次都会有之前的记录
        filter_cfg.FilterIdLow = can1_filter[0] << 5;//IDlist的基本就是把ID左移5位
        filter_cfg.FilterMaskIdLow = can1_filter[1] << 5;
        filter_cfg.FilterIdHigh = can1_filter[2] << 5;
        filter_cfg.FilterMaskIdHigh = can1_filter[3] << 5;
        HAL_CAN_ConfigFilter(&hcan1, &filter_cfg);
        if (3 == index % 4) {
            memset(can1_filter, 0, sizeof(can1_filter));//已经配置好了就清零，首次已经在定义中清零
        }
        can_device_list[0][index] = this;
    }
    else if (hcan == &hcan2 && can2_device_num < MAX_CAN_DEV_NUM) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO1;
        index = can2_device_num;//全都用index来操作,与id无关
        can2_device_num++;
        filter_cfg.FilterBank = index / 4 + 14;
        can2_filter[index % 4] = rx.stdid;
        //以下是保留之前的,每次都会有之前的记录
        filter_cfg.FilterIdLow = can2_filter[0] << 5;//0
        filter_cfg.FilterMaskIdLow = can2_filter[1] << 5;//1
        filter_cfg.FilterIdHigh = can2_filter[2] << 5;//2
        filter_cfg.FilterMaskIdHigh = can2_filter[3] << 5;//3
        HAL_CAN_ConfigFilter(&hcan2, &filter_cfg);
        if (3 == index % 4) {
            memset(can2_filter, 0, sizeof(can2_filter));//已经配置好了就清零，首次已经在定义中清零
        }
        can_device_list[1][index] = this;
    }
    taskEXIT_CRITICAL();
}

void can1SendTask(void *argument){
    CAN::can_tx_t can_tx;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef TxMessage = {.IDE = CAN_ID_STD,.RTR = CAN_RTR_DATA};

    for (;;) {
        osSemaphoreAcquire(can1_tx_cnt_semHandle,osWaitForever);
        osMessageQueueGet(can1SendQueueHandle,&can_tx,nullptr,osWaitForever);
        taskENTER_CRITICAL();
        TxMessage.StdId = can_tx.stdid;
        TxMessage.DLC = can_tx.len;
        HAL_StatusTypeDef hal_stat = HAL_CAN_AddTxMessage(&hcan1,&TxMessage,can_tx.buff,&tx_mailbox);
        if(hal_stat==HAL_OK) {
            __NOP();
        }
        taskEXIT_CRITICAL();
    }
}

uint32_t can2_send_ok = 0;
void can2SendTask(void *argument)
{
    CAN::can_tx_t can_tx;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef TxMessage = {.IDE = CAN_ID_STD,.RTR = CAN_RTR_DATA};

    for (;;)
    {
        osSemaphoreAcquire(can2_tx_cnt_semHandle,osWaitForever);
        osMessageQueueGet(can2SendQueueHandle,&can_tx,nullptr,osWaitForever);
        taskENTER_CRITICAL();
        TxMessage.StdId = can_tx.stdid;
        TxMessage.DLC = can_tx.len;
        HAL_StatusTypeDef hal_stat = HAL_CAN_AddTxMessage(&hcan2,&TxMessage,can_tx.buff,&tx_mailbox);
        if(hal_stat==HAL_OK) {
            can2_send_ok++;
            __NOP();
        }
        taskEXIT_CRITICAL();
    }
}

void CAN::can_tx_add(uint32_t can_tx_stdid,uint8_t *tx_buff,uint8_t tx_len){
    tx.is_enable = true;
    this->tx.stdid = can_tx_stdid;
    tx.buff = tx_buff;
    tx.len = (tx_len>8)? 8:(tx_len<0)? 0:tx_len;
}

void CAN::can_rx_tx_add(uint32_t rx_stdid, osSemaphoreId_t rxSem, can_rx_callback rx_callback,
                        uint32_t tx_stdid, uint8_t *tx_buff, uint8_t tx_len)
{
    can_rx_add(rx_stdid,rxSem,std::move(rx_callback));
    can_tx_add(tx_stdid,tx_buff,tx_len);
}

osStatus_t CAN::send_can_msg()
{
    if(tx.is_enable)
    {
        taskENTER_CRITICAL();
        osStatus_t stat;
        if(hcan==&hcan1)
        {
            stat = osMessageQueuePut(can1SendQueueHandle,&this->tx,0,0);
        }else{
            stat = osMessageQueuePut(can2SendQueueHandle,&this->tx,0,0);
        }
        taskEXIT_CRITICAL();
        if(stat == osOK)
        {
            tx.cnt++;
        }
        return stat;
    }
    return osError;
}

__STATIC_INLINE
void can_tx_complete_callback(CAN_HandleTypeDef *hcan){
    if (hcan == &hcan1) {
        osSemaphoreRelease(can1_tx_cnt_semHandle);//释放信号量
    } else {
        osSemaphoreRelease(can2_tx_cnt_semHandle);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
    can_tx_complete_callback(hcan);
};
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
    can_tx_complete_callback(hcan);
};
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
    can_tx_complete_callback(hcan);
};

void CAN::set_can_tx_disable(){
    tx.is_enable = false;
}

void CAN::set_can_tx_enable(){
    tx.is_enable = true;
}