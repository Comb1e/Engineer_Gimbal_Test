//
// Created by Henson on 2024-01-15.
//

#include "drv_DJI_motor.h"

//慎用6020
dji_motor_can_tx_t DJI_Motor::can_tx_list[6] =
                                {{false,&hcan1,8,0x200,0,},//0
                                 {false,&hcan1,8,0x1ff,0,},//1
                                 {false,&hcan1,8,0x2ff,0,},//2

                                 {false,&hcan2,8,0x200,0,},//3
                                 {false,&hcan2,8,0x1ff,0,},//4
                                 {false,&hcan2,8,0x2ff,0,}};//5

DJI_Motor::DJI_Motor() : Motor(), can_dev(){
    process_data_ptr_fun = nullptr;
}

void DJI_Motor::DJI_Motor_init(CAN_HandleTypeDef *hcan,uint32_t motor_id,bool _is_reverse){
    if(motor_id>0 && motor_id<9){
        id = motor_id;
    }else{
        return;
    }
    can_dev.can_init(hcan);
    this->is_reverse = _is_reverse;
}

void DJI_Motor_3508::DJI_Motor_3508_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse,osSemaphoreId_t rxSem){
    DJI_Motor_init(hcan,motorId,isReverse);
    if(hcan==&hcan1) {
        if(id<5){
            can_tx_list[0].is_enable = true;
            can_dev.tx.buff = can_tx_list[0].buff;
            can_dev.tx.stdid = can_tx_list[0].stdid;
        }else{
            can_tx_list[1].is_enable = true;
            can_dev.tx.buff = can_tx_list[1].buff;
            can_dev.tx.stdid = can_tx_list[1].stdid;
        }
    }
    else if(hcan==&hcan2) {
        if(id<5){
            can_tx_list[3].is_enable = true;
            can_dev.tx.buff = can_tx_list[3].buff;
            can_dev.tx.stdid = can_tx_list[3].stdid;
        }else{
            can_tx_list[4].is_enable = true;
            can_dev.tx.buff = can_tx_list[4].buff;
            can_dev.tx.stdid = can_tx_list[4].stdid;
        }
    }
    can_dev.tx.len = 8;
    can_dev.tx.is_enable = true;

    can_dev.can_rx_add(0x200+id,rxSem,[this](CAN *_can_dev,uint8_t *_rx_data) {
        this->motor_rx_data_update_callback(_can_dev,_rx_data);
    });
}

void DJI_Motor_2006::DJI_Motor_2006_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse, osSemaphoreId_t rxSem){
    DJI_Motor_init(hcan,motorId,isReverse);
    if(hcan==&hcan1){
        if(id<5){
            can_tx_list[0].is_enable = true;
            can_dev.tx.buff = can_tx_list[0].buff;
            can_dev.tx.stdid = can_tx_list[0].stdid;
        }else{
            can_tx_list[1].is_enable = true;
            can_dev.tx.buff = can_tx_list[1].buff;
            can_dev.tx.stdid = can_tx_list[1].stdid;
        }
    }
    else if(hcan==&hcan2) {
        if(id<5){
            can_tx_list[3].is_enable = true;
            can_dev.tx.buff = can_tx_list[3].buff;
            can_dev.tx.stdid = can_tx_list[3].stdid;
        }else{
            can_tx_list[4].is_enable = true;
            can_dev.tx.buff = can_tx_list[4].buff;
            can_dev.tx.stdid = can_tx_list[4].stdid;
        }
    }
    can_dev.tx.len = 8;
    can_dev.tx.is_enable = true;

    can_dev.can_rx_add(0x200+id,rxSem,[this](CAN *_can_dev,uint8_t *_rx_data) {
        this->motor_rx_data_update_callback(_can_dev,_rx_data);
    });
}

void DJI_Motor_6020::DJI_Motor_6020_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse, osSemaphoreId_t rxSem){
    DJI_Motor_init(hcan,motorId,isReverse);
    if(hcan==&hcan1) {
        if(id<5){
            can_tx_list[1].is_enable = true;
            can_dev.tx.buff = can_tx_list[1].buff;
            can_dev.tx.stdid = can_tx_list[1].stdid;
        }else{
            can_tx_list[2].is_enable = true;
            can_dev.tx.buff = can_tx_list[2].buff;
            can_dev.tx.stdid = can_tx_list[2].stdid;
        }
    }
    else if(hcan==&hcan2) {
        if(id<5){
            can_tx_list[4].is_enable = true;
            can_dev.tx.buff = can_tx_list[4].buff;
            can_dev.tx.stdid = can_tx_list[4].stdid;
        }else{
            can_tx_list[5].is_enable = true;
            can_dev.tx.buff = can_tx_list[5].buff;
            can_dev.tx.stdid = can_tx_list[5].stdid;
        }
    }
    can_dev.tx.len = 8;
    can_dev.tx.is_enable = true;

    can_dev.can_rx_add(0x204+id,rxSem,[this](CAN *_can_dev,uint8_t *_rx_data) {
        this->motor_rx_data_update_callback(_can_dev,_rx_data);
    });
}


void DJI_Motor::set_torque_to_can_tx_buff()
{
    uint8_t index = (id < 5 ? id : id - 4) * 2 - 1;

    can_dev.tx.buff[index-1] = high_byte(set_torque);
    can_dev.tx.buff[index] = low_byte(set_torque);
}

void DJI_Motor::motor_data_update(uint8_t *can_rx_data)
{
    motor_data_t *ptr = &(motor_data);

    raw.ecd = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
    raw.speed_rpm = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    raw.fb_torque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
    raw.temperature = can_rx_data[6];

    //get_offset
    rcv_cnt++;
    if (rcv_cnt > 50) {
        is_zero_offset = true;
    }
    /* initial value */
    if (is_zero_offset == false)
    {
        ptr->speed = (float)raw.speed_rpm / max_speed;

        VAL_LIMIT(ptr->speed,-1.0f,1.0f);
        ptr->round = (float)raw.ecd * (1.0f/max_ecd);
        VAL_LIMIT(ptr->round,0.0f,1.0f);
        ptr->offset_round = ptr->round;
        ptr->temperature = (int8_t)raw.temperature;
        stall_cnt = 0;
        return;
    }

    uint32_t x = taskENTER_CRITICAL_FROM_ISR();
    //round
    ptr->last_round = ptr->round;//电机编码反馈值
    ptr->round = (float)raw.ecd * (1.0f/max_ecd);;//电机编码反馈值
    VAL_LIMIT(ptr->round,0.0f,1.0f);
    if (ptr->round - ptr->last_round > 0.5f){
        ptr->spill_cnt--;
    }
    else if (ptr->round - ptr->last_round < -0.5f){
        ptr->spill_cnt++;
    }
    ptr->total_rounds = (float) ptr->spill_cnt + ptr->round - ptr->offset_round;
    ptr->total_rounds_without_offset = (float) ptr->spill_cnt + ptr->round;

    //speed
    ptr->last_speed = ptr->speed;
    ptr->speed = (float)raw.speed_rpm / max_speed;
    VAL_LIMIT(ptr->speed,-1.0f,1.0f);

    //temperature
    ptr->temperature = (int8_t)raw.temperature;

    if(ptr->temperature>60){
        is_overheat = true;
    }else{
        is_overheat = false;
    }

    //torque
    fb_torque = raw.fb_torque;
    if(ABS(fb_torque) >ABS(stall_torque_max) && ABS(ptr->speed) < ABS(stall_speed_max)) {
        stall_cnt++;
    }else{
        stall_cnt = 0;
    }

    get_truly.rounds = get_motor_total_rounds();
    get_truly.speed = get_motor_speed();
    if(is_reverse) {
        get_truly.torque = (float)-raw.fb_torque;
    }else{
        get_truly.torque = (float)raw.fb_torque;
    }

    if(process_data_ptr_fun != nullptr){
        process_data_ptr_fun();
    }
    taskEXIT_CRITICAL_FROM_ISR(x);
}

void DJI_Motor::motor_rx_data_update_callback(CAN *_can_dev, uint8_t *_rx_data) {
    update_frequency_Hz();
    motor_data_update(_rx_data);
//    osSemaphoreRelease(_can_dev->rx.semaphore);
}

int16_t DJI_Motor::get_set_torque(){
    return set_torque;
}

int16_t DJI_Motor::get_fb_torque(){
    return raw.fb_torque;
}

void DJI_Motor::set_motor_normalization_torque(float torque){
    torque = torque>1.0f?1.0f:(torque<-1.0f?-1.0f:torque);
    set_truly.torque = max_torque * torque;
    if(is_reverse) {
        set_torque= (int16_t)(-torque * max_torque);
    }else{
        set_torque = (int16_t)(torque * max_torque);
    }
}

void DJI_Motor::set_motor_speed(float speed){
    set_truly.speed = speed;
    set_motor_normalization_torque(pid_calculate(&(velPid), get_motor_speed(), speed));
}

void DJI_Motor::set_motor_free(){
    is_zero_offset = false;
    rcv_cnt = 0;
    set_torque = 0;
    set_truly.torque = 0;
}

void DJI_Motor::set_motor_rounds(float rounds) {
    set_truly.rounds = rounds;
    float speed_set = pid_calculate(&posPid, get_motor_total_rounds(), rounds);
    set_motor_speed(speed_set);
}

void DJI_Motor::set_motor_stall_parameter(int16_t _stall_torque_max,float _stall_speed_max){
    this->stall_torque_max = _stall_torque_max;
    this->stall_speed_max = _stall_speed_max;
}

void DJI_Motor::process_data_ptr_fun_register(void (*fun)()) {
    process_data_ptr_fun = fun;
}

DJI_Motor_3508::DJI_Motor_3508() : DJI_Motor()
{
    max_torque = MOTOR_MAX_CURRENT_M3508;
    max_speed = MOTOR_MAX_SPEED_M3508;
    max_ecd = MOTOR_MAX_ECD_M3508;
}

DJI_Motor_2006::DJI_Motor_2006() : DJI_Motor()
{
    max_torque = MOTOR_MAX_CURRENT_M2006;
    max_speed = MOTOR_MAX_SPEED_M2006;
    max_ecd = MOTOR_MAX_ECD_M2006;
}

DJI_Motor_6020::DJI_Motor_6020():DJI_Motor()
{
    max_torque = MOTOR_MAX_VOLTAGE_GM6020;//voltage
    max_speed = MOTOR_MAX_SPEED_GM6020;
    max_ecd = MOTOR_MAX_ECD_GM6020;
}

