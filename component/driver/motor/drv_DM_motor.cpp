//
// Created by Henson on 2024-01-15.
//

#include "drv_DM_motor.h"

//can需要5V供电 同时控制数据一发一回，不发不反馈
//注意线序,达妙电机线序很阴间
//绝对值编码器,但是也许由于是多圈计数，每次会变
//达妙反馈的位置就是最外面看到的位置,不是1：10中1的位置,反馈的速度也是最外面的速度

DM_Motor::DM_Motor(): Motor(), can_dev() {

}

float DM_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int DM_Motor::float_to_uint(float x, float x_min, float x_max, int bits){
/// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


void DM_Motor::motor_rx_data_update_callback(CAN *_can_dev, uint8_t *_rx_data){
    update_frequency_Hz();
    uint32_t x = taskENTER_CRITICAL_FROM_ISR();
    motor_data_update(_rx_data);
    taskEXIT_CRITICAL_FROM_ISR(x);
//    osSemaphoreRelease(_can_dev->rx.semaphore);
}


void DM_Motor::set_motor_speed(float speed){
    set_truly.speed = speed;
    if(mode==DM_MIT){
        if(mit_mode==DM_MIT_Torque){
            MIT_outer_set_motor_speed(speed);
        }
        else if(mit_mode == DM_MIT_Velocity){
            MIT_inter_set_motor_speed(speed);
        }
    }
    else if(mode==DM_VO){
        VO_inter_set_motor_speed(speed);
    }
}

void DM_Motor::set_motor_rounds(float rounds){
    set_truly.rounds = rounds;
    if(mode==DM_MIT){
        if(mit_mode==DM_MIT_Torque){
            MIT_outer_set_motor_total_rounds(rounds);
        }
        else if(mit_mode == DM_MIT_Position) {
            MIT_inter_set_motor_round(rounds);
        }
    }else if(mode==DM_PV){
        set_truly.speed = 0.15f;
        PV_inter_set_motor_total_rounds(rounds,0.15f);//0.15 * 30 rad/s很快了
    }
}

static float dm_raw_speed = 0;

void DM_Motor::motor_data_update(uint8_t *can_rx_data)
{
    motor_data_t *ptr = &(motor_data);
    dm_motor_raw_data_t *raw = &raw_data;

    raw->id = can_rx_data[0]&0xF;
    raw->err = can_rx_data[0]>>4;
    //若id>0x0f,则解算err出错
    raw->pos_rad = (can_rx_data[1]<<8)|can_rx_data[2];
    raw->vel_rad_s =(can_rx_data[3]<<4)|(can_rx_data[4]>>4);
    raw->torque = ((can_rx_data[4]&0xF)<<8)|can_rx_data[5];
    raw->t_mos = can_rx_data[6];
    raw->t_rotor = can_rx_data[7];

    if(is_use_absolute_encoder){
        rcv_cnt++;
        ptr->offset_round = 0;
        is_zero_offset = true;
        if (rcv_cnt < 50) {
            //round
            ptr->last_round = ptr->round;//电机编码反馈值
            ptr->round = (uint_to_float(raw->pos_rad, -max_pos_rad, max_pos_rad, 16)+max_pos_rad) * RAD2ROUND; // (-3.14,3.14) //电机编码反馈值
            //speed
            ptr->last_speed = ptr->speed;
            ptr->speed = uint_to_float(raw->vel_rad_s, -max_vel_rad_s, max_vel_rad_s, 12) / max_vel_rad_s; // (-30.0,30.0)
            ptr->speed = 0.95f*ptr->last_speed + 0.05f*ptr->speed;//自带滤波
            VAL_LIMIT(ptr->speed,-1.0f,1.0f);
            return ;
        }
    }else{
        //get_offset
        rcv_cnt++;
        if (rcv_cnt > 50) {
            is_zero_offset = true;
        }
        /* initial value */
        if (is_zero_offset == false)
        {
            ptr->last_speed = ptr->speed;
            ptr->speed = uint_to_float(raw->vel_rad_s, -max_vel_rad_s, max_vel_rad_s, 12) / max_vel_rad_s; // (-30.0,30.0)
            ptr->speed = 0.95f*ptr->last_speed + 0.05f*ptr->speed;//自带滤波
            VAL_LIMIT(ptr->speed,-1.0f,1.0f);

            //(0 - 2*P_MAX)
            ptr->last_round = ptr->round;//电机编码反馈值
            ptr->round = (uint_to_float(raw->pos_rad, -max_pos_rad, max_pos_rad, 16)+max_pos_rad) * RAD2ROUND; // (-3.14,3.14)
            ptr->offset_round = ptr->round;

            stall_cnt = 0;
            return;
        }
    }

    // 这里计圈好像有点问题 尤其是在第一次开的那一下 spill cnt 可能为1或者-1
    // round
    ptr->last_round = ptr->round;//电机编码反馈值
    ptr->round = (uint_to_float(raw->pos_rad, -max_pos_rad, max_pos_rad, 16)+max_pos_rad) * RAD2ROUND; // (-3.14,3.14) //电机编码反馈值
    if (ptr->round - ptr->last_round > max_pos_rad*RAD2ROUND){
        ptr->spill_cnt = ptr->spill_cnt - 1;
    }
    else if (ptr->round - ptr->last_round < -max_pos_rad*RAD2ROUND) {
        ptr->spill_cnt = ptr->spill_cnt + 1;
    }
    ptr->total_rounds = (float) ptr->spill_cnt * (2.0f*max_pos_rad*RAD2ROUND) + ptr->round - ptr->offset_round;//为0
    ptr->total_rounds_without_offset = (float) ptr->spill_cnt * (2.0f*max_pos_rad*RAD2ROUND) + ptr->round;

    //speed
    ptr->last_speed = ptr->speed;
    ptr->speed = uint_to_float(raw->vel_rad_s, -max_vel_rad_s, max_vel_rad_s, 12) / max_vel_rad_s; // (-30.0,30.0)
    dm_raw_speed = ptr->speed;
    dm_raw_speed;
    ptr->speed = 0.95f*ptr->last_speed + 0.05f*ptr->speed;//自带滤波
    VAL_LIMIT(ptr->speed,-1.0f,1.0f);

    //temperature
    ptr->temperature = (int8_t)raw->t_rotor;

    if(ptr->temperature>60){
        is_overheat = true;
    }else{
        is_overheat = false;
    }

    //torque
    fb_torque = uint_to_float(raw->torque, -max_torque, max_torque, 12); // (-10.0,10.0);
    if(ABS(fb_torque) >ABS(stall_torque_max) && ABS(ptr->speed) < ABS(stall_speed_max)) {
        stall_cnt++;
    }else{
        stall_cnt = 0;
    }

    get_truly.rounds = get_motor_total_rounds();
    get_truly.speed = get_motor_speed();
    if(is_reverse) {
        get_truly.torque = -fb_torque;
    }else{
        get_truly.torque = fb_torque;
    }
//    /*#### add enable can it again to solve can receive only one ID problem!!! ####**/
//    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}


//不断调用
void DM_Motor::set_ctrl_to_can_tx_buff(){
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint8_t *pbuf,*vbuf;
    switch (mode) {
        case DM_MIT://MIT模式
            pos_tmp = float_to_uint(ctrl_data.p_des, -max_pos_rad, max_pos_rad, 16);
            vel_tmp = float_to_uint(ctrl_data.v_des, -max_vel_rad_s, max_vel_rad_s, 12);
            tor_tmp = float_to_uint(ctrl_data.torq, -max_torque, max_torque, 12);
            kp_tmp = float_to_uint(ctrl_data.kp, DM_J4310_2EC_KP_MIN, DM_J4310_2EC_KP_MAX, 12);
            kd_tmp = float_to_uint(ctrl_data.kd, DM_J4310_2EC_KD_MIN, DM_J4310_2EC_KD_MAX, 12);

            can_dev.tx.buff[0] = (pos_tmp >> 8);
            can_dev.tx.buff[1] = pos_tmp;
            can_dev.tx.buff[2] = (vel_tmp >> 4);
            can_dev.tx.buff[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
            can_dev.tx.buff[4] = kp_tmp;
            can_dev.tx.buff[5] = (kd_tmp >> 4);
            can_dev.tx.buff[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
            can_dev.tx.buff[7] = tor_tmp;
            break;
        case DM_PV://位置速度模式
            pbuf=(uint8_t*)&ctrl_data.p_des;
            vbuf=(uint8_t*)&ctrl_data.v_des;
            can_dev.tx.buff[0] = *pbuf;
            can_dev.tx.buff[1] = *(pbuf+1);
            can_dev.tx.buff[2] = *(pbuf+2);
            can_dev.tx.buff[3] = *(pbuf+3);
            can_dev.tx.buff[4] = *vbuf;
            can_dev.tx.buff[5] = *(vbuf+1);
            can_dev.tx.buff[6] = *(vbuf+2);
            can_dev.tx.buff[7] = *(vbuf+3);
            break;
        case DM_VO://速度模式
            vbuf=(uint8_t*)&ctrl_data.v_des;
            can_dev.tx.buff[0] = *vbuf;
            can_dev.tx.buff[1] = *(vbuf+1);
            can_dev.tx.buff[2] = *(vbuf+2);
            can_dev.tx.buff[3] = *(vbuf+3);
            break;
        default:
            break;
    }
}

bool DM_Motor::recover_the_motor() {
    if ((dm_motor_state_enum)raw_data.err >= OverVoltage && (dm_motor_state_enum)raw_data.err <= Overload){
        motor_state = (dm_motor_state_enum)raw_data.err;
        set_motor_clear_error();
        set_motor_disable();
        set_motor_enable();
        return true;
    }
    return false;
}

void DM_Motor::DM_Motor_init(CAN_HandleTypeDef *hcan, uint32_t slaveId, uint16_t masterId, dm_motor_mode_e motorMode,bool isReverse)
{
    slave_id = slaveId;
    master_id = masterId;
    can_dev.can_init(hcan);
    this->is_reverse = isReverse;
    mode = motorMode;
}

void DM_Motor_J4310::DM_Motor_J4310_init(CAN_HandleTypeDef *hcan, uint32_t slaveId,uint32_t masterId,
                                         dm_motor_mode_e motorMode, bool isReverse, osSemaphoreId_t rxSem)
{
    DM_Motor_init(hcan,slaveId,masterId,motorMode,isReverse);
    switch (motorMode) {
        case DM_MIT:
            mit_mode = DM_MIT_Torque;
            can_dev.can_rx_tx_add(masterId,rxSem,
                                  [this](CAN *_can_dev,uint8_t *_rx_data) {this->motor_rx_data_update_callback(_can_dev,_rx_data);},
                                  slave_id, (uint8_t*)&tx_buff, 8);
            break;
        case DM_PV:
            can_dev.can_rx_tx_add(masterId,rxSem,
                                  [this](CAN *_can_dev,uint8_t *_rx_data) {this->motor_rx_data_update_callback(_can_dev,_rx_data);},
                                  0x100+slave_id, (uint8_t*)&tx_buff, 8);
            break;
        case DM_VO:
            can_dev.can_rx_tx_add(masterId,rxSem,
                                  [this](CAN *_can_dev,uint8_t *_rx_data) {this->motor_rx_data_update_callback(_can_dev,_rx_data);},
                                  0x200+slave_id, (uint8_t*)&tx_buff, 4);
            break;
        default: break;
    }
////    //无负载
//    pid_struct_init(&velPid, 1.0f, 0.05f, 1.05f, 0.00f, 1.2f);
//    pid_struct_init(&posPid, 0.05f,0.0f,10.0f,0,0);
////    //机械臂负载
////    pid_struct_init(&velPid, 1.0f, 0.05f, 1.05f, 0.00f, 1.2f);
////    pid_struct_init(&posPid,0.18f,0.0f,20.0f,0,0);
    set_motor_enable();
}

// --------------------- MIT --------------------- //

// MIT:  p_des * kp + v_des * kd + t_ff = out
void DM_Motor::MIT_inter_set_motor_normalization_torque(float torque) {
    ABS_LIMIT(torque,1.0f);
    set_truly.torque = max_torque * torque;
    if(is_reverse) {
        ctrl_data.torq = -torque * max_torque;
    }else{
        ctrl_data.torq = torque * max_torque;
    }
    ctrl_data.kp = 0;
    ctrl_data.kd = 0.4f;//0.3或0.4  原来是 0
//    ctrl_data.v_des = 0;
//    ctrl_data.p_des = 0;
}

void DM_Motor::MIT_outer_set_motor_speed(float set_speed){
    float _set_torque = pid_calculate(&velPid,
                                      get_motor_speed(),
                                      set_speed);
    MIT_inter_set_motor_normalization_torque(_set_torque);
}

void DM_Motor::MIT_outer_set_motor_total_rounds(float total_rounds) {
    float set_speed = pid_calculate( &posPid,
                                     get_motor_total_rounds_without_offset(),
                                     total_rounds);
    MIT_outer_set_motor_speed(set_speed);
}

void DM_Motor::MIT_inter_set_motor_speed(float speed){
    ABS_LIMIT(speed,1.0f);
    ctrl_data.v_des = speed * max_vel_rad_s;
    ctrl_data.kp = 0; //->p_ves = 0
    //kd不为0
    ctrl_data.kd = 1;//可改
    ctrl_data.torq = 0;//可改
}

//这个没搞明白，先不用
void DM_Motor::MIT_inter_set_motor_round(float rounds){
    float p_des = rounds*2*PI;
    ABS_LIMIT(p_des,DM_J4310_2EC_P_MAX);
    ctrl_data.p_des = p_des;
    ctrl_data.v_des = 0;//待测
    ctrl_data.kp = 1;//kp不能为0
    ctrl_data.kd = 1;//kd不能为0
    ctrl_data.torq = 0;//不确定
}

// --------------------- PV --------------------- //
void DM_Motor::PV_inter_set_motor_total_rounds(float total_rounds,float speed){//30*0.2f = 6
    ABS_LIMIT(total_rounds,max_pos_rad);
    ctrl_data.p_des = total_rounds*2*PI;
    ABS_LIMIT(speed,1.0f);
    ctrl_data.v_des = speed * max_vel_rad_s;
}

// --------------------- VO --------------------- //
void DM_Motor::VO_inter_set_motor_speed(float speed) {
    ABS_LIMIT(speed,1.0f);
    ctrl_data.v_des = speed * max_vel_rad_s;
}


void DM_Motor::set_motor_total_rounds_offset(float _total_rounds){
    float spill_cnt = 0;
    float round_f = get_modular(_total_rounds, (2.0f * max_pos_rad * RAD2ROUND), &spill_cnt);
    motor_data.spill_cnt = (int) spill_cnt;//int保留的是整数位,不考虑四舍五入,仅仅就是整数位,这里spill_cnt肯定为整数
    if (!is_reverse) {//正转
        motor_data.spill_cnt = (int) spill_cnt;
        motor_data.offset_round = motor_data.round - round_f;//可能为负,转向不影响offset设置
    } else {//反转
        motor_data.spill_cnt = -(int) spill_cnt;
        motor_data.offset_round = motor_data.round + round_f;//可能为负,转向不影响offset设置
    }
    is_zero_offset = true;
}

void DM_Motor::set_motor_stall_parameter(float _stall_torque_max,float _stall_speed_max){
    this->stall_torque_max = _stall_torque_max;
    this->stall_speed_max = _stall_speed_max;
}

//上电自检完毕后需发送“使能”命令才可以进行控制
void DM_Motor::set_motor_enable() {
    can_dev.tx.buff[0] = 0xff;
    can_dev.tx.buff[1] = 0xff;
    can_dev.tx.buff[2] = 0xff;
    can_dev.tx.buff[3] = 0xff;
    can_dev.tx.buff[4] = 0xff;
    can_dev.tx.buff[5] = 0xff;
    can_dev.tx.buff[6] = 0xff;
    can_dev.tx.buff[7] = 0xfc;
    for(uint8_t tmp=0;tmp<10;tmp++) {
        if(can_dev.send_can_msg()==osOK) {
            __NOP();
        }
        osDelay(3);
    }
    is_enable = true;
    motor_state = Enabled;
    osDelay(20);
}


void DM_Motor::set_motor_disable(){
    can_dev.tx.buff[0] = 0xff;
    can_dev.tx.buff[1] = 0xff;
    can_dev.tx.buff[2] = 0xff;
    can_dev.tx.buff[3] = 0xff;
    can_dev.tx.buff[4] = 0xff;
    can_dev.tx.buff[5] = 0xff;
    can_dev.tx.buff[6] = 0xff;
    can_dev.tx.buff[7] = 0xfd;

    for(uint8_t tmp=0;tmp<6;tmp++) {
        if(can_dev.send_can_msg()==osOK) {
            __NOP();
        }
        osDelay(2);
    }
    rcv_cnt = 0;
    is_enable = false;
    motor_state = Disabled;
    is_zero_offset = false;
    ctrl_data.torq = 0;
    ctrl_data.kp = 0;
    ctrl_data.kd = 0;
    set_truly.torque = 0;
}


void DM_Motor::set_motor_save_zero_offset() {
    can_dev.tx.buff[0] = 0xff;
    can_dev.tx.buff[1] = 0xff;
    can_dev.tx.buff[2] = 0xff;
    can_dev.tx.buff[3] = 0xff;
    can_dev.tx.buff[4] = 0xff;
    can_dev.tx.buff[5] = 0xff;
    can_dev.tx.buff[6] = 0xff;
    can_dev.tx.buff[7] = 0xfe;
    for(uint8_t tmp=0;tmp<6;tmp++) {
        if(can_dev.send_can_msg()==osOK) {
            __NOP();
        }
        osDelay(2);
    }
}

void DM_Motor::set_motor_clear_error() {
    can_dev.tx.buff[0] = 0xff;
    can_dev.tx.buff[1] = 0xff;
    can_dev.tx.buff[2] = 0xff;
    can_dev.tx.buff[3] = 0xff;
    can_dev.tx.buff[4] = 0xff;
    can_dev.tx.buff[5] = 0xff;
    can_dev.tx.buff[6] = 0xff;
    can_dev.tx.buff[7] = 0xfb;
    for(uint8_t tmp=0;tmp<6;tmp++) {
        if(can_dev.send_can_msg()==osOK) {
            __NOP();
        }
        osDelay(2);
    }
}

DM_Motor_J4310::DM_Motor_J4310() : DM_Motor()
{
    max_torque = MOTOR_MAX_TORQUE_J4310;
    max_vel_rad_s = MOTOR_MAX_VELOCITY_J4310;
    max_pos_rad = MOTOR_MAX_POSITION_J4310;
}





