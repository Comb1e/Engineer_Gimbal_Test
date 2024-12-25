//
// Created by Henson on 2023-11-18.
//

#include "drv_motor.h"

bool Motor::is_motor_lost(){
    return is_lost;
}
void Motor::set_motor_lost(){
    is_lost = true;
    lost_cnt++;
}
void Motor::set_motor_connect(){
    is_lost = false;
}

float Motor::get_motor_speed(){
    if(is_reverse) {
        return (float)-motor_data.speed;
    }else{
        return (float)motor_data.speed;
    }
}

float Motor::get_motor_last_speed(){
    if(is_reverse) {
        return (float)-motor_data.last_speed;
    }else{
        return (float)motor_data.last_speed;
    }
}

float Motor::get_motor_total_rounds(){
    if(is_reverse){
        return -motor_data.total_rounds;
    }else{
        return motor_data.total_rounds;
    }
}

float Motor::get_motor_total_rounds_without_offset(){
    if(is_reverse){
        return -motor_data.total_rounds_without_offset;
    }else{
        return motor_data.total_rounds_without_offset;
    }
}

float Motor::get_motor_offset_round(){
    return motor_data.offset_round;
}
float Motor::get_motor_current_round(){
    return motor_data.round;
}

int8_t Motor::get_temperature(){
    return motor_data.temperature;
}

double Motor::get_update_frequency_Hz(){
    return update_Hz;
}

void Motor::update_frequency_Hz(){
    double temp_ms = (double)get_time_ms_us();
    delta_t_s = (temp_ms - beginning_t_ms)/1000.0;
    update_Hz = 1.0/delta_t_s;
    beginning_t_ms = temp_ms;
}

void Motor::set_motor_offset(float _offset_round){
    motor_data.offset_round = _offset_round;
}

void Motor::set_motor_total_rounds_offset(float _total_rounds){
    float round_f = _total_rounds - (float)((int)(_total_rounds));
    motor_data.spill_cnt = (int)_total_rounds;
    if(!is_reverse){//正转
        motor_data.spill_cnt = (int)_total_rounds;
        motor_data.offset_round = motor_data.round - round_f;//可能为负,转向不影响offset设置
    }
    else{//反转
        motor_data.spill_cnt = -(int)_total_rounds;
        motor_data.offset_round = motor_data.round + round_f;//可能为负,转向不影响offset设置
    }
    is_zero_offset = true;
}

void Motor::set_motor_reverse(){
    if(!is_reverse){
        is_reverse = true;
        float _total_rounds = get_motor_total_rounds();
        set_motor_total_rounds_offset(_total_rounds);
    }
}
void Motor::set_motor_forward(){
    if(is_reverse){
        is_reverse = false;
        float _total_rounds = get_motor_total_rounds();
        set_motor_total_rounds_offset(_total_rounds);
    }
}
void Motor::set_motor_toggle(){
    float _total_rounds = get_motor_total_rounds();
    is_reverse = !is_reverse;
    set_motor_total_rounds_offset( _total_rounds);
}