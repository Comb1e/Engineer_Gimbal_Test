//
// Created by Henson on 2023-10-22.
//

#include "user_lib.h"

//Hal库时钟自己定为1000
uint32_t get_time_us(void) {
    return TIM7->CNT;
}

uint32_t get_time_ms(void) {
    return HAL_GetTick();
}

float get_time_ms_us(void) {
    return ((float)get_time_ms() + (float)get_time_us()/1000.0f);
}

float get_min_value(float a, float b) {
    if(b<a) {
        return b;
    }
    else {
        return a;
    }
}

float get_max_value(float a, float b) {
    if(b<a) {
        return a;
    }
    else {
        return b;
    }
}

bool is_val_between(float val, float min, float max){
    if(val>=min && val<=max){
        return true;
    }
    return false;
}

//绝对值限幅，两侧限幅相同
void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    }
    if (*a < -abs_max) {
        *a = -abs_max;
    }
}

//常数限幅，两侧限幅不同，这个是改变原值，但是也不返回值，返回值也可以的
void val_limit(float *a, float val_max,float val_min) {
    if (*a > val_max) {
        *a = val_max;
    }
    if (*a < val_min) {
        *a = val_min;
    }
}


uint32_t unsigned_32(uint8_t *p) {
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}

uint16_t unsigned_16(uint8_t *p) {
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

int16_t signed_16(uint8_t *p) {
    int16_t u;
    memcpy(&u, p, 2);
    return u;
}

//divisor必须为正 余数必须大于零 除数不能为0
//反馈值的范围应该是[0,divisor),quotient为整数
//商: quotient 除数: divisor 被除数: dividend
float get_modular(float dividend,float divisor,float *quotient){
    float remainder = dividend;
    float quo_temp = 0;
    if(divisor==0) {
        return remainder;
    }
    while(remainder<0) {
        quo_temp = quo_temp - 1;
        remainder = remainder + divisor;
    }
    while(remainder >= divisor) {
        quo_temp = quo_temp + 1;
        remainder = remainder - divisor;
    }
    *quotient = quo_temp;
    return remainder;
}


bool is_approx_equal_to(float current_val, float target_val, float _error){
    float err = current_val - target_val;
    return (fabsf(err)<=_error);
}


bool is_over_time_interval(uint32_t _time_ms){
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();;//这个不用static

    if((uint32_t)(current_time - last_time) >= _time_ms){
        last_time = current_time;
        return true;
    }
    else{
        return false;
    }
}


bool is_under_time_interval(uint32_t _time_ms){
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();;//这个不用static

    if((uint32_t)(current_time - last_time) <= _time_ms){
        last_time = current_time;
        return true;
    }
    else{
        return false;
    }
}