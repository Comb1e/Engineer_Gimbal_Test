//
// Created by Henson on 2023-11-18.
//

#include "drv_imu.h"
#include "user_lib.h"

float IMU::get_total_rounds(imu_e imuE){
    float total_rounds = 0.0f;
    switch(imuE) {
        case x:
            total_rounds = imu_data.gx.total_rounds; break;
        case y:
            total_rounds = imu_data.gy.total_rounds; break;
        case z:
            total_rounds = imu_data.gz.total_rounds; break;
        default:
            break;
    }
    return total_rounds;
}

float IMU::get_total_rounds_without_offset(imu_e imuE){
    float total_rounds_without_offset = 0.0f;
    switch(imuE) {
        case x:
            total_rounds_without_offset = imu_data.gx.total_rounds + (imu_data.gx.offset_deg/360.0f); break;
        case y:
            total_rounds_without_offset = imu_data.gy.total_rounds + (imu_data.gy.offset_deg/360.0f); break;
        case z:
            total_rounds_without_offset = imu_data.gz.total_rounds + (imu_data.gz.offset_deg/360.0f); break;
        default:
            break;
    }
    return total_rounds_without_offset;
}


float IMU::get_current_radian(imu_e imuE) {
    return get_current_deg(imuE)/180.0f*PI;
}

float IMU::get_current_deg(imu_e imuE){
    switch(imuE){
        case x:
            return imu_data.gx.deg;
        case y:
            return imu_data.gy.deg;
        case z:
            return imu_data.gz.deg;
        default:
            return 0;
    }
}

float IMU::get_total_radian(imu_e imuE){
    return get_total_rounds(imuE)*2*PI;
}

float IMU::get_total_radian_without_offset(imu_e imuE){
   return get_total_rounds_without_offset(imuE)*2*PI;
}

float IMU::getTemperature() {
    return imu_data.temperature;
}

void IMU::update_frequency_Hz(){
    double temp_ms = (double)get_time_ms_us();
    delta_t_s = (temp_ms - beginning_t_ms)/1000.0;
    update_Hz = 1.0f/delta_t_s;
    beginning_t_ms = temp_ms;
}

double IMU::get_update_frequency_Hz() {
    return update_Hz;
}

bool IMU::is_imu_lost(){
    return is_lost;
}
void IMU::set_imu_lost(){
    is_zero_offset = false;//相当于重开陀螺仪重新复位  todo
    is_lost = true;
}
void IMU::set_imu_connect(){
    is_lost = false;
}

bool IMU::is_imu_enable(){
    return is_enable;
}
void IMU::set_imu_enable(){
    is_enable = true;
}
void IMU::set_imu_disable(){
    is_enable = false;
}


void IMU::set_offset_deg(imu_e imuE,float offset_deg){
    switch(imuE){
        case x: imu_data.gx.offset_deg = offset_deg;
            break;
        case y: imu_data.gy.offset_deg = offset_deg;
            break;
        case z: imu_data.gz.offset_deg = offset_deg;
            break;
        default:
            break;
    }
}

float IMU::get_acceleration(IMU::imu_e imuE) {
    switch(imuE){
        case x:
            return imu_data.ax.acc;
        case y:
            return imu_data.ay.acc;
        case z:
            return imu_data.az.acc;
        default:
            return 0;
    }
}

float IMU::get_velocity(IMU::imu_e imuE) {
    switch(imuE){
        case x:
            return imu_data.ax.vel;
        case y:
            return imu_data.ay.vel;
        case z:
            return imu_data.az.vel;
        default:
            return 0;
    }
}

float IMU::get_position(IMU::imu_e imuE) {
    switch(imuE){
        case x:
            return imu_data.ax.pos;
        case y:
            return imu_data.ay.pos;
        case z:
            return imu_data.az.pos;
        default:
            return 0;
    }
}

void IMU::clear_position(imu_e imuE){
    taskENTER_CRITICAL();
    switch(imuE){
        case x:
            imu_data.ax.pos = 0;
        case y:
            imu_data.ay.pos = 0;
        case z:
            imu_data.az.pos = 0;
        default:
            __NOP();
    }
    taskEXIT_CRITICAL();
}

void IMU::clear_velocity(imu_e imuE){
    taskENTER_CRITICAL();
    switch(imuE){
        case x:
            imu_data.ax.vel = 0;
        case y:
            imu_data.ay.vel = 0;
        case z:
            imu_data.az.vel = 0;
        default:
            __NOP();
    }
    taskEXIT_CRITICAL();
}

void IMU::set_current_as_offset() {
    rcv_cnt = 0;
    is_zero_offset = false;//相当于重开陀螺仪重新复位
}


