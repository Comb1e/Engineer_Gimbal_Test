//
// Created by Henson on 2023-11-18.
//

#ifndef __DRV_IMU_H
#define __DRV_IMU_H

#include "common_inc.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/
class IMU
{
public:
    typedef enum {
        x = 1,
        y = 2,
        z = 3,
    }imu_e;

    typedef struct {
        float ang_acc;// we don't have it
        float ang_vel;
        float integral_deg;

        float deg;
        float last_deg;//有范围
        float delta_deg;//每次角度差值
        int32_t round_cnt;//圈数计算
        int32_t spill_cnt;//溢出计数
        float total_rounds;//总圈数
        float offset_deg;
    }gyro_t;

    typedef struct {
        float acc;//m/s2
        float vel;//m/s
        float pos;//m
        float acc_offset;
    }accelerometer_t;

    typedef struct{
        gyro_t gx;
        gyro_t gy;
        gyro_t gz;
        accelerometer_t ax;
        accelerometer_t ay;
        accelerometer_t az;
        struct {
            float W;
            float X;
            float Y;
            float Z;
        }quaternion;//四元数 16
        float temperature;
    }ImuData_t;

    //initial
    volatile bool is_lost = true;
    volatile bool is_enable = true;
    volatile bool is_zero_offset = false;
    volatile bool last_offset = false;


    //time counting
    volatile double beginning_t_ms = 0;
    volatile double ending_t_ms = 0;
    volatile double delta_t_s = 1;
    volatile double update_Hz = 0;

    //data counting
    volatile uint32_t error_data_cnt = 0;
    volatile uint32_t lost_cnt = 0;
    volatile uint32_t rcv_cnt = 0;

    ImuData_t imu_data={};

public:
    IMU() = default;

    bool is_imu_lost();
    void set_imu_lost();
    void set_imu_connect();
    bool is_imu_enable();
    void set_imu_disable();
    void set_imu_enable();

    float get_total_rounds(imu_e imuE);
    float get_total_rounds_without_offset(imu_e imuE);
    float get_total_radian(imu_e imuE);
    float get_total_radian_without_offset(imu_e imuE);
    float get_current_radian(imu_e imuE);
    float get_current_deg(imu_e imuE);

    void set_offset_deg(imu_e imuE,float offset_deg);
    void set_current_as_offset();
    float getTemperature();

    float get_acceleration(imu_e imuE);
    float get_velocity(imu_e imuE);
    float get_position(imu_e imuE);

    void clear_position(imu_e imuE);
    void clear_velocity(imu_e imuE);

    void update_frequency_Hz();
    double get_update_frequency_Hz();

//    virtual void update_imu()=0;
//    virtual void reset_imu()=0;
//    virtual void update_gyro()=0;
//    virtual void update_acc()=0;
//    virtual void update_msg()=0;
//    virtual void init_gyro_offset()=0;
//    virtual void init_acc_offset()=0;

};

#endif //__DRV_IMU_H
