//
// Created by Henson on 2023-11-18.
//

#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

#include "user_lib.h"
#include "pid.h"

#define RAD2ROUND   (1.0f/(2.0f*PI))
#define ROUND2RAD   (2.0f*PI)

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

class Motor{
public:
    volatile bool is_lost = true;
    volatile bool is_reverse = false;
    volatile bool is_zero_offset = false;
    volatile bool is_overheat = false;

    volatile uint32_t id = 0;
    volatile uint32_t rcv_cnt = 0;
    volatile uint32_t lost_cnt = 0;

    volatile double beginning_t_ms = 0;
    volatile double ending_t_ms = 0;
    volatile double delta_t_s = 1;
    volatile double update_Hz = 0;

//protected:
    typedef struct{
        float speed;                        //当前转速 归一化
        float last_speed;
        float round;                        //转子机械角度 编码器值
        float last_round;
        float offset_round;                 //校偏置 -2~2
        float total_rounds;                 //总圈准数,与方向有关
        float total_rounds_without_offset;  //总圈准数,与方向有关
        int8_t temperature;                 //电机温度℃
        int32_t spill_cnt;                  //圈数计算,大部分都为整数
    }motor_data_t;
    motor_data_t motor_data={};

    typedef struct{
        float speed;                        //设定当前归一化圈数
        float rounds;                       //设定当前圈数
        float torque;                       //设定当前转矩
    }set_truly_t;

    typedef struct{
        float speed;                        //设定当前归一化圈数
        float rounds;                       //设定当前圈数
        float torque;                       //设定当前转矩
    }get_truly_t;

    set_truly_t set_truly;
    get_truly_t get_truly;

public:
    Motor()= default;

    virtual bool is_motor_lost();
    virtual void set_motor_lost();
    virtual void set_motor_connect();

    virtual void set_motor_reverse();
    virtual void set_motor_forward();
    virtual void set_motor_toggle();

    float get_motor_speed();
    float get_motor_last_speed();
    float get_motor_total_rounds();
    float get_motor_total_rounds_without_offset();
    float get_motor_offset_round();
    float get_motor_current_round();
    int8_t get_temperature();

    double get_update_frequency_Hz();
    void update_frequency_Hz();

    void set_motor_offset(float _offset_round);
    virtual void set_motor_total_rounds_offset(float _total_rounds);

    virtual void set_motor_speed(float speed){};
    virtual void set_motor_rounds(float rounds){};

    virtual void set_motor_free(){};

private:

};


#endif //__DRV_MOTOR_H
