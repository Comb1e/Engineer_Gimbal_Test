//
// Created by Henson on 2024-01-16.
//

#ifndef __DRV_ACUTATOR_H
#define __DRV_ACTUATOR_H

#include "drv_DJI_motor.h"
#include "drv_hi229um.h"
#include "arm_parameters.h"

/*---------------------------- C++ Scope ---------------------------*/

class Actuator {
public:
    //initial
    DJI_Motor_2006 arm_roll;
    DJI_Motor_2006 pitch_roll_b;    //up->b
    DJI_Motor_2006 pitch_roll_f;    //down->f
    IMU_Hi229um arm_roll_IMU;//丢失则不使用，且无法代替
    bool is_x1_use_IMU_fb = false;

    //kine
    typedef struct {
        float arm_roll;     //roll
        float pitch_roll_b; //  back   up
        float pitch_roll_f; //  front  down
    }actuator_motor_set_rounds_t;
    actuator_motor_set_rounds_t set_rounds={};

    typedef struct{
        uint32_t arm_roll;
        uint32_t y_x2;
    }reset_step_t;
    reset_step_t reset_step={};

    //safe
    bool is_all_motors_safe = false;

    //reset flag
    bool is_x1_need_reset = false;
    bool is_y_x2_need_reset = false;

    bool is_x1_offset = false;
    bool is_y_x2_offset = false;

    bool is_x1_initial = false;
    bool is_y_x2_initial = false;

public:
    explicit Actuator();
    void actuator_init();

    //get_offset
    bool x1_reset_to_get_offset();
    bool y_x2_reset_to_get_offset();

    //IK set actuator xyx
    void actuator_set_x1(float x1_deg);
    void actuator_set_y_x2(float y_deg, float x2_deg);

    //FK get actuator xyx
    float get_actuator_x1_deg();
    float get_actuator_y_deg();
    float get_actuator_x2_deg();

    //set actuator xyx move
    void actuator_set_x1_move();
    void actuator_set_y_x2_move();
    void set_actuator_move();

    //is_in_set
    bool is_actuator_in_set_x1();
    bool is_actuator_in_set_y_x2();

    //reset
    void set_actuator_reset();
    bool is_offset_ok();
    //safe
    bool check_all_motors_safe();

    void set_reset_pid();

    void set_x1_not_use_IMU_fb();
    void set_x1_use_IMU_fb();

    void set_x1_reset();
    void set_y_x2_reset();

    void set_x1_reset_pid();
    void set_y_x2_reset_pid();

    void set_x1_running_pid();
    void set_y_x2_running_pid();

    bool is_x1_at_initial_pos();
    bool is_y_x2_at_initial_pos();
    bool is_actuator_at_initial_pos();

private:

};

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif //__DRV_ACTUATOR_H
