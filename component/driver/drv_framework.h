//
// Created by Henson on 2024-01-16.
//

#ifndef __DRV_FRAMEWORK_H
#define __DRV_FRAMEWORK_H

#include "drv_DJI_motor.h"
#include "drv_DM_motor.h"
#include "arm_parameters.h"
/*---------------------------- C++ Scope ---------------------------*/

class Framework {
public:
    //initial
    DJI_Motor_3508 uplift_l;
    DJI_Motor_3508 uplift_r;
    DJI_Motor_3508 extend_l;
    DJI_Motor_3508 extend_r;
    DJI_Motor_2006 slide;
    DM_Motor_J4310 arm_yaw;
    DM_Motor_J4310 arm_pitch;

    struct pid uplift_syn_master_pid={};
    struct pid uplift_syn_follower_pid={};

    struct pid extend_syn_master_pid={};
    struct pid extend_syn_follower_pid={};

    struct pid uplift_syn_master_torque_pid={};
    struct pid uplift_syn_follower_torque_pid={};

    //kine
    typedef struct {
        float arm_yaw;      //yaw
        float arm_pitch;      //yaw
        float slide;        //y
        float extend_l;       //x
        float extend_r;       //x
        float uplift_l;       //z
        float uplift_r;       //z
    }framework_kine_motor_set_rounds_t;
    framework_kine_motor_set_rounds_t set_rounds={};

    typedef struct{
        uint32_t uplift;
        uint32_t extend;
        uint32_t slide;
        uint32_t arm_yaw;
        uint32_t arm_pitch;
    }reset_step_t;
    reset_step_t reset_step={};

    //safe
    bool is_all_motors_safe = false;

    //reset flag
    bool is_uplift_need_reset = false;
    bool is_slide_need_reset = false;
    bool is_extend_need_reset = false;

    bool is_uplift_offset = false;
    bool is_extend_offset = false;
    bool is_slide_offset = false;

    bool is_uplift_initial = false;
    bool is_extend_initial = false;
    bool is_slide_initial = false;

    //arm_yaw
    bool is_arm_yaw_need_reset = false;
    bool is_arm_yaw_offset = false;
    bool is_arm_yaw_initial = false;

    //arm_pitch
    bool is_arm_pitch_need_reset = false;
    bool is_arm_pitch_offset = false;
    bool is_arm_pitch_initial = false;

    bool is_uplift_up_movement_ok = false;

public:
    explicit Framework();
    void framework_init();
    //get_offset
    bool uplift_reset_to_get_offset();
    bool extend_reset_to_get_offset();
    bool slide_reset_to_get_offset();

    //IK set xyz yaw ex
    void framework_set_extend(float extend_mm);
    void framework_set_slide(float slide_mm);
    void framework_set_uplift(float uplift_mm);
    void framework_set_arm_yaw(float arm_yaw_deg);
    void framework_set_arm_pitch(float arm_pitch_deg);

    //FK get xyz yaw ex
    float get_framework_extend_mm();
    float get_framework_slide_mm();
    float get_framework_uplift_mm();
    float get_framework_arm_yaw_deg();
    float get_framework_arm_pitch_deg();

    //set move
    void framework_set_extend_move();
    void framework_set_slide_move();
    void framework_set_uplift_move();
    void framework_set_arm_yaw_move();
    void framework_set_arm_pitch_move();
    void set_framework_move();

    //is in set
    bool is_framework_in_set_extend();
    bool is_framework_in_set_uplift();
    bool is_framework_in_set_slide();
    bool is_framework_in_set_arm_yaw();
    bool is_framework_in_set_arm_pitch();

    //reset flag
    bool is_offset_ok();
    void set_framework_reset();
    //safe
    void extend_motors_protection();
    bool check_all_motors_safe();
    void set_reset_pid();
    void set_running_pid();

    //pid
    void set_uplift_reset_pid();
    void set_uplift_running_pid();

    void set_extend_reset_pid();
    void set_extend_running_pid();

    void set_slide_reset_pid();
    void set_slide_running_pid();

    void set_arm_yaw_dm_reset_pid();
    void set_arm_yaw_dm_running_pid();


    void set_arm_pitch_dm_reset_pid();
    void set_arm_pitch_dm_running_pid();


    //reset
    void set_uplift_reset();
    void set_extend_reset();
    void set_slide_reset();
    void set_arm_yaw_reset();
    void set_arm_pitch_reset();

    bool is_uplift_at_initial_pos();
    bool is_extend_at_initial_pos();
    bool is_slide_at_initial_pos();

    bool is_framework_at_initial_pos();
    bool is_arm_yaw_running();
    bool is_arm_pitch_running();

    bool uplift_up_movement_to_avoid_arm_clash(uint32_t up_movement_ms);

};

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__DRV_FRAMEWORK_H
