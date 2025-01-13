//
// Created by Henson on 2024-01-14.
//

#include "task_ctrl.h"
#include "drv_arm.h"
#include "drv_comm.h"

// before
// 1. 大pitch
// 2. 大yaw
// 3. x和z
// 4. 横移和其它的几个轴

// current
// 1. z向上移动
// 2. 大pitch
// 3. 大yaw
// 4. x y z
// 5. xyx

//1 z
void upliftCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->framework.reset_step.uplift = 0;
    for(;;)
    {
        if(communication.arm->framework.is_uplift_need_reset)
        {
            communication.arm->framework.Uplift_Reset();
        }
        else if(false ==(communication.arm->framework.is_uplift_offset && communication.arm->framework.is_uplift_initial))
        {
            communication.arm->framework.uplift_l.set_torque = 0;
            communication.arm->framework.uplift_r.set_torque = 0;
//            VAL_LIMIT(uplift_set_torque,-1.0f,1.0f);
//            communication.arm->framework.uplift_l.set_torque = (int16_t)(-uplift_set_torque*16384.0f);
//            communication.arm->framework.uplift_r.set_torque = (int16_t)(uplift_set_torque*16384.0f);
        }
        else
        {
            communication.arm->framework.framework_set_uplift(communication.arm->set_joint.uplift_joint);
        }
        communication.arm->framework.framework_set_uplift_move();
        osDelay(3);
    }
}


//2 pitch
void armPitchCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->framework.reset_step.arm_pitch = 0;
    //先使能
    for(;;)
    {
        if(communication.arm->framework.is_arm_pitch_need_reset)
        {
            communication.arm->framework.Arm_Pitch_Reset();
        }
        else if(false==(communication.arm->framework.is_arm_pitch_offset && communication.arm->framework.is_arm_pitch_initial))
        {
            communication.arm->framework.arm_pitch.ctrl_data.kp = 0;
            communication.arm->framework.arm_pitch.ctrl_data.kd = 0;
            communication.arm->framework.arm_pitch.ctrl_data.torq = 0;
        }
        else
        {
            communication.arm->framework.framework_set_arm_pitch(communication.arm->set_joint.arm_pitch_joint);
        }
        if(communication.arm->framework.arm_pitch.recover_the_motor()==false)
        {
            communication.arm->framework.framework_set_arm_pitch_move();
        }
//        else{
//            communication.arm->framework.set_arm_pitch_reset();
//            osDelay(200);
//        }
        osDelay(3);
    }
}

//3 yaw
void armYawCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->framework.reset_step.arm_yaw = 0;
    //先使能
    for(;;){
        if(communication.arm->framework.is_arm_yaw_need_reset)
        {
            communication.arm->framework.Arm_Yaw_Reset();
        }
        else if(false==(communication.arm->framework.is_arm_yaw_offset && communication.arm->framework.is_arm_yaw_initial))
        {
            communication.arm->framework.arm_yaw.ctrl_data.kp = 0;
            communication.arm->framework.arm_yaw.ctrl_data.kd = 0;
            communication.arm->framework.arm_yaw.ctrl_data.torq = 0;
        }
        else
        {
            communication.arm->framework.framework_set_arm_yaw(communication.arm->set_joint.arm_yaw_joint);
        }
        if(communication.arm->framework.arm_yaw.recover_the_motor()==false)
        {
            communication.arm->framework.framework_set_arm_yaw_move();
        }
//        else{
//            this->set_arm_yaw_reset();
//            osDelay(200);
//        }
        osDelay(3);
    }
}

//4 x
void extendCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->framework.reset_step.extend = 0;
    for(;;)
    {
        if(communication.arm->framework.is_extend_need_reset)
        {
            communication.arm->framework.Extend_Reset();
        }
        else if(false == (communication.arm->framework.is_extend_offset && communication.arm->framework.is_extend_initial))
        {
            communication.arm->framework.extend_l.set_torque = 0;
            communication.arm->framework.extend_r.set_torque = 0;
        }
        else
        {
            communication.arm->framework.framework_set_extend(communication.arm->set_joint.extend_joint);
        }
        communication.arm->framework.extend_motors_protection();
        communication.arm->framework.framework_set_extend_move();
        osDelay(3);
    }
}

//4 y
void slideCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->framework.reset_step.slide = 0;
    for(;;)
    {
        if(communication.arm->framework.is_slide_need_reset)
        {
            communication.arm->framework.Slide_Reset();
        }
        else if(false==(communication.arm->framework.is_slide_offset && communication.arm->framework.is_slide_initial))
        {
            communication.arm->framework.slide.set_torque = 0;
        }
        else
        {
            communication.arm->framework.framework_set_slide(communication.arm->set_joint.slide_joint);
        }
        communication.arm->framework.framework_set_slide_move();
        osDelay(3);
    }
}


//5 x1
void armRollCtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200);
    communication.arm->actuator.reset_step.arm_roll = 0;
    for(;;)
    {
        if(communication.arm->actuator.is_x1_need_reset)
        {
            communication.arm->actuator.Arm_Roll_Reset();
        }
        else if(false==(communication.arm->actuator.is_x1_offset && g_arm.actuator.is_x1_initial))
        {
            communication.arm->actuator.arm_roll.set_torque = 0;
        }
        else
        {
            communication.arm->actuator.actuator_set_x1(communication.arm->set_joint.arm_roll_joint);
        }
        communication.arm->actuator.actuator_set_x1_move();
        osDelay(3);
    }
}
//5 yx2
void yX2CtrlTask(void *argument)
{
    while(!(communication.arm->is_all_peripheral_connect && communication.arm->is_init))
    {
        osDelay(1);
    }
    osDelay(200)        ;
    communication.arm->actuator.reset_step.y_x2 = 0;
    for(;;)
    {
        if(communication.arm->actuator.is_y_x2_need_reset)
        {
            communication.arm->actuator.Y_X2_Reset();
        }
        else if(false==(communication.arm->actuator.is_y_x2_offset && communication.arm->actuator.is_y_x2_initial))
        {
            communication.arm->actuator.pitch_roll_b.set_torque = 0;
            communication.arm->actuator.pitch_roll_f.set_torque = 0;
        }
        else
        {
            communication.arm->actuator.actuator_set_y_x2(communication.arm->set_joint.pitch_joint, communication.arm->set_joint.roll_joint);
        }
        communication.arm->actuator.actuator_set_y_x2_move();
        osDelay(3);
    }
}

