//
// Created by CYK on 2025/1/7.
//

#include "drv_reset.h"

void Framework::Uplift_Reset()
{
    switch(this->reset_step.uplift)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.uplift = 1;
            }
            this->uplift_l.set_torque = 0;
            this->uplift_r.set_torque = 0;
            break;
        }
        case 1://向上移动一下下，给定一个移动时间就可以
        {
            this->set_uplift_reset_pid();
            if(!this->uplift_l.is_zero_offset || !this->uplift_r.is_zero_offset)
            {
                break;
            }
            if(this->uplift_up_movement_to_avoid_arm_clash(600))
            {//pid在计算
                this->reset_step.uplift = 2;
                this->is_uplift_up_movement_ok = true;
            }
            break;
        }
        case 2:
        {
            if(this->is_arm_pitch_initial)//顺序 等待大pitch
            {
                if(this->uplift_reset_to_get_offset())
                {
                    this->reset_step.uplift = 3;
                }
            }
            else
            {//pitch没复位则一直向上走  pid在计算
                this->uplift_l.set_motor_normalization_torque(pid_calculate(&this->uplift_l.velPid, this->uplift_l.get_motor_speed(), 0.0f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动
                this->uplift_r.set_motor_normalization_torque(pid_calculate(&this->uplift_r.velPid, this->uplift_r.get_motor_speed(), 0.0f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动
            }
            break;
        }
        case 3:
        {
            this->framework_set_uplift(15);//pid 在计算
            if(this->is_framework_in_set_uplift())
            {
                this->is_uplift_initial = true;
                this->reset_step.uplift = 4;
            }
            break;
        }
        case 4:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->framework_set_uplift(communication.arm->set_joint.uplift_joint);
                if(this->is_framework_in_set_uplift())
                {
                    this->set_uplift_running_pid();
                    this->is_uplift_need_reset = false;
                    this->reset_step.uplift = 0;
                }
            }
            else
            {
                this->framework_set_uplift(15);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Framework::Arm_Pitch_Reset()
{
    switch(this->reset_step.arm_pitch)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.arm_pitch = 1;
            }
            this->arm_pitch.ctrl_data.kp = 0;
            this->arm_pitch.ctrl_data.kd = 0;
            this->arm_pitch.ctrl_data.torq = 0;
            break;
        }
        case 1:
        {
            this->set_arm_pitch_dm_reset_pid();
            if(!this->arm_pitch.is_zero_offset)
            {
                break;
            }
            this->is_arm_pitch_offset = true;
            if(this->is_uplift_up_movement_ok)
            {//抬升ok了之后 否则就等待
                this->arm_pitch.set_motor_enable();//这个要过一会发，不能一开始就发
                this->reset_step.arm_pitch = 2;
            }
            break;
        }
        case 2:
        {
            this->framework_set_arm_pitch(0.0f);
            if(this->is_framework_in_set_arm_pitch())
            {
                this->is_arm_pitch_initial = true;
                this->reset_step.arm_pitch = 3;
            }
            break;
        }
        case 3:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->framework_set_arm_pitch(communication.arm->set_joint.arm_pitch_joint);
                if(this->is_framework_in_set_arm_pitch())
                {
                    this->set_arm_pitch_dm_running_pid();
                    this->is_arm_pitch_need_reset = false;
                    this->reset_step.arm_pitch = 0;
                }
            }
            else
            {
                this->framework_set_arm_pitch(0.0f);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Framework::Arm_Yaw_Reset()
{
    switch(this->reset_step.arm_yaw)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.arm_yaw = 1;
            }
            this->arm_yaw.ctrl_data.kp = 0;
            this->arm_yaw.ctrl_data.kd = 0;
            this->arm_yaw.ctrl_data.torq = 0;
            break;
        }
        case 1:
        {
            this->set_arm_yaw_dm_reset_pid();
            if(!this->arm_yaw.is_zero_offset)
            {
                break;
            }
            //等待其它
            if(this->is_arm_pitch_initial)
            {
                this->is_arm_yaw_offset = true;
                this->arm_yaw.set_motor_enable();//这个要过一会发，不能一开始就发
                this->reset_step.arm_yaw = 2;
            }
            else
            {
                this->arm_yaw.ctrl_data.kp = 0;
                this->arm_yaw.ctrl_data.kd = 0;
                this->arm_yaw.ctrl_data.torq = 0;
            }
            break;
        }
        case 2:
        {
            this->framework_set_arm_yaw(0.0f);
            if(this->is_framework_in_set_arm_yaw())
            {
                this->is_arm_yaw_initial = true;
                this->reset_step.arm_yaw = 3;
            }
            break;
        }
        case 3:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->framework_set_arm_yaw(communication.arm->set_joint.arm_yaw_joint);
                if(this->is_framework_in_set_arm_yaw())
                {
                    this->set_arm_yaw_dm_running_pid();
                    this->is_arm_yaw_need_reset = false;
                    this->reset_step.arm_yaw = 0;
                }
            }
            else
            {
                this->framework_set_arm_yaw(0.0f);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Framework::Extend_Reset()
{
    switch(this->reset_step.extend)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.extend = 1;
            }
            this->extend_l.set_torque = 0;
            this->extend_r.set_torque = 0;
            break;
        }
        case 1:
        {
            this->set_extend_reset_pid();
            if(!this->extend_l.is_zero_offset || !this->extend_r.is_zero_offset)
            {
                break;
            }
            if(this->is_arm_pitch_initial && this->is_arm_yaw_initial)//顺序
            {
                if(this->extend_reset_to_get_offset())
                {
                    this->reset_step.extend = 2;
                }
            }
            else
            {
                this->extend_l.set_torque = 0;
                this->extend_r.set_torque = 0;
            }
            break;
        }
        case 2:
        {
            this->framework_set_extend(50.0f);
            if(this->is_framework_in_set_extend())
            {
                this->is_extend_initial = true;
                this->reset_step.extend = 3;
            }
            break;
        }
        case 3:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->framework_set_extend(communication.arm->set_joint.extend_joint);
                if(this->is_framework_in_set_extend())
                {
                    this->set_extend_running_pid();
                    this->is_extend_need_reset = false;
                    this->reset_step.extend = 0;
                }
            }
            else
            {
                this->framework_set_extend(50.0f);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Framework::Slide_Reset()
{
    switch(this->reset_step.slide)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.slide = 1;
            }
            this->slide.set_torque = 0;
            break;
        }
        case 1:
        {
            this->set_slide_reset_pid();
            if(!this->slide.is_zero_offset)
            {
                break;
            }
            if(this->is_arm_pitch_initial && this->is_arm_yaw_initial)//顺序
            {
                if(this->slide_reset_to_get_offset())
                {
                    this->reset_step.slide = 2;
                }
            }
            else
            {
                this->slide.set_torque = 0;
            }
            break;
        }
        case 2:
        {
            this->framework_set_slide(FRAME_SLIDE_MAX_MM/2);
            if(this->is_framework_in_set_slide())
            {
                this->is_slide_initial = true;
                this->reset_step.slide = 3;
            }
            break;
        }
        case 3:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->framework_set_slide(communication.arm->set_joint.slide_joint);
                if(this->is_framework_in_set_slide())
                {
                    this->set_slide_running_pid();
                    this->is_slide_need_reset = false;
                    this->reset_step.slide = 0;
                }
            }
            else
            {
                this->framework_set_slide(FRAME_SLIDE_MAX_MM/2);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Actuator::Arm_Roll_Reset()
{
    switch(this->reset_step.arm_roll)
    {
        case 0:
        {
            if(communication.arm->is_all_peripheral_connect && communication.arm->is_init)
            {
                this->reset_step.arm_roll = 1;
            }
            this->arm_roll.set_torque = 0;
            break;
        }
        case 1:
        {
            this->set_x1_reset_pid();
            if(!this->arm_roll.is_zero_offset)
            {
                break;
            }
            if(communication.arm->framework.is_arm_pitch_initial
            && communication.arm->framework.is_arm_yaw_initial
            && communication.arm->framework.is_extend_offset
            && communication.arm->framework.is_uplift_offset
            && communication.arm->framework.is_slide_offset)//顺序
            {
                if(this->x1_reset_to_get_offset())
                {
                    this->reset_step.arm_roll = 2;
                }
            }
            else
            {
                this->arm_roll.set_torque = 0;
            }
            break;
        }
        case 2:
        {
            this->actuator_set_x1(0.0f);
            if(this->is_actuator_in_set_x1())
            {
                this->is_x1_initial = true;
                this->reset_step.arm_roll = 3;
            }
            break;
        }
        case 3:
        {
            if(communication.arm->is_arm_at_initial_pos())
            {
                this->actuator_set_x1(communication.arm->set_joint.arm_roll_joint);
                if(this->is_actuator_in_set_x1())
                {
                    this->set_x1_running_pid();
                    this->is_x1_need_reset = false;
                    this->reset_step.arm_roll = 0;
                }
            }
            else
            {
                this->actuator_set_x1(0.0f);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Actuator::Y_X2_Reset()
{
    switch(this->reset_step.y_x2)
    {
        case 0:
        {
            if(g_arm.is_all_peripheral_connect && g_arm.is_init)
            {
                this->reset_step.y_x2 = 1;
            }
            this->pitch_roll_b.set_torque = 0;
            this->pitch_roll_f.set_torque = 0;
            break;
        }
        case 1:
        {
            this->set_y_x2_reset_pid();
            if(!this->pitch_roll_b.is_zero_offset || !this->pitch_roll_f.is_zero_offset)
            {
                break;
            }
            if(communication.arm->framework.is_arm_pitch_initial
            && communication.arm->framework.is_arm_yaw_initial
            && communication.arm->framework.is_extend_offset
            && communication.arm->framework.is_uplift_offset
            && communication.arm->framework.is_slide_offset)//顺序
            {
                if(this->y_x2_reset_to_get_offset())
                {
                    this->reset_step.y_x2 = 2;
                }
            }
            else
            {
                this->pitch_roll_b.set_torque = 0;
                this->pitch_roll_f.set_torque = 0;
            }
            break;
        }
        case 2:
        {
            this->actuator_set_y_x2(0.0f,0.0f);
            if(this->is_actuator_in_set_y_x2())
            {
                this->is_y_x2_initial = true;
                this->reset_step.y_x2 = 3;
            }
            break;
        }
        case 3:
        {
            if(g_arm.is_arm_at_initial_pos())
            {
                this->actuator_set_y_x2(communication.arm->set_joint.pitch_joint, communication.arm->set_joint.roll_joint);
                if(this->is_actuator_in_set_y_x2())
                {
                    this->set_y_x2_running_pid();
                    this->is_y_x2_need_reset = false;
                    this->reset_step.y_x2 = 0;
                }
            }
            else
            {
                this->actuator_set_y_x2(0.0f,0.0f);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

