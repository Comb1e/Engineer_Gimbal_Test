//
// Created by Henson on 2024-03-04.
//

#include "drv_arm.h"

#define s arm_sin_f32
#define c arm_cos_f32

Arm g_arm;

Arm::Arm():framework(Framework()),actuator(Actuator())
{

}

void slide_speed_filter() {
    //speed
    g_arm.framework.slide.motor_data.speed = g_arm.framework.slide.motor_data.last_speed*0.8f + g_arm.framework.slide.motor_data.speed*0.2f;
    VAL_LIMIT(g_arm.framework.slide.motor_data.speed,-1.0f,1.0f);
}

void Arm::Arm_init() {
    framework.framework_init();
    actuator.actuator_init();
    set_reset_pid();//必须在actuator后面 保证初始pid
    framework.slide.process_data_ptr_fun_register(slide_speed_filter);
    is_init = true;
}

void Arm::Update_Data()
{
    this->current_joint.extend_joint = (this->framework.extend_l.motor_data.total_rounds);
}

// --------------------------- 最顶层设置 --------------------------- //
void Arm::set_arm_arm_yaw_deg(float set_deg){
    VAL_LIMIT(set_deg,ARM_YAW_MIN_DEG,ARM_YAW_MAX_DEG);
    set_arm_yaw_deg = set_deg;
}

void Arm::set_arm_arm_pitch_deg(float set_deg){
    VAL_LIMIT(set_deg,ARM_PITCH_MIN_DEG,ARM_PITCH_MAX_DEG);
    set_arm_pitch_deg = set_deg;
}

void Arm::set_arm_pose(dof6_e dofE, float set_mm_deg){
    switch(dofE){
        case x:
            set.x_mm = set_mm_deg;
            VAL_LIMIT(set.x_mm,0-ARM_X_MM_OFFSET,1024-ARM_X_MM_OFFSET);
            break;
        case y:
            set.y_mm = set_mm_deg;
            VAL_LIMIT(set.y_mm,0-ARM_Y_MM_OFFSET,1024-ARM_Y_MM_OFFSET);
            break;
        case z:
            set.z_mm = set_mm_deg;
            VAL_LIMIT(set.z_mm,0-ARM_Z_MM_OFFSET,1024-ARM_Z_MM_OFFSET);
            break;
        case yaw:
            VAL_LIMIT(set_mm_deg,YAW_DEG_MIN,YAW_DEG_MAX);
            set.yaw_deg = set_mm_deg;
            break;
        case pitch:
            VAL_LIMIT(set_mm_deg,PITCH_DEG_MIN,PITCH_DEG_MAX);
            set.pitch_deg = set_mm_deg;
            break;
        case roll:
            VAL_LIMIT(set_mm_deg,ROLL_DEG_MIN,ROLL_DEG_MAX);
            set.roll_deg = set_mm_deg;
            break;
        default:
            break;
    }
}


// --------------------------- reset --------------------------- //

void Arm::set_arm_reset()
{
    is_enable = true;
    framework.set_framework_reset();//有reset pid
    actuator.set_actuator_reset();//有reset pid
    set_reset_pid();
    arm_reset_num++;
}

void Arm::set_actuator_single_reset()
{
    actuator.set_actuator_reset();
}

void Arm::set_framework_single_reset()
{
    framework.set_framework_reset();
}

bool Arm::is_arm_offset_ok() {
    if(framework.is_offset_ok() && actuator.is_offset_ok()){
        return true;
    }
    return false;
}

bool Arm::is_arm_at_initial_pos(){
    return framework.is_framework_at_initial_pos() && actuator.is_actuator_at_initial_pos();
}

bool Arm::is_arm_reset_all_ok() {
    if(g_arm.framework.reset_step.uplift==0 && g_arm.framework.reset_step.extend==0 &&
    g_arm.framework.reset_step.slide==0 && g_arm.framework.reset_step.arm_yaw==0 &&
    g_arm.framework.reset_step.arm_pitch==0 &&
    g_arm.actuator.reset_step.arm_roll==0 && g_arm.actuator.reset_step.y_x2==0 && is_enable==true)
    {
        return true;
    }
    else{
        return false;
    }
}

bool Arm::is_arm_at_x_lower_limit(){
    if(ABS(framework.get_framework_extend_mm()-(FRAME_EXTEND_MIN_MM))<5)
    {
        return true;
    }
    return false;
}

bool Arm::is_arm_at_x_higher_limit(){
    if(ABS(framework.get_framework_extend_mm()-(FRAME_EXTEND_MAX_MM))<5)
    {
        return true;
    }
    return false;
}

bool Arm::is_arm_at_y_lower_limit(){
    if(ABS(framework.get_framework_slide_mm()-FRAME_SLIDE_MIN_MM) < 2)
    {
        return true;
    }
    return false;
}

bool Arm::is_arm_at_y_higher_limit(){
    if(ABS(framework.get_framework_slide_mm()-FRAME_SLIDE_MAX_MM)<2)
    {
        return true;
    }
    return false;
}

// --------------------------- 整体控制 --------------------------- //
void Arm::set_arm_move() {
    if(g_arm.framework.is_all_motors_safe) {
        framework.set_framework_move();
    }
    if(g_arm.actuator.is_all_motors_safe) {
        actuator.set_actuator_move();
    }
}

void Arm::set_reset_pid(){
    framework.set_reset_pid();
    actuator.set_reset_pid();
}

void Arm::check_motors_overheat(){
    if(framework.extend_l.is_overheat||
        framework.extend_r.is_overheat||
        framework.uplift_l.is_overheat||
        framework.uplift_r.is_overheat||
        framework.arm_yaw.is_overheat||
        framework.arm_pitch.is_overheat||
        framework.slide.is_overheat||
        actuator.arm_roll.is_overheat||
        actuator.pitch_roll_b.is_overheat||
        actuator.pitch_roll_f.is_overheat)
    {
        is_motors_overheat = true;
    }
    else{
        is_motors_overheat = false;
    }
}


#undef s
#undef c
