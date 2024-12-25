//
// Created by Henson on 2024-01-14.
//

#include "task_ctrl.h"
#include "drv_arm.h"

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
void upliftCtrlTask(void *argument) {
    g_arm.framework.reset_step.uplift = 0;
    for(;;){
        if(g_arm.framework.is_uplift_need_reset) {
            switch(g_arm.framework.reset_step.uplift){
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.framework.reset_step.uplift = 1;
                    }
                    g_arm.framework.uplift_l.set_torque = 0;
                    g_arm.framework.uplift_r.set_torque = 0;
                case 1://向上移动一下下，给定一个移动时间就可以
                    g_arm.framework.set_uplift_reset_pid();
                    if(!g_arm.framework.uplift_l.is_zero_offset || !g_arm.framework.uplift_r.is_zero_offset){
                        break;
                    }
                    if(g_arm.framework.uplift_up_movement_to_avoid_arm_clash(600)){//pid在计算
                        g_arm.framework.reset_step.uplift = 2;
                        g_arm.framework.is_uplift_up_movement_ok = true;
                    }
                    break;
                case 2:
                    if(g_arm.framework.is_arm_pitch_initial)//顺序 等待大pitch
                    {
                        if(g_arm.framework.uplift_reset_to_get_offset()){
                            g_arm.framework.reset_step.uplift = 3;
                        }
                    }else{//pitch没复位则一直向上走  pid在计算
                        g_arm.framework.uplift_l.set_motor_normalization_torque(pid_calculate(&g_arm.framework.uplift_l.velPid, g_arm.framework.uplift_l.get_motor_speed(), 0.0f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动
                        g_arm.framework.uplift_r.set_motor_normalization_torque(pid_calculate(&g_arm.framework.uplift_r.velPid, g_arm.framework.uplift_r.get_motor_speed(), 0.0f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动
                    }
                    break;
                case 3:
                    g_arm.framework.framework_set_uplift(15);//pid 在计算
                    if(g_arm.framework.is_framework_in_set_uplift()){
                        g_arm.framework.is_uplift_initial = true;
                        g_arm.framework.reset_step.uplift = 4;
                    }
                    break;
                case 4:
                    if(g_arm.is_arm_at_initial_pos()){
                        g_arm.framework.framework_set_uplift(g_arm.kine.set.framework.uplift_mm);
                        if(g_arm.framework.is_framework_in_set_uplift()){
                            g_arm.framework.set_uplift_running_pid();
                            g_arm.framework.is_uplift_need_reset = false;
                            g_arm.framework.reset_step.uplift = 0;
                        }
                    }else{
                        g_arm.framework.framework_set_uplift(15);
                    }
                    break;
                default:
                    break;
            }
        }else if(false ==(g_arm.framework.is_uplift_offset && g_arm.framework.is_uplift_initial)){
            g_arm.framework.uplift_l.set_torque = 0;
            g_arm.framework.uplift_r.set_torque = 0;
//            VAL_LIMIT(uplift_set_torque,-1.0f,1.0f);
//            g_arm.framework.uplift_l.set_torque = (int16_t)(-uplift_set_torque*16384.0f);
//            g_arm.framework.uplift_r.set_torque = (int16_t)(uplift_set_torque*16384.0f);
        }
        else{
            g_arm.framework.framework_set_uplift(g_arm.kine.set.framework.uplift_mm);
        }
        g_arm.framework.framework_set_uplift_move();
        osDelay(3);
    }
}


//2 pitch
void armPitchCtrlTask(void *argument){
    g_arm.framework.reset_step.arm_pitch = 0;
    //先使能
    for(;;) {
        if(g_arm.framework.is_arm_pitch_need_reset) {
            switch(g_arm.framework.reset_step.arm_pitch) {
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.framework.reset_step.arm_pitch = 1;
                    }
                    g_arm.framework.arm_pitch.ctrl_data.kp = 0;
                    g_arm.framework.arm_pitch.ctrl_data.kd = 0;
                    g_arm.framework.arm_pitch.ctrl_data.torq = 0;
                case 1:
                    g_arm.framework.set_arm_pitch_dm_reset_pid();
                    if(!g_arm.framework.arm_pitch.is_zero_offset){
                        break;
                    }
                    g_arm.framework.is_arm_pitch_offset = true;
                    if(g_arm.framework.is_uplift_up_movement_ok){//抬升ok了之后 否则就等待
                        g_arm.framework.arm_pitch.set_motor_enable();//这个要过一会发，不能一开始就发
                        g_arm.framework.reset_step.arm_pitch = 2;
                    }
                    break;
                case 2:
                    g_arm.framework.framework_set_arm_pitch(0.0f);
                    if(g_arm.framework.is_framework_in_set_arm_pitch()){
                        g_arm.framework.is_arm_pitch_initial = true;
                        g_arm.framework.reset_step.arm_pitch = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.framework.framework_set_arm_pitch(g_arm.kine.set.framework.arm_pitch_deg);
                        if(g_arm.framework.is_framework_in_set_arm_pitch()){
                            g_arm.framework.set_arm_pitch_dm_running_pid();
                            g_arm.framework.is_arm_pitch_need_reset = false;
                            g_arm.framework.reset_step.arm_pitch = 0;
                        }
                    }else{
                        g_arm.framework.framework_set_arm_pitch(0.0f);
                    }
                    break;
                default:
                    break;
            }
        }
        else if(false==(g_arm.framework.is_arm_pitch_offset && g_arm.framework.is_arm_pitch_initial))
        {
            g_arm.framework.arm_pitch.ctrl_data.kp = 0;
            g_arm.framework.arm_pitch.ctrl_data.kd = 0;
            g_arm.framework.arm_pitch.ctrl_data.torq = 0;
        }
        else{
            g_arm.framework.framework_set_arm_pitch(g_arm.kine.set.framework.arm_pitch_deg);
        }
        if(g_arm.framework.arm_pitch.recover_the_motor()==false) {
            g_arm.framework.framework_set_arm_pitch_move();
        }
//        else{
//            g_arm.framework.set_arm_pitch_reset();
//            osDelay(200);
//        }
        osDelay(3);
    }
}

//3 yaw
void armYawCtrlTask(void *argument){
    g_arm.framework.reset_step.arm_yaw = 0;
    //先使能
    for(;;){
        if(g_arm.framework.is_arm_yaw_need_reset) {
            switch(g_arm.framework.reset_step.arm_yaw) {
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.framework.reset_step.arm_yaw = 1;
                    }
                    g_arm.framework.arm_yaw.ctrl_data.kp = 0;
                    g_arm.framework.arm_yaw.ctrl_data.kd = 0;
                    g_arm.framework.arm_yaw.ctrl_data.torq = 0;
                case 1:
                    g_arm.framework.set_arm_yaw_dm_reset_pid();
                    if(!g_arm.framework.arm_yaw.is_zero_offset){
                        break;
                    }
                    //等待其它
                    if(g_arm.framework.is_arm_pitch_initial){
                        g_arm.framework.is_arm_yaw_offset = true;
                        g_arm.framework.arm_yaw.set_motor_enable();//这个要过一会发，不能一开始就发
                        g_arm.framework.reset_step.arm_yaw = 2;
                    }else{
                        g_arm.framework.arm_yaw.ctrl_data.kp = 0;
                        g_arm.framework.arm_yaw.ctrl_data.kd = 0;
                        g_arm.framework.arm_yaw.ctrl_data.torq = 0;
                    }
                    break;
                case 2:
                    g_arm.framework.framework_set_arm_yaw(0.0f);
                    if(g_arm.framework.is_framework_in_set_arm_yaw()){
                        g_arm.framework.is_arm_yaw_initial = true;
                        g_arm.framework.reset_step.arm_yaw = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.framework.framework_set_arm_yaw(g_arm.kine.set.framework.arm_yaw_deg);
                        if(g_arm.framework.is_framework_in_set_arm_yaw()){
                            g_arm.framework.set_arm_yaw_dm_running_pid();
                            g_arm.framework.is_arm_yaw_need_reset = false;
                            g_arm.framework.reset_step.arm_yaw = 0;
                        }
                    }else{
                        g_arm.framework.framework_set_arm_yaw(0.0f);
                    }
                    break;
                default:
                    break;
            }
        }
        else if(false==(g_arm.framework.is_arm_yaw_offset && g_arm.framework.is_arm_yaw_initial))
        {
            g_arm.framework.arm_yaw.ctrl_data.kp = 0;
            g_arm.framework.arm_yaw.ctrl_data.kd = 0;
            g_arm.framework.arm_yaw.ctrl_data.torq = 0;
        }
        else{
            g_arm.framework.framework_set_arm_yaw(g_arm.kine.set.framework.arm_yaw_deg);
        }
        if(g_arm.framework.arm_yaw.recover_the_motor()==false) {
            g_arm.framework.framework_set_arm_yaw_move();
        }
//        else{
//            g_arm.framework.set_arm_yaw_reset();
//            osDelay(200);
//        }
        osDelay(3);
    }
}

//4 x
void extendCtrlTask(void *argument) {
    g_arm.framework.reset_step.extend = 0;
    for(;;){
        if(g_arm.framework.is_extend_need_reset) {
            switch(g_arm.framework.reset_step.extend){
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.framework.reset_step.extend = 1;
                    }
                    g_arm.framework.extend_l.set_torque = 0;
                    g_arm.framework.extend_r.set_torque = 0;
                    break;
                case 1:
                    g_arm.framework.set_extend_reset_pid();
                    if(!g_arm.framework.extend_l.is_zero_offset || !g_arm.framework.extend_r.is_zero_offset){
                        break;
                    }
                    if(g_arm.framework.is_arm_pitch_initial
                       && g_arm.framework.is_arm_yaw_initial)//顺序
                    {
                        if(g_arm.framework.extend_reset_to_get_offset()){
                            g_arm.framework.reset_step.extend = 2;
                        }
                    }
                    else{
                        g_arm.framework.extend_l.set_torque = 0;
                        g_arm.framework.extend_r.set_torque = 0;
                    }
                    break;
                case 2:
                    g_arm.framework.framework_set_extend(50.0f);
                    if(g_arm.framework.is_framework_in_set_extend()){
                        g_arm.framework.is_extend_initial = true;
                        g_arm.framework.reset_step.extend = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.framework.framework_set_extend(g_arm.kine.set.framework.extend_mm);
                        if(g_arm.framework.is_framework_in_set_extend()){
                            g_arm.framework.set_extend_running_pid();
                            g_arm.framework.is_extend_need_reset = false;
                            g_arm.framework.reset_step.extend = 0;
                        }
                    }else{
                        g_arm.framework.framework_set_extend(50.0f);
                    }
                    break;
                default:
                    break;
            }
        }else if(false ==(g_arm.framework.is_extend_offset && g_arm.framework.is_extend_initial)) {
            g_arm.framework.extend_l.set_torque = 0;
            g_arm.framework.extend_r.set_torque = 0;
        }
        else{
            g_arm.framework.framework_set_extend(g_arm.kine.set.framework.extend_mm);
        }
        g_arm.framework.extend_motors_protection();
        g_arm.framework.framework_set_extend_move();
        osDelay(3);
    }
}

//4 y
void slideCtrlTask(void *argument){
    g_arm.framework.reset_step.slide = 0;
    for(;;){
        if(g_arm.framework.is_slide_need_reset) {
            switch(g_arm.framework.reset_step.slide) {
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.framework.reset_step.slide = 1;
                    }
                    g_arm.framework.slide.set_torque = 0;
                    break;
                case 1:
                    g_arm.framework.set_slide_reset_pid();
                    if(!g_arm.framework.slide.is_zero_offset){
                        break;
                    }
                    if(g_arm.framework.is_arm_pitch_initial
                        && g_arm.framework.is_arm_yaw_initial)//顺序
                    {
                        if(g_arm.framework.slide_reset_to_get_offset()){
                            g_arm.framework.reset_step.slide = 2;
                        }
                    }else{
                        g_arm.framework.slide.set_torque = 0;
                    }
                    break;
                case 2:
                    g_arm.framework.framework_set_slide(FRAME_SLIDE_MAX_MM/2);
                    if(g_arm.framework.is_framework_in_set_slide()){
                        g_arm.framework.is_slide_initial = true;
                        g_arm.framework.reset_step.slide = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.framework.framework_set_slide(g_arm.kine.set.framework.slide_mm);
                        if(g_arm.framework.is_framework_in_set_slide()){
                            g_arm.framework.set_slide_running_pid();
                            g_arm.framework.is_slide_need_reset = false;
                            g_arm.framework.reset_step.slide = 0;
                        }
                    }else{
                        g_arm.framework.framework_set_slide(FRAME_SLIDE_MAX_MM/2);
                    }
                    break;
                default:
                    break;
            }
        }else if(false==(g_arm.framework.is_slide_offset && g_arm.framework.is_slide_initial)){
            g_arm.framework.slide.set_torque = 0;
        }
        else{
            g_arm.framework.framework_set_slide(g_arm.kine.set.framework.slide_mm);
        }
        g_arm.framework.framework_set_slide_move();
        osDelay(3);
    }
}


//5 x1
void armRollCtrlTask(void *argument){
    g_arm.actuator.reset_step.arm_roll = 0;
    for(;;){
        if(g_arm.actuator.is_x1_need_reset) {
            switch(g_arm.actuator.reset_step.arm_roll){
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.actuator.reset_step.arm_roll = 1;
                    }
                    g_arm.actuator.arm_roll.set_torque = 0;
                    break;
                case 1:
                    g_arm.actuator.set_x1_reset_pid();
                    if(!g_arm.actuator.arm_roll.is_zero_offset){
                        break;
                    }
                    if(g_arm.framework.is_arm_pitch_initial
                       && g_arm.framework.is_arm_yaw_initial
                       && g_arm.framework.is_extend_offset
                       && g_arm.framework.is_uplift_offset
                       && g_arm.framework.is_slide_offset)//顺序
                    {
                        if(g_arm.actuator.x1_reset_to_get_offset()){
                            g_arm.actuator.reset_step.arm_roll = 2;
                        }
                    }else{
                        g_arm.actuator.arm_roll.set_torque = 0;
                    }
                    break;
                case 2:
                    g_arm.actuator.actuator_set_x1(0.0f);
                    if(g_arm.actuator.is_actuator_in_set_x1()){
                        g_arm.actuator.is_x1_initial = true;
                        g_arm.actuator.reset_step.arm_roll = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.actuator.actuator_set_x1(g_arm.kine.set.act.euler_x1_deg);
                        if(g_arm.actuator.is_actuator_in_set_x1()){
                            g_arm.actuator.set_x1_running_pid();
                            g_arm.actuator.is_x1_need_reset = false;
                            g_arm.actuator.reset_step.arm_roll = 0;
                        }
                    }else{
                        g_arm.actuator.actuator_set_x1(0.0f);
                    }
                    break;
                default:
                    break;
            }
        }else if(false==(g_arm.actuator.is_x1_offset && g_arm.actuator.is_x1_initial)){
            g_arm.actuator.arm_roll.set_torque = 0;
        }
        else{
            g_arm.actuator.actuator_set_x1(g_arm.kine.set.act.euler_x1_deg);
        }
        g_arm.actuator.actuator_set_x1_move();
        osDelay(3);
    }
}


//5 yx2
void yX2CtrlTask(void *argument){
    g_arm.actuator.reset_step.y_x2 = 0;
    for(;;){
        if(g_arm.actuator.is_y_x2_need_reset) {
            switch(g_arm.actuator.reset_step.y_x2){
                case 0:
                    if(g_arm.is_all_peripheral_connect && g_arm.is_init){
                        g_arm.actuator.reset_step.y_x2 = 1;
                    }
                    g_arm.actuator.pitch_roll_b.set_torque = 0;
                    g_arm.actuator.pitch_roll_f.set_torque = 0;
                    break;
                case 1:
                    g_arm.actuator.set_y_x2_reset_pid();
                    if(!g_arm.actuator.pitch_roll_b.is_zero_offset || !g_arm.actuator.pitch_roll_f.is_zero_offset){
                        break;
                    }
                    if(g_arm.framework.is_arm_pitch_initial
                       && g_arm.framework.is_arm_yaw_initial
                       && g_arm.framework.is_extend_offset
                       && g_arm.framework.is_uplift_offset
                       && g_arm.framework.is_slide_offset)//顺序
                    {
                        if(g_arm.actuator.y_x2_reset_to_get_offset()){
                            g_arm.actuator.reset_step.y_x2 = 2;
                        }
                    }else{
                        g_arm.actuator.pitch_roll_b.set_torque = 0;
                        g_arm.actuator.pitch_roll_f.set_torque = 0;
                    }
                    break;
                case 2:
                    g_arm.actuator.actuator_set_y_x2(0.0f,0.0f);
                    if(g_arm.actuator.is_actuator_in_set_y_x2()){
                        g_arm.actuator.is_y_x2_initial = true;
                        g_arm.actuator.reset_step.y_x2 = 3;
                    }
                    break;
                case 3:
                    if(g_arm.is_arm_at_initial_pos()) {
                        g_arm.actuator.actuator_set_y_x2(g_arm.kine.set.act.euler_y_deg, g_arm.kine.set.act.euler_x2_deg);
                        if(g_arm.actuator.is_actuator_in_set_y_x2()){
                            g_arm.actuator.set_y_x2_running_pid();
                            g_arm.actuator.is_y_x2_need_reset = false;
                            g_arm.actuator.reset_step.y_x2 = 0;
                        }
                    }else{
                        g_arm.actuator.actuator_set_y_x2(0.0f,0.0f);
                    }
                    break;
                default:
                    break;
            }
        }else if(false==(g_arm.actuator.is_y_x2_offset && g_arm.actuator.is_y_x2_initial)){
            g_arm.actuator.pitch_roll_b.set_torque = 0;
            g_arm.actuator.pitch_roll_f.set_torque = 0;
        }
        else{
            g_arm.actuator.actuator_set_y_x2(g_arm.kine.set.act.euler_y_deg, g_arm.kine.set.act.euler_x2_deg);
        }
        g_arm.actuator.actuator_set_y_x2_move();
        osDelay(3);
    }
}

