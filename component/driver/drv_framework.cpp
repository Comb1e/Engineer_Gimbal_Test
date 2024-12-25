//
// Created by Henson on 2024-01-16.
//

#include "drv_framework.h"
#include "GlobalCfg.h"
/*  俯视图
 *
 *                  ^X
 *                  |
 *                  |
 *                  |
 *                  |
 * Y                |
 * <----------------+Z
 *
 */

Framework::Framework():
        uplift_l(DJI_Motor_3508()), uplift_r(DJI_Motor_3508()), extend_l(DJI_Motor_3508()), extend_r(DJI_Motor_3508()),
        slide(DJI_Motor_2006()), arm_yaw(DM_Motor_J4310()), arm_pitch(DM_Motor_J4310())
{

}

void Framework::framework_init() {
    //can2
    uplift_l.DJI_Motor_3508_init(&UPLIFT_L_CAN, 1, true, upliftLeftBinarySemHandle);
    uplift_r.DJI_Motor_3508_init(&UPLIFT_R_CAN, 2, false, upliftRightBinarySemHandle);
    extend_l.DJI_Motor_3508_init(&EXTEND_L_CAN, 3, true, extendLeftBinarySemHandle);
    extend_r.DJI_Motor_3508_init(&EXTEND_R_CAN, 4, false, extendRightBinarySemHandle);
    slide.DJI_Motor_2006_init(&SLIDE_CAN, 5, false, slideBinarySemHandle);
    //can1
    arm_yaw.DM_Motor_J4310_init(&ARM_YAW_CAN, 0x01, 0x02, DM_Motor::DM_MIT, false, armYawDMBinarySemHandle);
    arm_pitch.DM_Motor_J4310_init(&ARM_PITCH_CAN, 0x03, 0x04, DM_Motor::DM_MIT, false, armPitchDMBinarySemHandle);
}


// ------------------------ loop reset to get offset ------------------------ //
/*  logic
    reset z -> x -> y   direction?
    back to initial y -> x/z
    之前是 z ->x/y
*/
void Framework::set_uplift_reset_pid() {
    //uplift
    pid_struct_init(&uplift_l.velPid, 0.8f, 0.0f, 8.4f, 0, 0);
    pid_struct_init(&uplift_l.posPid, 0.12f, 0.1f, 0.16f, 0.0045f, 0.13f);
    uplift_l.velPid.f_pid_clear_mem(&uplift_l.velPid);
    uplift_l.posPid.f_pid_clear_mem(&uplift_l.posPid);

    pid_struct_init(&uplift_r.velPid, 0.8f, 0.0f, 8.4f, 0, 0);
    pid_struct_init(&uplift_r.posPid, 0.12f, 0.1f, 0.16f, 0.0045f, 0.13f);
    uplift_r.velPid.f_pid_clear_mem(&uplift_r.velPid);
    uplift_r.posPid.f_pid_clear_mem(&uplift_r.posPid);

    //synpid
    pid_struct_init(&uplift_syn_master_pid,0.1f,0,0.05f,0,0);
    pid_struct_init(&uplift_syn_follower_pid,0.1f,0,0.05f,0,0);
    uplift_syn_master_pid.f_pid_clear_mem(&uplift_syn_master_pid);
    uplift_syn_follower_pid.f_pid_clear_mem(&uplift_syn_follower_pid);

    //not use
    pid_struct_init(&uplift_syn_master_torque_pid,0.007f,0.0f,0.001f,0.0f,0);
    pid_struct_init(&uplift_syn_follower_torque_pid,0.007f,0.0f,0.001f,0.0f,0);
    uplift_syn_master_torque_pid.f_pid_clear_mem(&uplift_syn_master_torque_pid);
    uplift_syn_follower_torque_pid.f_pid_clear_mem(&uplift_syn_follower_torque_pid);
}

void Framework::set_uplift_running_pid() {
    //uplift
    pid_struct_init(&uplift_l.velPid, 0.8f, 0.0f, 8.4f, 0, 0);
    pid_struct_init(&uplift_l.posPid, 0.4f, 0.1f, 0.12f, 0.0045f, 0.13f);
    uplift_l.velPid.f_pid_clear_mem(&uplift_l.velPid);
    uplift_l.posPid.f_pid_clear_mem(&uplift_l.posPid);

    pid_struct_init(&uplift_r.velPid, 0.8f, 0.0f, 8.4f, 0, 0);
    pid_struct_init(&uplift_r.posPid, 0.4f, 0.1f, 0.12f, 0.0045f, 0.13f);
    uplift_r.velPid.f_pid_clear_mem(&uplift_r.velPid);
    uplift_r.posPid.f_pid_clear_mem(&uplift_r.posPid);

    //synpid
    pid_struct_init(&uplift_syn_master_pid,0.12f,0,0.04f,0,0);
    pid_struct_init(&uplift_syn_follower_pid,0.12f,0,0.04f,0,0);
    uplift_syn_master_pid.f_pid_clear_mem(&uplift_syn_master_pid);
    uplift_syn_follower_pid.f_pid_clear_mem(&uplift_syn_follower_pid);

    //not use
    pid_struct_init(&uplift_syn_master_torque_pid,0.007f,0.0f,0.001f,0.0f,0);
    pid_struct_init(&uplift_syn_follower_torque_pid,0.007f,0.0f,0.001f,0.0f,0);
    uplift_syn_master_torque_pid.f_pid_clear_mem(&uplift_syn_master_torque_pid);
    uplift_syn_follower_torque_pid.f_pid_clear_mem(&uplift_syn_follower_torque_pid);
}

void Framework::set_extend_reset_pid() {
    //extend √
    pid_struct_init(&extend_l.velPid,  0.8f, 0, 8.5, 0, 1.5);
    pid_struct_init(&extend_l.posPid,  0.12f, 0.04, 0.11f, 0.003, 0.1);// p 0.2
    extend_l.velPid.f_pid_clear_mem(&extend_l.velPid);
    extend_l.posPid.f_pid_clear_mem(&extend_l.posPid);

    pid_struct_init(&extend_r.velPid, 0.8f, 0, 8.5, 0, 1.5);
    pid_struct_init(&extend_r.posPid, 0.12f, 0.04, 0.11f, 0.003, 0.1);// p 0.2
    extend_r.velPid.f_pid_clear_mem(&extend_r.velPid);
    extend_r.posPid.f_pid_clear_mem(&extend_r.posPid);

    //synpid
    pid_struct_init(&extend_syn_master_pid,0.08f,0,0.08f,0,0);
    pid_struct_init(&extend_syn_follower_pid,0.08f,0,0.08f,0,0);
    extend_syn_master_pid.f_pid_clear_mem(&extend_syn_master_pid);
    extend_syn_follower_pid.f_pid_clear_mem(&extend_syn_follower_pid);
}

void Framework::set_extend_running_pid() {
    //extend
    pid_struct_init(&extend_l.velPid,  0.8f, 0, 8.5, 0, 1.5);
    pid_struct_init(&extend_l.posPid,  0.5f, 0.04, 0.11f, 0.003, 0.1);// p 0.2
    extend_l.velPid.f_pid_clear_mem(&extend_l.velPid);
    extend_l.posPid.f_pid_clear_mem(&extend_l.posPid);

    pid_struct_init(&extend_r.velPid, 0.8f, 0, 8.5, 0, 1.5);
    pid_struct_init(&extend_r.posPid, 0.5f, 0.04, 0.11f, 0.003, 0.1);// p 0.2
    extend_r.velPid.f_pid_clear_mem(&extend_r.velPid);
    extend_r.posPid.f_pid_clear_mem(&extend_r.posPid);

    //synpid
    pid_struct_init(&extend_syn_master_pid,0.08f,0,0.08f,0,0);
    pid_struct_init(&extend_syn_follower_pid,0.08f,0,0.08f,0,0);
    extend_syn_master_pid.f_pid_clear_mem(&extend_syn_master_pid);
    extend_syn_follower_pid.f_pid_clear_mem(&extend_syn_follower_pid);
}

void Framework::set_slide_reset_pid() {
    //slide √
    pid_struct_init(&slide.velPid,  0.8f, 0.0f, 12.0f, 0.0f, 1.0f);
    pid_struct_init(&slide.posPid,  0.15f, 0, 0.1, 0, 0.2);//转的太快会有问题，读不到
    slide.velPid.f_pid_clear_mem(&slide.velPid);
    slide.posPid.f_pid_clear_mem(&slide.posPid);
}

void Framework::set_slide_running_pid() {
    //slide
    pid_struct_init(&slide.velPid,  0.8f, 0.0f, 12.0f, 0.0f, 1.0f);
    pid_struct_init(&slide.posPid,  0.4f, 0, 0.1, 0, 0.2);//转的太快会有问题，读不到
    slide.velPid.f_pid_clear_mem(&slide.velPid);
    slide.posPid.f_pid_clear_mem(&slide.posPid);
}

void Framework::set_arm_yaw_dm_reset_pid() {
    //arm_yaw
    pid_struct_init(&arm_yaw.velPid, 0.45f, 0.00f, 6.2f, 0.0f, 1.0f);
    pid_struct_init(&arm_yaw.posPid, 0.08f, 0, 3.1f, 0, 1.0f);
    arm_yaw.velPid.f_pid_clear_mem(&arm_yaw.velPid);
    arm_yaw.posPid.f_pid_clear_mem(&arm_yaw.posPid);
}

void Framework::set_arm_yaw_dm_running_pid() {
    //arm_yaw
    pid_struct_init(&arm_yaw.velPid, 0.8f, 0.00f, 8.0f, 0.0f, 1.0f);
    pid_struct_init(&arm_yaw.posPid, 0.35f, 0, 3.0f, 0, 1.0f);
    arm_yaw.velPid.f_pid_clear_mem(&arm_yaw.velPid);
    arm_yaw.posPid.f_pid_clear_mem(&arm_yaw.posPid);
}


void Framework::set_arm_pitch_dm_reset_pid() {
    //arm_pitch
    pid_struct_init(&arm_pitch.velPid, 0.35, 0, 6.5, 0, 1);
    pid_struct_init(&arm_pitch.posPid, 0.1, 0.04, 4.5, 0.006, 1.2);
    arm_pitch.velPid.f_pid_clear_mem(&arm_pitch.velPid);
    arm_pitch.posPid.f_pid_clear_mem(&arm_pitch.posPid);
}

void Framework::set_arm_pitch_dm_running_pid() {
    //arm_pitch
    pid_struct_init(&arm_pitch.velPid, 0.6, 0, 6.5, 0, 1);
    pid_struct_init(&arm_pitch.posPid, 0.35, 0.04, 4.5, 0.006, 1.2);
    arm_pitch.velPid.f_pid_clear_mem(&arm_pitch.velPid);
    arm_pitch.posPid.f_pid_clear_mem(&arm_pitch.posPid);
}

void Framework::set_reset_pid(){
    ///framework
    set_uplift_reset_pid();
    set_extend_reset_pid();
    set_slide_reset_pid();
    set_arm_yaw_dm_reset_pid();
    set_arm_pitch_dm_reset_pid();
}

void Framework::set_running_pid(){
    ///framework
    set_uplift_running_pid();
    set_extend_running_pid();
    set_slide_running_pid();
    set_arm_yaw_dm_running_pid();
    set_arm_pitch_dm_running_pid();
}

void Framework::set_uplift_reset(){
    reset_step.uplift = 0;
    is_uplift_need_reset = true;
    is_uplift_offset = false;
    is_uplift_initial = false;
    is_uplift_up_movement_ok = false;
}
void Framework::set_extend_reset(){
    reset_step.extend = 0;
    is_extend_need_reset = true;
    is_extend_offset = false;
    is_extend_initial = false;
}
void Framework::set_slide_reset(){
    reset_step.slide = 0;
    is_slide_need_reset = true;
    is_slide_offset = false;
    is_slide_initial = false;
}
void Framework::set_arm_yaw_reset(){
    reset_step.arm_yaw = 0;
    is_arm_yaw_need_reset = true;
    is_arm_yaw_offset = false;
    is_arm_yaw_initial = false;
}

void Framework::set_arm_pitch_reset(){
    reset_step.arm_pitch = 0;
    is_arm_pitch_need_reset = true;
    is_arm_pitch_offset = false;
    is_arm_pitch_initial = false;
}

bool Framework::is_uplift_at_initial_pos(){
    return is_uplift_initial;
}
bool Framework::is_extend_at_initial_pos(){
    return is_extend_initial;
}
bool Framework::is_slide_at_initial_pos(){
    return is_slide_initial;
}
bool Framework::is_arm_yaw_running(){
    return is_arm_yaw_initial;
}

bool Framework::is_arm_pitch_running(){
    return is_arm_pitch_initial;
}

bool Framework::is_framework_at_initial_pos(){
    return is_uplift_at_initial_pos() && is_extend_at_initial_pos() && is_slide_at_initial_pos() &&
            is_arm_pitch_running() && is_arm_yaw_running();
}


// ----------------------------------- reset to get offset ----------------------------------- //
bool Framework::extend_reset_to_get_offset() {
    if(is_extend_offset){
        return true;
    }
    //1. set speed
    extend_l.set_motor_speed(-0.12f);
    extend_l.set_motor_stall_parameter(7000,0.03f);

    extend_r.set_motor_speed(-0.12f);//向后复位
    extend_r.set_motor_stall_parameter(7000,0.03f);

    //2. while (check current)
    if(extend_l.stall_cnt<20 && extend_r.stall_cnt<20){
        return false;//3. exit
    }
    else{
        extend_l.set_motor_total_rounds_offset(FRAME_EXTEND_L_MIN_ROUNDS+FRAME_EXTEND_L_MIN_ROUNDS_OFFSET);
        extend_l.velPid.f_pid_clear_mem(&extend_l.velPid);
        extend_l.posPid.f_pid_clear_mem(&extend_l.posPid);

        extend_r.set_motor_total_rounds_offset(FRAME_EXTEND_R_MIN_ROUNDS+FRAME_EXTEND_R_MIN_ROUNDS_OFFSET);
        extend_r.velPid.f_pid_clear_mem(&extend_r.velPid);
        extend_r.posPid.f_pid_clear_mem(&extend_r.posPid);
        is_extend_offset = true;
        return true;//3. exit
    }
}

bool Framework::slide_reset_to_get_offset() {
    if(is_slide_offset){
        return true;
    }
    //1. set speed
    slide.set_motor_speed(0.1f);//向左复位
    slide.set_motor_stall_parameter(4000,0.03f);

    //2. while (check current)
    if(slide.stall_cnt<20){
        return false;//3. exit
    }
    else{
        slide.set_motor_total_rounds_offset(FRAME_SLIDE_MAX_ROUNDS+FRAME_SLIDE_MAX_ROUNDS_OFFSET);
        slide.velPid.f_pid_clear_mem(&slide.velPid);
        slide.posPid.f_pid_clear_mem(&slide.posPid);
        is_slide_offset = true;
        return true;//3. exit
    }
}

//检查这个函数即可
bool Framework::uplift_up_movement_to_avoid_arm_clash(uint32_t up_movement_ms) {
    static bool is_first_in_new_movement = true;
    static uint32_t time_ms_at_first_in_new_movement = 0;

    uplift_l.set_motor_normalization_torque(pid_calculate(&uplift_l.velPid, uplift_l.get_motor_speed(), 0.09f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动
    uplift_r.set_motor_normalization_torque(pid_calculate(&uplift_r.velPid, uplift_r.get_motor_speed(), 0.09f) + FRAME_UPLIFT_OFFSET_TORQUE);//向上移动

    if(is_first_in_new_movement){// 结束了之后为true
        time_ms_at_first_in_new_movement = HAL_GetTick();
        is_first_in_new_movement = false;
    }

    if(HAL_GetTick()>(time_ms_at_first_in_new_movement+up_movement_ms)){
        is_first_in_new_movement = true;
        return true;
    }else{
        return false;
    }
}


bool Framework::uplift_reset_to_get_offset() {
    if(is_uplift_offset) {
        return true;
    }
    //1. set speed
    //todo 注意这里初始向下偏置给大0.3 pid算出来就大小不行,即算出来的值的绝对值都不到 0.3*16384
    uplift_l.set_motor_normalization_torque(pid_calculate(&uplift_l.velPid, uplift_l.get_motor_speed(), -0.12f) + FRAME_UPLIFT_OFFSET_TORQUE);//向下复位
    uplift_l.set_motor_stall_parameter(7000,0.03f);

    uplift_r.set_motor_normalization_torque(pid_calculate(&uplift_r.velPid, uplift_r.get_motor_speed(), -0.12f) + FRAME_UPLIFT_OFFSET_TORQUE);//向下复位
    uplift_r.set_motor_stall_parameter(7000,0.03f);

    //2. while (check current)
    if(uplift_l.stall_cnt<30 && uplift_r.stall_cnt<30){
        return false;//3. exit
    }
    else{
        uplift_l.set_motor_total_rounds_offset(FRAME_UPLIFT_L_MIN_ROUNDS+FRAME_UPLIFT_L_MIN_ROUNDS_OFFSET);
        uplift_l.velPid.f_pid_clear_mem(&uplift_l.velPid);
        uplift_l.posPid.f_pid_clear_mem(&uplift_l.posPid);

        uplift_r.set_motor_total_rounds_offset(FRAME_UPLIFT_R_MIN_ROUNDS+FRAME_UPLIFT_R_MIN_ROUNDS_OFFSET);
        uplift_r.velPid.f_pid_clear_mem(&uplift_r.velPid);
        uplift_r.posPid.f_pid_clear_mem(&uplift_r.posPid);

        is_uplift_offset = true;
        return true;//3. exit
    }
}

// ------------------------------------------ FK ------------------------------------------ //
float Framework::get_framework_extend_mm(){
    float extend_l_mm = 0.0f, extend_r_mm = 0.0f;
    extend_l_mm = FRAME_EXTEND_MIN_MM +
            (extend_l.get_motor_total_rounds()-FRAME_EXTEND_L_MIN_ROUNDS)/(FRAME_EXTEND_L_MAX_ROUNDS-FRAME_EXTEND_L_MIN_ROUNDS)*(FRAME_EXTEND_MAX_MM-FRAME_EXTEND_MIN_MM);
    extend_r_mm = FRAME_EXTEND_MIN_MM +
            (extend_r.get_motor_total_rounds()-FRAME_EXTEND_R_MIN_ROUNDS)/(FRAME_EXTEND_R_MAX_ROUNDS-FRAME_EXTEND_R_MIN_ROUNDS)*(FRAME_EXTEND_MAX_MM-FRAME_EXTEND_MIN_MM);
    return 0.5f*(extend_l_mm+extend_r_mm);
}

float Framework::get_framework_slide_mm(){
    float slide_mm = 0.0f;
    slide_mm = FRAME_SLIDE_MIN_MM +
                (slide.get_motor_total_rounds()-FRAME_SLIDE_MIN_ROUNDS)/(FRAME_SLIDE_MAX_ROUNDS-FRAME_SLIDE_MIN_ROUNDS)*(FRAME_SLIDE_MAX_MM - FRAME_SLIDE_MIN_MM);
    return slide_mm;
}

float Framework::get_framework_uplift_mm(){
    float uplift_l_mm = 0.0f, uplift_r_mm = 0.0f;
    uplift_l_mm = FRAME_UPLIFT_MIN_MM +
                (uplift_l.get_motor_total_rounds()-FRAME_UPLIFT_L_MIN_ROUNDS)/(FRAME_UPLIFT_L_MAX_ROUNDS-FRAME_UPLIFT_L_MIN_ROUNDS)*(FRAME_UPLIFT_MAX_MM - FRAME_UPLIFT_MIN_MM);
    uplift_r_mm = FRAME_UPLIFT_MIN_MM +
                (uplift_r.get_motor_total_rounds()-FRAME_UPLIFT_R_MIN_ROUNDS)/(FRAME_UPLIFT_R_MAX_ROUNDS-FRAME_UPLIFT_R_MIN_ROUNDS)*(FRAME_UPLIFT_MAX_MM - FRAME_UPLIFT_MIN_MM);
    return 0.5f*(uplift_l_mm + uplift_r_mm);
}

float Framework::get_framework_arm_yaw_deg(){
    float arm_yaw_deg = 0.0f;
    arm_yaw_deg = ARM_YAW_MIN_DEG + ((arm_yaw.get_motor_total_rounds()-ARM_YAW_ZERO_OFFSET_ROUNDS) - ARM_YAW_MIN_ROUNDS) / (ARM_YAW_MAX_ROUNDS - ARM_YAW_MIN_ROUNDS) * (ARM_YAW_MAX_DEG - ARM_YAW_MIN_DEG);
    return arm_yaw_deg;
}

float Framework::get_framework_arm_pitch_deg(){
    float arm_pitch_deg = 0.0f;
    arm_pitch_deg = ARM_PITCH_MIN_DEG + ((arm_pitch.get_motor_total_rounds()-ARM_PITCH_ZERO_OFFSET_ROUNDS) - ARM_PITCH_MIN_ROUNDS) / (ARM_PITCH_MAX_ROUNDS - ARM_PITCH_MIN_ROUNDS) * (ARM_PITCH_MAX_DEG - ARM_PITCH_MIN_DEG);
    return arm_pitch_deg;
}

// ------------------------------------- IK ------------------------------------- //
void Framework::framework_set_extend(float extend_mm) {
    float speed_l_syn_master=0,speed_r_syn_follower=0,speed_l_set=0,speed_r_set=0;
    VAL_LIMIT(extend_mm, FRAME_EXTEND_MIN_MM, FRAME_EXTEND_MAX_MM);
    set_rounds.extend_l = FRAME_EXTEND_L_MIN_ROUNDS + (extend_mm - FRAME_EXTEND_MIN_MM) / (FRAME_EXTEND_MAX_MM - FRAME_EXTEND_MIN_MM) * (FRAME_EXTEND_L_MAX_ROUNDS - FRAME_EXTEND_L_MIN_ROUNDS);
    set_rounds.extend_r = FRAME_EXTEND_R_MIN_ROUNDS + (extend_mm - FRAME_EXTEND_MIN_MM) / (FRAME_EXTEND_MAX_MM - FRAME_EXTEND_MIN_MM) * (FRAME_EXTEND_R_MAX_ROUNDS - FRAME_EXTEND_R_MIN_ROUNDS);
    VAL_LIMIT(set_rounds.extend_l,FRAME_EXTEND_L_MIN_ROUNDS,FRAME_EXTEND_L_MAX_ROUNDS);
    VAL_LIMIT(set_rounds.extend_r,FRAME_EXTEND_R_MIN_ROUNDS,FRAME_EXTEND_R_MAX_ROUNDS);

//    speed_l_syn_master = pid_calculate(&extend_syn_master_pid, extend_l.get_motor_total_rounds(), extend_r.get_motor_total_rounds());
//    speed_r_syn_follower = pid_calculate(&extend_syn_follower_pid, extend_r.get_motor_total_rounds(), extend_l.get_motor_total_rounds());

    speed_l_set = pid_calculate(&extend_l.posPid, extend_l.get_motor_total_rounds(), set_rounds.extend_l);
    speed_r_set = pid_calculate(&extend_r.posPid, extend_r.get_motor_total_rounds(), set_rounds.extend_r);

    extend_l.set_motor_speed(speed_l_syn_master+speed_l_set);
    extend_r.set_motor_speed(speed_r_syn_follower+speed_r_set);
}

void Framework::framework_set_slide(float slide_mm) {
    VAL_LIMIT(slide_mm, FRAME_SLIDE_MIN_MM, FRAME_SLIDE_MAX_MM);
    set_rounds.slide = FRAME_SLIDE_MIN_ROUNDS + (slide_mm - FRAME_SLIDE_MIN_MM) / (FRAME_SLIDE_MAX_MM - FRAME_SLIDE_MIN_MM) * (FRAME_SLIDE_MAX_ROUNDS - FRAME_SLIDE_MIN_ROUNDS);
    VAL_LIMIT(set_rounds.slide,FRAME_SLIDE_MIN_ROUNDS,FRAME_SLIDE_MAX_ROUNDS);
    slide.set_motor_rounds(set_rounds.slide);
}

void Framework::framework_set_uplift(float uplift_mm) {
    static uint32_t l_ok_t = 0,r_ok_t = 0;
    float speed_set = 0.0f;
    float speed_syn = 0.0f;
    float speed_syn_master_torque = 0.0f;
    float speed_syn_follower_torque = 0.0f;
    float uplift_l_set_torque = 0,uplift_r_set_torque=0;
    VAL_LIMIT(uplift_mm, FRAME_UPLIFT_MIN_MM, FRAME_UPLIFT_MAX_MM);
    set_rounds.uplift_l = FRAME_UPLIFT_L_MIN_ROUNDS + (uplift_mm - FRAME_UPLIFT_MIN_MM) / (FRAME_UPLIFT_MAX_MM - FRAME_UPLIFT_MIN_MM) * (FRAME_UPLIFT_L_MAX_ROUNDS - FRAME_UPLIFT_L_MIN_ROUNDS);
    set_rounds.uplift_r = FRAME_UPLIFT_R_MIN_ROUNDS + (uplift_mm - FRAME_UPLIFT_MIN_MM) / (FRAME_UPLIFT_MAX_MM - FRAME_UPLIFT_MIN_MM) * (FRAME_UPLIFT_R_MAX_ROUNDS - FRAME_UPLIFT_R_MIN_ROUNDS);
    VAL_LIMIT(set_rounds.uplift_l,FRAME_UPLIFT_L_MIN_ROUNDS,FRAME_UPLIFT_L_MAX_ROUNDS);
    VAL_LIMIT(set_rounds.uplift_r,FRAME_UPLIFT_R_MIN_ROUNDS,FRAME_UPLIFT_R_MAX_ROUNDS);

//    speed_syn_master_torque = pid_calculate(&uplift_syn_master_torque_pid, ABS(uplift_l.get_set_torque()), ABS(uplift_r.get_set_torque()));
//    speed_syn_follower_torque = pid_calculate(&uplift_syn_follower_torque_pid, ABS(uplift_r.get_set_torque()), ABS(uplift_l.get_set_torque()));

    speed_syn_master_torque = 0;
    speed_syn_follower_torque = 0;

    //uplift l
    speed_syn = pid_calculate(&uplift_syn_master_pid, uplift_l.get_motor_total_rounds(), uplift_r.get_motor_total_rounds());
    speed_set = pid_calculate(&uplift_l.posPid, uplift_l.get_motor_total_rounds(), set_rounds.uplift_l);
    uplift_l.set_truly.rounds = set_rounds.uplift_l;

    uplift_l_set_torque = pid_calculate(&uplift_l.velPid, uplift_l.get_motor_speed(), speed_set+speed_syn)+FRAME_UPLIFT_OFFSET_TORQUE+speed_syn_master_torque;
    uplift_l.set_truly.speed = speed_set+speed_syn;

    if(ABS(uplift_l.get_motor_total_rounds()-set_rounds.uplift_l) < 0.04f) {
        if(HAL_GetTick()>(l_ok_t+350)){
            VAL_LIMIT(uplift_l_set_torque,-(FRAME_UPLIFT_OFFSET_TORQUE+0.04f),FRAME_UPLIFT_OFFSET_TORQUE+0.04f);//4096
        }
    }else{
        l_ok_t = HAL_GetTick();
    }
    uplift_l.set_motor_normalization_torque(uplift_l_set_torque);

    //uplift r
    speed_syn = pid_calculate(&uplift_syn_follower_pid, uplift_r.get_motor_total_rounds(), uplift_l.get_motor_total_rounds());
    speed_set = pid_calculate(&uplift_r.posPid, uplift_r.get_motor_total_rounds(), set_rounds.uplift_r);
    uplift_r.set_truly.rounds = set_rounds.uplift_r;

    uplift_r_set_torque = pid_calculate(&uplift_r.velPid, uplift_r.get_motor_speed(), speed_set+speed_syn)+FRAME_UPLIFT_OFFSET_TORQUE+speed_syn_follower_torque;
    uplift_r.set_truly.speed = speed_set+speed_syn;

    if(ABS(uplift_r.get_motor_total_rounds()-set_rounds.uplift_r) < 0.04f) {
        if (HAL_GetTick() > (r_ok_t + 350)) {
            VAL_LIMIT(uplift_r_set_torque, -(FRAME_UPLIFT_OFFSET_TORQUE + 0.04f),FRAME_UPLIFT_OFFSET_TORQUE + 0.04f);//4096
        }
    }else{
        r_ok_t = HAL_GetTick();
    }
    uplift_r.set_motor_normalization_torque(uplift_r_set_torque);
}


void Framework::framework_set_arm_yaw(float arm_yaw_deg) {
    VAL_LIMIT(arm_yaw_deg, ARM_YAW_MIN_DEG, ARM_YAW_MAX_DEG);
    set_rounds.arm_yaw = ARM_YAW_MIN_ROUNDS + (arm_yaw_deg - ARM_YAW_MIN_DEG) / (ARM_YAW_MAX_DEG - ARM_YAW_MIN_DEG) * (ARM_YAW_MAX_ROUNDS - ARM_YAW_MIN_ROUNDS) + ARM_YAW_ZERO_OFFSET_ROUNDS;
    VAL_LIMIT(set_rounds.arm_yaw,ARM_YAW_MIN_ROUNDS+ARM_YAW_ZERO_OFFSET_ROUNDS, ARM_YAW_MAX_ROUNDS+ARM_YAW_ZERO_OFFSET_ROUNDS);
    arm_yaw.set_motor_rounds(set_rounds.arm_yaw);
}

void Framework::framework_set_arm_pitch(float arm_pitch_deg) {
    VAL_LIMIT(arm_pitch_deg, ARM_PITCH_MIN_DEG, ARM_PITCH_MAX_DEG);
    set_rounds.arm_pitch = ARM_PITCH_MIN_ROUNDS + (arm_pitch_deg - ARM_PITCH_MIN_DEG) / (ARM_PITCH_MAX_DEG - ARM_PITCH_MIN_DEG) * (ARM_PITCH_MAX_ROUNDS - ARM_PITCH_MIN_ROUNDS) + ARM_PITCH_ZERO_OFFSET_ROUNDS;
    VAL_LIMIT(set_rounds.arm_pitch,ARM_PITCH_MIN_ROUNDS+ARM_PITCH_ZERO_OFFSET_ROUNDS, ARM_PITCH_MAX_ROUNDS+ARM_PITCH_ZERO_OFFSET_ROUNDS);
    arm_pitch.set_motor_rounds(set_rounds.arm_pitch);
}

// ------------------------------------- set_move ------------------------------------- //

void Framework::framework_set_extend_move(){
    extend_l.set_torque_to_can_tx_buff();
    extend_r.set_torque_to_can_tx_buff();
}
void Framework::framework_set_slide_move(){
    slide.set_torque_to_can_tx_buff();
}
void Framework::framework_set_uplift_move(){
    uplift_l.set_torque_to_can_tx_buff();
    uplift_r.set_torque_to_can_tx_buff();
}

//单独发送can消息
void Framework::framework_set_arm_yaw_move(){
    if(!arm_yaw.is_zero_offset) {
        memset(arm_yaw.can_dev.tx.buff,0,8);;
    }else{
        arm_yaw.set_ctrl_to_can_tx_buff();
    }
    arm_yaw.can_dev.send_can_msg();
}
//单独发送can消息
void Framework::framework_set_arm_pitch_move(){
    if(!arm_pitch.is_zero_offset) {
        memset(arm_pitch.can_dev.tx.buff,0,8);
    }else{
        arm_pitch.set_ctrl_to_can_tx_buff();
    }
    arm_pitch.can_dev.send_can_msg();
}

void Framework::set_framework_move() {
    framework_set_extend_move();
    framework_set_slide_move();
    framework_set_uplift_move();
    framework_set_arm_yaw_move();
    framework_set_arm_pitch_move();
}

// ---------------------------------- is_in_target_pos ---------------------------------- //
bool Framework::is_framework_in_set_extend() {
    if(ABS(extend_l.get_motor_total_rounds()-set_rounds.extend_l) < 0.8f
        && ABS(extend_r.get_motor_total_rounds()-set_rounds.extend_r < 0.8f))
    {
        return true;
    }
    else{
        return false;
    }
}

bool Framework::is_framework_in_set_uplift() {
    if(ABS(uplift_l.get_motor_total_rounds()-set_rounds.uplift_l < 0.5f
       && ABS(uplift_r.get_motor_total_rounds()-set_rounds.uplift_r) < 0.5f))
    {
        return true;
    }else{
        return false;
    }
}

bool Framework::is_framework_in_set_slide(){
    if(ABS(slide.get_motor_total_rounds()-set_rounds.slide) < 0.5f) {
        return true;
    }else{
        return false;
    }
}

bool Framework::is_framework_in_set_arm_yaw(){
    if(ABS(arm_yaw.get_motor_total_rounds() - set_rounds.arm_yaw) < 0.004f) {
        return true;
    }else{
        return false;
    }
}

bool Framework::is_framework_in_set_arm_pitch(){
    if(ABS(arm_pitch.get_motor_total_rounds() - set_rounds.arm_pitch) < 0.004f) {
        return true;
    }else{
        return false;
    }
}

bool Framework::is_offset_ok(){
    bool ret = is_uplift_offset && is_extend_offset && is_slide_offset && is_arm_yaw_offset && is_arm_pitch_offset;
    return ret;
}


void Framework::set_framework_reset() {
    set_uplift_reset();
    set_extend_reset();
    set_slide_reset();
    set_arm_yaw_reset();
    set_arm_pitch_reset();
    set_reset_pid();
}

void Framework::extend_motors_protection(){
    if(set_rounds.uplift_l < FRAME_EXTEND_L_MIN_ROUNDS+3.0f && set_rounds.uplift_r < FRAME_EXTEND_R_MIN_ROUNDS+3.0f)
    {//圈数到
        if(ABS(set_rounds.uplift_l-uplift_l.get_motor_total_rounds())>0.2f || (ABS(set_rounds.uplift_r-uplift_r.get_motor_total_rounds())>0.2f))
        {
            if(extend_l.stall_cnt>5000 && extend_r.stall_cnt>5000)//堵转时长到
            {
                extend_l.set_motor_total_rounds_offset(FRAME_EXTEND_L_MIN_ROUNDS+FRAME_EXTEND_L_MIN_ROUNDS_OFFSET);
                extend_r.set_motor_total_rounds_offset(FRAME_EXTEND_R_MIN_ROUNDS+FRAME_EXTEND_L_MIN_ROUNDS_OFFSET);
            }
        }
    }
}

//位置限制的需要进行复位则需要单独偏置(提前量好位置),如果是全自动的就是可以在这里面设置偏置,自动量好位置
bool Framework::check_all_motors_safe(){
    osStatus_t stat;
    //uplift
    stat = osSemaphoreAcquire(uplift_l.can_dev.rx.semaphore,8);
    if(stat != osOK) {
        uplift_l.set_motor_lost();
    }else{
        uplift_l.set_motor_connect();
    }
    stat = osSemaphoreAcquire(uplift_r.can_dev.rx.semaphore,8);
    if(stat != osOK) {
        uplift_r.set_motor_lost();
    }else{
        uplift_r.set_motor_connect();
    }
    //extend
    stat = osSemaphoreAcquire(extend_l.can_dev.rx.semaphore,8);
    if(stat != osOK) {
        extend_l.set_motor_lost();
    }else{
        extend_l.set_motor_connect();
    }
    stat = osSemaphoreAcquire(extend_r.can_dev.rx.semaphore,8);
    if(stat != osOK) {
        extend_r.set_motor_lost();
    }else{
        extend_r.set_motor_connect();
    }
    //slide
    stat = osSemaphoreAcquire(slide.can_dev.rx.semaphore,15);
    if(stat != osOK) {
        slide.set_motor_lost();
    }else{
        slide.set_motor_connect();
    }
    //arm_yaw_dm
    stat = osSemaphoreAcquire(arm_yaw.can_dev.rx.semaphore, 12);
    if(stat != osOK) {
        if(arm_yaw.is_enable){
            arm_yaw.set_motor_enable();
        }
        arm_yaw.set_motor_lost();
    }else{
        arm_yaw.set_motor_connect();
    }

    stat = osSemaphoreAcquire(arm_pitch.can_dev.rx.semaphore, 12);
    if(stat != osOK) {
        if(arm_pitch.is_enable){
            arm_pitch.set_motor_enable();
        }
        arm_pitch.set_motor_lost();
    }else{
        arm_pitch.set_motor_connect();
    }

    //return
    bool b_uplift_l,b_uplift_r,b_extend_l,b_extend_r,b_slide,b_arm_yaw_dm,b_arm_pitch_dm;
    b_uplift_l = uplift_l.is_motor_lost();
    b_uplift_r = uplift_r.is_motor_lost();
    b_extend_l = extend_l.is_motor_lost();
    b_extend_r = extend_r.is_motor_lost();
    b_slide = slide.is_motor_lost();
//    b_arm_yaw_dm = arm_yaw.is_motor_lost();
//    b_arm_pitch_dm = arm_pitch.is_motor_lost();
    if(b_uplift_l || b_uplift_r || b_extend_l || b_extend_r || b_slide){
        return false;
    }else{
        return true;
    }
}


