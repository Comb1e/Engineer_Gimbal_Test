//
// Created by Henson on 2024-01-16.
//

#include "drv_actuator.h"
#include "GlobalCfg.h"

Actuator::Actuator(): arm_roll(), pitch_roll_b(), pitch_roll_f(), arm_roll_IMU()
{

}

void Actuator::actuator_init() {
    arm_roll.DJI_Motor_2006_init(&ARM_ROLL_CAN, 1, true, armRollBinarySemHandle);
    pitch_roll_b.DJI_Motor_2006_init(&PITCH_ROLL_U_CAN, 2, false, pitchRollUpBinarySemHandle);
    pitch_roll_f.DJI_Motor_2006_init(&PITCH_ROLL_D_CAN, 3, true, pitchRollDownBinarySemHandle);
    arm_roll_IMU.IMU_Hi229um_init(false,&ARM_ROLL_IMU,armRollImuBinarySemHandle);
    set_x1_not_use_IMU_fb();
}


bool Actuator::is_x1_at_initial_pos(){
    return is_x1_initial;
}
bool Actuator::is_y_x2_at_initial_pos(){
    return is_y_x2_initial;
}
bool Actuator::is_actuator_at_initial_pos(){
    return is_x1_at_initial_pos() && is_y_x2_at_initial_pos();
}

//
void Actuator::set_x1_reset() {
    reset_step.arm_roll = 0;
    is_x1_need_reset = true;
    is_x1_offset = false;
    is_x1_initial = false;;
}

void Actuator::set_y_x2_reset(){
    reset_step.y_x2 = 0;
    is_y_x2_need_reset = true;
    is_y_x2_offset = false;
    is_y_x2_initial = false;
}

void Actuator::set_x1_reset_pid(){
    //arm_roll √
    //针对陀螺仪
    pid_struct_init(&arm_roll.velPid,  0.8f, 0, 12, 0, 0);
    pid_struct_init(&arm_roll.posPid,  0.13f, 0.0f, 0.009f, 0.0, 0);
    arm_roll.velPid.f_pid_clear_mem(&arm_roll.velPid);
    arm_roll.posPid.f_pid_clear_mem(&arm_roll.posPid);
}

void Actuator::set_x1_running_pid(){
    //arm_roll  √
    if(is_x1_use_IMU_fb) {
        //针对陀螺仪
        pid_struct_init(&arm_roll.velPid,  0.8f, 0, 12, 0, 0);
        pid_struct_init(&arm_roll.posPid,  0.145f, 0.0f, 0.008f, 0.0, 0);
    }else{
        pid_struct_init(&arm_roll.velPid,  0.8f, 0, 12, 0, 0);
        pid_struct_init(&arm_roll.posPid,  0.2f, 0.0f, 0.16f, 0.0, 0.3f);
    }
    arm_roll.velPid.f_pid_clear_mem(&arm_roll.velPid);
    arm_roll.posPid.f_pid_clear_mem(&arm_roll.posPid);
}
void Actuator::set_y_x2_reset_pid() {
    //pitch_roll √
    pid_struct_init(&pitch_roll_b.velPid, 1.0f, 0, 13, 0, 0);
    pid_struct_init(&pitch_roll_b.posPid, 0.14f*MAX_VELOCITY_MATCH_RATIO, 0, 0.14, 0, 0.1);
    pitch_roll_b.velPid.f_pid_clear_mem(&pitch_roll_b.velPid);
    pitch_roll_b.posPid.f_pid_clear_mem(&pitch_roll_b.posPid);

    pid_struct_init(&pitch_roll_f.velPid, 1.0f, 0, 13, 0, 0);
    pid_struct_init(&pitch_roll_f.posPid, 0.14f*MAX_VELOCITY_MATCH_RATIO, 0, 0.14, 0, 0.1);
    pitch_roll_f.velPid.f_pid_clear_mem(&pitch_roll_f.velPid);
    pitch_roll_f.posPid.f_pid_clear_mem(&pitch_roll_f.posPid);
}

void Actuator::set_y_x2_running_pid(){
    //pitch_roll
    pid_struct_init(&pitch_roll_b.velPid, 1.0f, 0, 13, 0, 0);
    pid_struct_init(&pitch_roll_b.posPid, 0.2f * MAX_VELOCITY_MATCH_RATIO, 0, 0.14, 0, 0.1);
    pitch_roll_b.velPid.f_pid_clear_mem(&pitch_roll_b.velPid);
    pitch_roll_b.posPid.f_pid_clear_mem(&pitch_roll_b.posPid);

    pid_struct_init(&pitch_roll_f.velPid, 1.0f, 0, 13, 0, 0);
    pid_struct_init(&pitch_roll_f.posPid, 0.2f * MAX_VELOCITY_MATCH_RATIO, 0, 0.14, 0, 0.1);
    pitch_roll_f.velPid.f_pid_clear_mem(&pitch_roll_f.velPid);
    pitch_roll_f.posPid.f_pid_clear_mem(&pitch_roll_f.posPid);
}

void Actuator::set_reset_pid() {
    ///actuator
    set_x1_reset_pid();
    set_y_x2_reset_pid();
}

void Actuator::set_x1_not_use_IMU_fb() {
    //arm_roll  √
    is_x1_use_IMU_fb = false;
    set_x1_running_pid();
}

void Actuator::set_x1_use_IMU_fb() {
    //arm_roll  √
    is_x1_use_IMU_fb = true;
    set_x1_running_pid();
}

// ----------------------------------- reset to get offset ----------------------------------- //
bool Actuator::x1_reset_to_get_offset()
{
    if(is_x1_offset) {
        return true;
    }
    //1. set speed and pid
    static uint8_t offset_cnt = 0;
//    //todo
//    if(arm_roll_IMU.is_imu_lost()){
//        arm_roll.set_motor_speed(0);
//    }else{
        float speed_set = 0;
        speed_set = pid_calculate(
                &arm_roll.posPid,
                arm_roll_IMU.get_total_rounds_without_offset(IMU::imu_e::x)*360.0f - HORIZONTAL_ARM_ROLL_DEG,
                0.0f);
        arm_roll.set_motor_speed(speed_set);
//    }

    if(ABS(arm_roll.get_motor_speed())<0.02f) {
        offset_cnt++;
        if(offset_cnt>50){
            offset_cnt = 0;
            arm_roll.set_motor_total_rounds_offset(0);
            arm_roll.velPid.f_pid_clear_mem(&arm_roll.velPid);
            arm_roll.posPid.f_pid_clear_mem(&arm_roll.posPid);
            is_x1_offset = true;
            return true;
        }
        return false;
    }else{
        offset_cnt=0;
        return false;
    }
}

bool Actuator::y_x2_reset_to_get_offset()
{
    if(is_y_x2_offset) {
        return true;
    }
    float reset_speed = -0.06f;
    //1. set speed and pid
    pitch_roll_b.set_motor_speed(reset_speed);
    pitch_roll_b.set_motor_stall_parameter(2200, 0.02f);//2150

    pitch_roll_f.set_motor_speed(reset_speed);
    pitch_roll_f.set_motor_stall_parameter(2200, 0.02f);//2150

    //2. while (check current)
    if(pitch_roll_b.stall_cnt < 10 && pitch_roll_f.stall_cnt < 10) {//5
        return false;//3. exit
    }
    else{
        pitch_roll_b.set_motor_total_rounds_offset(Y_X2_U_OFFSET_ROUNDS);
        pitch_roll_f.set_motor_total_rounds_offset(Y_X2_D_OFFSET_ROUNDS);

        pitch_roll_b.velPid.f_pid_clear_mem(&pitch_roll_b.velPid);
        pitch_roll_b.posPid.f_pid_clear_mem(&pitch_roll_b.posPid);

        pitch_roll_f.velPid.f_pid_clear_mem(&pitch_roll_f.velPid);
        pitch_roll_f.posPid.f_pid_clear_mem(&pitch_roll_f.posPid);

        is_y_x2_offset = true;
        return true;//3. exit
    }
}

// ------------------------------------------ IK ------------------------------------------ //
void Actuator::actuator_set_x1(float x1_deg) {
    VAL_LIMIT(x1_deg, ACTUATOR_X1_MIN_DEG, ACTUATOR_X1_MAX_DEG);
    set_rounds.arm_roll = x1_deg * MOTOR_ROTATING_RATIO_M2006 * (1.0f / X1_DEDUCTION_RATIO) / 360.0f;
    VAL_LIMIT(set_rounds.arm_roll, ACTUATOR_X1_MIN_DEG* MOTOR_ROTATING_RATIO_M2006 * (1.0f / X1_DEDUCTION_RATIO) / 360.0f,
              ACTUATOR_X1_MAX_DEG* MOTOR_ROTATING_RATIO_M2006 * (1.0f / X1_DEDUCTION_RATIO) / 360.0f);
    if(!arm_roll_IMU.is_imu_lost()){
        if(is_x1_use_IMU_fb){
            float speed_set = pid_calculate(
                    &arm_roll.posPid,
                    arm_roll_IMU.get_total_rounds_without_offset(IMU::imu_e::x)*360.0f - HORIZONTAL_ARM_ROLL_DEG,
                    x1_deg);
            arm_roll.set_motor_speed(speed_set);
        }
        else{
            arm_roll.set_motor_rounds(set_rounds.arm_roll);
        }
    }else{
        arm_roll.set_motor_rounds(set_rounds.arm_roll);
    }
}

void Actuator::actuator_set_y_x2(float y_deg, float x2_deg) {
    VAL_LIMIT(y_deg, ACTUATOR_Y_MIN_DEG, ACTUATOR_Y_MAX_DEG);
    VAL_LIMIT(x2_deg, ACTUATOR_X2_MIN_DEG, ACTUATOR_X2_MAX_DEG);

    set_rounds.pitch_roll_b =
            y_deg * MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) / 360.0f
            - x2_deg * MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) * (1.0f/(SMALL_BEVEL_GEAR/BIG_BEVEL_GEAR)) / 360.0f;
    set_rounds.pitch_roll_f =
             y_deg * MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) / 360.0f
             + x2_deg * MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) * (1.0f/(SMALL_BEVEL_GEAR/BIG_BEVEL_GEAR)) / 360.0f;

    pitch_roll_b.set_motor_rounds(set_rounds.pitch_roll_b);
    pitch_roll_f.set_motor_rounds(set_rounds.pitch_roll_f);
}

// ------------------------------------------ FK ------------------------------------------ //
float Actuator::get_actuator_x1_deg() {
    float x1_deg = 0.0f;
    x1_deg = arm_roll.get_motor_total_rounds() / (MOTOR_ROTATING_RATIO_M2006 * (1.0f / X1_DEDUCTION_RATIO) / 360.0f);//改了offset读出来就没影响
    return x1_deg;
}

float Actuator::get_actuator_y_deg() {
    float y_deg = 0.0f;
    y_deg = (pitch_roll_f.get_motor_total_rounds() + pitch_roll_b.get_motor_total_rounds())
            /(2*MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) / 360.0f);
    return y_deg;
}

float Actuator::get_actuator_x2_deg() {
    float x2_deg = 0.0f;
    x2_deg = (pitch_roll_f.get_motor_total_rounds() - pitch_roll_b.get_motor_total_rounds())
            /(2*MOTOR_ROTATING_RATIO_M2006 * (SMALL_BELT_GEAR / Y_X2_BELT_GEAR) * (1.0f/(SMALL_BEVEL_GEAR/BIG_BEVEL_GEAR)) / 360.0f);
    return x2_deg;
}

// ------------------------------------- set_move ------------------------------------- //

void Actuator::actuator_set_x1_move(){
    arm_roll.set_torque_to_can_tx_buff();
}
void Actuator::actuator_set_y_x2_move(){
    pitch_roll_b.set_torque_to_can_tx_buff();
    pitch_roll_f.set_torque_to_can_tx_buff();
}
void Actuator::set_actuator_move(){
    actuator_set_x1_move();
    actuator_set_y_x2_move();
}

// ---------------------------------- is_in_target_pos ---------------------------------- //
bool Actuator::is_actuator_in_set_x1()
{
    if(ABS(arm_roll.get_motor_total_rounds()-set_rounds.arm_roll) < 0.4f) {
        return true;
    }
    else{
        return false;
    }
}

bool Actuator::is_actuator_in_set_y_x2()
{
    if(ABS(pitch_roll_b.get_motor_total_rounds() - set_rounds.pitch_roll_b) < 0.5f
        && ABS(pitch_roll_f.get_motor_total_rounds() - set_rounds.pitch_roll_f) < 0.5f)
    {
        return true;
    }
    else{
        return false;
    }
}

bool Actuator::is_offset_ok() {
    if(is_x1_offset && is_y_x2_offset){
        return true;
    }
    return false;
}

void Actuator::set_actuator_reset(){
    set_x1_reset();
    set_y_x2_reset();
    set_reset_pid();
}

//位置限制的需要进行复位则需要单独偏置(提前量好位置),如果是全自动的就是可以在这里面设置偏置,自动量好位置
bool Actuator::check_all_motors_safe(){
    osStatus_t stat;
    //arm_roll
    stat = osSemaphoreAcquire(arm_roll.can_dev.rx.semaphore,8);
    if(stat != osOK) {
        arm_roll.set_motor_lost();
    }else{
        arm_roll.set_motor_connect();
    }
    //pitch_roll_u
    stat = osSemaphoreAcquire(pitch_roll_b.can_dev.rx.semaphore, 8);
    if(stat != osOK) {
        pitch_roll_b.set_motor_lost();
    }else{
        pitch_roll_b.set_motor_connect();
    }
    //pitch_roll_d
    stat = osSemaphoreAcquire(pitch_roll_f.can_dev.rx.semaphore, 8);
    if(stat != osOK) {
        pitch_roll_f.set_motor_lost();
    }else{
        pitch_roll_f.set_motor_connect();
    }
    //return
    bool b_arm_roll,b_pitch_roll_u,b_pitch_roll_d;
    b_arm_roll = arm_roll.is_motor_lost();
    b_pitch_roll_u = pitch_roll_b.is_motor_lost();
    b_pitch_roll_d = pitch_roll_f.is_motor_lost();
    if(b_arm_roll || b_pitch_roll_u || b_pitch_roll_d){
        return false;
    }else{
        return true;
    }
}
