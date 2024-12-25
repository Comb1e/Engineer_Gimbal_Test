//
// Created by Henson on 2024-03-24.
//

#include "task_test.h"
#include "drv_arm.h"
#include "drv_key.h"
#include "drv_buzzer.h"
#include "drv_DM_motor.h"

void test_slide();
void test_extend();
void test_uplift();
void test_arm_yaw();
void test_x1();
void test_y_x2();
void test_pitch_roll_u();
void test_pitch_roll_d();
void test_pitch_roll_u_rounds();
void test_kinematic();
void test_buzzer();
void test_arm_roll();

volatile float test_speed = 0.05f;
volatile float test_rounds = 0.0f;
volatile uint32_t ctrl_uint = 0;

DM_Motor_J4310 dm_motor_4310;
DJI_Motor_2006 dji_2006;

void test_new_extend();
void test_new_uplift();
void test_new_slide();
void test_new_x1();
void test_new_y_x2();

//void test_arm_pitch();

void test_arm_yaw();

bool g_good = false;


void testTask(void *argument) {
    while(!g_arm.is_init) {
        osDelay(1);
    }
//    g_arm.framework.set_extend_reset_pid();
//    g_arm.framework.set_uplift_reset_pid();
//    g_arm.framework.set_slide_reset_pid();
//    g_arm.framework.set_arm_yaw_dm_reset_pid();
//    g_arm.actuator.set_x1_reset_pid();
//    g_arm.actuator.set_y_x2_reset_pid();
    for(;;){
        switch(ctrl_uint){
            case 0:
//                test_arm_roll();
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                break;
        }
        osDelay(2);
    }
}


void test_arm_roll(){
    if(is_key_down()){
        g_good = true;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        g_arm.actuator.set_x1_reset_pid();
    }

    if(g_good == false){
//        g_arm.framework.framework_set_arm_pitch(0.0f);
        g_arm.actuator.arm_roll.set_motor_speed(0.000f);
        g_arm.actuator.actuator_set_x1_move();
    }
    else{
//        g_arm.framework.arm_pitch.set_motor_speed(0.000f);
        g_arm.actuator.arm_roll.set_motor_speed(0.2f);
//        g_arm.actuator.arm_roll.set_motor_rounds(0.0f);
        g_arm.actuator.actuator_set_x1_move();
    }
}

void test_arm_pitch(){
    if(is_key_down()){
        g_good = true;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        g_arm.framework.arm_pitch.set_motor_enable();
        g_arm.framework.set_arm_pitch_dm_reset_pid();
    }

    if(g_good == false){
//        g_arm.framework.framework_set_arm_pitch(0.0f);
        g_arm.framework.arm_pitch.set_motor_speed(0.000f);
        g_arm.framework.framework_set_arm_pitch_move();
    }
    else{
//        g_arm.framework.arm_pitch.set_motor_speed(0.000f);
        g_arm.framework.framework_set_arm_pitch(0.0f);
        g_arm.framework.framework_set_arm_pitch_move();
    }
}

void test_new_extend() {
    if(is_key_down()){
        g_good = true;
    }
    if (g_good) {
        g_arm.framework.framework_set_extend(test_rounds);
    }else{
        g_arm.framework.extend_l.set_torque = 0;
        g_arm.framework.extend_r.set_torque = 0;
    }
    g_arm.framework.framework_set_extend_move();
}

void test_new_uplift() {
    if(is_key_down()){
        g_good = true;
    }
    if (g_good) {
        g_arm.framework.framework_set_uplift(test_rounds);
    }else{
        g_arm.framework.uplift_l.set_torque = 0;
        g_arm.framework.uplift_r.set_torque = 0;
    }
    g_arm.framework.framework_set_uplift_move();
}

void test_new_slide() {
    if(is_key_down()){
        g_good = true;
    }
    if (g_good) {
        g_arm.framework.framework_set_slide(test_rounds);
    }else{
        g_arm.framework.slide.set_torque = 0;
    }
    g_arm.framework.framework_set_slide_move();
}


void test_new_x1() {
    if(is_key_down()){
        g_good = true;
    }
    if (g_good) {
        g_arm.actuator.actuator_set_x1(test_rounds);
    }else{
        g_arm.actuator.arm_roll.set_torque = 0;
    }
    g_arm.actuator.actuator_set_x1_move();
}

void test_new_y_x2() {
    if(is_key_down()){
        g_good = true;
    }
    if (g_good) {
        g_arm.actuator.actuator_set_y_x2(test_rounds,0);
    }else{
        g_arm.actuator.pitch_roll_b.set_torque = 0;
        g_arm.actuator.pitch_roll_f.set_torque = 0;
    }
    g_arm.actuator.actuator_set_y_x2_move();
}

void test_buzzer(){
    buzzer_on();
    osDelay(500);
    buzzer_off();
    osDelay(500);
}


void test_uplift() {
    if(!g_good && is_key_down()) {
        g_arm.framework.uplift_l.set_motor_normalization_torque(
                pid_calculate(&g_arm.framework.uplift_l.velPid, g_arm.framework.uplift_l.get_motor_speed(),
                              test_speed) + 0.13f);
        g_arm.framework.uplift_l.set_torque_to_can_tx_buff();
        g_arm.framework.uplift_l.can_dev.send_can_msg();
        g_arm.framework.uplift_l.set_motor_stall_parameter(8500,0.03f);

        g_arm.framework.uplift_r.set_motor_normalization_torque(
                pid_calculate(&g_arm.framework.uplift_r.velPid, g_arm.framework.uplift_r.get_motor_speed(),
                              test_speed) + 0.13f);
        g_arm.framework.uplift_r.set_torque_to_can_tx_buff();
        g_arm.framework.uplift_r.can_dev.send_can_msg();
        g_arm.framework.uplift_r.set_motor_stall_parameter(8500,0.03f);

        if(g_arm.framework.uplift_l.stall_cnt<30 && g_arm.framework.uplift_r.stall_cnt<30){
            __NOP();
        }else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.framework.uplift_l.set_motor_total_rounds_offset(FRAME_UPLIFT_L_MIN_ROUNDS);
            g_arm.framework.uplift_l.velPid.f_pid_clear_mem(&g_arm.framework.uplift_l.velPid);
            g_arm.framework.uplift_l.posPid.f_pid_clear_mem(&g_arm.framework.uplift_l.posPid);

            g_arm.framework.uplift_r.set_motor_total_rounds_offset(FRAME_UPLIFT_R_MIN_ROUNDS);
            g_arm.framework.uplift_r.velPid.f_pid_clear_mem(&g_arm.framework.uplift_r.velPid);
            g_arm.framework.uplift_r.posPid.f_pid_clear_mem(&g_arm.framework.uplift_r.posPid);
        }
    }else{
        g_arm.framework.uplift_l.set_motor_normalization_torque(
                pid_calculate(&g_arm.framework.uplift_l.velPid, g_arm.framework.uplift_l.get_motor_speed(), 0) + 0.15f);
        g_arm.framework.uplift_l.set_torque_to_can_tx_buff();
        g_arm.framework.uplift_l.can_dev.send_can_msg();

        g_arm.framework.uplift_r.set_motor_normalization_torque(
                pid_calculate(&g_arm.framework.uplift_r.velPid, g_arm.framework.uplift_r.get_motor_speed(), 0) + 0.15f);
        g_arm.framework.uplift_r.set_torque_to_can_tx_buff();
        g_arm.framework.uplift_r.can_dev.send_can_msg();
    }
}

void test_extend() {
    if(!g_good && is_key_down()) {
        g_arm.framework.extend_l.set_motor_speed(test_speed);
        g_arm.framework.extend_l.set_torque_to_can_tx_buff();
        g_arm.framework.extend_l.can_dev.send_can_msg();
        g_arm.framework.extend_l.set_motor_stall_parameter(6000,0.03f);

        g_arm.framework.extend_r.set_motor_speed(test_speed);
        g_arm.framework.extend_r.set_torque_to_can_tx_buff();
        g_arm.framework.extend_r.can_dev.send_can_msg();
        g_arm.framework.extend_r.set_motor_stall_parameter(6000,0.03f);

        if(g_arm.framework.extend_l.stall_cnt<20 && g_arm.framework.extend_r.stall_cnt<20){
            __NOP();
        }else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.framework.extend_l.set_motor_total_rounds_offset(FRAME_EXTEND_L_MIN_ROUNDS);
            g_arm.framework.extend_l.velPid.f_pid_clear_mem(&g_arm.framework.extend_l.velPid);
            g_arm.framework.extend_l.posPid.f_pid_clear_mem(&g_arm.framework.extend_l.posPid);

            g_arm.framework.extend_r.set_motor_total_rounds_offset(FRAME_EXTEND_R_MIN_ROUNDS);
            g_arm.framework.extend_r.velPid.f_pid_clear_mem(&g_arm.framework.extend_r.velPid);
            g_arm.framework.extend_r.posPid.f_pid_clear_mem(&g_arm.framework.extend_r.posPid);
        }
    }else{
        g_arm.framework.extend_l.set_torque = 0;
        g_arm.framework.extend_l.set_torque_to_can_tx_buff();
        g_arm.framework.extend_l.can_dev.send_can_msg();

        g_arm.framework.extend_r.set_torque = 0;
        g_arm.framework.extend_r.set_torque_to_can_tx_buff();
        g_arm.framework.extend_r.can_dev.send_can_msg();
    }
}


void test_slide(){
    if(!g_good && is_key_down()){
        //1. set speed
        g_arm.framework.slide.set_motor_speed(test_speed);//向左复位
        g_arm.framework.slide.set_torque_to_can_tx_buff();
        g_arm.framework.slide.can_dev.send_can_msg();
        g_arm.framework.slide.set_motor_stall_parameter(6000,0.03f);
        //2. while (check current)
        if(g_arm.framework.slide.stall_cnt<20){
            __NOP();
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.framework.slide.set_motor_total_rounds_offset(FRAME_SLIDE_MAX_ROUNDS);
            g_arm.framework.slide.velPid.f_pid_clear_mem(&g_arm.framework.slide.velPid);
            g_arm.framework.slide.posPid.f_pid_clear_mem(&g_arm.framework.slide.posPid);
        }
    }
    else{
        g_arm.framework.slide.set_torque = 0;
        g_arm.framework.slide.set_torque_to_can_tx_buff();
        g_arm.framework.slide.can_dev.send_can_msg();
    }
}

void test_arm_yaw(){
    if(is_key_down()) {
        g_good = true;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        g_arm.framework.arm_yaw.set_motor_enable();
        g_arm.framework.set_arm_yaw_dm_reset_pid();
    }

    if(g_good == false){
//        g_arm.framework.framework_set_arm_yaw(0.0f);
        g_arm.framework.arm_yaw.set_motor_speed(0.000f);
        g_arm.framework.framework_set_arm_yaw_move();
    }
    else{
//        g_arm.framework.arm_yaw.set_motor_speed(0.000f);
        g_arm.framework.framework_set_arm_yaw(0.0f);
        g_arm.framework.framework_set_arm_yaw_move();
    }
}



void test_x1(){
    if(!g_good && is_key_down()) {
        static uint8_t offset_cnt = 0;
        float speed_set = pid_calculate(
                &g_arm.actuator.arm_roll.posPid,
                g_arm.actuator.arm_roll_IMU.get_current_deg(IMU::imu_e::x) - HORIZONTAL_ARM_ROLL_DEG,
                0.0f);
        g_arm.actuator.arm_roll.set_motor_speed(speed_set);
        g_arm.actuator.arm_roll.set_torque_to_can_tx_buff();
        g_arm.actuator.arm_roll.can_dev.send_can_msg();

        if (ABS(g_arm.actuator.arm_roll.get_motor_speed()) < 0.02f) {
            offset_cnt++;
            if (offset_cnt > 50) {
                offset_cnt = 0;
                g_arm.actuator.arm_roll.set_motor_total_rounds_offset(0);
                g_arm.actuator.arm_roll.velPid.f_pid_clear_mem(&g_arm.actuator.arm_roll.velPid);
                g_arm.actuator.arm_roll.posPid.f_pid_clear_mem(&g_arm.actuator.arm_roll.posPid);
            }
        }else{
            offset_cnt = 0;
        }
    }else{
        g_arm.actuator.arm_roll.set_motor_speed(0);
        g_arm.actuator.arm_roll.set_torque_to_can_tx_buff();
        g_arm.actuator.arm_roll.can_dev.send_can_msg();
    }
}

void test_pitch_roll_u(){
    if(!g_good && is_key_down()){
        //1. set speed
        g_arm.actuator.pitch_roll_b.set_motor_speed(test_speed);//向左复位
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
        g_arm.actuator.pitch_roll_b.set_motor_stall_parameter(6000, 0.03f);
        //2. while (check current)
        if(g_arm.actuator.pitch_roll_b.stall_cnt < 20){
            __NOP();
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.actuator.pitch_roll_b.set_motor_total_rounds_offset(0);
            g_arm.actuator.pitch_roll_b.velPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.velPid);
            g_arm.actuator.pitch_roll_b.posPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.posPid);
        }
    }else{
        g_arm.actuator.pitch_roll_b.set_motor_speed(0);
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
    }
}

void test_pitch_roll_d(){
    if(!g_good && is_key_down()){
        //1. set speed
        g_arm.actuator.pitch_roll_f.set_motor_speed(test_speed);//向左复位
        g_arm.actuator.pitch_roll_f.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_f.can_dev.send_can_msg();
        g_arm.actuator.pitch_roll_f.set_motor_stall_parameter(6000, 0.03f);
        //2. while (check current)
        if(g_arm.actuator.pitch_roll_f.stall_cnt < 20){
            __NOP();
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.actuator.pitch_roll_f.set_motor_total_rounds_offset(0);
            g_arm.actuator.pitch_roll_f.velPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_f.velPid);
            g_arm.actuator.pitch_roll_f.posPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_f.posPid);
        }
    }else{
        g_arm.actuator.pitch_roll_f.set_motor_speed(0);
        g_arm.actuator.pitch_roll_f.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_f.can_dev.send_can_msg();
    }
}

void test_pitch_roll_u_rounds(){
    if(!g_good && is_key_down()){
        //1. set speed
        g_arm.actuator.pitch_roll_b.set_motor_rounds(test_rounds);//向左复位
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
        g_arm.actuator.pitch_roll_b.set_motor_stall_parameter(6000, 0.03f);
        //2. while (check current)
        if(g_arm.actuator.pitch_roll_b.stall_cnt < 20){
            __NOP();
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.actuator.pitch_roll_b.set_motor_total_rounds_offset(0);
            g_arm.actuator.pitch_roll_b.velPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.velPid);
            g_arm.actuator.pitch_roll_b.posPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.posPid);
        }
    }else{
        g_arm.actuator.pitch_roll_b.set_motor_rounds(0);//向左复位
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
    }
}


void test_y_x2() {
    if(!g_good && is_key_down()){
        //1. set speed
        g_arm.actuator.pitch_roll_b.set_motor_speed(test_speed);//向左复位
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
        g_arm.actuator.pitch_roll_b.set_motor_stall_parameter(4200, 0.03f);

        //1. set speed
        g_arm.actuator.pitch_roll_f.set_motor_speed(test_speed);//向左复位
        g_arm.actuator.pitch_roll_f.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_f.can_dev.send_can_msg();
        g_arm.actuator.pitch_roll_f.set_motor_stall_parameter(4200, 0.03f);

        //2. while (check current)
        if(g_arm.actuator.pitch_roll_f.stall_cnt < 20 && g_arm.actuator.pitch_roll_b.stall_cnt < 20){
            __NOP();
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            g_good = true;
            g_arm.actuator.pitch_roll_f.set_motor_total_rounds_offset(0);
            g_arm.actuator.pitch_roll_f.velPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_f.velPid);
            g_arm.actuator.pitch_roll_f.posPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_f.posPid);

            g_arm.actuator.pitch_roll_b.set_motor_total_rounds_offset(0);
            g_arm.actuator.pitch_roll_b.velPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.velPid);
            g_arm.actuator.pitch_roll_b.posPid.f_pid_clear_mem(&g_arm.actuator.pitch_roll_b.posPid);
        }
    }else{
        g_arm.actuator.pitch_roll_f.set_motor_speed(0);
        g_arm.actuator.pitch_roll_f.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_f.can_dev.send_can_msg();

        g_arm.actuator.pitch_roll_b.set_motor_speed(0);
        g_arm.actuator.pitch_roll_b.set_torque_to_can_tx_buff();
        g_arm.actuator.pitch_roll_b.can_dev.send_can_msg();
    }
}