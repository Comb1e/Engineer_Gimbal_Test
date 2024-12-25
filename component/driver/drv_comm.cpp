//
// Created by Henson on 2024-03-07.
//

#include "drv_comm.h"
#include "drv_arm.h"
#include "freertos_inc.h"

CommCAN::CommCAN() : can_basic_dev(), can_more_dev(){

}

void CommUSB::CommUSB_init(){

}

void CommCAN::CommCANInit(CAN_HandleTypeDef *hcan){
    //basic
    can_basic_dev.can_init(hcan);
    can_basic_dev.can_rx_tx_add(COMM_CAN_BASIC_RX_STDID, CommCanBasicBinarySemHandle,
                                [this](CAN *_can_dev, uint8_t *_rx_data) {
                                    this->can_rx_basic_data_callback(_can_dev, _rx_data);
                                }, COMM_CAN_BASIC_TX_STDID, (uint8_t *) &tx_basic_raw, 8);

    //more
    can_more_dev.can_init(hcan);
    can_more_dev.can_rx_tx_add(COMM_CAN_MORE_RX_STDID, CommCanMoreBinarySemHandle,
                               [this](CAN *_can_dev, uint8_t *_rx_data) {
                                   this->can_rx_more_data_callback(_can_dev, _rx_data);
                               }, COMM_CAN_MORE_TX_STDID, (uint8_t *) &tx_more_raw, 8);

    //custom
    can_custom_dev.can_init(hcan);
    can_custom_dev.can_rx_add(COMM_CAN_CUSTOM_RX_STDID,CommCanCustomBinarySemHandle,std::move( [this](CAN *_can_dev, uint8_t *_rx_data) {
        this->can_rx_custom_data_callback(_can_dev, _rx_data);
    }));
}


void CommCAN::can_rx_custom_data_callback(CAN *device,uint8_t *data){
    memcpy(&this->rx_custom_raw, data, sizeof(this->rx_custom_raw));
}

void CommCAN::can_rx_basic_data_callback(CAN *device,uint8_t *data){
    memcpy(&this->rx_basic_raw,data,sizeof(this->rx_basic_raw));
}
void CommCAN::can_rx_more_data_callback(CAN *device,uint8_t *data){
    memcpy(&this->rx_more_raw,data,sizeof(this->rx_more_raw));
}

Communication::Communication(): can(CommCAN()),usb(CommUSB())
{

}

void Communication::Communication_init(){
    can.CommCANInit(&COMM_DATA_CAN);
    usb.CommUSB_init();
}

// --------------------------- 通信反馈控制 --------------------------- //
void Arm::set_communication_basic_data_to_rx_data(){
    taskENTER_CRITICAL();
    communication.can.rx_set_data.x_mm = communication.can.rx_basic_raw.x_mm;
    communication.can.rx_set_data.x_mm /= 2.0f;
    communication.can.rx_set_data.y_mm = communication.can.rx_basic_raw.y_mm;
    communication.can.rx_set_data.y_mm /= 2.0f;
    communication.can.rx_set_data.z_mm = communication.can.rx_basic_raw.z_mm;
    communication.can.rx_set_data.z_mm /= 2.0f;
    communication.can.rx_set_data.yaw_deg = communication.can.rx_basic_raw.yaw_deg;
    communication.can.rx_set_data.yaw_deg /= 2.0f;
    communication.can.rx_set_data.pitch_deg = communication.can.rx_basic_raw.pitch_deg;
    communication.can.rx_set_data.pitch_deg /= 2.0f;
    communication.can.rx_set_data.roll_deg = communication.can.rx_basic_raw.roll_deg;
    communication.can.rx_set_data.roll_deg /= 2.0f;

    communication.can.rx_set_data.x_mm -= ARM_X_MM_OFFSET;
    communication.can.rx_set_data.y_mm -= ARM_Y_MM_OFFSET;
    communication.can.rx_set_data.z_mm -= ARM_Z_MM_OFFSET;

    taskEXIT_CRITICAL();
}

void Arm::set_communication_more_data_to_rx_data() {
    taskENTER_CRITICAL();

    communication.can.rx_set_data.arm_yaw_deg = communication.can.rx_more_raw.arm_yaw_deg;
    communication.can.rx_set_data.arm_yaw_deg /= 8.0f;
    communication.can.rx_set_data.arm_pitch_deg = communication.can.rx_more_raw.arm_pitch_deg;
    communication.can.rx_set_data.arm_pitch_deg /= 8.0f;

    communication.can.rx_custom_ctrl_data.is_using_custom_ctrl = communication.can.rx_more_raw.is_using_custom_ctrl;

    //rising edge
    if(communication.can.rx_set_data.is_actuator_reset==false && (bool)communication.can.rx_more_raw.is_sucker_reset==true
       && is_arm_reset_all_ok())
    {
        set_actuator_single_reset();
        set_arm_arm_yaw_deg(communication.can.rx_set_data.arm_yaw_deg);
        set_arm_arm_pitch_deg(communication.can.rx_set_data.arm_pitch_deg);
        set_arm_pose(x,communication.can.rx_set_data.x_mm);
        set_arm_pose(y,communication.can.rx_set_data.y_mm);
        set_arm_pose(z,communication.can.rx_set_data.z_mm);
    }
    communication.can.rx_set_data.is_actuator_reset = communication.can.rx_more_raw.is_sucker_reset;

    //rising edge
    if(communication.can.rx_set_data.is_arm_need_reset==false && (bool)communication.can.rx_more_raw.is_arm_need_reset==true
       && is_arm_reset_all_ok())
    {
        __set_FAULTMASK(1);  //关闭全局中断
        HAL_NVIC_SystemReset();
    }
    communication.can.rx_set_data.is_arm_need_reset = communication.can.rx_more_raw.is_arm_need_reset;

    if(communication.can.rx_more_raw.pose_select_mode==0){//single
        communication.can.rx_set_data.is_single_solution = true;
        communication.can.rx_set_data.is_concentric_double_solution = false;
        communication.can.rx_set_data.is_eccentric_double_solution = false;
        set_pose_select_single_solution();
    }
    else if(communication.can.rx_more_raw.pose_select_mode==1){//concentric
        communication.can.rx_set_data.is_single_solution = false;
        communication.can.rx_set_data.is_concentric_double_solution = true;
        communication.can.rx_set_data.is_eccentric_double_solution = false;
        set_pose_select_concentric_double_solution();
    }
    else if(communication.can.rx_more_raw.pose_select_mode==2){//eccentric
        communication.can.rx_set_data.is_single_solution = false;
        communication.can.rx_set_data.is_concentric_double_solution = false;
        communication.can.rx_set_data.is_eccentric_double_solution = true;
        set_pose_select_eccentric_double_solution();
    }
    taskEXIT_CRITICAL();
}


void Arm::set_communication_to_arm_pose(){
    set_arm_arm_yaw_deg(communication.can.rx_set_data.arm_yaw_deg);
    set_arm_arm_pitch_deg(communication.can.rx_set_data.arm_pitch_deg);
    set_arm_pose(x,communication.can.rx_set_data.x_mm);
    set_arm_pose(y,communication.can.rx_set_data.y_mm);
    set_arm_pose(z,communication.can.rx_set_data.z_mm);
    set_arm_pose(yaw,communication.can.rx_set_data.yaw_deg);
    set_arm_pose(pitch,communication.can.rx_set_data.pitch_deg);
    set_arm_pose(roll,communication.can.rx_set_data.roll_deg);
}

void Arm::set_FK_data_to_communication() {
    taskENTER_CRITICAL();
    communication.can.tx_fb_data.x_mm = get.x_mm;
    communication.can.tx_fb_data.y_mm = get.y_mm;
    communication.can.tx_fb_data.z_mm = get.z_mm;
    communication.can.tx_fb_data.yaw_deg = get.yaw_deg;
    communication.can.tx_fb_data.pitch_deg = get.pitch_deg;
    communication.can.tx_fb_data.roll_deg = get.roll_deg;

    communication.can.tx_fb_data.arm_yaw_deg = kine.get.framework.arm_yaw_deg;
    communication.can.tx_fb_data.arm_pitch_deg = kine.get.framework.arm_pitch_deg;

    communication.can.tx_fb_data.framework_extend_mm = kine.get.framework.extend_mm;
    communication.can.tx_fb_data.framework_slide_mm = kine.get.framework.slide_mm;
    communication.can.tx_fb_data.framework_uplift_mm = kine.get.framework.uplift_mm;

    communication.can.tx_basic_raw.x_mm = roundf(communication.can.tx_fb_data.x_mm + ARM_X_MM_OFFSET);
    communication.can.tx_basic_raw.y_mm = roundf(communication.can.tx_fb_data.y_mm + ARM_Y_MM_OFFSET);
    communication.can.tx_basic_raw.z_mm = roundf(communication.can.tx_fb_data.z_mm + ARM_Z_MM_OFFSET);
    communication.can.tx_basic_raw.yaw_deg = roundf(communication.can.tx_fb_data.yaw_deg);
    communication.can.tx_basic_raw.pitch_deg = roundf(communication.can.tx_fb_data.pitch_deg);
    communication.can.tx_basic_raw.roll_deg = roundf(communication.can.tx_fb_data.roll_deg);

    communication.can.tx_more_raw.arm_yaw_deg = roundf(communication.can.tx_fb_data.arm_yaw_deg);
    communication.can.tx_more_raw.arm_pitch_deg = roundf(communication.can.tx_fb_data.arm_pitch_deg);

    communication.can.tx_more_raw.framework_extend_mm = roundf(communication.can.tx_fb_data.framework_extend_mm);
    communication.can.tx_more_raw.framework_slide_mm = roundf(communication.can.tx_fb_data.framework_slide_mm);
    communication.can.tx_more_raw.framework_uplift_mm = roundf(communication.can.tx_fb_data.framework_uplift_mm);

    communication.can.tx_more_raw.total_arm_xy_dist_mm = roundf(communication.can.tx_fb_data.total_arm_xy_dist_mm);//0-512
    communication.can.tx_more_raw.total_arm_xy_yaw_deg = roundf(communication.can.tx_fb_data.total_arm_xy_yaw_deg);//-90-90

    communication.can.tx_more_raw.is_imu_lost = g_arm.actuator.arm_roll_IMU.is_imu_lost();
    communication.can.tx_more_raw.is_motors_lost = !(g_arm.framework.is_all_motors_safe && g_arm.actuator.is_all_motors_safe);
    communication.can.tx_more_raw.is_motors_overheat = g_arm.is_motors_overheat;
    taskEXIT_CRITICAL();
}


void Arm::send_FK_data_to_communication() {
    communication.can.can_basic_dev.send_can_msg();
    communication.can.can_more_dev.send_can_msg();
}