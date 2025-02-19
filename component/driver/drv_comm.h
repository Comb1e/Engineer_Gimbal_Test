//
// Created by Henson on 2024-03-07.
//

#ifndef __DRV_COMM_H
#define __DRV_COMM_H

#include "interface_can.h"
#include "GlobalCfg.h"
#include "drv_arm.h"
#include "drv_dr16.h"
#include "freertos_inc.h"

/*---------------------------- C++ Scope ---------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

#define USB_FRAME_HEAD 0x55
#define USB_FRAME_TAIL 0xAA

#define USB_INFO_TX_BUF_NUM 60

#pragma pack(1)
typedef struct
{
    uint8_t is_arm_pump_holding_on : 1;
    uint8_t is_left_pump_holding_on : 1;
    uint8_t is_right_pump_holding_on : 1;
    uint8_t is_wheel_lf_lost : 1;
    uint8_t is_wheel_lb_lost : 1;
    uint8_t is_wheel_rb_lost : 1;
    uint8_t is_wheel_rf_lost : 1;
    uint8_t is_gimbal_slide_lost : 1;
    float chassis_gyro_totoal_rounds;//底盘陀螺仪的yaw
}can_rx_raw_data_t;//40位

typedef struct
{
    uint8_t is_chassis_vel_control : 1;//底盘是否是速度控制
    int8_t chassis_spin;//(-128)-(127)
    int16_t chassis_x;//合适的进制转换
    int16_t chassis_y;
    uint8_t is_arm_pump_open : 1;
    uint8_t is_left_pump_open : 1;
    uint8_t is_right_pump_open : 1;
    uint8_t is_usb_lost : 1;
}can_tx_data_t;//44位

typedef struct
{
    uint8_t frame_head;
    // 以下为关节状态设定信息
    float uplift_joint;
    float extend_joint;
    float slide_joint;
    float arm_yaw_joint;
    float arm_pitch_joint;
    float arm_roll_joint;
    float pitch_joint;
    float roll_joint;
    //以下为气泵打开指令
    uint8_t is_arm_pump_open;
    uint8_t is_left_pump_open;
    uint8_t is_right_pump_open;
    //以下为底盘控制信息
    uint8_t is_chassis_vel_control;//底盘是否是速度控制
    int8_t chassis_spin;
    int16_t chassis_x;//合适的进制转换
    int16_t chassis_y;
    uint8_t frame_tail;
}usb_rx_raw_t;//43

typedef struct
{
    uint8_t frame_head;
    uint8_t remote_ctrl[18];
    // 以下为关节状态信息
    float uplift_joint;
    float extend_joint;
    float slide_joint;
    float arm_yaw_joint;
    float arm_pitch_joint;
    float arm_roll_joint;
    float pitch_joint;
    float roll_joint;
    //以下为气泵是否吸住
    uint8_t is_arm_pump_holding_on;
    uint8_t is_left_pump_holding_on;
    uint8_t is_right_pump_holding_on;

    uint8_t is_rc_online;//关控保护
    float chassis_gyro_totoal_rounds;//底盘陀螺仪的yaw
    uint8_t frame_tail;
}usb_tx_t;//60

#pragma pack()

#ifdef __cplusplus
}
#endif

class Comm_CAN
{
public:
    explicit Comm_CAN();

    bool is_enable = true;

    can_rx_raw_data_t rx_raw_data;
    can_tx_data_t tx_data;

    CAN can_device;

    void Init(CAN_HandleTypeDef *hcan);

    void Comm_Can_RX_Callback(CAN *can_device,uint8_t *data);

    friend class Communication;
};

void Comm_Can_RX_Callback(CAN *can_device,uint8_t *data);

class Comm_USB
{
public:
    Comm_USB()= default;

    bool is_enable = false;
    bool is_lost = true;
    bool data_valid_flag;

    usb_rx_raw_t rx_raw={};
    usb_tx_t tx_data={};

    void Init();
    void Start_Receive_Data();
    void Transmit_Data();
    void Update_RX_Data();

    friend class Communication;
};


class Communication
{
public:
    explicit Communication();

    bool usb_lost_flag;
    bool can_lost_flag;
    bool rc_control_flag;

    Comm_CAN *can;
    Comm_USB *usb;
    Arm *arm;
    rc_device_t *rc;

    void PTR_Init(Comm_CAN *comm_can,Comm_USB *comm_usb,Arm *Arm,rc_device_t *rc);
    void Init();
    void Update_CAN_TX_Data();
    void Update_TX_Data();
    void Update_USB_Lost_Flag();
    void Update_CAN_Lost_Flag();
    bool Check_USB_Lost_Flag();
    void Send_MSG();
    void Update_Arm_Control();
    void Set_Chassis_Control_RC();
    void Update_RC_Control();
    void Set_Chassis_Control_USB();
};

extern Comm_CAN comm_can;
extern Comm_USB comm_usb;
extern Communication communication;

/*---------------------------- C Scope ---------------------------*/

#endif //__DRV_COMM_H
