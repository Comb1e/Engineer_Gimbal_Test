//
// Created by Henson on 2024-03-07.
//

#ifndef __DRV_COMM_H
#define __DRV_COMM_H

#include "interface_can.h"
#include "GlobalCfg.h"

/*---------------------------- C++ Scope ---------------------------*/

#define ARM_X_MM_OFFSET     (-200)
#define ARM_Y_MM_OFFSET     400
#define ARM_Z_MM_OFFSET     150

#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(1)

typedef struct {
    uint16_t x_mm : 11;//0-1024      200-1224
    uint16_t y_mm : 11;//0-1024      -400
    uint16_t z_mm : 11;//0-1024      -150
    int16_t yaw_deg : 10;//-256-256
    int16_t pitch_deg : 9;//-128-128
    int16_t roll_deg : 10;// -256-256
    uint16_t reserve_00 : 2;
} rx_basic_raw_t;

typedef struct {
    uint16_t is_sucker_reset : 1;
    uint16_t is_arm_need_reset : 1;
    uint16_t pose_select_mode : 2;// 0: single  1:concentric double  2: eccentric double
    int16_t arm_pitch_deg : 11;//-128-128
    int16_t arm_yaw_deg : 11;//-128-128
    uint16_t is_using_custom_ctrl : 1;//bool
    uint16_t reserve_04 : 5;
    uint16_t reserve_05 : 8;
    uint16_t reserve_06 : 8;
    uint16_t reserve_07 : 8;
    uint16_t reserve_08 : 8;
} rx_more_raw_t;


typedef struct {
    uint16_t frame_x_mm : 10;//0-1024  660
    uint16_t frame_y_mm : 8;//0-256    145
    uint16_t frame_z_mm : 10;//0-1024  660
    int16_t euler_x1_deg : 9;//-256-256 大roll轴
    int16_t euler_y_deg : 9;//-256-256  小pitch
    int16_t euler_x2_deg : 9;//-256-256 小roll轴
    int16_t arm_yaw_deg : 9; //-128-128
} rx_custom_raw_t;

typedef struct {
    uint16_t x_mm : 10;//0-1024
    uint16_t y_mm : 10;//-372.5-513
    uint16_t z_mm : 10;//-54-624
    int16_t yaw_deg : 9;//-256-256
    int16_t pitch_deg : 8;//-128-128
    int16_t roll_deg : 9;// -256-256
    uint16_t reserve_0 : 8;
} tx_basic_raw_t;

typedef struct {
    int16_t arm_yaw_deg : 8;//-128-128
    int16_t arm_pitch_deg : 8;//-128-128
    uint16_t framework_extend_mm : 10;//0-1024
    uint16_t framework_slide_mm : 8;//0-256
    uint16_t framework_uplift_mm : 10;//0-1024
    uint16_t total_arm_xy_dist_mm : 9;//0-512
    int16_t total_arm_xy_yaw_deg : 8;//-90-90
    int16_t is_imu_lost : 1;
    int16_t is_motors_lost : 1;
    int16_t is_motors_overheat : 1;
} tx_more_raw_t;

typedef struct {
    uint16_t reserve_00 : 8;
    uint16_t reserve_01 : 8;
    uint16_t reserve_02 : 8;
    uint16_t reserve_03 : 8;
    uint16_t reserve_04 : 8;
    uint16_t reserve_05 : 8;
    uint16_t reserve_06 : 8;
    uint16_t reserve_07 : 8;
} tx_custom_raw_t;
#pragma pack()
#ifdef __cplusplus
}
#endif

typedef struct{
    float round;                        //转子机械角度 编码器值
    float last_round;
    float offset_round;                 //校偏置 -2~2
    float total_rounds;                 //总圈准数,与方向有关
    float total_rounds_without_offset;  //总圈准数,与方向有关
    float total_deg;
    int32_t spill_cnt;                  //圈数计算,大部分都为整数
}custom_euler_t;

typedef struct{
    bool is_using_custom_ctrl;
    uint32_t rcv_cnt;
    bool is_offset_ok;
    float frame_x_mm;//0-1024  660
    float frame_y_mm;//0-256   145
    float frame_z_mm;//0-1024  660
    custom_euler_t euler_x1;
    custom_euler_t euler_y;
    custom_euler_t euler_x2;
    custom_euler_t arm_yaw;
}custom_ctrl_data_t;

class CommCAN{
public:
    struct{
        float x_mm;
        float y_mm;
        float z_mm;
        float yaw_deg;
        float pitch_deg;
        float roll_deg;
        float arm_yaw_deg;
        float arm_pitch_deg;
        bool is_actuator_reset;
        bool is_arm_need_reset;//边沿触发而非电平触发

        bool is_single_solution;
        bool is_concentric_double_solution;
        bool is_eccentric_double_solution;
    }rx_set_data={};


    custom_ctrl_data_t rx_custom_ctrl_data={};


    struct{
        float x_mm;
        float y_mm;
        float z_mm;
        float yaw_deg;
        float pitch_deg;
        float roll_deg;
        float arm_yaw_deg;
        float arm_pitch_deg;

        float framework_extend_mm;
        float framework_slide_mm;
        float framework_uplift_mm;

        float total_arm_xy_dist_mm;
        float total_arm_xy_yaw_deg;
    }tx_fb_data={};

    bool is_enable = true;
    CAN can_basic_dev;
    CAN can_more_dev;
    CAN can_custom_dev;
    rx_basic_raw_t rx_basic_raw={};
    rx_more_raw_t rx_more_raw={};
    rx_custom_raw_t rx_custom_raw={};
    tx_basic_raw_t tx_basic_raw={};
    tx_more_raw_t tx_more_raw={};
    tx_custom_raw_t tx_custom_raw={};
public:
    explicit CommCAN();
    void CommCANInit(CAN_HandleTypeDef *hcan);
    void can_rx_basic_data_callback(CAN *device,uint8_t *data);
    void can_rx_more_data_callback(CAN *device,uint8_t *data);
    void can_rx_custom_data_callback(CAN *device,uint8_t *data);
};


class CommUSB{
public:
#pragma pack(1)
    typedef struct {
        uint16_t reserve_0 : 8;
        uint16_t reserve_1 : 8;
        uint16_t reserve_2 : 8;
        uint16_t reserve_3 : 8;
        uint16_t reserve_4 : 8;
        uint16_t reserve_5 : 8;
        uint16_t reserve_6 : 8;
        uint16_t reserve_7 : 8;
    } rx_raw_t;
    typedef struct {
        uint16_t reserve_0 : 8;
        uint16_t reserve_1 : 8;
        uint16_t reserve_2 : 8;
        uint16_t reserve_3 : 8;
        uint16_t reserve_4 : 8;
        uint16_t reserve_5 : 8;
        uint16_t reserve_6 : 8;
        uint16_t reserve_7 : 8;
    } tx_raw_t;
#pragma pack()
    bool is_enable = false;
    bool is_lost = true;
    rx_raw_t rx_raw={};
    tx_raw_t tx_raw={};
public:
    CommUSB()= default;
    void CommUSB_init();
};


class Communication{
public:
    CommCAN can;
    CommUSB usb;
public:
    explicit Communication();
    void Communication_init();
private:

};


/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif //__DRV_COMM_H
