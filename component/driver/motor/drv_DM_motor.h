//
// Created by Henson on 2024-01-15.
//

#ifndef __DRV_DM_MOTOR_H
#define __DRV_DM_MOTOR_H

#include "interface_can.h"
#include "freertos_inc.h"
#include "user_lib.h"
#include "drv_motor.h"
#include "pid.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

#define DM_J4310_2EC_T_MAX          (10.0f)
#define DM_J4310_2EC_V_MAX          (30.0f)
#define DM_J4310_2EC_P_MAX          (3.141593f)

#define DM_J4310_2EC_KP_MIN         0.0f
#define DM_J4310_2EC_KP_MAX         500.0f
#define DM_J4310_2EC_KD_MIN         0.0f
#define DM_J4310_2EC_KD_MAX         5.0f

//J4310
#define MOTOR_MAX_TORQUE_J4310      DM_J4310_2EC_T_MAX       // for J4310
#define MOTOR_MAX_VELOCITY_J4310    DM_J4310_2EC_V_MAX       // for J4310
#define MOTOR_MAX_POSITION_J4310    DM_J4310_2EC_P_MAX
#define MOTOR_ROTATING_RATIO_J4310  (10.0f/1.0f)             //10 : 1 for M3508

class DM_Motor:public Motor {
public:
#pragma pack(1)
    typedef struct{
        uint16_t p_des : 16;
        uint16_t v_des : 12;
        uint16_t kp : 12;
        uint16_t kd : 12;
        uint16_t t_ff : 12;
    } dm_motor_MIT_can_tx_buff_t;

    typedef struct{
        float p_des;
        float v_des;
    } dm_motor_PV_can_tx_buff_t;

    typedef struct{
        float v_des;
    } dm_motor_VO_can_tx_buff_t;
#pragma pack()

    typedef struct {
        uint8_t id;
        uint8_t err;
        uint16_t pos_rad;       //转子机械角度 编码器值
        uint16_t vel_rad_s;       //转速以rad/s为单位
        uint16_t torque;    //转矩
        uint8_t t_mos;      //驱动上平均温度℃
        uint8_t t_rotor;    //电机线圈温度℃
    } dm_motor_raw_data_t;

    typedef struct{
        float p_des;//rad
        float v_des;//rad/s
        float kp;
        float kd;
        float torq;
    } dm_motor_tx_t;

    typedef enum{
        MOTOR_MODE_NONE = 0,
        DM_MIT,//MIT模式
        DM_PV,//速度位置模式
        DM_VO,//速度模式
    }dm_motor_mode_e;

    typedef enum{
        DM_MIT_Torque = 0U,    /*!< MIT模式下对力矩进行控制 */
        DM_MIT_Velocity = 1U,     /*!< MIT模式下对速度进行控制 */
        DM_MIT_Position = 2U   /*!< MIT模式下对位置进行控制 */
    } dm_motor_mit_mode_e;


    typedef enum {
        Disabled = 1U,             /*!< 失能电机，关闭电机 */
        Enabled = 2U,              /*!< 电机没有异常状态，已经使能并可以正常工作 */

        OverVoltage = 0x8,         /*!< 电机超压，电压高于32V，过压将退出“使能模式” */
        UnderVoltage = 0x9,        /*!< 电机欠压，电压低于15V，欠压将退出“使能模式” */
        OverCurrent = 0xA,         /*!< 电机过电流，电流高于9.8A，过流将退出“使能模式” */
        MosOverTemperature = 0xB,  /*!< MOS过温，温度大于120℃将退出“使能模式” */
        CoilOverTemperature = 0xC, /*!< 线圈过温，温度大于120℃将退出“使能模式” */
        CommunicationLos = 0xD,    /*!< 通讯丢失防护，设定周期内没有收到 CAN 指令将自动退出“使能模式” */
        Overload = 0xE             /*!< 电机过载 */
    } dm_motor_state_enum;

    //basic
    bool is_enable = false;
    bool is_use_absolute_encoder = true;
    uint32_t slave_id = 0;          //电机ID,slave_id tx
    uint32_t master_id = 0;         //电机ID,master_id rx
    struct pid velPid={};           //功能中初始化
    struct pid posPid={};           //功能中初始化
    CAN can_dev;                    //can数据,初始化

    //data
    dm_motor_raw_data_t raw_data{};     //原始数据,接收不用初始化
    dm_motor_tx_t ctrl_data{};
    uint8_t tx_buff[8]={};

    //mode
    dm_motor_mode_e mode = DM_MIT;  //电机模式
    dm_motor_state_enum motor_state = Disabled;
    dm_motor_mit_mode_e mit_mode = DM_MIT_Torque;

    float max_torque = 0.0f;
    float max_vel_rad_s = 0.0f;
    float max_pos_rad = 0.0f;


    //stall
    float fb_torque = 0;
    uint32_t stall_cnt = 0;
    float stall_torque_max = 0.5f; // torque && speed
    float stall_speed_max = 0.03f; // torque && speed
public:
    explicit DM_Motor();
    void DM_Motor_init(CAN_HandleTypeDef *hcan, uint32_t slaveId, uint16_t masterId, dm_motor_mode_e motor_mode,bool isReverse);

    void set_motor_speed(float speed) override;
    void set_motor_rounds(float rounds) override;
    void set_motor_stall_parameter(float _stall_torque_max,float _stall_speed_max);
    void motor_rx_data_update_callback(CAN *_can_dev, uint8_t *rx_data);
    void motor_data_update(uint8_t *can_rx_data);

    void MIT_inter_set_motor_round(float rounds);
    void MIT_inter_set_motor_speed(float speed);
    void MIT_inter_set_motor_normalization_torque(float torque);
    void MIT_outer_set_motor_total_rounds(float total_rounds);
    void MIT_outer_set_motor_speed(float speed);

    void PV_inter_set_motor_total_rounds(float total_rounds,float speed);

    void VO_inter_set_motor_speed(float speed);


    bool recover_the_motor();
    void set_ctrl_to_can_tx_buff();
    void set_motor_enable();
    void set_motor_disable();
    void set_motor_clear_error();
    void set_motor_save_zero_offset();
    void set_motor_total_rounds_offset(float _total_rounds) override;

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

class DM_Motor_J4310:public DM_Motor {
public:
    DM_Motor_J4310();
    void DM_Motor_J4310_init(CAN_HandleTypeDef *hcan, uint32_t slaveId, uint32_t masterId,
                             dm_motor_mode_e motorMode, bool isReverse, osSemaphoreId_t rxSem);
};


#endif //__DRV_DM_MOTOR_H
