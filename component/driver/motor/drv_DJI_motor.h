//
// Created by Henson on 2024-01-15.
//

#ifndef __DRV_DJI_MOTOR_H
#define __DRV_DJI_MOTOR_H

#include "drv_motor.h"
#include "interface_can.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

//M3508
#define MOTOR_MAX_CURRENT_M3508     (16384.0f)                  // 16384 for M3508
#define MOTOR_MAX_SPEED_M3508       (9600.0f)                   // 9600 for M3508
#define MOTOR_MAX_ECD_M3508         (8192.0f)
#define MOTOR_ROTATING_RATIO_M3508  (19.0f)                     //19 : 1 for M3508

//M2006
#define MOTOR_MAX_CURRENT_M2006     (10000.0f)                  // 10000 for M2006
#define MOTOR_MAX_SPEED_M2006       (21000.0f)                  // 21000  for M2006
#define MOTOR_MAX_ECD_M2006         (8192.0f)
#define MOTOR_ROTATING_RATIO_M2006  (36.0f)                     //36 : 1 for M2006

//GM6020
#define MOTOR_MAX_VOLTAGE_GM6020    (30000.0f)                  // 30000 for GM6020
#define MOTOR_MAX_SPEED_GM6020      (320.0f)                    // 320  for GM6020
#define MOTOR_MAX_ECD_GM6020        (8192.0f)

typedef struct {
    uint8_t is_enable;
    CAN_HandleTypeDef *hcan;
    uint8_t len;
    uint16_t stdid;
    uint8_t buff[8];
}dji_motor_can_tx_t;

class DJI_Motor:public Motor{
public:
    typedef struct{
        uint16_t ecd;           //转子机械角度 编码器值
        int16_t speed_rpm;      //转速以rpm为单位
        int16_t fb_torque;      //反馈的转矩
        uint8_t temperature;    //电机温度
    } raw_data_t;

    int16_t set_torque = 0;         //要发送的电流,后续设置不用初始化
    int16_t fb_torque = 0;
    raw_data_t raw={};             //原始数据,接收不用初始化
    CAN can_dev;                  //can数据,初始化
    struct pid velPid={};          //功能中初始化
    struct pid posPid={};          //功能中初始化

    float max_torque = 0.0f;
    float max_speed = 0.0f;
    float max_ecd = 0.0f;

    uint32_t stall_cnt = 0;
    int16_t stall_torque_max = 6000; // torque && speed
    float stall_speed_max = 0.03f; // torque && speed

    void (*process_data_ptr_fun)();

public:
    explicit DJI_Motor();
    void DJI_Motor_init(CAN_HandleTypeDef *hcan,uint32_t motor_id,bool _is_reverse);

    void set_torque_to_can_tx_buff();
    void set_motor_normalization_torque(float torque);
    void set_motor_speed(float speed) override;
    void set_motor_rounds(float rounds) override;
    void set_motor_free() override;
    void set_motor_stall_parameter(int16_t _stall_torque_max,float _stall_speed_max);

    void motor_rx_data_update_callback(CAN *_can_dev, uint8_t *rx_data);
    void motor_data_update(uint8_t *can_rx_data);
    void process_data_ptr_fun_register(void (*fun)());

    int16_t get_set_torque();
    int16_t get_fb_torque();
    static dji_motor_can_tx_t can_tx_list[6];
};

class DJI_Motor_3508:public DJI_Motor{
public:
    DJI_Motor_3508();
    void DJI_Motor_3508_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse, osSemaphoreId_t rxSem);
};

class DJI_Motor_2006:public DJI_Motor{
public:
    DJI_Motor_2006();
    void DJI_Motor_2006_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse, osSemaphoreId_t rxSem);
};

class DJI_Motor_6020:public DJI_Motor{
public:
    DJI_Motor_6020();
    void DJI_Motor_6020_init(CAN_HandleTypeDef *hcan, uint32_t motorId, bool isReverse, osSemaphoreId_t rxSem);
};

#endif //__DRV_DJI_MOTOR_H
