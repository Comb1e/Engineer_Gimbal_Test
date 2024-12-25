//
// Created by Henson on 2023-11-18.
//

#ifndef __DRV_HI229UM_H
#define __DRV_HI229UM_H

#include "bsp_usart.h"
#include "user_lib.h"
#include "common_inc.h"
#include "crc.h"
#include "drv_imu.h"

#define DRV_HI229UM_BUF_SIZE 100

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*---------------------------- C++ Scope ---------------------------*/

class IMU_Hi229um:public IMU{
public:
#pragma pack(1)
    union hi229um_raw{
        struct {
            uint16_t head;
            uint16_t len;
            uint16_t crc;
        }frame_head;//6
        struct {
            struct {
                uint16_t head;
                uint16_t len;
                uint16_t crc;
            }frame_head_91;//6
            uint8_t tag;//1
            uint8_t user_id;//1
            uint16_t reserving_bytes;//2
            float pressure;//气压pa 4
            uint32_t time_ms;//ms 4
            struct {
                float x;
                float y;
                float z;
            }acc;//加速度g 12
            struct {
                float x;
                float y;
                float z;
            }ang_v;//角速度deg/s 12
            struct {
                float x;
                float y;
                float z;
            }magnetic;//磁场uT 12
            struct {
                float roll;//for x
                float pitch;//for y
                float yaw;//for z
            }euler_ang;//欧拉角deg 12
            struct {
                float W;
                float X;
                float Y;
                float Z;
            }quaternion;//四元数 16
        }data_91;

        struct {
            struct {
                uint16_t head;
                uint16_t len;
                uint16_t crc;
            }frame_head_90;//6
            struct {
                uint8_t label;
                uint8_t id;
            }usr_id;//2
            struct {
                uint8_t label;
                int16_t x;
                int16_t y;
                int16_t z;
            }acc;//加速度0.001g 7
            struct {
                uint8_t label;
                int16_t x;
                int16_t y;
                int16_t z;
            }ang_v;//角速度0.1°/s 7
            struct {
                uint8_t label;
                int16_t x;
                int16_t y;
                int16_t z;
            }magnetic;//磁场强度0.001Gauss 7
            struct {
                uint8_t label;
                int16_t pitch;//for y
                int16_t roll;//for x
                int16_t yaw;//for z
            }euler_ang;//欧拉角0.01° or 0.1° 7
            struct {
                uint8_t label;
                float pressure;
            }air;//气压pa 5
        }data_90;
        uint8_t buf[DRV_HI229UM_BUF_SIZE];
    };
#pragma pack()
    UART_HandleTypeDef* huart=nullptr;
    union hi229um_raw raw={0};
    bool is_using_91_mode=true;
    osSemaphoreId_t rx_semaphore=nullptr;
    const float g = 9.80665f;//重力加速度
public:
    IMU_Hi229um();
    void IMU_Hi229um_init(bool is_using_91_mode, UART_HandleTypeDef *huart, osSemaphoreId_t sem_rx);
    void start_receive_dma();
    void update_gyro();
    void update_acc();
    void update_msg();
    void init_gyro_offset();
    void init_acc_offset();
    void reset_imu();
    void update_imu();
    bool is_data_legal();
    void instruction_set(const char *instruction);
protected:
    static uint16_t U2(uint8_t *p) {uint16_t u;memcpy(&u, p, 2);return u;}
    bool is_crc_passing();
    void update_single_gyro(IMU::imu_e imuE, float ang_ceiling);
};

extern const char mode_cmd_91[];
extern const char mode_cmd_90[];
extern const char mode_cmd_6_axis[];
extern const char mode_cmd_9_axis[];
extern const char dir_cmd_00[];
extern const char dir_cmd_x_90[];
extern const char dir_cmd_x_neg_90[];
extern const char dir_cmd_x_180[];
extern const char dir_cmd_y_90[];
extern const char dir_cmd_y_neg_90[];
extern const char dir_cmd_y_180[];
extern const char dir_cmd_z_90[];
extern const char dir_cmd_z_neg_90[];
extern const char dir_cmd_z_180[];
extern const char dir_cmd_x_180_y_90[];

extern const char dir_cmd_921600[];
extern const char dir_cmd_460800[];
extern const char dir_cmd_115200[];
extern const char dir_cmd_rst[];
extern const char dir_cmd_400[];

#endif //__DRV_HI229UM_H
