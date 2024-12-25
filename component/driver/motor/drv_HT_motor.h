//
// Created by Henson on 2024-01-15.
//

#ifndef __DRV_HT_MOTOR_H
#define __DRV_HT_MOTOR_H

#include "drv_motor.h"
/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

//class HT_Motor:public Motor{
//public:
//    uint32_t motor_id;
//    uint8_t send_data[8];
//    uint8_t get_motor_measure(const uint8_t data[8]);
//    void get_send_data();
//
//private:
//    uint16_t p_temp;
//    uint16_t v_temp;
//    uint16_t kp_temp;
//    uint16_t kv_temp;
//    uint16_t torque_temp;
//    uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
//        float span = x_max - x_min;
//        float offset = x_min;
//
//        return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
//    }
//    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
//        float span = x_max - x_min;
//        float offset = x_min;
//        return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
//    }
//};
//
//
//
//class HT_Motor_2205:public HT_Motor{
//
//public:
//
//
//private:
//
//
//};
#endif //__DRV_HT_MOTOR_H
