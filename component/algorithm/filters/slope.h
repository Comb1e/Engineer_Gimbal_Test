//
// Created by Henson on 2024-01-17.
//

#ifndef __SLOPE_H
#define __SLOPE_H

#include "common_inc.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

class Slope {
public:
    int32_t count;
    int32_t scale;
    float out;
    typedef struct {
        float out;//输出
        float acc;//加速增量
        float dec;//减速增量
        float out_max;//输出限幅
        float target;
        uint32_t current_time;
        uint32_t last_time;
    } slope_speed_t;

    slope_speed_t slope_speed;
public:
    Slope(int32_t scale);
    Slope(float out, float acc, float dec, float out_max, float target);
    float slope_calc();
    float get_slope_speed(float target_speed);
};

#endif //__SLOPE_H
