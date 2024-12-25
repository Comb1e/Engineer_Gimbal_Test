//
// Created by Henson on 2023-11-18.
//

#ifndef __PID_H
#define __PID_H

#include "user_lib.h"
#include <math.h>

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

struct pid_param {
    float p;
    float i;
    float d;
    float input_max_err;//不用
    float max_out;
    float integral_higher_limit;
    float integral_lower_limit;
};

struct pid{
    struct pid_param param;

    bool enable;

    float set;
    float get;

    float err;
    float last_err;
    float penultimate_err;

    float pout;
    float iout;
    float dout;
    float out;

    //以下两个函数指针基本用不到，仅供学习
    void (*f_param_init)(struct pid *pid,
                         float max_output,
                         float integral_limit,
                         float p,
                         float i,
                         float d);

    void (*f_pid_clear_mem)(struct pid *pid);
    void (*f_pid_delete)(struct pid *pid);
    void (*f_pid_reset)(struct pid *pid, float p, float i, float d);
};


void pid_struct_init(
        struct pid * pid,
        float maxout,
        float intergral_limit,

        float kp,
        float ki,
        float kd);

//__attribute__ ((section(".RamFunc")))
__RAM_FUNC float pid_calculate(struct pid *pid, float get, float set);
float pid_increase(struct pid *pid, float get, float set);
void set_pid_integral_limit(struct pid *pid,float max,float min);


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

#endif //__PID_H


