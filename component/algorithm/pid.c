//
// Created by Henson on 2023-11-18.
//

#include "pid.h"
static void pid_param_init(
        struct pid * pid,
        float maxout,
        float integral_limit,
        float kp,
        float ki,
        float kd) {

    pid->param.integral_higher_limit = integral_limit;
    pid->param.integral_lower_limit = -integral_limit;
    pid->param.max_out = maxout;

    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(struct pid *pid, float kp, float ki, float kd) {
    if (kp >= 0)pid->param.p = kp;
    if(ABS(ki) <= 1e-6){
        pid->param.i = 0;
        pid->iout = 0;
    }
    else if (ki > 0)
        pid->param.i = ki;
    if (kd >= 0)pid->param.d = kd;
//    pid->pout = 0;
//    pid->iout = 0;
//    pid->dout = 0;
//    pid->out = 0;
}

//pid参数完全不变，但是消除从前的i的记录的影响
void pid_clear_mem(struct pid *pid){
    pid->iout = 0;
}

void pid_delete(struct pid *pid) {//输出为0
    pid->param.p = 0;
    pid->param.i = 0;
    pid->param.d = 0;
    pid->iout = 0;
}


/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output   0-outmax
  */ //0-outmax已限幅
__RAM_FUNC float pid_calculate(struct pid *pid, float get, float set) {
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if (fabsf(pid->param.input_max_err) > 1e-6)
        abs_limit(&(pid->err), pid->param.input_max_err);
    pid->pout = pid->param.p * pid->err;
    pid->iout += pid->param.i * pid->err;
//    pid->iout += pid->param.i * abs_zero(pid->err,0.1);
    pid->dout = pid->param.d * (pid->err - pid->last_err);

//    abs_limit(&(pid->iout), pid->param.integral_limit);
    val_limit(&(pid->iout), pid->param.integral_higher_limit,pid->param.integral_lower_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.max_out);
    pid->last_err = pid->err;

    if (pid->enable == 0) {
        pid->out = 0;
    }
    return pid->out;
}


/**
  * @brief     pid_increase
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float pid_increase(struct pid *pid, float get, float set) {
    pid->get = get;
    pid->set = set;
    pid->err = set - get;

    if (fabsf(pid->param.input_max_err)>1e-6)
        abs_limit(&(pid->err), pid->param.input_max_err);

    pid->pout = pid->param.p * (pid->err-pid->last_err);
    pid->iout = pid->param.i * pid->err;
    pid->dout = pid->param.d * (pid->err - 2 * pid->last_err + pid->penultimate_err);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.max_out);
    if (pid->enable == 0) {
        pid->out = 0;
    }
    pid->penultimate_err = pid->last_err;
    pid->last_err = pid->err;
    return pid->out;
}

/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void pid_struct_init(
        struct pid *pid,
        float maxout,
        float integral_limit,

        float kp,
        float ki,
        float kd)
{
    pid->enable = 1;
    pid->f_param_init = pid_param_init;
    pid->f_pid_clear_mem = pid_clear_mem;
    pid->f_pid_reset = pid_reset;
    pid->f_pid_delete = pid_delete;

    pid->f_param_init(pid, maxout, integral_limit, kp, ki, kd);
}

void set_pid_integral_limit(struct pid *pid,float max,float min){
    pid->param.integral_higher_limit = max;
    pid->param.integral_lower_limit = min;
}
