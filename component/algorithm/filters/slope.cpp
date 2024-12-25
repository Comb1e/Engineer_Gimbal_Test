//
// Created by Henson on 2024-01-17.
//

#include "slope.h"

Slope::Slope(int32_t scale) {
    count = 0;
    this->scale = scale;
}

float Slope::slope_calc() {
    if (scale <= 0)
        return 0;
    count++;
    if (count >= scale)
        count = scale;

    out = count / ((float)scale);
    return out;
}

Slope::Slope(float out,float acc,float dec,float out_max,float target){
    slope_speed.out = out;
    slope_speed.acc = acc;
    slope_speed.dec = dec;
    slope_speed.out_max = out_max;
    slope_speed.target = target;
}

float Slope::get_slope_speed(float target_speed){
    //限制输入的最大速度
    if(target_speed>slope_speed.out_max){
        target_speed = slope_speed.out_max;
    }else if(target_speed < -slope_speed.out_max){
        target_speed = -slope_speed.out_max;
    }
    //判断速度方向和 加减速状态
    bool is_acc = true;
    int dir; //1 向前 -1 向后
    if(slope_speed.out>0){
        dir = 1;
        if(target_speed<slope_speed.out)
            is_acc=false;//比上一次设定速度小
    }else{
        dir = -1;
        if(target_speed>slope_speed.out)
            is_acc=false;//绝对值比上一次设定速度小
    }
    //计算速度差
    float speed_err = fabsf(target_speed - slope_speed.out);

    //计算速度大小
    float acc;
    if(is_acc){
        acc = slope_speed.acc;
    }else{//减速
        acc = - slope_speed.dec;
    }

    if(speed_err < fabsf(acc))//acc本身很小,speed如果相差不大就不用滤波
        slope_speed.out = target_speed;
    else {
        slope_speed.out = fabsf(slope_speed.out) + acc;//
        slope_speed.out = slope_speed.out * (float)dir;//这两步简化了if判断来控制加减速
    }
    return slope_speed.out;
}