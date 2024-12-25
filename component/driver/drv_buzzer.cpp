//
// Created by Henson on 2024-03-27.
//

#include "drv_buzzer.h"

bool is_buzzer_on = false;
void buzzer_init(){
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void buzzer_on() {
    if(!is_buzzer_on){
        __HAL_TIM_PRESCALER(&htim4, 10);//0-1000  频率
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 18000);//10000-20000  音量  18000
        is_buzzer_on = true;
    }
}

void buzzer_off(void) {
    if(is_buzzer_on){
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
        is_buzzer_on = false;
    }
}
